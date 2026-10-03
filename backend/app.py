"""Drone mission supervisor demo: FastAPI backend.

Simplified simulation -- AI mission supervision, not flight stabilization.
Chain shown in the UI: Jev proposal -> deterministic safety check -> executed action.
"""
import asyncio
import math
import os
import time
from collections import deque
from contextlib import asynccontextmanager
from datetime import datetime
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from . import sim as simmod
from .jev import JevError, make_client
from .recorder import Recorder

ROOT = Path(__file__).resolve().parent
env_file = ROOT.parent / ".env"
if env_file.exists():
    for line in env_file.read_text().splitlines():
        if "=" in line and not line.strip().startswith("#"):
            k, v = line.split("=", 1)
            os.environ.setdefault(k.strip(), v.strip())

TICK = 0.1
DECISION_PERIOD = 1.0
SPEED = float(os.getenv("SIM_SPEED", "1"))             # sim-seconds per wall-second (headless soak)
DEADLINE_MS = float(os.getenv("JEV_DEADLINE_MS", "1000"))  # a decision slower than this misses its cycle
TELEMETRY_PERIOD = 1.0
RUNS_DIR = ROOT.parent / "runs"
MISSION_PRIORITY = "Complete the inspection waypoints without unnecessary delay, while preserving battery reserve."


LOW_BATTERY_OFFER = 45.0   # %: from here Jev is offered LAND_AT_SITE (nearest waypoint/site) to prevent a forced landing
N_DRONES = min(int(os.getenv("N_DRONES", "10")), len(simmod.DRONES))


class Agent:
    """One drone + its own Jev supervision state."""

    def __init__(self, fleet, idx):
        self.fleet = fleet
        self.sim = simmod.Sim(fleet.world, idx, emit=self._emit)
        self.jev = fleet.jev
        self.name = self.sim.name
        self.ai_enabled = True
        self.decisions = deque(maxlen=30)
        self.pending = None
        self.last_choice = self.last_proposal = None
        self.offered_level = None
        self.jev_status = "idle"
        self.fallback_active = False
        self.last_brake = -10.0

    def _emit(self, kind, msg, **kw):
        self.fleet.emit(self.name, kind, msg)
        if kind == "SAFETY" and kw.get("forced"):
            # deterministic rule overrode whatever Jev last proposed
            self._record(proposal=self.last_proposal, verdict="OVERRIDDEN", rule=kw["rule"],
                         reason=msg, executed=kw["forced"], source="SAFETY")

    def _record(self, proposal, verdict, rule, reason, executed, source, error=None, snap_age=None,
                state=None, latency_ms=None):
        self.fleet.did += 1
        row = {
            "id": self.fleet.did, "ts": datetime.now().strftime("%H:%M:%S.%f")[:-3], "t": round(self.sim.t, 1),
            "proposal": proposal, "verdict": verdict, "rule": rule, "reason": reason,
            "executed": executed, "source": source, "error": error, "snapshot_age": snap_age,
        }
        self.decisions.appendleft(row)
        s = self.sim
        self.fleet.log_decision({
            **{k: row[k] for k in ("id", "t", "verdict", "rule", "reason", "executed", "source", "error")},
            "wall": datetime.now().isoformat(timespec="milliseconds"),
            "drone": self.name, "scenario": s.scenario, "lap": s.laps + 1, "mode": s.mode,
            "battery": round(s.battery, 1), "latency_ms": latency_ms,
            "proposal": proposal, "jev_state": state,
            # ground truth the drone cannot see, for offline calibration
            "truth": {"obstruction_active": s.obstruction_active(), "in_wind": s.in_wind(), "traffic": self._traffic_truth(),
                      "home_reachable": s.battery >= s.return_cost()},
        })
        if latency_ms is not None:
            self.fleet.note_latency(self.name, latency_ms, error is not None)

    def reset(self):
        self.sim.reset()
        self.decisions.clear()
        self.pending = None
        self.last_choice = self.last_proposal = None
        self.offered_level = None
        self.ai_enabled = True
        self.fallback_active = False
        self.jev_status = "idle"

    def operator(self, action):
        s = self.sim
        if action == "hold":
            self.ai_enabled = False
            s.epoch += 1  # any in-flight Jev response is now stale
            self.pending = None
            s.apply("HOLD", "OPERATOR")
            self._emit("OPERATOR", "Operator override: AI supervision off, holding (in-flight Jev response discarded)")
        elif action == "resume":
            self.ai_enabled = True
            if s.mode == "HOLD":
                s.apply("HOLD", "FALLBACK")
                s.hold_started = s.t
            self._emit("OPERATOR", "AI supervision resumed")
        else:
            raise ValueError(action)

    # ---- traffic ------------------------------------------------------------------
    def _relevant(self, q):
        """Is q airborne traffic this drone should know about? Anything near in altitude, plus any drone
        climbing or descending (takeoff/landing columns sweep through every level)."""
        s = self.sim
        if q is s or q.mode in simmod.GROUNDED:
            return False
        return abs(q.alt - s.alt) < simmod.TRAFFIC_ALT_RANGE or q.mode in ("TAKEOFF", "LANDING", "DIVERT")

    def _traffic(self):
        """Nearby airborne drones in 3D, as this drone can observe them (altitude, vertical speed and target
        altitude as a transponder would give; no exact closest-approach)."""
        s, out = self.sim, []
        for o in self.fleet.agents:
            q = o.sim
            if not self._relevant(q):
                continue
            d = simmod.dist(s.pos, q.pos)
            if d > simmod.TRAFFIC_RANGE:
                continue
            (vx, vy), (wx, wy) = s.velocity(), q.velocity()
            rx, ry = q.pos[0] - s.pos[0], q.pos[1] - s.pos[1]
            closing = -((rx * (wx - vx) + ry * (wy - vy)) / d) if d > 1e-6 else 0.0
            ang = lambda dx, dy: math.degrees(math.atan2(dy, dx))
            mine = ang(vx, vy) if (vx or vy) else ang(rx, ry)
            theirs = ang(wx, wy) if (wx or wy) else mine
            dz = q.alt - s.alt
            out.append({
                "drone": q.name, "distance_m": round(d), "closing_speed_mps": round(closing, 1),
                "bearing_from_my_heading_deg": round((ang(rx, ry) - mine + 180) % 360 - 180),
                "their_heading_relative_deg": round((theirs - mine + 180) % 360 - 180),
                "their_speed_mps": round(q.speed, 1), "their_battery_pct": round(q.battery, 1),
                "their_mode": q.mode, "same_next_waypoint": bool(s.target_name() and s.target_name() == q.target_name()),
                "seconds_to_contact": round(d / closing, 1) if closing > 0.5 else None,   # distance / closing speed
                "same_level": abs(dz) < simmod.SEPARATION_V,
                "their_altitude_delta_m": round(dz),
                "their_vertical_speed_mps": round(q.vz(), 1),
                "their_target_altitude_m": round(q.alt_target()),
                # a drone taking off / landing / diverting cannot decide or yield, so the other drone never has priority over it
                "i_have_priority": q.mode in ("MISSION", "HOLD") and (s.battery, s.name) < (q.battery, q.name),
            })
        # most threatening first: near horizontally and near in altitude
        return sorted(out, key=lambda t: t["distance_m"] + 4 * max(0, abs(t["their_altitude_delta_m"]) - simmod.SEPARATION_V))[:5]

    def _traffic_truth(self):
        """Hidden ground truth for calibration: would an unmanaged pass lose 3D separation within the horizon?"""
        s = self.sim
        worst = None
        for o in self.fleet.agents:
            q = o.sim
            if not self._relevant(q) or simmod.dist(s.pos, q.pos) > simmod.TRAFFIC_RANGE:
                continue
            h, t, loss = simmod.cpa3(s, q)
            if worst is None or (not loss, h) < (not worst[3], worst[0]):
                worst = (h, t, q.name, loss)
        return None if worst is None else {"cpa_m": round(worst[0], 1), "t_s": round(worst[1], 1), "with": worst[2],
                                           "loss_of_separation": worst[3]}

    # ---- decision cycle -----------------------------------------------------------
    def _admissible(self):
        """Deterministically pre-checked candidates (Jev may only choose among these)."""
        s = self.sim
        acts = ["CONTINUE", "HOLD"]
        if self._traffic():
            if s.speed > simmod.YIELD_SPEED + 1.0:   # slowing only means something above the yield cap
                acts.append("SLOW_FOR_TRAFFIC")
            if self.fleet.level_target(s) is not None:
                acts.append("CHANGE_LEVEL")
        if s.alternate_available():
            acts += [a for a in s.alts if s.alt_feasible(a)]
        home_ok = s.battery >= s.return_cost()
        if home_ok:
            acts.append("RETURN_HOME")
        if s.best_site() and (not home_ok or s.battery - s.return_cost() < 35.0 or s.battery <= LOW_BATTERY_OFFER):
            acts.append("LAND_AT_SITE")
        return acts + ["ABSTAIN"]

    def _consequences(self, admissible):
        s = self.sim
        out = {}
        for a in admissible:
            if a == "CONTINUE":
                out[a] = f"Finishing the plan and returning costs about {s.mission_cost():.0f}% battery."
            elif a == "SLOW_FOR_TRAFFIC":
                out[a] = f"Cap speed at {simmod.YIELD_SPEED:.0f} m/s for {simmod.TRAFFIC_SLOW_S:.0f}s; costs a little time, keeps flying."
            elif a == "CHANGE_LEVEL":
                tgt = self.offered_level = self.fleet.level_target(s)
                out[a] = (f"{'Climb' if tgt > s.alt else 'Descend'} to a clear level at {tgt:.0f} m (no other drone near or heading to it) for "
                          f"{simmod.TRAFFIC_LEVEL_S:.0f}s; route and speed unchanged.")
            elif a == "HOLD":
                out[a] = f"Hovering burns {s.last_drain or 0.5:.2f}%/s; hold is limited to {simmod.HOLD_LIMIT_JEV:.0f}s."
            elif a in s.alts:
                out[a] = (f"Detour adds {s.alt_cost(a) - s.direct_cost():.0f}% battery versus the blocked corridor; "
                          f"{s.battery - s.alt_cost(a):.0f}% would remain after clearing it and returning home.")
            elif a == "RETURN_HOME":
                out[a] = f"{s.home_distance():.0f} m to home, about {s.return_cost():.0f}% battery."
            elif a == "LAND_AT_SITE":
                site = s.best_site()
                out[a] = f"Nearest reachable safe landing zone {site['name']} is {site['dist']:.0f} m away, about {site['cost']:.0f}% battery."
        return out

    def _jev_state(self, admissible):
        s = self.sim
        return {
            "phase": s.mode,
            "my_altitude_m": round(s.alt),
            "my_vertical_speed_mps": round(s.vz(), 1),
            "my_target_altitude_m": round(s.alt_target()),
            "my_cruise_altitude_m": round(s.cruise_alt),
            "level_change_seconds_left": max(0, round(s.level_until - s.t)) if s.level_alt is not None else 0,
            "nearest_landing_zone": (lambda z: z and {"name": z["name"], "distance_m": round(z["dist"])})(s.best_site()),
            "mission_priority": MISSION_PRIORITY,
            "battery_pct": round(s.battery, 1),
            "battery_needed_to_return_pct": round(s.return_cost(), 1),
            "mission_cost_remaining_pct": round(s.mission_cost(), 1),
            "distance_to_home_m": round(s.home_distance()),
            "next_waypoint": s.target_name(),
            "waypoints_remaining": max(0, len(s.plan) - s.idx),
            "route": s.route,
            "speed_mps": round(s.speed, 1),
            "in_gusty_zone": s.in_wind(),
            "drain_pct_per_s": round(s.last_drain, 2),
            "stopped_at_corridor_entry": s.blocked_stop,
            "seconds_in_hold": None if s.hold_started is None else round(s.t - s.hold_started),
            "corridor_observations": (
                "alternate route in use; main corridor no longer relevant" if s.route != "MAIN" else
                "main corridor already passed" if not s.corridor_ahead() else
                [{"seconds_ago": round(s.t - t), "reading": r} for t, r in s.obs]
                or "corridor not yet in sensor range"),
            "traffic": self._traffic(),
            "admissible_actions": admissible,
            "candidate_consequences": self._consequences(admissible),
        }

    async def decide_once(self):
        s = self.sim
        if not self.ai_enabled or s.mode not in ("MISSION", "HOLD"):
            return
        epoch, t0 = s.epoch, time.monotonic()
        admissible = self._admissible()
        state = self._jev_state(admissible)
        self.pending = {"since": s.t, "admissible": admissible}
        proposal = error = None
        try:
            if s.world.cfg.get("outage"):
                raise JevError("injected outage (no call made)")
            proposal = await self.jev.decide(state, admissible)
            self.jev_status = "ok"
        except JevError as e:
            error = str(e)
            self.jev_status = "error"
        self.pending = None
        elapsed = time.monotonic() - t0
        age = round(elapsed, 2)
        latency_ms = None if s.world.cfg.get("outage") else round(elapsed * 1000, 1)

        if s.epoch != epoch or not self.ai_enabled:
            self._emit("SAFETY", "DISCARDED_STALE: Jev response from before reset/operator override ignored")
            return
        if s.mode not in ("MISSION", "HOLD"):
            return  # a deterministic rule took over while we waited; already recorded
        if proposal:
            proposal.pop("request", None)
            self.last_proposal = proposal
        self._arbitrate(proposal, error, age, state, latency_ms)

    NOUL_YIELD = 0.55          # traffic-noul at/above which a non-priority drone may not just CONTINUE
    NOUL_YIELD_SLOW = 0.70     # ... or merely SLOW (slowing does not separate drones on a collision course)
    RISK_YIELD = 3.0           # same gate on Jev's collision_risk score (0-4): "high risk" or worse
    RISK_YIELD_SLOW = 3.5

    SHORT_CONTACT_S = 6.0      # time to contact below which the lower thresholds apply
    NOUL_YIELD_SHORT = 0.40
    RISK_YIELD_SHORT = 2.0

    def _noul_yield(self, proposal, state, choice):
        """Use Jev's own traffic_conflict noul and collision_risk score to gate its maneuver: if it signals a likely
        conflict but picked CONTINUE/SLOW while the drone it is closing on has priority, the drone yields instead.
        Thresholds drop when time to contact is short (closing fast at close range leaves no time for a late answer).
        Returns the traffic entry being yielded to, else None."""
        tn = proposal.get("traffic_noul") if proposal else None
        risk = proposal.get("collision_risk") if proposal else None     # score 0-4: Jev's own collision-risk rating
        if (tn is None and risk is None) or not state or choice not in ("CONTINUE", "SLOW_FOR_TRAFFIC"):
            return None
        if self.fleet.level_in_progress(self.sim):
            return None
        threats = [t for t in state.get("traffic", []) if t["closing_speed_mps"] > 1.0 and t["distance_m"] <= 120
                   and not t["i_have_priority"]
                   and (t["same_level"] or t["their_mode"] in ("TAKEOFF", "LANDING", "DIVERT"))]
        if not threats:
            return None
        nearest = min(threats, key=lambda t: t["distance_m"])
        ttc = nearest.get("seconds_to_contact")
        if ttc is not None and ttc < self.SHORT_CONTACT_S:
            tn_thr, risk_thr = self.NOUL_YIELD_SHORT, self.RISK_YIELD_SHORT
        elif choice == "SLOW_FOR_TRAFFIC":
            tn_thr, risk_thr = self.NOUL_YIELD_SLOW, self.RISK_YIELD_SLOW
        else:
            tn_thr, risk_thr = self.NOUL_YIELD, self.RISK_YIELD
        return nearest if ((tn or 0.0) >= tn_thr or (risk or 0.0) >= risk_thr) else None

    def _arbitrate(self, proposal, error, age, state=None, latency_ms=None):
        """Deterministic safety layer between Jev proposal and execution."""
        s = self.sim
        verdict, rule, reason, action, source = "ACCEPTED", None, "Proposal passed safety checks", None, "JEV"
        choice = proposal["choice"] if proposal else None

        if error:
            verdict, rule, action, source = "FALLBACK", "JEV_FAILURE", "HOLD", "FALLBACK"
            reason = f"Jev unavailable ({error}); bounded hold, then return"
        elif choice == "ABSTAIN":
            verdict, rule, action, source = "FALLBACK", "ABSTAIN", "HOLD", "FALLBACK"
            reason = "Jev abstained; bounded hold, then return"
        elif choice in s.alts and not (s.alternate_available() and s.alt_feasible(choice)):
            verdict, rule, action, source = "REJECTED", "ROUTE_UNAVAILABLE", "HOLD", "FALLBACK"
            reason = "Alternate route unavailable or not energy-feasible; proposal rejected"
        elif choice == "LAND_AT_SITE" and not s.best_site():
            verdict, rule, action, source = "REJECTED", "NO_SITE", "HOLD", "FALLBACK"
            reason = "No approved landing site reachable; proposal rejected"
        elif choice == "CHANGE_LEVEL" and self.fleet.level_target(s) is None:
            verdict, rule, action, source = "REJECTED", "NO_FREE_LEVEL", "SLOW_FOR_TRAFFIC", "FALLBACK"
            reason = "No clear level available; slowing instead"
        elif choice == "RETURN_HOME" and s.battery < s.return_cost():
            verdict, rule, source = "OVERRIDDEN", "RETURN_INFEASIBLE", "SAFETY"
            action = "LAND_AT_SITE" if s.best_site() else "LAND"
            reason = "Not enough battery to reach home; diverting instead"
        elif (yielding_to := self._noul_yield(proposal, state, choice)) is not None:
            action = "CHANGE_LEVEL" if self.fleet.level_target(s) is not None else "HOLD"
            verdict, rule, source = "OVERRIDDEN", "NOUL_YIELD", "SAFETY"
            reason = (f"Traffic-noul {proposal.get('traffic_noul') or 0:.2f}, collision risk {proposal.get('collision_risk')}/4: {yielding_to['drone']} is {yielding_to['distance_m']} m away "
                      f"closing at {yielding_to['closing_speed_mps']} m/s and has priority; yielding by {action} instead of {choice}")
        elif choice == "CONTINUE" and s.next_leg_blocked():
            verdict, rule, action, source = "OVERRIDDEN", "BLOCKED_PATH", "HOLD", "SAFETY"
            reason = "Corridor is blocked; blocked paths cannot be entered"
        else:
            action = choice

        was_fallback = self.fallback_active
        self.fallback_active = verdict == "FALLBACK"
        if action == "CHANGE_LEVEL":
            offered = self.offered_level
            ok = offered is not None and ((self.fleet.level_in_progress(s) and offered == s.level_alt) or
                                          (abs(offered - s.alt) >= simmod.SEPARATION_V and self.fleet.level_clear(s, offered)))
            s.level_alt = offered if ok else self.fleet.level_target(s)   # what Jev was told, unless it went stale
        changed = s.apply(action, source, rule)
        self._record(proposal, verdict, rule, reason, action, source, error=error, snap_age=age,
                     state=state, latency_ms=latency_ms)
        if verdict != "ACCEPTED":
            self._emit("SAFETY" if verdict in ("OVERRIDDEN", "REJECTED") else "FALLBACK",
                       f"{verdict}: {reason}" + (f" [{rule}]" if rule else ""))
        elif choice != self.last_choice or changed or was_fallback:
            sc = f" (score {proposal['score']:.1f}/{proposal['score_max']}, noul {proposal['noul']:.2f})"
            self._emit("JEV", f"Jev proposed {choice}{sc} -> executed {action}")
        self.last_choice = choice

    def view_control(self):
        if not self.ai_enabled:
            return "OPERATOR"
        return "FALLBACK" if self.fallback_active else "AI SUPERVISION"

    def view(self):
        control = self.view_control()
        return {
            "sim": self.sim.snapshot(), "control": control,
            "jev": {"status": self.jev_status, "pending": self.pending},
            "decisions": list(self.decisions),
        }


class Fleet:
    def __init__(self):
        self.events = deque(maxlen=300)
        self._eid = 0
        self.did = 0
        self.world = simmod.World()
        self.jev = make_client()
        self.agents = [Agent(self, i) for i in range(N_DRONES)]
        self.rec = None
        self.lat = []                      # Jev round-trip latencies (ms), successful calls only
        self.lat_errors = 0
        self.collisions = 0
        self.crashes = 0
        self.conflicts = 0
        self._near = set()                 # pairs currently inside the separation envelope
        self.backstop = os.getenv("BACKSTOP", "1") != "0"
        self.backstop_fires = 0
        self._rtc = set()                  # drones currently holding a level instead of returning to cruise
        self._episodes = {}                # same-level threats currently in progress
        self.threats = 0
        self.outcomes = {}
        self._next_tel = 0.0
        self.counts = {}                   # (verdict, rule) -> n, across resets of this run

    # ---- recording ------------------------------------------------------------
    def start_recording(self):
        if os.getenv("RECORD", "1") != "0" and self.rec is None:
            self.rec = Recorder(RUNS_DIR)

    def finish(self):
        """Close any encounter still in progress so its outcome is counted."""
        for key in list(self._episodes):
            self._close_episode(key)

    def stop_recording(self):
        self.finish()
        if self.rec:
            self.rec.summary(self.metrics())
            self.rec.close()
            self.rec = None

    def log_decision(self, row):
        key = f"{row['verdict']}:{row['rule'] or '-'}"
        self.counts[key] = self.counts.get(key, 0) + 1
        if self.rec:
            self.rec.write("decisions", row)

    def note_latency(self, drone, ms, failed):
        if failed:
            self.lat_errors += 1
        else:
            self.lat.append(ms)

    def metrics(self):
        xs = sorted(self.lat)

        def pct(q):
            return round(xs[min(len(xs) - 1, int(q * len(xs)))], 1) if xs else None
        return {
            "sim_time_s": round(self.world.t, 1), "scenario": self.world.scenario, "loop": self.world.loop,
            "jev": {"calls": len(xs) + self.lat_errors, "errors": self.lat_errors,
                    "avg_ms": round(sum(xs) / len(xs), 1) if xs else None,
                    "p50_ms": pct(0.5), "p95_ms": pct(0.95), "p99_ms": pct(0.99),
                    "max_ms": round(xs[-1], 1) if xs else None,
                    "deadline_ms": DEADLINE_MS, "deadline_misses": sum(1 for x in xs if x > DEADLINE_MS)},
            "decisions": dict(sorted(self.counts.items())),
            "crashes": self.crashes, "collisions": self.collisions, "backstop": {"enabled": self.backstop, "activations": self.backstop_fires},
            "traffic": {"threat_encounters": self.threats, "outcomes": dict(sorted(self.outcomes.items()))}, "separation_losses": self.conflicts,
            "laps": {a.name: a.sim.laps for a in self.agents},
        }

    def emit(self, drone, kind, msg):
        self._eid += 1
        ev = {"id": self._eid, "ts": datetime.now().strftime("%H:%M:%S.%f")[:-3],
              "t": round(self.world.t, 1), "drone": drone, "kind": kind, "msg": msg}
        self.events.append(ev)
        if kind == "CRASH":
            self.crashes += 1
        if self.rec:
            self.rec.write("events", {**ev, "wall": datetime.now().isoformat(timespec="milliseconds"),
                                      "scenario": self.world.scenario})

    def reset(self, scenario):
        self._near.clear()
        self._rtc.clear()
        self._episodes.clear()
        self._next_tel = 0.0
        self.world.reset(scenario)
        self.events.clear()
        for a in self.agents:
            a.reset()
        self.emit("-", "STATE", f"Reset: {simmod.SCENARIOS[scenario]['label']} (scenario hidden from Jev)")

    def start(self):
        for a in self.agents:
            a.sim.start()  # all drones lift off together

    def set_loop(self, enabled):
        self.world.loop = bool(enabled)
        self.emit("-", "STATE", f"Continuous mission loop {'ON' if enabled else 'OFF'}")

    FLYING = ("TAKEOFF", "MISSION", "HOLD", "RETURN_HOME", "DIVERT", "LANDING")

    def _backstop(self, dt=0.1):
        """Deterministic traffic floor, independent of Jev (rule TRAFFIC_BACKSTOP).
        1. Braking: a drone never closes on a same-level drone ahead faster than it could stop STOP_H short of it.
        2. Deadlock escape: held for ESCAPE_AFTER seconds by a drone that is stationary (or by a higher-priority one),
           it steps up one level, but only into a column that is clear. It descends again once the column below is clear.
        Priority: landing/taking-off first, then lowest battery.  Every activation is recorded as an OVERRIDDEN decision."""
        live = [a for a in self.agents if a.sim.mode in self.FLYING]
        for a in live:
            a.sim.speed_limit, a.sim.blocker = None, None
        if not self.backstop:
            for a in live:
                a.sim.bs_alt = None
            self._return_gate(live)
            return
        for a in live:                                                   # pass 1: braking limits
            s = a.sim
            tgt = s.aim()
            if s.mode in ("LANDING", "TAKEOFF") or tgt is None:
                continue
            d0 = simmod.dist(s.pos, tgt) or 1.0
            hx, hy = (tgt[0] - s.pos[0]) / d0, (tgt[1] - s.pos[1]) / d0
            best = None
            for b in live:
                o = b.sim
                if o is s or not (o.occupies(s.alt) or s.occupies(o.alt)):
                    continue
                d = simmod.dist(s.pos, o.pos)
                if d > simmod.BRAKE_RANGE or (o.pos[0] - s.pos[0]) * hx + (o.pos[1] - s.pos[1]) * hy <= 0:
                    continue   # other is not ahead: free to move (including away from a drone that is too close)
                # the other drone needs room to stop too, if it is moving toward us: stopping distances add up
                wx, wy = o.velocity()
                toward = max(0.0, ((s.pos[0] - o.pos[0]) * wx + (s.pos[1] - o.pos[1]) * wy) / d) if d > 1e-6 else 0.0
                room = d - simmod.STOP_H - toward * toward / (2 * simmod.MAX_ACCEL)
                lim = 0.0 if room <= 0 else math.sqrt(2 * simmod.MAX_ACCEL * room)
                if best is None or lim < best[0]:
                    best = (lim, o)
            if best and best[0] < simmod.CRUISE_SPEED:
                s.speed_limit, s.blocker = best[0], best[1]
                if best[0] < s.speed - 1.0 and self.world.t - a.last_brake > 4.0:
                    a.last_brake = self.world.t
                    self._note_backstop(a, best[1], "BRAKE_FOR_TRAFFIC",
                                        f"{s.name} {simmod.dist(s.pos, best[1].pos):.0f} m from {best[1].name} at the same level, braking to stay {simmod.STOP_H:.0f} m clear")
        for a in live:                                                   # vertical moves must not sweep through traffic
            s = a.sim
            s.vert_hold, s.nudge = False, None
            if s.mode == "TAKEOFF":
                continue
            if s.mode == "LANDING":
                if self._vert_blocked(s, 0.0, radius=simmod.LAND_CLEAR_H):
                    s.vert_hold = True
                    below = [o for o in live if o.sim is not s and simmod.dist(s.pos, o.sim.pos) < simmod.LAND_CLEAR_H
                             and o.sim.alt < s.alt]
                    if below:
                        ax = sum(s.pos[0] - o.sim.pos[0] for o in below) or 1.0
                        ay = sum(s.pos[1] - o.sim.pos[1] for o in below)
                        n = math.hypot(ax, ay) or 1.0
                        s.nudge = (ax / n, ay / n)
                continue
            goal = s.goal_alt(0.0) if s.mode == "DIVERT" else s.goal_alt()
            if abs(goal - s.alt) > 0.5:
                s.vert_hold = self._vert_blocked(s, goal, radius=simmod.DIVERT_CLEAR_H if s.mode == "DIVERT" else None)
        prio = lambda x: (x.mode not in ("LANDING", "TAKEOFF"), x.battery, x.name)
        for a in live:                                                   # pass 2: deadlock escape / return to cruise
            s = a.sim
            held = s.speed_limit is not None and s.speed_limit < 1.0 and s.speed < 1.0
            s.hold_t = s.hold_t + dt if held else 0.0
            if s.mode in ("LANDING", "TAKEOFF"):
                continue
            o = s.blocker
            if held and s.hold_t > simmod.ESCAPE_AFTER and o is not None and o.mode not in ("LANDING", "TAKEOFF"):
                mutual = o.blocker is s
                if (not mutual or prio(s) > prio(o)) and o.speed < 1.0:
                    new = (s.bs_alt if s.bs_alt is not None else s.alt) + simmod.LEVEL_STEP
                    if new <= s.cruise_alt + 3 * simmod.LEVEL_STEP and not self._vert_blocked(s, new):
                        s.bs_alt = new
                        s.hold_t = 0.0
                        self._note_backstop(a, o, "ESCAPE_UP", f"{s.name} held by {o.name}, stepping up to a clear level ({new:.0f} m)")
            elif s.bs_alt is not None and not self._vert_blocked(s, s.cruise_alt):
                s.bs_alt = None                                           # column below is clear: back to cruise

    RTC_RADIUS = 100.0   # m: traffic this close that sits on the climb/descent back to cruise holds the drone's level

    def _return_gate(self, live):
        """Always on, even with the full backstop off. When a level change ends the drone flies back to its cruise
        altitude by itself (Jev has no say in that); it must not sweep through another drone's level on the way.
        It holds its current level until the path is clear. Recorded as an OVERRIDDEN RETURN_TO_CRUISE_HOLD."""
        for a in live:
            s = a.sim
            s.vert_hold, s.nudge = False, None
            returning = (s.mode in ("MISSION", "HOLD", "RETURN_HOME") and s.bs_alt is None
                         and not (s.level_alt is not None and s.t < s.level_until)
                         and abs(s.cruise_alt - s.alt) > 0.5)
            if returning and self._vert_blocked(s, s.cruise_alt, radius=self.RTC_RADIUS):
                s.vert_hold = True
                if s.name not in self._rtc:
                    self._rtc.add(s.name)
                    msg = (f"RETURN_TO_CRUISE_HOLD: {s.name} holds {s.alt:.0f} m instead of returning to "
                           f"{s.cruise_alt:.0f} m, other traffic occupies a level on the way")
                    self.emit(s.name, "SAFETY", msg)
                    a._record(proposal=a.last_proposal, verdict="OVERRIDDEN", rule="RETURN_TO_CRUISE_HOLD",
                              reason=msg, executed="HOLD_LEVEL", source="SAFETY")
            else:
                self._rtc.discard(s.name)

    def _vert_blocked(self, s, goal, radius=None):
        """Would moving from s.alt to `goal` pass through (or land in) a level nearby traffic occupies?
        The band around the drone's own level is excluded: those drones are the ones it is already separating from."""
        sgn = 1 if goal > s.alt else -1
        xs, x = [], s.alt + sgn * simmod.LEVEL_BAND
        while (x - goal) * sgn < 0:
            xs.append(x)
            x += sgn * 5.0
        xs.append(goal)
        r = simmod.COLUMN_H if radius is None else radius
        return any(o.sim is not s and o.sim.mode in self.FLYING and simmod.dist(s.pos, o.sim.pos) < r
                   and any(o.sim.occupies(x) for x in xs) for o in self.agents)

    def _note_backstop(self, agent, other, executed, msg):
        self.backstop_fires += 1
        full = f"TRAFFIC_BACKSTOP: {msg}"
        self.emit(agent.name, "SAFETY", full)
        agent._record(proposal=agent.last_proposal, verdict="OVERRIDDEN", rule="TRAFFIC_BACKSTOP", reason=full,
                      executed=executed, source="SAFETY")
        ep = self._episode_for(agent.name, other.name)
        if ep:
            ep["backstop"] = True

    def level_clear(self, s, c):
        """No other airborne drone nearby is at, heading to, or sweeping (climbing/descending through) altitude c."""
        m = simmod.SEPARATION_V + 2
        for o in self.agents:
            q = o.sim
            if q is s or q.mode in simmod.GROUNDED or simmod.dist(s.pos, q.pos) > simmod.LEVEL_CLEAR_H:
                continue
            if abs(q.alt - c) < m or abs(q.goal_alt() - c) < m:
                return False
            if q.mode in ("LANDING", "DIVERT") and c < q.alt + m:                 # sweeps every level below it
                return False
            if q.mode == "TAKEOFF" and c < q.goal_alt() + m:                      # sweeps every level up to cruise
                return False
        return True

    @staticmethod
    def level_in_progress(s):
        """A CHANGE_LEVEL window is open and the drone has not yet reached its target level."""
        return s.level_alt is not None and s.t < s.level_until and abs(s.level_alt - s.alt) > 1.0

    def level_target(self, s):
        """The level for a CHANGE_LEVEL: stay committed to the level already being flown to until its window ends
        (re-picking every decision made drones reverse mid-manoeuvre); otherwise the nearest clear one."""
        if self.level_in_progress(s):
            return s.level_alt
        return self.free_level(s)

    def path_clear(self, s, c):
        """Climbing/descending from the current altitude to level c must not sweep through another drone's
        altitude. Drones sharing our own level are what the change leaves behind, so they don't count."""
        lo, hi = sorted((s.alt, c))
        for o in self.agents:
            q = o.sim
            if q is s or q.mode in simmod.GROUNDED or simmod.dist(s.pos, q.pos) > simmod.LEVEL_CLEAR_H:
                continue
            if (abs(q.alt - s.alt) < simmod.LEVEL_BAND and q.mode not in ("TAKEOFF", "LANDING", "DIVERT")
                    and abs(q.vz()) < 0.5):   # level and not climbing/descending: it is what we are leaving behind
                continue
            x = lo
            while x <= hi:
                if q.occupies(x):
                    return False
                x += 2.0
        return True

    def free_level(self, s):
        """Nearest flight level (cruise +/- k*LEVEL_STEP) that is clear of nearby traffic."""
        cands = sorted({s.cruise_alt + k * simmod.LEVEL_STEP for k in (-2, -1, 1, 2, 3)}, key=lambda a: (abs(a - s.alt), a))
        return next((c for c in cands if c >= 25.0 and abs(c - s.alt) >= simmod.SEPARATION_V and self.level_clear(s, c)
                           and self.path_clear(s, c)), None)

    # ---- encounter bookkeeping: did Jev resolve the conflict, or did the backstop have to? -------------------
    def _episode_for(self, a, b):
        return self._episodes.get(tuple(sorted((a, b))))

    def _track_episodes(self):
        """A threat encounter starts when two same-level drones are on course to lose separation (ground truth) and
        ends when they are clear of each other or one is down. Outcome is decided at the end."""
        for i, a1 in enumerate(self.agents):
            for a2 in self.agents[i + 1:]:
                p, q = a1.sim, a2.sim
                key = tuple(sorted((p.name, q.name)))
                d = simmod.dist(p.pos, q.pos)
                ep = self._episodes.get(key)
                down = p.mode in simmod.GROUNDED or q.mode in simmod.GROUNDED
                if ep is None:
                    if (not down and d < simmod.TRAFFIC_RANGE and abs(p.cruise_alt - q.cruise_alt) < simmod.SEPARATION_V
                            and simmod.cpa(p, q)[0] < simmod.SEPARATION_H):
                        self._episodes[key] = {"pair": key, "t0": round(self.world.t, 1), "min_h": d,
                                               "backstop": False, "collision": False, "loss": False, "min_3d": d}
                        self.threats += 1
                    continue
                ep["min_h"] = min(ep["min_h"], d)
                v = abs(p.alt - q.alt)
                ep["min_3d"] = min(ep["min_3d"], math.hypot(d, v))
                ep["loss"] = ep["loss"] or (d < simmod.SEPARATION_H and v < simmod.SEPARATION_V)
                if ep["collision"] or down or d > simmod.TRAFFIC_RANGE * 1.2:
                    self._close_episode(key)

    def _close_episode(self, key):
        ep = self._episodes.pop(key)
        if ep["collision"]:
            outcome = "COLLISION"
        elif ep["backstop"]:
            outcome = "BACKSTOP"
        elif ep["loss"]:
            outcome = "SEPARATION_LOSS"
        else:
            outcome = "JEV_RESOLVED"
        self.outcomes[outcome] = self.outcomes.get(outcome, 0) + 1
        if self.rec:
            self.rec.write("events", {"kind": "ENCOUNTER", "t": round(self.world.t, 1), "pair": ep["pair"],
                                      "outcome": outcome, "min_distance_m": round(ep["min_h"], 1), "min_3d_m": round(ep["min_3d"], 1),
                                      "t0": ep["t0"], "scenario": self.world.scenario, "backstop_enabled": self.backstop})

    def tick(self, dt):
        self.world.t += dt
        self._backstop(dt)
        for a in self.agents:
            a.sim.tick(dt)
            q = a.sim
            if q.mode == "CRASHED" and self.world.loop and self.world.t - q.crashed_at >= simmod.CRASH_RECOVERY:
                q.replace()
        self._collisions()
        self._track_episodes()
        if self.rec and self.world.t >= self._next_tel:
            self._next_tel = self.world.t + TELEMETRY_PERIOD
            for a in self.agents:
                q = a.sim
                self.rec.write("telemetry", {
                    "t": round(self.world.t, 1), "drone": q.name, "scenario": self.world.scenario, "lap": q.laps + 1,
                    "mode": q.mode, "route": q.route, "x": round(q.pos[0], 1), "y": round(q.pos[1], 1),
                    "alt": round(q.alt, 1), "speed": round(q.speed, 1), "battery": round(q.battery, 1),
                    "drain": round(q.last_drain, 2), "in_wind": q.in_wind(), "target": q.target_name(),
                    "command": q.command, "source": q.command_source, "control": a.view_control()})

    def _collisions(self):
        """Physical truth, independent of Jev and of the safety rules: contact destroys both airframes."""
        sims = [a.sim for a in self.agents if a.sim.mode != "CRASHED"]
        for i, p in enumerate(sims):
            for q in sims[i + 1:]:
                if p.mode in simmod.GROUNDED and q.mode in simmod.GROUNDED:
                    continue
                h = simmod.dist(p.pos, q.pos)
                v = abs(p.alt - q.alt)
                key = (p.name, q.name)
                if h < simmod.COLLISION_H and v < simmod.COLLISION_V:
                    self.collisions += 1
                    self._near.discard(key)
                    ep = self._episodes.get(tuple(sorted(key)))
                    if ep:
                        ep["collision"] = True
                    for x, o in ((p, q), (q, p)):
                        x.crash(f"COLLISION with {o.name}")
                elif h < simmod.SEPARATION_H and v < simmod.SEPARATION_V:
                    if key not in self._near:
                        self._near.add(key)
                        self.conflicts += 1
                        self.emit(p.name, "SAFETY", f"SEPARATION_LOSS: {p.name}/{q.name} {h:.0f} m apart, {v:.0f} m vertical")
                else:
                    self._near.discard(key)

    def agent(self, name):
        for a in self.agents:
            if a.name == name:
                return a
        raise KeyError(name)

    def state(self):
        return {
            "map": {
                "size": [400, 300],
                "drones": {a.name: a.sim.geometry() for a in self.agents},
                "waypoints": simmod.WAYPOINTS,
                "sites": simmod.LANDING_SITES,
                "wind": {"zone": simmod.WIND_ZONE, "active": self.world.wind_on()},
            },
            "scenario": self.world.scenario,
            "loop": self.world.loop,
            "metrics": self.metrics(),
            "recording": str(self.rec.dir.name) if self.rec else None,
            "t": round(self.world.t, 1),
            "limits": {"reserve": simmod.BATTERY_RESERVE, "emergency": simmod.BATTERY_EMERGENCY},
            "jev": {"label": self.jev.label, "model": getattr(self.jev, "model", None)},
            "drones": [a.view() for a in self.agents],
            "events": list(self.events),
            "scenarios": {k: v["label"] for k, v in simmod.SCENARIOS.items()},
        }


sup = Fleet()


async def sim_loop():
    nxt = time.monotonic()
    while True:
        sup.tick(TICK)
        nxt += TICK / SPEED
        await asyncio.sleep(max(0, nxt - time.monotonic()))


async def decision_loop(agent, offset):
    await asyncio.sleep(offset)
    while True:
        t0 = time.monotonic()
        try:
            await agent.decide_once()
        except Exception as e:  # never let the loop die; surface it
            agent._emit("FALLBACK", f"Supervisor error: {type(e).__name__}: {e}")
        await asyncio.sleep(max(0.05, DECISION_PERIOD / SPEED - (time.monotonic() - t0)))


@asynccontextmanager
async def lifespan(app):
    sup.start_recording()
    sup.reset("mixed")
    tasks = [asyncio.create_task(sim_loop())] + [
        asyncio.create_task(decision_loop(a, 0.1 * i)) for i, a in enumerate(sup.agents)]
    yield
    for t in tasks:
        t.cancel()
    sup.stop_recording()


app = FastAPI(title="Drone mission supervisor demo", lifespan=lifespan)


class ResetBody(BaseModel):
    scenario: str = "mixed"


class BackstopBody(BaseModel):
    enabled: bool = True


class LoopBody(BaseModel):
    enabled: bool = True


class OperatorBody(BaseModel):
    action: str
    drone: str = "D1"


@app.get("/api/state")
async def get_state():
    return sup.state()


@app.get("/api/health")
async def health():
    return {"ok": True, "jev_mode": sup.jev.label, "jev_key_set": bool(getattr(sup.jev, "key", True)), "drones": N_DRONES}


@app.post("/api/reset")
async def reset(body: ResetBody):
    if body.scenario not in simmod.SCENARIOS:
        raise HTTPException(400, "unknown scenario")
    sup.reset(body.scenario)
    return {"ok": True}


@app.post("/api/start")
async def start():
    sup.start()
    return {"ok": True}


@app.post("/api/loop")
async def loop(body: LoopBody):
    sup.set_loop(body.enabled)
    return {"ok": True}


@app.post("/api/backstop")
async def set_backstop(body: BackstopBody):
    sup.backstop = body.enabled
    sup.emit("-", "STATE", f"Deterministic traffic backstop {'ON' if body.enabled else 'OFF (Jev alone)'}")
    return {"ok": True}


@app.get("/api/metrics")
async def get_metrics():
    return sup.metrics()


@app.post("/api/operator")
async def operator(body: OperatorBody):
    try:
        sup.agent(body.drone).operator(body.action)
    except (ValueError, KeyError):
        raise HTTPException(400, "unknown operator action or drone")
    return {"ok": True}


app.mount("/static", StaticFiles(directory=ROOT / "static"), name="static")


@app.get("/")
async def index():
    return FileResponse(ROOT / "static" / "index.html")
