"""Simplified kinematic mission simulation (NOT flight dynamics).

Units: meters, seconds, percent. Map frame: x east, y south (canvas style).
All thresholds are demo parameters, not real flight-safety values.

Each drone flies its own sector (5 waypoints) with one corridor leg that can be
obstructed and two predefined alternates of different energy cost. Two approved
landing sites and a gusty wind zone make return / divert / continue a real trade-off.
"""
import math
from collections import deque

# One shared map: named waypoints every drone can be tasked with, each drone flies its own ordered mission
# through them. corridor = (entry idx, exit idx) leg of the mission that can be obstructed; each drone has two
# predefined alternates (extra via points) of different energy cost around that leg.
WAYPOINTS = {
    "W1": (40, 200), "W2": (120, 150), "W3": (200, 100), "W4": (280, 150), "W5": (360, 200),
    "W6": (120, 50), "W7": (280, 50), "W8": (200, 190), "W9": (200, 50), "W10": (40, 100), "W11": (360, 100),
}
_HAND_ALTS = {  # D1-D3: hand-placed alternates
    "D1": {"USE_ALT_1": [(60, 100)], "USE_ALT_2": [(170, 130), (170, 70)]},
    "D2": {"USE_ALT_1": [(200, 45)], "USE_ALT_2": [(250, 110), (290, 95)]},
    "D3": {"USE_ALT_1": [(340, 100)], "USE_ALT_2": [(240, 130), (240, 70)]},
}
_MISSIONS = [
    ["W1", "W2", "W6", "W3", "W8"], ["W8", "W3", "W7", "W4", "W2"], ["W5", "W4", "W7", "W3", "W8"],
    ["W10", "W2", "W3", "W4", "W11"], ["W11", "W4", "W3", "W2", "W10"], ["W2", "W8", "W4", "W7", "W9"],
    ["W4", "W8", "W2", "W6", "W9"], ["W1", "W8", "W3", "W9", "W6"], ["W5", "W8", "W3", "W9", "W7"],
    ["W3", "W8", "W2", "W10", "W6"],
]


def _clamp(p):
    return (round(min(390.0, max(10.0, p[0]))), round(min(290.0, max(10.0, p[1]))))


def _alternates(a, b):
    """Two detours around leg a->b: a short one to one side, a longer two-point one to the other."""
    (ax, ay), (bx, by) = a, b
    L = math.hypot(bx - ax, by - ay) or 1.0
    nx, ny = -(by - ay) / L, (bx - ax) / L
    mid = ((ax + bx) / 2, (ay + by) / 2)
    return {"USE_ALT_1": [_clamp((mid[0] + 40 * nx, mid[1] + 40 * ny))],
            "USE_ALT_2": [_clamp((ax + (bx - ax) * .25 - 70 * nx, ay + (by - ay) * .25 - 70 * ny)),
                          _clamp((ax + (bx - ax) * .75 - 70 * nx, ay + (by - ay) * .75 - 70 * ny))]}


DRONES = []
for _i, _m in enumerate(_MISSIONS):
    _nm = f"D{_i + 1}"
    DRONES.append({"name": _nm, "home": (30.0 + 40.0 * _i, 270.0), "alt": 45.0, "mission": _m, "corridor": (1, 2),
                   "alts": _HAND_ALTS.get(_nm) or _alternates(WAYPOINTS[_m[1]], WAYPOINTS[_m[2]])})
LANDING_SITES = {"LS_W": (95.0, 205.0), "LS_E": (305.0, 205.0)}
# every waypoint and approved site is a safe ground landing zone
ZONES = {**{n: (float(x), float(y)) for n, (x, y) in WAYPOINTS.items()}, **LANDING_SITES}
WIND_ZONE = (150.0, 110.0, 260.0, 200.0)  # x0, y0, x1, y1
WIND_DRAIN = 2.0     # battery drain multiplier inside the zone
WIND_SPEED = 9.0     # ground-speed cap inside the zone (m/s)
SENSOR_RANGE = 90.0  # m from corridor entry at which a blockage becomes observable

CRUISE_SPEED = 15.0
MAX_ACCEL = 4.0
CLIMB_RATE = 4.0
DESCENT_RATE = 3.0
ARRIVE_RADIUS = 3.0

BATTERY_RESERVE = 25.0    # % -> forced return
BATTERY_EMERGENCY = 10.0  # % -> forced landing
RETURN_MARGIN = 5.0       # % kept above the computed return cost
FINISH_FLOOR = 15.0       # % an alternate must leave after clearing the corridor and returning
LAND_MARGIN = 5.0
HOLD_LIMIT_JEV = 30.0     # s
HOLD_LIMIT_FALLBACK = 10.0
CHARGE_RATE = 2.0         # %/s while parked on a pad or landing site (demo speed)
CHARGE_TARGET = 100.0
CRASH_RECOVERY = 30.0     # s before a replacement airframe appears on the home pad (loop mode only)
DESCENT_RESERVE = 3.0     # % kept above the energy needed to descend from the current altitude
COLLISION_H, COLLISION_V = 4.0, 4.0     # m: contact
SEPARATION_H, SEPARATION_V = 15.0, 8.0  # m: loss-of-separation warning
GROUNDED = ("IDLE", "LANDED", "CHARGING", "CRASHED")
TRAFFIC_ALT_RANGE = 35.0   # m: drones this close in altitude (or climbing/descending) are shown as traffic
TRAFFIC_RANGE = 150.0    # m: same-level traffic inside this range is shown to Jev
TRAFFIC_HORIZON = 10.0   # s: look-ahead for a predicted loss of separation
BACKSTOP_H = 50.0        # m: deterministic last-resort deconfliction if Jev has not resolved the conflict
BACKSTOP_RELEASE = 80.0  # m: backstop releases (hysteresis) beyond this range
YIELD_SPEED = 5.0        # m/s: speed cap for a yielding drone
TRAFFIC_SLOW_S = 6.0     # s: duration of a Jev SLOW_FOR_TRAFFIC command
TRAFFIC_LEVEL_S = 12.0   # s: duration of a Jev CHANGE_LEVEL command
LEVEL_STEP = SEPARATION_V + 2.0  # m: spacing between flight levels
EVADE_H = 25.0           # m: pair this close and closing -> evasive vertical move at EVADE_RATE
EVADE_RATE = 10.0        # m/s: emergency vertical rate (evasion, or landing on a nearly empty battery)
STOP_H = 15.0            # m: hard floor, never close within this of a same-level drone ahead
BRAKE_RANGE = 110.0      # m: range at which the hard floor starts limiting speed
ESCAPE_AFTER = 2.0       # s held by traffic before stepping up a level
COLUMN_H = 40.0          # m: horizontal radius that must be clear to climb / descend a level
LEVEL_BAND = SEPARATION_V + 1.0
LAUNCH_CLEAR_H = 30.0    # m: nothing may be this close to a pad when a drone relaunches
LAND_CLEAR_H = 8.0       # m: a landing drone only needs this much clear directly below (others brake 15 m from it)
DIVERT_CLEAR_H = 20.0    # m: same for a diverting drone, which also moves sideways while descending
NUDGE_SPEED = 6.0        # m/s: sideways slide of a landing drone whose column is blocked
LAP_RESERVE = 30.0       # % that must remain after a whole mission for the next one to start without recharging
LEVEL_CLEAR_H = 200.0    # m: radius within which other drones occupy a flight level

# corridors: per-drone obstruction spec. kind: temp (flickers clear before ending),
# persist (never clears), flicker (noisy readings, truly clears at dur).
INF = float("inf")
SCENARIOS = {
    "mixed": {"label": "Mixed fleet",
              "battery": {"D1": 100.0, "D2": 85.0, "D3": 62.0}, "wind": True,
              "corridors": {"D1": {"kind": "temp", "dur": 18.0},
                            "D2": {"kind": "persist", "dur": INF},
                            "D3": {"kind": "flicker", "dur": 30.0, "seed": 3}}},
    "clear": {"label": "Clear route",
              "battery": {"D1": 100.0, "D2": 100.0, "D3": 100.0}, "wind": False, "corridors": {}},
    "temporary": {"label": "Temporary obstruction",
                  "battery": {"D1": 100.0, "D2": 100.0, "D3": 100.0}, "wind": False,
                  "corridors": {"D1": {"kind": "temp", "dur": 18.0}, "D2": {"kind": "temp", "dur": 12.0},
                                "D3": {"kind": "temp", "dur": 22.0}}},
    "persistent": {"label": "Persistent obstruction",
                   "battery": {"D1": 100.0, "D2": 100.0, "D3": 100.0}, "wind": False,
                   "corridors": {"D1": {"kind": "persist", "dur": INF}, "D3": {"kind": "persist", "dur": INF}}},
    "ambiguous": {"label": "Ambiguous readings",
                  "battery": {"D1": 100.0, "D2": 100.0, "D3": 100.0}, "wind": False,
                  "corridors": {"D1": {"kind": "flicker", "dur": 30.0, "seed": 1},
                                "D2": {"kind": "flicker", "dur": 40.0, "seed": 5},
                                "D3": {"kind": "temp", "dur": 12.0}}},
    "jev_outage": {"label": "Jev outage",
                   "battery": {"D1": 100.0, "D2": 80.0, "D3": 70.0}, "wind": True, "outage": True,
                   "corridors": {"D1": {"kind": "persist", "dur": INF}, "D2": {"kind": "temp", "dur": 15.0}}},
    "stress": {"label": "Max stress (10 drones)",
               "battery": {"D1": 100.0, "D2": 55.0, "D3": 70.0, "D4": 45.0, "D5": 90.0, "D6": 38.0, "D7": 80.0,
                           "D8": 62.0, "D9": 100.0, "D10": 50.0}, "wind": True,
               "corridors": {"D1": {"kind": "temp", "dur": 18.0}, "D2": {"kind": "persist", "dur": INF},
                             "D3": {"kind": "flicker", "dur": 30.0, "seed": 3}, "D4": {"kind": "temp", "dur": 12.0},
                             "D6": {"kind": "persist", "dur": INF}, "D7": {"kind": "flicker", "dur": 40.0, "seed": 5},
                             "D9": {"kind": "temp", "dur": 22.0}}},
    "windy": {"label": "Gusty wind",
              "battery": {"D1": 100.0, "D2": 75.0, "D3": 100.0}, "wind": True,
              "corridors": {"D2": {"kind": "persist", "dur": INF}}},
    "low_battery": {"label": "Low battery",
                    "battery": {"D1": 52.0, "D2": 70.0, "D3": 34.0}, "wind": True,
                    "corridors": {"D1": {"kind": "persist", "dur": INF}}},
}

FLYING = ("TAKEOFF", "MISSION", "HOLD", "RETURN_HOME", "DIVERT", "LANDING")


def cpa(p, q, horizon=TRAFFIC_HORIZON):
    """Ground-truth closest approach of two drones if neither changes course: (distance m, seconds)."""
    rx, ry = q.pos[0] - p.pos[0], q.pos[1] - p.pos[1]
    (px, py), (qx, qy) = p.velocity(), q.velocity()
    vx, vy = qx - px, qy - py
    vv = vx * vx + vy * vy
    t = 0.0 if vv < 1e-9 else max(0.0, min(horizon, -(rx * vx + ry * vy) / vv))
    return math.hypot(rx + vx * t, ry + vy * t), t


def cpa3(p, q, horizon=TRAFFIC_HORIZON, step=0.5):
    """3D ground truth if nobody changes course/speed: (horizontal m, seconds, loses_separation).
    Loss means within SEPARATION_H horizontally AND SEPARATION_V vertically at the same instant."""
    (px, py), (qx, qy) = p.velocity(), q.velocity()
    best = None
    t = 0.0
    while t <= horizon + 1e-9:
        h = math.hypot(q.pos[0] + qx * t - p.pos[0] - px * t, q.pos[1] + qy * t - p.pos[1] - py * t)
        v = abs(q.alt_at(t) - p.alt_at(t))
        key = (0 if v < SEPARATION_V else 1, h)
        if best is None or key < best[0]:
            best = (key, h, t)
        t += step
    return best[1], best[2], best[0][0] == 0 and best[1] < SEPARATION_H


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class World:
    """Shared clock and hidden per-corridor obstruction state."""

    def __init__(self):
        self.scenario = "mixed"
        self.t = 0.0
        self.sensed_at = {}
        self.sims = []     # every Sim sharing this airspace (for site claims)
        self.loop = False  # relaunch after charging instead of staying parked

    def reset(self, scenario):
        self.scenario, self.t, self.sensed_at = scenario, 0.0, {}

    def launch_clear(self, s):
        """No airborne drone near this pad, and nobody diverting to land on it."""
        for o in self.sims:
            if o is s or o.mode in GROUNDED:
                continue
            if dist(o.pos, s.pos) < LAUNCH_CLEAR_H:
                return False
            if o.mode == "DIVERT" and o.divert and dist(o.divert[1], s.pos) < LAUNCH_CLEAR_H:
                return False
        return True

    @property
    def cfg(self):
        return SCENARIOS[self.scenario]

    def wind_on(self):
        return self.cfg["wind"]

    def battery(self, name):
        return self.cfg["battery"].get(name, 100.0)

    def spec(self, name):
        return self.cfg["corridors"].get(name)

    def obstruction_active(self, name):
        spec, at = self.spec(name), self.sensed_at.get(name)
        if spec is None or at is None:
            return False
        return self.t - at < spec["dur"]

    def reading(self, name):
        """Noisy sensor view; what a drone can actually observe."""
        if not self.obstruction_active(name):
            return "clear"
        spec = self.spec(name)
        if spec["kind"] == "temp":
            remaining = spec["dur"] - (self.t - self.sensed_at[name])
            return "clear" if remaining < 7.0 and int(self.t) % 2 == 0 else "blocked"
        if spec["kind"] == "flicker":
            return "blocked" if (int(self.t) * 7 + spec["seed"]) % 10 < 6 else "clear"
        return "blocked"


class Sim:
    def __init__(self, world, idx=0, emit=lambda kind, msg, **kw: None):
        cfg = DRONES[idx]
        self.world, self.emit = world, emit
        self.name, self.home, self.base_alt = cfg["name"], cfg["home"], cfg["alt"]
        self.cruise_alt = self.base_alt
        world.sims.append(self)
        self.main_plan = list(cfg["mission"])
        self.points = {n: tuple(map(float, WAYPOINTS[n])) for n in self.main_plan}
        self.alts = {}
        for act, via in cfg["alts"].items():
            names = []
            for j, p in enumerate(via):
                n = f"A{act[-1]}.{j + 1}"
                self.points[n] = p
                names.append(n)
            self.alts[act] = names
        self.corr = cfg["corridor"]
        self.epoch = 0
        self.reset()

    @property
    def t(self):
        return self.world.t

    @property
    def scenario(self):
        return self.world.scenario

    def pt(self, name):
        return self.points[name]

    def geometry(self):
        return {"home": list(self.home), "points": self.points, "main_plan": self.main_plan,
                "alts": self.alts, "corridor": [self.main_plan[i] for i in self.corr]}

    # ---- lifecycle -------------------------------------------------
    def reset(self):
        self.epoch += 1
        self.start_at = None
        self.pos = list(self.home)
        self.alt = 0.0
        self.speed = 0.0
        self.battery = self.world.battery(self.name)
        self.cruise_alt = self.world.cfg.get("same_alt", self.base_alt)  # all drones share one level when set
        self.mode = "IDLE"
        self.command = None          # last executed action
        self.command_source = None   # JEV / SAFETY / FALLBACK / OPERATOR
        self.plan = list(self.main_plan)
        self.route = "MAIN"
        self.idx = 0
        self.track = [tuple(self.home)]
        self.hold_started = None
        self.hold_limit = None
        self.blocked_stop = False
        self.divert = None           # (site name, pos)
        self.crash_cause = None
        self.crashed_at = None
        self.speed_limit = None      # set by the fleet backstop
        self.slow_until = 0.0        # Jev SLOW_FOR_TRAFFIC window (sim time)
        self.level_until = 0.0       # Jev CHANGE_LEVEL window (sim time)
        self.evade = False           # fleet backstop: collision imminent, use emergency vertical rate
        self.nudge = None            # fleet: unit vector to slide along while the landing column is blocked
        self.vert_hold = False       # fleet: a vertical move toward the goal level would sweep through other traffic
        self.blocker = None          # drone currently limiting this one's speed
        self.hold_t = 0.0            # s continuously held by traffic
        self.bs_alt = None           # absolute altitude commanded by the fleet backstop
        self.level_alt = None        # absolute altitude for a Jev CHANGE_LEVEL (chosen clear of traffic)
        self.laps = 0
        self.last_drain = 0.0
        self.obs = deque(maxlen=10)  # (t, "blocked"|"clear")
        self._last_obs_t = -1.0

    def start(self, delay=0.0):
        if self.mode == "IDLE" and self.start_at is None:
            self.start_at = self.t + delay

    def _reset_mission(self):
        self.epoch += 1  # discard any in-flight Jev response from the previous lap
        self.world.sensed_at.pop(self.name, None)
        self.plan, self.route, self.idx = list(self.main_plan), "MAIN", 0
        self.track = [(round(self.pos[0], 1), round(self.pos[1], 1))]
        self.hold_started = self.hold_limit = self.divert = None
        self.blocked_stop = False
        self.obs.clear()
        self._last_obs_t = -1.0
        self.command = self.command_source = None

    def relaunch(self):
        """Start a fresh lap from wherever the drone is parked (home pad or a landing site)."""
        self._reset_mission()
        self.mode = "TAKEOFF"
        self.emit("STATE", f"Recharged to {self.battery:.0f}%, starting mission {self.laps + 1}")

    def lap_cost(self):
        """Battery % to fly the whole mission from here (no return: every waypoint is a landing zone)."""
        return self._cost(self._len([tuple(self.pos)] + [self.pt(n) for n in self.main_plan]))

    def _mission_done(self):
        self.laps += 1
        if self.world.loop:
            self._reset_mission()
            if self.battery - self.lap_cost() >= LAP_RESERVE:
                self.command, self.command_source = "CONTINUE", "MISSION"
                self.emit("STATE", f"Mission {self.laps} complete at {self.battery:.0f}%, starting mission {self.laps + 1} immediately")
                return
            site = self.best_site()
            if site:
                self.divert = (site["name"], site["pos"])
                self.mode, self.command, self.command_source = "DIVERT", "LAND_AT_SITE", "MISSION"
                self.emit("STATE", f"Mission {self.laps} complete at {self.battery:.0f}%, landing at nearest zone {site['name']} to recharge")
                return
        self.emit("STATE", "Mission complete, returning home")
        self.command, self.command_source = "RETURN_HOME", "MISSION"
        self.mode = "RETURN_HOME"

    def crash(self, cause):
        if self.mode == "CRASHED":
            return
        self.mode, self.crash_cause, self.crashed_at = "CRASHED", cause, self.t
        self.alt = self.speed = 0.0
        self.command, self.command_source = "CRASH", "PHYSICS"
        self.emit("CRASH", f"CRASH: {cause} at ({self.pos[0]:.0f}, {self.pos[1]:.0f})", cause=cause)

    def replace(self):
        """Replacement airframe on the home pad after a crash (loop mode)."""
        self.epoch += 1
        self.pos = list(self.home)
        self.alt = self.speed = 0.0
        self.battery = 20.0
        self.crash_cause = self.crashed_at = None
        self.mode = "CHARGING"
        self.track = [tuple(self.home)]
        self.emit("STATE", "Replacement airframe on home pad, charging")

    def aim(self):
        """Where the drone is currently flying to (None when hovering / climbing / landing)."""
        if self.mode == "MISSION":
            return self.pt(self.target_name()) if self.target_name() and not self.next_leg_blocked() else None
        if self.mode == "RETURN_HOME":
            return self.home
        if self.mode == "DIVERT":
            return self.divert[1] if self.divert else None
        return None

    def occupies(self, alt):
        """Does this drone occupy altitude `alt` (including levels it is sweeping through)?"""
        if self.mode in ("LANDING", "DIVERT"):
            return alt <= self.alt + LEVEL_BAND
        if self.mode == "TAKEOFF":
            return self.alt - LEVEL_BAND <= alt <= self.cruise_alt + LEVEL_BAND
        return abs(alt - self.alt) < LEVEL_BAND

    def velocity(self):
        """Current velocity vector (m/s) toward whatever the drone is flying to."""
        tgt = self.aim()
        if tgt is None or self.speed <= 0:
            return (0.0, 0.0)
        d = dist(self.pos, tgt)
        return (0.0, 0.0) if d < 1e-6 else ((tgt[0] - self.pos[0]) / d * self.speed, (tgt[1] - self.pos[1]) / d * self.speed)

    def vz(self):
        """Current vertical speed (m/s, + climbing): what other drones' transponders would report."""
        if self.mode in GROUNDED or self.mode in ("CHARGING", "IDLE", "LANDED") or self.vert_hold:
            return 0.0
        if self.mode == "TAKEOFF":
            return CLIMB_RATE
        if self.mode == "LANDING":
            return -DESCENT_RATE if self.alt > 0 else 0.0
        d = self.goal_alt() - self.alt
        return 0.0 if abs(d) < 0.5 else (CLIMB_RATE if d > 0 else -DESCENT_RATE)

    def alt_target(self):
        """Altitude this drone is currently heading for."""
        if self.mode == "TAKEOFF":
            return self.cruise_alt
        if self.mode == "LANDING":
            return 0.0
        return self.goal_alt()

    def alt_at(self, t):
        """Altitude after t more seconds if nothing changes."""
        vz, goal = self.vz(), self.alt_target()
        a = self.alt + vz * t
        return min(a, goal) if vz > 0 else max(a, goal) if vz < 0 else self.alt

    def descent_cost(self):
        return self.alt / DESCENT_RATE * self._base_drain(0.0)

    def landing_note(self):
        return "inside gusty zone" if self.in_wind() else None

    # ---- environment -------------------------------------------------
    def in_wind(self):
        x0, y0, x1, y1 = WIND_ZONE
        return self.world.wind_on() and x0 <= self.pos[0] <= x1 and y0 <= self.pos[1] <= y1

    def cruise(self):
        return WIND_SPEED if self.in_wind() else CRUISE_SPEED

    @staticmethod
    def _base_drain(speed):
        return 0.5 + 0.02 * speed  # %/s

    def _drain(self, speed):
        return self._base_drain(speed) * (WIND_DRAIN if self.in_wind() else 1.0)

    def obstruction_active(self):
        return self.world.obstruction_active(self.name)

    def _sense(self):
        if self.corridor_ahead() and dist(self.pos, self.pt(self.main_plan[self.corr[0]])) <= SENSOR_RANGE:
            w = self.world
            if self.name not in w.sensed_at and w.spec(self.name) is not None:
                w.sensed_at[self.name] = self.t
            if self.t - self._last_obs_t >= 1.0:
                self._last_obs_t = self.t
                self.obs.append((round(self.t, 1), w.reading(self.name)))

    # ---- geometry / energy helpers ---------------------------------------
    def target_name(self):
        return self.plan[self.idx] if self.idx < len(self.plan) else None

    def corridor_ahead(self):
        """True while the main-route corridor has not yet been passed."""
        return self.route == "MAIN" and self.idx <= self.corr[1]

    def next_leg_blocked(self):
        if self.route == "MAIN" and self.idx == self.corr[1]:
            return self.obstruction_active()
        return False

    def alternate_available(self):
        return self.corridor_ahead() and self.mode in ("MISSION", "HOLD")

    def alt_plan(self, act):
        ins = self.corr[1]
        return self.main_plan[:ins] + self.alts[act] + self.main_plan[ins:]

    def home_distance(self):
        return dist(self.pos, self.home)

    def _land_margin(self):
        """Energy kept for getting down: at least LAND_MARGIN, more when high up (stepped-up levels)."""
        return max(LAND_MARGIN, self.descent_cost() + 2.0)

    def _cost(self, length):
        return length / CRUISE_SPEED * self._base_drain(CRUISE_SPEED)

    def _len(self, pts):
        return sum(dist(a, b) for a, b in zip(pts, pts[1:]))

    def return_cost(self, frm=None):
        """Battery % to fly home from `frm` and land (demo energy model, no wind)."""
        return self._cost(dist(frm or self.pos, self.home)) + self._land_margin()

    def mission_cost(self):
        """Battery % to finish the current plan and return home."""
        pts = [tuple(self.pos)] + [self.pt(n) for n in self.plan[self.idx:]]
        return self._cost(self._len(pts)) + self.return_cost(pts[-1])

    def alt_cost(self, act):
        """Battery % to clear the corridor via `act` and still return home from the far side."""
        ins = self.corr[1]
        plan = self.alt_plan(act)
        pts = [tuple(self.pos)] + [self.pt(n) for n in plan[min(self.idx, ins):ins + len(self.alts[act]) + 1]]
        return self._cost(self._len(pts)) + self.return_cost(pts[-1])

    def direct_cost(self):
        pts = [tuple(self.pos)] + [self.pt(n) for n in self.main_plan[self.idx:self.corr[1] + 1]]
        return self._cost(self._len(pts)) + self.return_cost(pts[-1])

    def alt_feasible(self, act):
        return self.battery - self.alt_cost(act) >= FINISH_FLOOR

    def site_taken(self, name):
        """Another drone is parked at, or already landing at, this site."""
        pos = ZONES[name]
        for o in self.world.sims:
            if o is self or o.mode == "CRASHED":
                continue
            if o.mode in ("LANDED", "CHARGING", "IDLE") and dist(o.pos, pos) < SEPARATION_H:
                return True
            if o.mode in ("DIVERT", "LANDING") and o.divert and o.divert[0] == name:
                return True
            if o.mode not in GROUNDED and o.speed < 2.0 and o.alt > 0 and dist(o.pos, pos) < SEPARATION_H:
                return True   # hovering over it (waiting at a blocked corridor, holding): landing would descend onto it
        return False

    def zone_blocked(self, name):
        """Parked on, or hovering over, this zone (a landing there would descend onto it)."""
        if name not in ZONES:
            return False
        pos = ZONES[name]
        for o in self.world.sims:
            if o is self or o.mode == "CRASHED":
                continue
            if o.mode in ("LANDED", "CHARGING") and dist(o.pos, pos) < SEPARATION_H:
                return True
            if o.mode not in GROUNDED and o.speed < 2.0 and o.alt > 0 and dist(o.pos, pos) < SEPARATION_H:
                return True
        return False

    def best_site(self):
        """Nearest approved, unclaimed landing site reachable on current battery, or None."""
        opts = sorted(((dist(self.pos, p), n, p) for n, p in ZONES.items()))
        for d, n, p in opts:
            if self.site_taken(n):
                continue
            if self.battery >= self._cost(d) + self._land_margin():
                return {"name": n, "pos": p, "dist": d, "cost": self._cost(d) + self._land_margin()}
        return None

    # ---- commands (single writer) ------------------------------------
    def apply(self, action, source, rule=None):
        """Execute a validated action. Returns True if the vehicle state changed."""
        if self.mode in ("IDLE", "LANDING", "LANDED"):
            return False
        changed = False
        if action == "HOLD":
            if self.mode != "HOLD":
                self.mode = "HOLD"
                self.hold_started = self.t
                changed = True
            self.hold_limit = None if source == "OPERATOR" else (
                HOLD_LIMIT_FALLBACK if source == "FALLBACK" else HOLD_LIMIT_JEV)
        elif action == "CONTINUE":  # traffic windows (slow / level change) expire on their own
            if self.mode == "HOLD":
                self.mode = "MISSION"
                self.hold_started = None
                changed = True
        elif action in ("SLOW_FOR_TRAFFIC", "CHANGE_LEVEL"):
            if action == "CHANGE_LEVEL":
                if self.level_alt is None:
                    return False
                self.level_until = self.t + TRAFFIC_LEVEL_S
            else:
                self.slow_until = self.t + TRAFFIC_SLOW_S
            if self.mode == "HOLD":
                self.mode = "MISSION"
                self.hold_started = None
            changed = True
        elif action in self.alts:
            if not self.alternate_available():
                return False
            self.plan = self.alt_plan(action)
            self.idx = min(self.idx, self.corr[1])
            self.route = action[4:]
            self.mode = "MISSION"
            self.hold_started = None
            changed = True
        elif action == "RETURN_HOME":
            if self.mode != "RETURN_HOME":
                self.mode = "RETURN_HOME"
                self.hold_started = None
                changed = True
        elif action == "LAND_AT_SITE":
            site = self.best_site()
            if site is None:
                return False
            if self.mode != "DIVERT":
                self.mode = "DIVERT"
                self.divert = (site["name"], site["pos"])
                self.hold_started = None
                changed = True
        elif action == "LAND":
            if self.mode != "LANDING":
                self.mode = "LANDING"
                changed = True
        if changed or action != self.command:
            self.command, self.command_source = action, source
        return changed

    # ---- physics tick ---------------------------------------------------
    def tick(self, dt):
        if self.mode == "IDLE" and self.start_at is not None and self.t >= self.start_at:
            self.mode = "TAKEOFF"
            self.emit("STATE", "Takeoff started")
        if self.mode in ("IDLE", "CRASHED"):
            return
        if self.mode == "LANDED":
            if self.world.loop:
                self.mode = "CHARGING"
                self.emit("STATE", f"Parked at ({self.pos[0]:.0f}, {self.pos[1]:.0f}), charging from {self.battery:.0f}%")
            return
        if self.mode == "CHARGING":
            self.battery = min(CHARGE_TARGET, self.battery + CHARGE_RATE * dt)
            if self.battery >= CHARGE_TARGET and self.world.launch_clear(self):
                self.relaunch()
            return
        self._safety()
        if self.mode == "TAKEOFF":
            self._move_toward(None, 0.0, dt)
            self.alt = min(self.cruise_alt, self.alt + CLIMB_RATE * dt)
            if self.alt >= self.cruise_alt:
                self.mode = "MISSION"
                self.command, self.command_source = "CONTINUE", "MISSION"
                self.emit("STATE", "Takeoff complete, mission started")
        elif self.mode == "MISSION":
            self._fly_plan(dt)
            self._track_alt(dt)
        elif self.mode == "HOLD":
            self._move_toward(None, 0.0, dt)
            self._sense()
            self._track_alt(dt)
        elif self.mode == "RETURN_HOME":
            self._track_alt(dt)
            if self._move_toward(self.home, self.cruise(), dt):
                self.mode = "LANDING"
                self.emit("STATE", "Over home, landing")
        elif self.mode == "DIVERT":
            if self.divert and self.zone_blocked(self.divert[0]):
                site = self.best_site()
                if site:
                    self.divert = (site["name"], site["pos"])
                    self.emit("STATE", f"Landing zone occupied, diverting to {site['name']} instead")
                else:
                    self._force("LANDING", "LAND", "NO_REACHABLE_SITE",
                                "NO_REACHABLE_SITE: landing zone occupied and no other in reach, landing in place")
            self._track_alt(dt, base=0.0)  # descend while flying in: less hover time at the end
            if self._move_toward(self.divert[1], self.cruise(), dt):
                self.mode = "LANDING"
                self.emit("STATE", f"Over landing site {self.divert[0]}, landing")
        elif self.mode == "LANDING":
            self._move_toward(None, 0.0, dt)
            if self.nudge:   # something sits in the column below: slide to a free spot before descending
                self.pos[0] = min(395.0, max(5.0, self.pos[0] + self.nudge[0] * NUDGE_SPEED * dt))
                self.pos[1] = min(295.0, max(5.0, self.pos[1] + self.nudge[1] * NUDGE_SPEED * dt))
            # a nearly empty battery gets an emergency descent rather than running dry in the air
            rate = EVADE_RATE if self.battery < self.descent_cost() + 0.5 else DESCENT_RATE
            if not self.vert_hold:
                self.alt = max(0.0, self.alt - rate * dt)
            if self.alt <= 0.0 and self.speed < 0.1:
                self.mode = "LANDED"
                self.emit("STATE", f"Landed, battery {self.battery:.0f}%")
        if self.mode != "LANDED":
            self.last_drain = self._drain(self.speed)
            self.battery = max(0.0, self.battery - self.last_drain * dt)
            if self.battery <= 0.0 and self.alt > 0.0:
                self.crash("BATTERY_DEPLETED in flight")
                return
        if not self.track or dist(self.track[-1], self.pos) > 2.0:
            self.track.append((round(self.pos[0], 1), round(self.pos[1], 1)))

    def goal_alt(self, base=None):
        if self.bs_alt is not None:
            return self.bs_alt
        if self.level_alt is not None and self.t < self.level_until:
            return self.level_alt
        return self.cruise_alt if base is None else base

    def _track_alt(self, dt, base=None):
        """Hold cruise level (or `base`), unless the backstop / a Jev level change says otherwise."""
        goal = self.goal_alt(base)
        if self.vert_hold:
            return
        step = (EVADE_RATE if self.evade else CLIMB_RATE if goal > self.alt else DESCENT_RATE) * dt
        self.alt += max(-step, min(step, goal - self.alt))

    def _fly_plan(self, dt):
        name = self.target_name()
        if name is None:
            self._mission_done()
            return
        self._sense()
        if self.next_leg_blocked():
            self._move_toward(None, 0.0, dt)
            if not self.blocked_stop:
                self.blocked_stop = True
                self.emit("SAFETY", "BLOCKED_PATH: corridor blocked, drone will not enter",
                          rule="BLOCKED_PATH")
            return
        self.blocked_stop = False
        if self._move_toward(self.pt(name), self.cruise(), dt):
            self.emit("STATE", f"Reached {name}")
            self.idx += 1

    def _move_toward(self, target, desired_speed, dt):
        """Bounded-acceleration move. Returns True on arrival."""
        if target is None:
            desired_speed = 0.0
        else:
            lim = self.speed_limit
            if self.t < self.slow_until:
                lim = YIELD_SPEED if lim is None else min(lim, YIELD_SPEED)
            if lim is not None:
                desired_speed = min(desired_speed, lim)
            d = dist(self.pos, target)
            # decelerate so we can stop at the target (bounded decel)
            desired_speed = min(desired_speed, math.sqrt(2 * MAX_ACCEL * max(d - ARRIVE_RADIUS, 0.0)) + 1.0)
        dv = max(-MAX_ACCEL * dt, min(MAX_ACCEL * dt, desired_speed - self.speed))
        self.speed = max(0.0, self.speed + dv)
        if target is not None and self.speed > 0:
            d = dist(self.pos, target)
            step = self.speed * dt
            if d <= max(step, ARRIVE_RADIUS):
                self.pos[0], self.pos[1] = target
                return True
            self.pos[0] += (target[0] - self.pos[0]) / d * step
            self.pos[1] += (target[1] - self.pos[1]) / d * step
        elif target is not None and dist(self.pos, target) <= ARRIVE_RADIUS:
            return True
        return False

    # ---- deterministic safety (independent of Jev) -------------------------
    def _force(self, mode, action, kind_rule, msg):
        self.mode = mode
        self.hold_started = None
        self.command, self.command_source = action, "SAFETY"
        self.emit("SAFETY", msg, rule=kind_rule, forced=action)

    def _safety(self):
        b = self.battery
        floor = max(BATTERY_EMERGENCY, self.descent_cost() + DESCENT_RESERVE)
        if self.mode == "DIVERT" and self.divert:
            # flying in while descending: energy still needed is the longer of the flight and the descent
            fly = dist(self.pos, self.divert[1]) / CRUISE_SPEED
            down = self.alt / DESCENT_RATE
            need = (fly * self._base_drain(CRUISE_SPEED) + max(0.0, down - fly) * self._base_drain(0.0)) \
                * (WIND_DRAIN if self.world.wind_on() else 1.0) + 1.0
            trigger, floor = b < need, need
        else:
            trigger = self.mode in ("MISSION", "HOLD", "TAKEOFF", "RETURN_HOME") and b <= floor
        if trigger:
            site = self.best_site()
            if site and self.mode not in ("TAKEOFF", "DIVERT"):
                self.divert = (site["name"], site["pos"])
                self._force("DIVERT", "LAND_AT_SITE", "EMERGENCY_DIVERT",
                            f"EMERGENCY_DIVERT: {b:.0f}% <= {floor:.0f}%, landing at {site['name']} ({site['dist']:.0f} m)")
            else:
                note = self.landing_note()
                self._force("LANDING", "LAND", "EMERGENCY_BATTERY",
                            f"EMERGENCY_BATTERY: {b:.0f}% <= {floor:.0f}%, no safe site in reach, landing in place"
                            + (f" ({note})" if note else ""))
        elif self.mode in ("MISSION", "HOLD") and (b <= BATTERY_RESERVE or b <= self.return_cost() + RETURN_MARGIN):
            site, home_ok = self.best_site(), b >= self.return_cost()
            if home_ok and not (site and site["dist"] < self.home_distance()):
                self._force("RETURN_HOME", "RETURN_HOME", "RESERVE_RETURN",
                            f"RESERVE_RETURN: {b:.0f}% vs {self.return_cost():.0f}% needed to get home, forcing return")
            elif site:
                self.divert = (site["name"], site["pos"])
                rule = "RESERVE_LAND" if home_ok else "HOME_UNREACHABLE"
                self._force("DIVERT", "LAND_AT_SITE", rule,
                            f"{rule}: {b:.0f}% battery, nearest safe landing zone {site['name']} ({site['dist']:.0f} m) "
                            f"is closer than home ({self.home_distance():.0f} m), landing there")
            else:
                self._force("LANDING", "LAND", "NO_REACHABLE_SITE",
                            f"NO_REACHABLE_SITE: {b:.0f}% cannot reach home or a site, landing in place")
        elif self.mode == "RETURN_HOME" and b < self.return_cost() + 2.0 and self.best_site() \
                and self.best_site()["dist"] < self.home_distance():
            site = self.best_site()
            self.divert = (site["name"], site["pos"])
            self._force("DIVERT", "LAND_AT_SITE", "HOME_UNREACHABLE",
                        f"HOME_UNREACHABLE: {b:.0f}% on the way home, landing at nearest safe zone {site['name']} ({site['dist']:.0f} m)")
        elif self.mode == "HOLD" and self.hold_limit is not None and self.t - self.hold_started > self.hold_limit:
            self._force("RETURN_HOME", "RETURN_HOME", "HOLD_TIMEOUT",
                        f"HOLD_TIMEOUT: hold exceeded {self.hold_limit:.0f}s, returning home")

    # ---- view --------------------------------------------------------------
    def snapshot(self):
        return {
            "id": self.name,
            "home": list(self.home),
            "t": round(self.t, 1),
            "epoch": self.epoch,
            "mode": self.mode,
            "route": self.route,
            "plan": self.plan,
            "pos": [round(self.pos[0], 1), round(self.pos[1], 1)],
            "alt": round(self.alt, 1),
            "speed": round(self.speed, 1),
            "battery": round(self.battery, 1),
            "target": self.target_name(),
            "blocked_stop": self.blocked_stop,
            "obstruction_active": self.obstruction_active(),
            "in_wind": self.in_wind(),
            "divert": self.divert[0] if self.divert and self.mode in ("DIVERT", "LANDING") else None,
            "command": self.command,
            "command_source": self.command_source,
            "hold_remaining": (None if self.mode != "HOLD" or self.hold_limit is None
                               else round(max(0.0, self.hold_limit - (self.t - self.hold_started)), 1)),
            "waiting": self.mode == "IDLE" and self.start_at is not None,
            "laps": self.laps,
            "yielding": self.bs_alt is not None or self.t < self.slow_until or self.t < self.level_until,
            "crash_cause": self.crash_cause,
            "track": self.track[-300:],
        }
