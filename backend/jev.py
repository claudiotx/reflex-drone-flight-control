"""Jev client: live HTTP adapter plus an explicit, clearly-labelled mock."""
import json
import os
import time

import httpx

ACTION_DESCRIPTIONS = {
    "CONTINUE": "Keep flying the current mission route.",
    "HOLD": "Hover in place and wait; hold time is limited.",
    "USE_ALT_1": "Take predefined alternate path 1 around the main corridor.",
    "USE_ALT_2": "Take predefined alternate path 2 around the main corridor.",
    "SLOW_FOR_TRAFFIC": "Keep flying the route but at a reduced speed for a few seconds so other same-level traffic can pass first.",
    "CHANGE_LEVEL": "Keep flying the route but move 10 m or more to a separate, clear level for about 12 seconds, to pass above or below conflicting traffic (works for head-on and overtaking, where slowing alone cannot).",
    "RETURN_HOME": "Abort the mission and fly back home to land.",
    "LAND_AT_SITE": "Divert to the nearest safe landing zone (every waypoint and landing site is a safe ground landing spot) and land there. Use this before the battery gets low rather than risking a forced landing.",
    "ABSTAIN": "Evidence is insufficient or conflicting; no selection.",
}
COLLISION_LEVELS = [
    "No risk of collision",
    "Low risk: other drones will pass at a safe distance",
    "Moderate risk: a conflict is possible and should be watched",
    "High risk: a collision is likely unless someone changes course, level or speed soon",
    "Imminent: a collision is about to happen",
]
SCORE_LEVELS = [
    "No meaningful disruption",
    "Minor delay",
    "Temporary interruption",
    "Substantial interruption requiring an alternative",
    "Mission cannot be completed",
]


class JevError(Exception):
    pass


TRAFFIC_INSTRUCTIONS = (
    " Other drones are listed in 3D. A conflict means coming within 15 m horizontally AND 8 m vertically at the same "
    "moment. A drone far above or below you (their_altitude_delta_m) is not a conflict unless it is climbing or "
    "descending toward your altitude (their_vertical_speed_mps, their_target_altitude_m); drones taking off or landing "
    "sweep through every level. Act early: seconds_to_contact (distance divided by closing speed) below about 6 s with a likely conflict means decide now. The drone with "
    "the lowest battery has priority and should proceed; a higher-battery drone in a conflict should yield by "
    "SLOW_FOR_TRAFFIC (crossing paths) or CHANGE_LEVEL (head-on or same-line traffic; slowing cannot fix those; check "
    "the other drone is not already heading to the same level) or HOLD. Do not yield when there is no real conflict.")


def build_request(model, state, admissible):
    notes = state.get("candidate_consequences", {})
    traffic = bool(state.get("traffic"))
    req = {
        "model": model,
        "state": json.dumps(state),
        "questions": {
            "maneuver": {
                "type": "choice",
                "instructions": "Select the best operational response among the supplied admissible "
                                "actions for this drone mission. Use ABSTAIN if the observations do not "
                                "support a selection. Prevent low battery: when the battery is low or the remaining mission would "
                                "leave too little reserve, choose LAND_AT_SITE (nearest safe landing zone) or RETURN_HOME "
                                "early instead of continuing." + (TRAFFIC_INSTRUCTIONS if traffic else ""),
                "criteria": {a: f"{ACTION_DESCRIPTIONS[a]} {notes.get(a, '')}".strip() for a in admissible},
            },
            "disruption": {
                "type": "score",
                "instructions": "Rate how much the current situation (blockage, energy margin, wind) disrupts this drone's inspection mission.",
                "criteria": SCORE_LEVELS,
            },
            "persistent_blockage": {
                "type": "noul",
                "instructions": "Do the recent corridor observations indicate that the blockage is "
                                "persistent rather than momentary? Answer low if there is no blockage "
                                "or it appears to be clearing.",
            },
        },
    }


    if traffic:
        req["questions"]["traffic_conflict"] = {
            "type": "noul",
            "instructions": "Considering the listed traffic, will this drone come within 15 m horizontally and 8 m "
                            "vertically of another drone within the next 10 seconds (counting any climb or descent "
                            "they are making) if nobody changes course or speed? Answer low if the other drones are "
                            "moving apart, at a different altitude, or will pass at a safe distance.",
        }
        req["questions"]["collision_risk"] = {
            "type": "score",
            "instructions": "Rate the chance that this drone collides with another drone within the next 10 seconds "
                            "if nobody changes course, level or speed (count any climb or descent they are making).",
            "criteria": COLLISION_LEVELS,
        }
    return req


def parse_response(data, admissible):
    """Validate and normalize a gateway response. Raises JevError when invalid."""
    try:
        answers = data["answers"]
        ch, sc, nl = answers["maneuver"], answers["disruption"], answers["persistent_blockage"]
        choice = ch["choice"]
        score = float(sc["score"])
        noul = float(nl["noul"])
    except (KeyError, TypeError, ValueError) as e:
        raise JevError(f"malformed response: {e!r}")
    if choice not in admissible:
        raise JevError(f"choice {choice!r} not in admissible set")
    if not (0 <= score <= len(SCORE_LEVELS) - 1) or not (0 <= noul <= 1):
        raise JevError("score/noul out of range")
    tn = answers.get("traffic_conflict", {}).get("noul")
    cr = answers.get("collision_risk", {}).get("score")
    return {
        "collision_risk": None if cr is None else float(cr),
        "traffic_noul": None if tn is None else float(tn),
        "choice": choice,
        "choice_confidence": ch.get("confidence"),
        "choice_probabilities": ch.get("probabilities"),
        "score": score,
        "score_max": len(SCORE_LEVELS) - 1,
        "noul": noul,
        "model": data.get("model"),
        "usage": data.get("usage"),
    }


class LiveJev:
    label = "LIVE JEV"

    def __init__(self):
        self.endpoint = os.getenv("JEV_ENDPOINT", "https://inference.do-ai.run/v1/systemone")
        self.model = os.getenv("JEV_MODEL", "typesafe-jev-1.13.0")
        self.key = os.getenv("JEV_API_KEY", "")
        self.timeout = float(os.getenv("JEV_TIMEOUT", "8"))
        self.client = httpx.AsyncClient(timeout=self.timeout)

    async def decide(self, state, admissible):
        if not self.key:
            raise JevError("JEV_API_KEY not set")
        t0 = time.monotonic()
        try:
            r = await self.client.post(
                self.endpoint,
                headers={"Authorization": f"Bearer {self.key}", "Content-Type": "application/json"},
                json=build_request(self.model, state, admissible),
            )
        except httpx.TimeoutException:
            raise JevError(f"timeout after {self.timeout:.0f}s")
        except httpx.HTTPError as e:
            raise JevError(f"transport error: {type(e).__name__}")
        latency = time.monotonic() - t0
        if r.status_code != 200:
            raise JevError(f"HTTP {r.status_code}: {r.text[:120]}")
        try:
            out = parse_response(r.json(), admissible)
        except ValueError:
            raise JevError("response is not JSON")
        out["latency"] = round(latency, 2)
        out["request"] = {"state": state, "admissible": admissible}
        return out


class MockJev:
    """Heuristic stand-in for offline UI testing. Always labelled MOCK in the UI."""
    label = "MOCK"
    model = "mock-heuristic"
    delay = 0.3

    async def decide(self, state, admissible):
        import asyncio
        await asyncio.sleep(self.delay)
        tr = state.get("traffic") or []
        if tr:
            # yield only when closing on a higher-priority (lower-battery) drone from close range
            threat = [t for t in tr if t["closing_speed_mps"] > 3 and t["distance_m"] < 150 and t.get("same_level", True)]
            if threat and not any(t["i_have_priority"] for t in threat) and "SLOW_FOR_TRAFFIC" in admissible:
                nose_on = any(abs(t["their_heading_relative_deg"]) > 120 or t["bearing_from_my_heading_deg"] == 0 and abs(t["their_heading_relative_deg"]) < 30 for t in threat)
                return {"choice": "CHANGE_LEVEL" if nose_on else "SLOW_FOR_TRAFFIC", "choice_confidence": None, "choice_probabilities": None,
                        "score": 2.0, "score_max": 4, "noul": 0.1, "traffic_noul": 0.85, "collision_risk": 3.0, "model": self.model,
                        "usage": None, "latency": self.delay}
        obs = state.get("corridor_observations")
        if not isinstance(obs, list):
            obs = []
        blocked = [o for o in obs if o["reading"] == "blocked"]
        latest_clear = bool(obs) and obs[-1]["reading"] == "clear"
        alt = next((a for a in ("USE_ALT_1", "USE_ALT_2") if a in admissible), None)
        thin = state["battery_pct"] - state["mission_cost_remaining_pct"] < 10
        zone = state.get("nearest_landing_zone")
        if thin and "LAND_AT_SITE" in admissible and zone and zone["distance_m"] < state["distance_to_home_m"]:
            choice, score, noul = "LAND_AT_SITE", 3.5, 0.2
        elif thin and "RETURN_HOME" in admissible:
            choice, score, noul = "RETURN_HOME", 3.0, 0.2
        elif not obs:
            choice, score, noul = "CONTINUE", 0.0, 0.05
        elif len(blocked) >= 6 and not latest_clear and alt:
            choice, score, noul = alt, 3.0, 0.9
        elif len(blocked) >= 3 and len(blocked) < len(obs) and not latest_clear:
            choice, score, noul = "ABSTAIN", 2.0, 0.5
        elif blocked:
            choice, score, noul = "HOLD", 2.0, 0.35
        else:
            choice, score, noul = "CONTINUE", 0.5, 0.05
        if choice not in admissible:
            choice = "ABSTAIN"
        return {"choice": choice, "choice_confidence": None, "choice_probabilities": None,
                "score": score, "score_max": 4, "noul": noul, "traffic_noul": 0.1 if tr else None, "collision_risk": 0.0 if tr else None,
                "model": self.model, "usage": None, "latency": self.delay,
                "request": {"state": state, "admissible": admissible}}


def make_client():
    return MockJev() if os.getenv("JEV_MODE", "live").lower() == "mock" else LiveJev()
