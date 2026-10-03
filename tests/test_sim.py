import pytest
from backend import sim as S
from backend.jev import parse_response, JevError


def mk(scenario, idx=0):
    w = S.World(); w.reset(scenario)
    return S.Sim(w, idx)


def run(sims, secs, dt=0.1):
    sims = sims if isinstance(sims, list) else [sims]
    for _ in range(int(secs / dt)):
        sims[0].world.t += dt
        for s in sims:
            s.tick(dt)


def test_clear_mission_completes_all_drones_together():
    w = S.World(); w.reset("clear")
    sims = [S.Sim(w, i) for i in range(3)]
    for s in sims:
        s.start()
    run(sims, 8)
    assert all(s.mode in ("TAKEOFF", "MISSION") for s in sims)  # simultaneous lift-off
    run(sims, 200)
    assert all(s.mode == "LANDED" and s.battery > 25 for s in sims)


def test_blocked_path_not_entered():
    s = mk("persistent"); s.start()
    run(s, 60)
    assert s.blocked_stop and s.target_name() == s.main_plan[2]
    assert s.pos == list(map(float, s.pt(s.main_plan[1])))  # stays at corridor entry


@pytest.mark.parametrize("alt", ["USE_ALT_1", "USE_ALT_2"])
def test_alternate_route_gets_through(alt):
    s = mk("persistent"); s.start()
    run(s, 40)
    assert s.alt_feasible(alt) and s.apply(alt, "JEV")
    run(s, 200)
    assert s.mode == "LANDED" and s.route == alt[4:]


def test_alternates_differ_in_cost():
    s = mk("persistent"); s.start()
    run(s, 40)
    assert s.alt_cost("USE_ALT_1") < s.alt_cost("USE_ALT_2")


def test_low_battery_forces_return_or_divert():
    s = mk("low_battery", 2); s.start()
    run(s, 60)
    assert s.command_source == "SAFETY" and s.command in ("RETURN_HOME", "LAND_AT_SITE", "LAND")


def test_unreachable_home_diverts_to_site():
    s = mk("clear", 0); s.start()
    run(s, 30)
    s.battery = 12.0  # above emergency, but home is too far for the energy left
    s.pos[:] = [300.0, 40.0]
    run(s, 0.2)
    assert s.mode in ("DIVERT", "LANDING") or s.command == "LAND"


def test_hold_timeout_returns():
    s = mk("clear"); s.start()
    run(s, 15); s.apply("HOLD", "FALLBACK"); run(s, 12)
    assert s.mode in ("RETURN_HOME", "LANDING")


def test_wind_zone_costs_more_battery():
    s = mk("windy"); s.pos[:] = [200.0, 150.0]
    assert s.in_wind() and s._drain(15) == pytest.approx(s._base_drain(15) * S.WIND_DRAIN)


def test_parse_rejects_unknown_choice():
    data = {"answers": {"maneuver": {"choice": "FLY_AWAY"}, "disruption": {"score": 1}, "persistent_blockage": {"noul": .5}}}
    with pytest.raises(JevError):
        parse_response(data, ["HOLD"])


# ---- crashes, collisions, loop, recorder -------------------------------------------
def test_battery_depletion_in_flight_crashes():
    s = mk("clear"); s.start()
    run(s, 15)
    s.mode = "MISSION"; s.battery = 0.05  # bypass safety to force depletion while airborne
    s._safety = lambda: None
    run(s, 1)
    assert s.mode == "CRASHED" and "BATTERY" in s.crash_cause


def test_emergency_prefers_reachable_site_over_landing_in_place():
    s = mk("clear", 0); s.start()
    run(s, 30)
    s.pos[:] = [95.0, 190.0]; s.battery = 10.4   # 15 m from LS_W
    run(s, 0.2)
    assert s.mode == "DIVERT" and s.divert[0] == "LS_W"
    run(s, 30)
    assert s.mode in ("LANDED", "CHARGING")


def test_collision_destroys_both_and_far_apart_is_safe():
    from backend.app import Fleet
    f = Fleet(); f.world.reset("clear")
    a, b = f.agents[0].sim, f.agents[1].sim
    for q in (a, b):
        q.mode, q.alt = "MISSION", 40.0
    a.pos[:] = [100.0, 100.0]; b.pos[:] = [100.0, 130.0]
    f._collisions(); assert a.mode == b.mode == "MISSION" and f.collisions == 0
    b.pos[:] = [102.0, 101.0]
    f._collisions()
    assert a.mode == b.mode == "CRASHED" and f.collisions == 1 and f.crashes == 2


def test_loop_recharges_and_relaunches_from_landing_site():
    s = mk("clear", 0); s.world.loop = True; s.start()
    run(s, 15)
    s.pos[:] = [95.0, 190.0]; s.battery = 9.0
    run(s, 40)
    assert s.mode == "CHARGING" or s.laps >= 1
    run(s, 80)
    assert s.laps >= 1 and s.mode != "CRASHED" and s.mode != "LANDED"


def test_jev_outage_never_crashes_fleet():
    """With zero Jev proposals, the deterministic layer alone must keep every drone alive."""
    from backend.app import Fleet
    from backend.jev import JevError
    f = Fleet(); f.reset("jev_outage"); f.world.loop = True
    for a in f.agents:
        a.sim.start()
    for _ in range(1500):           # 150 s; each drone's hold fallback applied once a second
        f.tick(0.1)
        if int(f.world.t * 10) % 10 == 0:
            for a in f.agents:
                if a.sim.mode in ("MISSION", "HOLD"):
                    a._arbitrate(None, "injected outage", 0.0)
    assert f.crashes == 0 and f.collisions == 0


def _head_on(backstop, jev=None):
    from backend.app import Fleet
    f = Fleet(); f.reset("clear"); f.backstop = backstop
    a, b = f.agents[0].sim, f.agents[1].sim
    for q, frm, to, bat in ((a, (100.0, 100.0), (220.0, 100.0), 30.0), (b, (220.0, 100.0), (100.0, 100.0), 60.0)):
        q.mode, q.alt, q.pos[:], q.battery, q.speed = "DIVERT", 45.0, list(frm), bat, 15.0
        q.divert = ("X", to)
    for _ in range(350):
        f.tick(0.1)
    return f, a, b


def test_head_on_same_altitude_collides_with_no_traffic_management():
    f, a, b = _head_on(False)
    assert f.collisions == 1 and a.mode == b.mode == "CRASHED"
    assert f.outcomes.get("COLLISION") == 1


def test_backstop_prevents_collision_and_is_recorded_as_override():
    f, a, b = _head_on(True)
    assert f.collisions == 0 and f.crashes == 0 and f.backstop_fires >= 1
    f.finish()
    assert f.outcomes.get("BACKSTOP") == 1
    rules = {r["rule"] for ag in f.agents[:2] for r in ag.decisions}
    assert "TRAFFIC_BACKSTOP" in rules


def test_jev_slow_for_traffic_command_caps_speed():
    s = mk("clear"); s.start(); run(s, 12)
    assert s.apply("SLOW_FOR_TRAFFIC", "JEV")
    run(s, 4)
    assert s.speed <= S.YIELD_SPEED + 0.1
    run(s, 6)   # window expires by itself
    assert s.t >= s.slow_until


def test_jev_alone_resolves_head_on_without_backstop():
    """Mock Jev yields the higher-battery drone via SLOW_FOR_TRAFFIC; the backstop is disabled."""
    import asyncio
    from backend.app import Fleet
    from backend.jev import MockJev
    MockJev.delay = 0.0
    f = Fleet(); f.reset("clear"); f.backstop = False
    f.jev = f.agents[0].jev = f.agents[1].jev = MockJev()
    a, b = f.agents[0].sim, f.agents[1].sim
    for q, frm, to, bat in ((a, (100.0, 100.0), (240.0, 100.0), 70.0), (b, (240.0, 100.0), (100.0, 100.0), 90.0)):
        q.mode, q.alt, q.pos[:], q.battery, q.speed, q.route, q.idx = "MISSION", 45.0, list(frm), bat, 15.0, "MAIN", 0
        q.points["X"] = to; q.plan = ["X"]
    async def go():
        for step in range(350):
            f.tick(0.1)
            if step % 10 == 0:
                for ag in f.agents[:2]:
                    await ag.decide_once()
    asyncio.run(go())
    f.finish()
    assert f.collisions == 0 and f.backstop_fires == 0
    assert f.threats == 1 and f.outcomes == {"JEV_RESOLVED": 1}


def test_low_battery_lands_at_nearest_waypoint_automatically():
    s = mk("clear", 0); s.start(); run(s, 30)
    s.pos[:] = [205.0, 105.0]; s.battery = 24.0     # reserve; W3 is 7 m away, home is far
    run(s, 0.3)
    assert s.mode == "DIVERT" and s.divert[0] == "W3" and s.command_source == "SAFETY"
    run(s, 40)
    assert s.mode == "LANDED" and s.pos == [200.0, 100.0]


def test_jev_is_offered_early_landing_when_battery_is_low():
    from backend.app import Fleet
    f = Fleet(); f.reset("clear")
    a = f.agents[0]; q = a.sim
    q.start(); 
    for _ in range(300):
        f.tick(0.1)
    q.battery = 44.0
    assert "LAND_AT_SITE" in a._admissible()
    q.battery = 90.0
    assert "LAND_AT_SITE" not in a._admissible()


def test_nearly_empty_battery_lands_instead_of_crashing():
    s = mk("clear"); s.start(); run(s, 15)
    s.mode, s.alt, s.battery = "LANDING", 45.0, 3.0   # a normal descent would cost ~7.5%
    run(s, 15)
    assert s.mode in ("LANDED", "CHARGING") and s.alt == 0.0


def test_traffic_view_is_3d_and_shows_climbing_drones():
    """A drone climbing through our altitude must be visible (with its vertical speed) well before it is level."""
    from backend.app import Fleet
    f = Fleet(); f.reset("clear")
    a, b = f.agents[0], f.agents[1]
    a.sim.start(); run(a.sim, 14)                       # a is cruising at 45 m
    b.sim.pos[:] = [a.sim.pos[0] + 60, a.sim.pos[1]]
    b.sim.mode, b.sim.alt = "TAKEOFF", 12.0             # b is climbing 33 m below
    tr = a._traffic()
    assert tr and tr[0]["drone"] == b.sim.name
    assert tr[0]["their_mode"] == "TAKEOFF" and tr[0]["their_vertical_speed_mps"] > 0
    assert tr[0]["same_level"] is False and tr[0]["their_altitude_delta_m"] < 0
    # an unrelated cruising drone far below is not traffic
    b.sim.mode, b.sim.alt = "MISSION", 5.0
    assert a._traffic() == []


def test_cpa3_accounts_for_climb():
    a, b = mk("clear", 0), mk("clear", 1)
    a.world.sims = [a, b]
    a.mode, a.alt, a.pos[:] = "MISSION", 45.0, [100.0, 100.0]
    b.mode, b.alt, b.pos[:] = "TAKEOFF", 10.0, [100.0, 100.0]
    b.speed = 0.0
    h, t, loss = S.cpa3(a, b)
    assert loss and t > 5                                # b climbs into a's level within the horizon
    a.alt = a.cruise_alt = 70.0
    b.alt = 0.0
    assert not S.cpa3(a, b)[2]                           # b only reaches 40 m in 10 s: still 30 m below


def test_level_change_stays_committed_until_window_ends():
    from backend.app import Fleet
    f = Fleet(); f.reset("clear")
    a = f.agents[0]; s = a.sim
    s.start(); run(s, 14)
    first = f.level_target(s)
    s.level_alt, s.level_until = first, s.t + S.TRAFFIC_LEVEL_S
    # another drone moves onto that level: a fresh pick would change, the committed one must not
    b = f.agents[1].sim
    b.mode, b.alt, b.pos[:] = "MISSION", first, [s.pos[0] + 30, s.pos[1]]
    assert f.free_level(s) != first
    assert f.level_target(s) == first
    run(s, S.TRAFFIC_LEVEL_S + 1)                       # window over: free to re-pick
    assert f.level_target(s) == f.free_level(s)


def test_level_change_never_targets_current_altitude():
    from backend.app import Fleet
    f = Fleet(); f.reset("clear")
    s = f.agents[0].sim
    s.start(); run(s, 14)
    s.alt = s.cruise_alt + S.LEVEL_STEP          # already on the +1 level, which is the nearest "clear" one
    assert abs(f.free_level(s) - s.alt) >= S.SEPARATION_V
    s.level_alt, s.level_until = s.alt, s.t + 12  # committed level we have already reached: not a change
    assert abs(f.level_target(s) - s.alt) >= S.SEPARATION_V


def test_level_change_path_must_not_cross_a_conflicting_drone_altitude():
    from backend.app import Fleet
    f = Fleet(); f.reset("clear")
    s, q = f.agents[0].sim, f.agents[1].sim
    s.start(); run(s, 14)
    s.alt = 35.0                                         # separated below q
    q.mode, q.alt, q.pos[:] = "MISSION", 45.0, [s.pos[0] + 40, s.pos[1]]
    assert not f.path_clear(s, 55.0)                     # climbing to 55 would pass through q at 45
    assert f.path_clear(s, 25.0)                         # going down is clear
    assert f.free_level(s) != 55.0


def test_jev_state_has_own_vertical_state():
    from backend.app import Fleet
    f = Fleet(); f.reset("clear")
    a = f.agents[0]
    a.sim.start(); run(a.sim, 14)
    st = a._jev_state(a._admissible())
    for k in ("my_vertical_speed_mps", "my_target_altitude_m", "my_cruise_altitude_m", "level_change_seconds_left"):
        assert k in st


def test_slow_for_traffic_only_offered_when_it_changes_something():
    from backend.app import Fleet
    f = Fleet(); f.reset("clear")
    a, b = f.agents[0], f.agents[1]
    a.sim.start(); run(a.sim, 14)
    b.sim.mode, b.sim.alt, b.sim.pos[:] = "MISSION", 45.0, [a.sim.pos[0] + 40, a.sim.pos[1]]
    a.sim.speed = 12.0
    assert "SLOW_FOR_TRAFFIC" in a._admissible()
    a.sim.speed = S.YIELD_SPEED                           # already at the cap: slowing is a no-op
    assert "SLOW_FOR_TRAFFIC" not in a._admissible() and "HOLD" in a._admissible()


def test_path_check_counts_a_drone_that_is_climbing_through_our_level():
    from backend.app import Fleet
    f = Fleet(); f.reset("clear")
    s, q = f.agents[0].sim, f.agents[1].sim
    s.start(); run(s, 14)
    s.alt = 55.0
    q.mode, q.alt, q.pos[:] = "MISSION", 54.0, [s.pos[0] + 40, s.pos[1]]
    q.level_alt, q.level_until = 65.0, q.t + 12          # q is climbing 54 -> 65 right now
    assert abs(q.vz()) > 0 and not f.path_clear(s, 75.0)  # our climb to 75 would cross it
    q.level_alt, q.cruise_alt = None, 54.0                # q sits level: it is what we leave behind
    assert f.path_clear(s, 75.0)


def test_return_to_cruise_waits_for_traffic_and_is_recorded_even_without_backstop():
    from backend.app import Fleet
    f = Fleet(); f.reset("clear"); f.backstop = False
    a, b = f.agents[0], f.agents[1]
    s, q = a.sim, b.sim
    s.start(); run(s, 14)
    s.alt = 25.0; s.level_alt, s.level_until = 25.0, 0.0      # level change over: wants to go back to 45
    q.mode, q.alt, q.pos[:] = "MISSION", 45.0, [s.pos[0] + 60, s.pos[1]]
    f.tick(0.1)
    assert s.vert_hold and s.alt <= 25.5                         # holds its level, does not climb into q
    rows = [r for r in a.decisions if r["rule"] == "RETURN_TO_CRUISE_HOLD"]
    assert rows and rows[-1]["verdict"] == "OVERRIDDEN"
    q.pos[:] = [s.pos[0] + 400, s.pos[1]]                        # traffic gone: free to return
    f.tick(0.1)
    assert not s.vert_hold


def _noul_setup(other_priority, other_mode="MISSION"):
    from backend.app import Fleet
    f = Fleet(); f.reset("clear"); f.backstop = False
    a, b = f.agents[0], f.agents[1]
    a.sim.start(); run(a.sim, 14)
    entry = {"drone": b.sim.name, "distance_m": 40, "closing_speed_mps": 12.0, "same_level": True, "their_mode": other_mode,
             "i_have_priority": not other_priority}
    state = {"traffic": [entry]}
    prop = {"choice": "CONTINUE", "traffic_noul": 0.7, "score": 1.0, "score_max": 4, "noul": 0.1}
    return f, a, prop, state


def test_noul_yield_overrides_continue_when_other_has_priority():
    f, a, prop, state = _noul_setup(other_priority=True)
    a._arbitrate(prop, None, 0.0, state=state)
    r = a.decisions[-1]
    assert r["verdict"] == "OVERRIDDEN" and r["rule"] == "NOUL_YIELD" and r["executed"] in ("CHANGE_LEVEL", "HOLD")


def test_noul_yield_does_not_fire_for_priority_drone_or_low_noul():
    f, a, prop, state = _noul_setup(other_priority=False)          # we have priority: we proceed
    a._arbitrate(prop, None, 0.0, state=state)
    assert a.decisions[-1]["verdict"] == "ACCEPTED"
    f, a, prop, state = _noul_setup(other_priority=True)
    prop["traffic_noul"] = 0.3                                       # Jev sees no conflict
    a._arbitrate(prop, None, 0.0, state=state)
    assert a.decisions[-1]["verdict"] == "ACCEPTED"


def test_nobody_has_priority_over_a_drone_that_cannot_decide():
    from backend.app import Fleet
    f = Fleet(); f.reset("clear")
    a, b = f.agents[0], f.agents[1]
    a.sim.start(); run(a.sim, 14)
    a.sim.battery = 20.0                                             # lower battery would normally mean priority
    b.sim.mode, b.sim.alt, b.sim.pos[:] = "TAKEOFF", 40.0, [a.sim.pos[0] + 30, a.sim.pos[1]]
    t = a._traffic()
    assert t and t[0]["their_mode"] == "TAKEOFF" and t[0]["i_have_priority"] is False


def test_collision_risk_score_alone_can_trigger_a_yield():
    f, a, prop, state = _noul_setup(other_priority=True)
    prop["traffic_noul"], prop["collision_risk"] = 0.2, 3.0      # noul calm, but Jev rates the risk high
    a._arbitrate(prop, None, 0.0, state=state)
    assert a.decisions[-1]["rule"] == "NOUL_YIELD"
    f, a, prop, state = _noul_setup(other_priority=True)
    prop["traffic_noul"], prop["collision_risk"] = 0.2, 1.0
    a._arbitrate(prop, None, 0.0, state=state)
    assert a.decisions[-1]["verdict"] == "ACCEPTED"


def test_collision_risk_question_only_asked_with_traffic_and_parsed():
    from backend.jev import build_request
    req = build_request("m", {"traffic": [{"drone": "D2"}]}, ["CONTINUE"])
    assert req["questions"]["collision_risk"]["type"] == "score" and len(req["questions"]["collision_risk"]["criteria"]) == 5
    assert "collision_risk" not in build_request("m", {"traffic": []}, ["CONTINUE"])["questions"]
    data = {"answers": {"maneuver": {"choice": "CONTINUE"}, "disruption": {"score": 1}, "persistent_blockage": {"noul": 0.1},
                        "collision_risk": {"score": 3}}}
    assert parse_response(data, ["CONTINUE"])["collision_risk"] == 3.0


def test_short_time_to_contact_lowers_the_yield_thresholds():
    f, a, prop, state = _noul_setup(other_priority=True)
    prop["traffic_noul"], prop["collision_risk"] = 0.45, 2.1       # below the normal gate (0.55 / 3.0)
    state["traffic"][0]["seconds_to_contact"] = 12.0                # plenty of time: Jev's call stands
    a._arbitrate(prop, None, 0.0, state=state)
    assert a.decisions[-1]["verdict"] == "ACCEPTED"
    f, a, prop, state = _noul_setup(other_priority=True)
    prop["traffic_noul"], prop["collision_risk"] = 0.45, 1.0
    state["traffic"][0]["seconds_to_contact"] = 2.5                 # about to meet: act on a weaker signal
    a._arbitrate(prop, None, 0.0, state=state)
    assert a.decisions[-1]["rule"] == "NOUL_YIELD"


def test_traffic_entry_has_seconds_to_contact():
    from backend.app import Fleet
    f = Fleet(); f.reset("clear")
    a, b = f.agents[0], f.agents[1]
    a.sim.start(); run(a.sim, 14)
    b.sim.mode, b.sim.alt, b.sim.pos[:] = "MISSION", a.sim.alt, [a.sim.pos[0] + 80, a.sim.pos[1]]
    t = a._traffic()[0]
    assert "seconds_to_contact" in t
