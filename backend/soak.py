"""Headless soak run: python -m backend.soak --minutes 60 --scenario mixed [--speed 10] [--mock]

Drives the same Fleet/sim/Jev loops as the server (no browser), with loop mode on,
and records telemetry/decisions/events/summary.json under runs/<timestamp>/.
--speed N runs the sim N times faster than wall clock (decision cadence scales too).
"""
import argparse
import asyncio
import os
import sys


async def main(a):
    from . import app as A
    A.sup.start_recording()
    A.sup.reset(a.scenario)
    A.sup.set_loop(not a.no_loop)
    A.sup.start()
    tasks = [asyncio.create_task(A.sim_loop())] + [
        asyncio.create_task(A.decision_loop(ag, 0.1 * i)) for i, ag in enumerate(A.sup.agents)]
    goal = a.minutes * 60
    try:
        while A.sup.world.t < goal:
            await asyncio.sleep(5 / A.SPEED)
            m = A.sup.metrics()
            print(f"t={m['sim_time_s']:.0f}s laps={m['laps']} crashes={m['crashes']} "
                  f"jev={m['jev']['calls']} calls p95={m['jev']['p95_ms']}ms", flush=True)
    finally:
        for t in tasks:
            t.cancel()
        A.sup.stop_recording()
    print("run dir:", A.RUNS_DIR)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--minutes", type=float, default=60)
    ap.add_argument("--scenario", default="mixed")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--mock", action="store_true", help="offline mock Jev (not a real experiment)")
    ap.add_argument("--no-loop", action="store_true")
    args = ap.parse_args()
    os.environ["SIM_SPEED"] = str(args.speed)
    if args.mock:
        os.environ["JEV_MODE"] = "mock"
    sys.exit(asyncio.run(main(args)))
