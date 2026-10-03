"""Offline analysis of a recorded run: python tools/analyze.py runs/<timestamp>

Latency (avg/p95/p99), decision/override mix, and a calibration table for the
`persistent_blockage` noul answer against the simulator's hidden ground truth.
"""
import json
import sys
from pathlib import Path


def load(p):
    return [json.loads(l) for l in p.read_text().splitlines() if l.strip()] if p.exists() else []


def pct(xs, q):
    return xs[min(len(xs) - 1, int(q * len(xs)))] if xs else None


def main(d):
    d = Path(d)
    dec, ev, tel = load(d / "decisions.jsonl"), load(d / "events.jsonl"), load(d / "telemetry.jsonl")
    lat = sorted(r["latency_ms"] for r in dec if r.get("latency_ms") is not None and not r.get("error"))
    print(f"decisions {len(dec)}  telemetry rows {len(tel)}  events {len(ev)}")
    if lat:
        print(f"latency ms  avg {sum(lat)/len(lat):.0f}  p50 {pct(lat,.5):.0f}  p95 {pct(lat,.95):.0f}  "
              f"p99 {pct(lat,.99):.0f}  max {lat[-1]:.0f}")
    mix = {}
    for r in dec:
        k = f"{r['verdict']}/{r['rule'] or '-'}"
        mix[k] = mix.get(k, 0) + 1
    print("verdict/rule:", dict(sorted(mix.items(), key=lambda kv: -kv[1])))
    print("crashes:", sum(e.get("kind") == "CRASH" for e in ev),
          " separation losses:", sum("SEPARATION_LOSS" in e.get("msg", "") for e in ev))

    # calibration: P(truth blocked) by noul bin, only where the corridor was in play
    bins = [[0, 0] for _ in range(5)]
    for r in dec:
        p = r.get("proposal")
        if not p or r.get("mode") not in ("MISSION", "HOLD") or r["jev_state"] is None:
            continue
        if not isinstance(r["jev_state"].get("corridor_observations"), list):
            continue
        b = min(4, int(p["noul"] * 5))
        bins[b][0] += 1
        bins[b][1] += bool(r["truth"]["obstruction_active"])
    print("\nnoul bin      n   actually blocked")
    for i, (n, k) in enumerate(bins):
        print(f"{i/5:.1f}-{(i+1)/5:.1f}  {n:6d}   {k/n:.0%}" if n else f"{i/5:.1f}-{(i+1)/5:.1f}  {n:6d}   -")


def traffic(dec, ev):
    enc = [e for e in ev if e.get("kind") == "ENCOUNTER"]
    out = {}
    for e in enc:
        out[e["outcome"]] = out.get(e["outcome"], 0) + 1
    print(f"\nthreat encounters {len(enc)}  outcomes {out}")
    if enc:
        print(f"closest 3D approach in any encounter: {min(e['min_3d_m'] for e in enc):.1f} m")
    acts = {}
    for r in dec:
        if r.get("proposal") and r["proposal"].get("traffic_noul") is not None:
            acts[r["executed"]] = acts.get(r["executed"], 0) + 1
    print("actions taken when traffic was in range:", acts)
    bins = [[0, 0] for _ in range(5)]
    for r in dec:
        p, t = r.get("proposal"), (r.get("truth") or {}).get("traffic")
        if p and p.get("traffic_noul") is not None and t:
            b = min(4, int(p["traffic_noul"] * 5))
            bins[b][0] += 1
            bins[b][1] += bool(t["loss_of_separation"])
    print("\ntraffic noul  n   actually on course to lose separation")
    for i, (n, k) in enumerate(bins):
        print(f"{i/5:.1f}-{(i+1)/5:.1f}  {n:6d}   " + (f"{k/n:.0%}" if n else "-"))


if __name__ == "__main__":
    main(sys.argv[1])
    d = Path(sys.argv[1])
    traffic(load(d / "decisions.jsonl"), load(d / "events.jsonl"))
