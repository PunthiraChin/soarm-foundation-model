"""Run the offline one-command demo and optionally emit its JSONL trace."""
import argparse
import json
from .voice import VoiceIngress
from .fixture import Bench
from .planner import FixturePlanner
from .runtime import ControlLoop


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("goal", nargs="?", default="หยิบก้อนสีฟ้าไปวางในถาด")
    parser.add_argument("--trace", action="store_true", help="print each mock event")
    args = parser.parse_args()
    bench, planner = Bench(), FixturePlanner()
    loop = ControlLoop(bench.clock, planner, bench.cameras, bench.ik, bench.plant)
    VoiceIngress(loop).deliver(1, {"kind": "goal", "text": args.goal})
    while loop.state not in {"done", "ask", "stopped", "fault"}:
        bench.advance()
        loop.tick()
    if args.trace:
        for event in loop.log:
            print(json.dumps(event, ensure_ascii=False))
    print(json.dumps(dict(mode="synthetic", state=loop.state, checkpoints=len(planner.requests),
                          writes=len(bench.plant.writes), simulated_ms=bench.clock.ms,
                          last_decision=next((e["reason"] for e in reversed(loop.log)
                                             if e["event"] == "decision"), None)), ensure_ascii=False))
    return 0 if loop.state in {"done", "ask"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
