"""Append-only JSONL run recorder: telemetry, decisions, events, plus a summary.json."""
import json
from datetime import datetime
from pathlib import Path


class Recorder:
    STREAMS = ("telemetry", "decisions", "events")

    def __init__(self, root):
        self.dir = Path(root) / datetime.now().strftime("%Y%m%d-%H%M%S")
        self.dir.mkdir(parents=True, exist_ok=True)
        self._f = {k: open(self.dir / f"{k}.jsonl", "a", buffering=1) for k in self.STREAMS}

    def write(self, stream, row):
        f = self._f.get(stream)
        if f and not f.closed:
            f.write(json.dumps(row, default=str, separators=(",", ":")) + "\n")

    def summary(self, data):
        (self.dir / "summary.json").write_text(json.dumps(data, indent=2, default=str))

    def close(self):
        for f in self._f.values():
            f.close()
