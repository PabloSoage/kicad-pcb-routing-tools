#!/usr/bin/env python3
"""
Remove the router's dangling copper: the tracks and vias KiCad's own DRC calls
track_dangling / via_dangling, and nothing else.

The autorouter leaves stubs behind (rip-ups, abandoned escapes). Rather than
re-deriving connectivity, this asks kicad-cli: run DRC with the zones refilled
(so a via that feeds a plane is NOT dangling), delete each reported item, and
repeat, because removing a stub can leave the segment before it dangling.
Items are matched by the UUID the report gives for each.

Also removed: supply-net tracks the report calls items_not_allowed (26 Oct 2026).
The router's plane finalize leaves 0.01-0.2 mm supply stubs at pads on L4 that reach
into the islands' no-tracks keep-outs; KiCad 10.0.6 reports them, the plane connects
those pads anyway. A signal track in a keep-out is left for the DRC to show.

Usage:  python layout/drop_dangling.py board.kicad_pcb [--max-rounds 6]
"""

import json
import os
import re
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from toolpaths import KICAD_CLI                      # noqa: E402


def drc(board):
    out = os.path.join(tempfile.mkdtemp(), "drc.json")
    subprocess.run([KICAD_CLI, "pcb", "drc", "--refill-zones", "--format", "json", "-o", out, board],
                   capture_output=True)
    return json.load(open(out, encoding="utf-8"))


def dangling(report):
    """UUIDs of the items the report calls dangling, and of supply stubs in keep-outs."""
    dang = {it["uuid"] for v in report["violations"]
            if v["type"] in ("track_dangling", "via_dangling") for it in v["items"]}
    stubs = {it["uuid"] for v in report["violations"] if v["type"] == "items_not_allowed"
             for it in v["items"] if re.search(r"\[[+]", it["description"])}
    return dang | stubs


def main():
    board = sys.argv[1]
    rounds = int(sys.argv[sys.argv.index("--max-rounds") + 1]) if "--max-rounds" in sys.argv else 6
    total = 0
    for r in range(rounds):
        targets = dangling(drc(board))
        if not targets:
            break
        text = open(board, encoding="utf-8").read()
        removed = 0

        def keep(m):
            nonlocal removed
            uid = re.search(r'\(uuid "([^"]+)"\)', m.group(0))
            if uid and uid.group(1) in targets:
                removed += 1
                return ""
            return m.group(0)
        text = re.sub(r"\n\t\((?:segment|via)\n.*?\n\t\)", keep, text, flags=re.S)
        open(board, "w", encoding="utf-8", newline="\n").write(text)
        print(f"round {r + 1}: {len(targets)} reported, {removed} removed")
        total += removed
        if not removed:
            break
    print(f"{total} dangling items removed")


if __name__ == "__main__":
    main()
