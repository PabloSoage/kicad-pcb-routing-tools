#!/usr/bin/env python3
"""
Remove the copper behind DRC errors, so the nets can be routed again (11 Oct 2026).

For each clearance, hole_to_hole, hole_clearance, shorting_items or
tracks_crossing violation in a kicad-cli DRC report (JSON), one track or via
is deleted, matched by the UUID the report gives:
  * a pad against a track or via: the track or via;
  * two pieces of copper: the one NOT in the RF50 class (the IF lines are
    routed last and on purpose), else the second.
The nets it touched are printed, one per line, for route_finish.sh's ONLY.

Usage:  python layout/rip_violations.py board.kicad_pcb drc.json [out.kicad_pcb]
"""

import json
import re
import sys

ERRORS = ("clearance", "hole_to_hole", "hole_clearance", "shorting_items", "tracks_crossing")
RF = re.compile(r"\[/IF_(RX|TX)_[ABC]\]")


def main():
    board, report = sys.argv[1], sys.argv[2]
    out = sys.argv[3] if len(sys.argv) > 3 else board
    text = open(board, encoding="utf-8").read()
    d = json.load(open(report, encoding="utf-8"))
    kill, nets = set(), set()
    for v in d["violations"]:
        if v["type"] not in ERRORS:
            continue
        copper = [i for i in v["items"] if not i["description"].startswith("Pad")]
        if not copper:
            continue
        pick = [i for i in copper if not RF.search(i["description"])] or copper
        it = pick[-1]
        kill.add(it["uuid"])
        m = re.search(r"\[([^\]]+)\]", it["description"])
        if m:
            nets.add(m.group(1))
    n = 0

    def drop(m):
        nonlocal n
        u = re.search(r'\(uuid "([^"]+)"\)', m.group(0))
        if u and u.group(1) in kill:
            n += 1
            return ""
        return m.group(0)
    text = re.sub(r"\n\t\((?:segment|via|arc)\n.*?\n\t\)", drop, text, flags=re.S)
    open(out, "w", encoding="utf-8", newline="\n").write(text)
    print(f"{n} of {len(kill)} items removed", file=sys.stderr)
    print("\n".join(sorted(nets)))


if __name__ == "__main__":
    main()
