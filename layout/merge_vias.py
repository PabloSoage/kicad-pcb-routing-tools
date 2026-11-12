#!/usr/bin/env python3
"""
Merge vias of one net that sit closer than the hole-to-hole rule (3 Nov 2026): a local region pass
can leave a second via 0.2 mm from the first (DRC hole_to_hole warning, drills touching). For each
such pair the second via is deleted and the track ends that sat inside it move to the kept via's
centre. Text edit of the .kicad_pcb (no pcbnew); run the DRC afterwards.

Usage:  python layout/merge_vias.py <in.kicad_pcb> <out.kicad_pcb> [--hole-gap 0.25]
"""

import argparse
import math
import re

ap = argparse.ArgumentParser()
ap.add_argument("inp"); ap.add_argument("out")
ap.add_argument("--hole-gap", type=float, default=0.25, help="board hole-to-hole minimum, mm")
a = ap.parse_args()

t = open(a.inp, encoding="utf-8").read()
VIA = re.compile(r"\n\t\(via\n(?:\t\t.*\n)*?\t\)")
vias = []
for m in VIA.finditer(t):
    b = m.group(0)
    x, y = map(float, re.search(r"\(at ([-\d.]+) ([-\d.]+)\)", b).groups())
    net = re.search(r'\(net (?:"([^"]*)"|(\d+))\)', b)
    vias.append(dict(span=m.span(), x=x, y=y, net=net.group(1) or net.group(2),
                     drill=float(re.search(r"\(drill ([\d.]+)\)", b).group(1)),
                     size=float(re.search(r"\(size ([\d.]+)\)", b).group(1))))

drop, moves = set(), []                              # moves: (x, y, r of the dropped via, kept x, y, net)
for i, u in enumerate(vias):
    if i in drop:
        continue
    for j in range(i + 1, len(vias)):
        v = vias[j]
        if j in drop or v["net"] != u["net"]:
            continue
        d = math.hypot(u["x"] - v["x"], u["y"] - v["y"])
        if d - (u["drill"] + v["drill"]) / 2 < a.hole_gap:
            drop.add(j)
            moves.append((v["x"], v["y"], v["size"] / 2, u["x"], u["y"], u["net"]))
            print(f"{u['net']}: via ({v['x']}, {v['y']}) merged into ({u['x']}, {u['y']}), {d:.3f} mm apart")

for j in sorted(drop, key=lambda j: -vias[j]["span"][0]):
    s, e = vias[j]["span"]
    t = t[:s] + t[e:]


def move_ends(m):
    b = m.group(0)
    net = re.search(r'\(net (?:"([^"]*)"|(\d+))\)', b)
    net = net and (net.group(1) or net.group(2))
    for x, y, r, kx, ky, n in moves:
        if n != net:
            continue
        def end(e):
            ex, ey = float(e.group(2)), float(e.group(3))
            if math.hypot(ex - x, ey - y) < r:
                return f"({e.group(1)} {kx:.6f} {ky:.6f})"
            return e.group(0)
        b = re.sub(r"\((start|end) ([-\d.]+) ([-\d.]+)\)", end, b)
    return b


t = re.sub(r"\n\t\(segment\n(?:\t\t.*\n)*?\t\)", move_ends, t)
open(a.out, "w", encoding="utf-8", newline="\n").write(t)
print(f"{len(drop)} vias merged")
