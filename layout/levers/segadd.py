"""segadd.py in out NET LAYER x,y x,y ... [--width 0.1] [--via x,y]...
Add a track of NET on LAYER through the given points (one segment per pair), and through vias
(0.45/0.2) at the --via points. Text edit, no checks: run the DRC after."""
import argparse, uuid

ap = argparse.ArgumentParser()
ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("net"); ap.add_argument("layer")
ap.add_argument("pts", nargs="*")
ap.add_argument("--width", type=float, default=0.1)
ap.add_argument("--via", action="append", default=[])
a = ap.parse_args()
t = open(a.inp, encoding="utf-8").read()
pts = [tuple(map(float, p.split(","))) for p in a.pts]
txt = ""
for p, q in zip(pts, pts[1:]):
    txt += (f'\n\t(segment\n\t\t(start {p[0]:.6f} {p[1]:.6f})\n\t\t(end {q[0]:.6f} {q[1]:.6f})\n\t\t(width {a.width})\n'
            f'\t\t(layer "{a.layer}")\n\t\t(net "{a.net}")\n\t\t(uuid "{uuid.uuid4()}")\n\t)')
for v in a.via:
    x, y = map(float, v.split(","))
    txt += (f'\n\t(via\n\t\t(at {x:.6f} {y:.6f})\n\t\t(size 0.45)\n\t\t(drill 0.2)\n\t\t(layers "F.Cu" "B.Cu")\n'
            f'\t\t(net "{a.net}")\n\t\t(uuid "{uuid.uuid4()}")\n\t)')
i = t.find("\n\t(segment")
if i < 0: i = t.find("\n\t(via")
t = t[:i] + txt + t[i:]
open(a.out, "w", encoding="utf-8", newline="\n").write(t)
print(f"{len(pts) - 1 if pts else 0} segments, {len(a.via)} vias added to {a.net}")
