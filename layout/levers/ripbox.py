"""ripbox.py in out REGEX [--box x0 y0 x1 y1] [--layer L]...
Delete the segments/vias/arcs of the nets matching REGEX; with --box only those with an end
(or the via) inside the box, with --layer only on those layers. Several REGEX may be given as
'a|b'. Repeat the whole option set with ';' between groups on the command line of run scripts."""
import re, argparse

ap = argparse.ArgumentParser()
ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("rx")
ap.add_argument("--box", nargs=4, type=float)
ap.add_argument("--layer", action="append", default=[])
a = ap.parse_args()
t = open(a.inp, encoding="utf-8").read()
n = 0


def inside(x, y):
    x0, y0, x1, y1 = a.box
    return x0 <= x <= x1 and y0 <= y <= y1


def drop(m):
    global n
    s = m.group(0)
    net = re.search(r'\(net "([^"]*)"\)', s)
    if not net or not re.search(a.rx, net.group(1)):
        return s
    if a.layer and not any(f'"{l}"' in s for l in a.layer):
        return s
    if a.box:
        pts = [(float(p.group(1)), float(p.group(2)))
               for p in re.finditer(r"\((?:start|end|at|mid) ([-\d.]+) ([-\d.]+)\)", s)]
        if not any(inside(*p) for p in pts):
            return s
    n += 1
    return ""


t = re.sub(r"\n\t\((?:segment|via|arc)\n.*?\n\t\)", drop, t, flags=re.S)
open(a.out, "w", encoding="utf-8", newline="\n").write(t)
print(f"{a.rx}: {n} items ripped")
