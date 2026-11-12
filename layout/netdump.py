"""netdump.py board REGEX: print the pads (PAD net ref.pad side x y), vias (VIA net x y) and tracks
(SEG net layer x1 y1 x2 y2 mm) of the nets matching REGEX."""
import re, sys, math
t = open(sys.argv[1], encoding="utf-8").read()
pat = re.compile(sys.argv[2])
# pads
for fp in re.finditer(r"\n\t\(footprint .*?\n\t\)", t, re.S):
    f = fp.group(0)
    ref = re.search(r'\(property "Reference" "([^"]+)"', f).group(1)
    at = re.search(r"\n\t\t\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)", f)
    X, Y, A = float(at.group(1)), float(at.group(2)), math.radians(float(at.group(3) or 0))
    side = "B" if '\n\t\t(layer "B.Cu")' in f else "F"
    for p in re.finditer(r'\(pad "([^"]*)" \w+ \w+\s*\(at ([-\d.]+) ([-\d.]+).*?\(net "([^"]*)"\)', f, re.S):
        if pat.search(p.group(4)):
            px, py = float(p.group(2)), float(p.group(3))
            print("PAD", p.group(4), f"{ref}.{p.group(1)}", side, round(X + px*math.cos(A) + py*math.sin(A), 3), round(Y - px*math.sin(A) + py*math.cos(A), 3))
for m in re.finditer(r"\n\t\((segment|arc|via)\n(.*?)\n\t\)", t, re.S):
    k, b = m.groups()
    n = re.search(r'\(net "([^"]*)"\)', b)
    if not n or not pat.search(n.group(1)): continue
    if k == "via":
        at = re.search(r"\(at ([-\d.]+) ([-\d.]+)\)", b).groups(); print("VIA", n.group(1), *at)
    else:
        s = re.search(r"\(start ([-\d.]+) ([-\d.]+)\)", b).groups(); e = re.search(r"\(end ([-\d.]+) ([-\d.]+)\)", b).groups()
        L = re.search(r'\(layer "([^"]+)"\)', b).group(1)
        print("SEG", n.group(1), L, *s, *e, round(math.dist(tuple(map(float,s)), tuple(map(float,e))),2))
