"""segdel.py in out NET x1,y1:x2,y2 ... : delete NET's segments with these end points (either
order, within 0.01 mm). Text edit; pairs with segadd.py for small hand edits."""
import re, sys, math

inp, out, net, *specs = sys.argv[1:]
want = [tuple(tuple(map(float, p.split(","))) for p in s.split(":")) for s in specs]
t = open(inp, encoding="utf-8").read()
hit = [0] * len(want)


def f(m):
    b = m.group(0)
    if f'(net "{net}")' not in b: return b
    s = tuple(map(float, re.search(r"\(start ([-\d.]+) ([-\d.]+)\)", b).groups()))
    e = tuple(map(float, re.search(r"\(end ([-\d.]+) ([-\d.]+)\)", b).groups()))
    for i, (p, q) in enumerate(want):
        if (math.dist(s, p) < 0.01 and math.dist(e, q) < 0.01) or (math.dist(s, q) < 0.01 and math.dist(e, p) < 0.01):
            hit[i] += 1; return ""
    return b


t = re.sub(r"\n\t\(segment\n.*?\n\t\)", f, t, flags=re.S)
open(out, "w", encoding="utf-8", newline="\n").write(t)
for s, h in zip(specs, hit):
    if h != 1: print(f"{s}: {h} segments deleted")
print(f"{sum(hit)} segments deleted")
