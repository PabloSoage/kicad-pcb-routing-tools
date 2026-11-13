"""keepnets.py base routed out REGEX: out = base with the copper (tracks, arcs, vias) of the nets
matching REGEX taken from routed instead. Every other net stays exactly as in base: KRT's route.py
re-grades nearby nets at the end even without --rip-existing-nets ."""
import re, sys
base, routed, out, pat = sys.argv[1:5]; rx = re.compile(pat)
tb = open(base, encoding="utf-8").read(); tr = open(routed, encoding="utf-8").read()
def net(b):
    m = re.search(r'\(net (?:\d+ )?"([^"]*)"\)', b); return m[1] if m else ""
blk = re.compile(r'\n\t\((?:segment|via|arc)\b.*?\n\t\)', re.S)
n0 = len(blk.findall(tb))
tb = blk.sub(lambda m: "" if rx.search(net(m.group(0))) else m.group(0), tb)
add = [m.group(0) for m in blk.finditer(tr) if rx.search(net(m.group(0)))]
i = tb.find("\n\t(zone"); i = i if i > 0 else tb.rfind("\n)")
open(out, "w", encoding="utf-8", newline="\n").write(tb[:i] + "".join(add) + tb[i:])
print(f"kept {len(add)} items of the matching nets from {routed}; base had {n0} items")
