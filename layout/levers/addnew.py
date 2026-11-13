"""addnew.py base routed out REGEX: out = base plus the tracks and vias of the nets matching REGEX
that routed has and base does not. Removals in routed are ignored: KRT's route.py may re-grade a
big net's existing copper while joining one new pad, and that must not reach the board."""
import re, sys
base, routed, out, pat = sys.argv[1:5]; rx = re.compile(pat)
blk = re.compile(r'\n\t\((?:segment|via|arc)\b.*?\n\t\)', re.S)
def key(b):
    g = tuple(re.findall(r'\((?:start|end|at|mid) ([-\d.]+) ([-\d.]+)\)', b)); w = re.search(r'\((?:width|size) ([\d.]+)', b)
    l = re.search(r'\(layers? ([^)]*)\)', b); n = re.search(r'\(net (?:\d+ )?"([^"]*)"\)', b)
    return (n[1] if n else "", g, w[1] if w else "", l[1] if l else "")
tb = open(base, encoding="utf-8").read(); tr = open(routed, encoding="utf-8").read()
have = {key(m.group(0)) for m in blk.finditer(tb)}
add = [m.group(0) for m in blk.finditer(tr) if rx.search(key(m.group(0))[0]) and key(m.group(0)) not in have]
i = tb.find("\n\t(zone"); i = i if i > 0 else tb.rfind("\n)")
open(out, "w", encoding="utf-8", newline="\n").write(tb[:i] + "".join(add) + tb[i:])
print("added", len(add))
