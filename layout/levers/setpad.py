"""setpad.py in out REF:PAD=NET ...: give pads a net by text (a trial before the schematic exists:
the DRC's parity check will flag it). Replaces the pad's (net ...) line or adds one."""
import re, sys
inp, out, *items = sys.argv[1:]
t = open(inp, encoding="utf-8").read()
for it in items:
    rp, net = it.split("="); ref, pad = rp.split(":")
    i = t.find(f'(property "Reference" "{ref}"'); s = t.rfind("\n\t(footprint", 0, i); e = t.find("\n\t)", i)
    fp = t[s:e]
    m = re.search(r'\(pad "' + re.escape(pad) + r'" .*?\n\t\t\)', fp, re.S)
    assert m, it
    blk = m.group(0)
    if re.search(r'\(net (?:\d+ )?"[^"]*"\)', blk): blk2 = re.sub(r'\(net (?:\d+ )?"[^"]*"\)', f'(net "{net}")', blk, count=1)
    else:   # no (net ...) yet: add one before the pad's closing parenthesis
        k = blk.rfind("\n\t\t)"); blk2 = blk[:k] + f'\n\t\t\t(net "{net}")' + blk[k:]
    fp = fp[:m.start()] + blk2 + fp[m.end():]; t = t[:s] + fp + t[e:]
open(out, "w", encoding="utf-8", newline="\n").write(t)
print("set", len(items))
