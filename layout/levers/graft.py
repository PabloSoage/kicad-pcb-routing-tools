# graft.py base src out NETREGEX [rename a=b ...]: add src's tracks/vias of matching nets into base (renamed)
import re,sys
base,src,out,pat,*ren=sys.argv[1:]; ren=[r.split("=",1) for r in ren]
tb=open(base,encoding="utf-8").read(); ts=open(src,encoding="utf-8").read(); rx=re.compile(pat)
add=[]
for m in re.finditer(r'\n\t\((segment|via|arc)\b.*?\n\t\)',ts,re.S):
    b=m.group(0); n=re.search(r'\(net (?:\d+ )?"([^"]*)"\)',b)
    if n and rx.search(n[1]):
        for a,c in ren: b=b.replace(f'"{a}"',f'"{c}"')
        add.append(b)
i=tb.find("\n\t(zone"); i=i if i>0 else tb.rfind("\n)")
open(out,"w",encoding="utf-8").write(tb[:i]+"".join(add)+tb[i:]); print("grafted",len(add))
