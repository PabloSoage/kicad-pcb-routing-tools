import re,sys
def items(p):
    t=open(p,encoding="utf-8").read(); d={}
    for m in re.finditer(r'\n\t\((segment|via|arc)\b(.*?)\n\t\)',t,re.S):
        b=m[2]; n=re.search(r'\(net (?:\d+ )?"([^"]*)"\)',b)
        geo=tuple(re.findall(r'\((?:start|end|at|mid) ([-\d.]+) ([-\d.]+)\)',b)); w=re.search(r'\((?:width|size) ([\d.]+)',b)
        l=re.search(r'\(layers? ([^)]*)\)',b)
        d.setdefault(n[1] if n else "",set()).add((m[1],geo,w[1] if w else "",l[1] if l else ""))
    return d
a,b=items(sys.argv[1]),items(sys.argv[2])
for n in sorted(set(a)|set(b)):
    if a.get(n,set())!=b.get(n,set()): print(n, "-",len(a.get(n,set())-b.get(n,set())),"+",len(b.get(n,set())-a.get(n,set())))
