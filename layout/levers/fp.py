import re,sys,math
t=open(sys.argv[1],encoding="utf-8").read()
ref=sys.argv[2]
i=t.find(f'(property "Reference" "{ref}"')
s=t.rfind("\n\t(footprint",0,i); e=t.find("\n\t)",i)
fp=t[s:e]
at=re.search(r"\n\t\t\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)",fp)
X,Y,R=float(at[1]),float(at[2]),float(at[3] or 0)
for m in re.finditer(r'\(pad "([^"]+)"[^\n]*\n((?:\t\t\t.*\n)*?)\t\t\)',fp):
    b=m[2]; p=re.search(r"\(at ([-\d.]+) ([-\d.]+)",b)
    n=re.search(r'\(net (?:\d+ )?"([^"]*)"\)',b)
    px,py=float(p[1]),float(p[2]); a=math.radians(-R)
    x=X+px*math.cos(a)-py*math.sin(a); y=Y+px*math.sin(a)+py*math.cos(a)
    print(m[1], f"{x:.2f} {y:.2f}", n[1] if n else "-")
