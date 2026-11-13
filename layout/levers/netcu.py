import re,sys
t=open(sys.argv[1],encoding="utf-8").read(); pat=re.compile(sys.argv[2])
x0,y0,x1,y1=map(float,sys.argv[3:7]) if len(sys.argv)>6 else (-1e9,-1e9,1e9,1e9)
inb=lambda x,y:x0<=x<=x1 and y0<=y<=y1
for m in re.finditer(r'\n\t\(segment\s*\n?\s*\(start ([-\d.]+) ([-\d.]+)\)\s*\(end ([-\d.]+) ([-\d.]+)\)\s*\(width ([\d.]+)\)\s*\(layer "([^"]+)"\)\s*\(net (?:\d+ )?"([^"]*)"\)',t):
    a,b,c,d=map(float,m.group(1,2,3,4))
    if pat.search(m[7]) and (inb(a,b) or inb(c,d)): print("seg",m[7],m[6],m[5],f"{a:.3f},{b:.3f} -> {c:.3f},{d:.3f}")
for m in re.finditer(r'\n\t\(via\s*(?:\(\w+\)\s*)*\(at ([-\d.]+) ([-\d.]+)\)\s*\(size ([\d.]+)\)\s*\(drill ([\d.]+)\)\s*\(layers ([^)]*)\)[\s\S]*?\(net (?:\d+ )?"([^"]*)"\)',t):
    a,b=float(m[1]),float(m[2])
    if pat.search(m[6]) and inb(a,b): print("via",m[6],m[3],m[4],m[5],f"{a:.3f},{b:.3f}")
