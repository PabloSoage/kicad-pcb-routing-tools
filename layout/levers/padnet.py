# padnet.py in out REF:PAD... : take those pads off their nets (remove the pad's (net ...) line)
import re,sys
inp,out,*pads=sys.argv[1:]; t=open(inp,encoding="utf-8").read()
for rp in pads:
    ref,pad=rp.split(":")
    i=t.find(f'(property "Reference" "{ref}"'); s=t.rfind("\n\t(footprint",0,i); e=t.find("\n\t)",i)
    fp=t[s:e]; m=re.search(r'\(pad "'+re.escape(pad)+r'" .*?\n\t\t\)',fp,re.S)
    blk=re.sub(r'\n\t\t\t\(net (?:\d+ )?"[^"]*"\)','',m.group(0),count=1)
    fp=fp[:m.start()]+blk+fp[m.end():]; t=t[:s]+fp+t[e:]
open(out,"w",encoding="utf-8").write(t)
