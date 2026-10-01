"""Extract the RW612 ball map from NXP's FRDM-RW612 schematic PDF.

    pip install pymupdf
    python tools/extract_rw612_balls.py        # -> lib/rw612_balls.json

Pages 3 (U6B power) and 4 (U6A signals) hold the RW612 symbol; ball numbers are black
text, pin names are dark blue (0x004080) and sit ~4.7 pt below their ball number.
"""
import pymupdf, re, json, collections, os
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
doc=pymupdf.open(os.path.join(ROOT,'docs','FRDM-RW612-SCH.pdf'))
BALL=re.compile(r'^[A-HJ-NPR](1[0-5]|[1-9])$')
allpins={}; unitof={}
for pno,unit in [(2,'B'),(3,'A')]:
    pg=doc[pno]; M=pg.rotation_matrix
    spans=[]
    for b in pg.get_text('dict')['blocks']:
        for l in b.get('lines',[]):
            horiz = abs(l['dir'][1])>0.5   # page rotated 90 -> displayed horizontal
            for s in l['spans']:
                r=pymupdf.Rect(s['bbox'])*M
                t=s['text'].strip()
                if not t: continue
                spans.append(dict(t=t,c=s['color'],x0=r.x0,y0=r.y0,x1=r.x1,y1=r.y1,cx=(r.x0+r.x1)/2,cy=(r.y0+r.y1)/2,h=horiz))
    balls=[s for s in spans if s['c']==0 and BALL.match(s['t'])]
    names=[s for s in spans if s['c']==0x4080]
    res={}
    for b in [b for b in balls if b['h']]:
        c=[n for n in names if n['h'] and 3.7<(n['cy']-b['cy'])<5.7 and abs(n['cx']-b['cx'])<120]
        res[b['t']]=min(c,key=lambda n:abs(n['cx']-b['cx']))['t'] if c else None
    rows=collections.defaultdict(list)
    for b in balls:
        if not b['h']: rows[round(b['y0']/5)].append(b)
    nrows=collections.defaultdict(list)
    for n in names:
        if not n['h']: nrows[round(n['y1']/5)].append(n)
    for k,bl in rows.items():
        bl.sort(key=lambda s:s['cx'])
        kk=min(nrows,key=lambda kk:abs(kk*5-bl[0]['y0']))
        nl=sorted(nrows[kk],key=lambda s:s['cx'])
        print('page',pno+1,'vertical row',len(bl),len(nl))
        for b,n in zip(bl,nl): res[b['t']]=n['t']
    for k,v in res.items():
        if k in allpins: print('dup',k)
        allpins[k]=v; unitof[k]=unit
    print('page',pno+1,'balls',len(res))
json.dump({'pins':allpins,'unit':unitof},open(os.path.join(ROOT,'lib','rw612_balls.json'),'w'),indent=1,sort_keys=True)
print(len(allpins),'unmatched',[k for k,v in allpins.items() if v is None])
rowsL='ABCDEFGHJKLMNPR'
missing=[f'{r}{c}' for r in rowsL for c in range(1,16) if f'{r}{c}' not in allpins]
print('grid positions not used:',len(missing),missing)
