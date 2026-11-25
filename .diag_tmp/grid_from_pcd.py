#!/usr/bin/env python3
"""从（已验证 2.5 cm 自洽的）地图系点云直接生成 2D 栅格图。

free     = 观测到地面的格（z 在 [-0.15, 0.12]）
occupied = 有中高度结构的格（z 在 [0.25, 2.0]，且至少 3 个点）
unknown  = 其余（写 205，符合 map_server 语义）
"""
import argparse, sys
import numpy as np
from scipy import ndimage

def read_pcd(path):
    with open(path,'rb') as f:
        hdr={}
        while True:
            line=f.readline().decode('ascii','replace').strip()
            if not line or line.startswith('#'): continue
            k,_,v=line.partition(' '); hdr[k.upper()]=v.strip()
            if k.upper()=='DATA': break
        payload=f.read()
    fields=hdr['FIELDS'].split(); sizes=[int(x) for x in hdr['SIZE'].split()]; types=hdr['TYPE'].split()
    dt={'F':{4:'f4',8:'f8'},'U':{1:'u1',2:'u2',4:'u4',8:'u8'},'I':{1:'i1',2:'i2',4:'i4',8:'i8'}}
    return np.frombuffer(payload,dtype=np.dtype([(f,'<'+dt[t][s]) for f,t,s in zip(fields,types,sizes)]),count=int(hdr['POINTS']))

def write_pgm(path, a):
    with open(path,'wb') as f:
        f.write(b'P5\n# generated from pcd\n%d %d\n255\n' % (a.shape[1], a.shape[0]))
        f.write(a.tobytes())

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--pcd', required=True)
    ap.add_argument('--out-prefix', required=True)
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--margin', type=float, default=1.0)
    args=ap.parse_args()

    p=read_pcd(args.pcd)
    X=p['x'].astype(float); Y=p['y'].astype(float); Z=p['z'].astype(float)
    m=np.isfinite(X)&np.isfinite(Y)&np.isfinite(Z)
    X,Y,Z=X[m],Y[m],Z[m]
    res=args.res
    x0=np.floor((X.min()-args.margin)/res)*res
    y0=np.floor((Y.min()-args.margin)/res)*res
    x1=np.ceil((X.max()+args.margin)/res)*res
    y1=np.ceil((Y.max()+args.margin)/res)*res
    W=int(round((x1-x0)/res)); H=int(round((y1-y0)/res))
    ci=np.floor((X-x0)/res).astype(np.int64)
    cj=(H-1-np.floor((Y-y0)/res).astype(np.int64))
    ok=(ci>=0)&(ci<W)&(cj>=0)&(cj<H)
    ci,cj=ci[ok],cj[ok]; Z=Z[ok]

    ground=(Z>=-0.15)&(Z<=0.12)
    wall=(Z>=0.25)&(Z<=2.0)

    gcount=np.zeros((H,W),np.int32); wcount=np.zeros((H,W),np.int32)
    np.add.at(gcount,(cj[ground],ci[ground]),1)
    np.add.at(wcount,(cj[wall],ci[wall]),1)

    free=gcount>=3
    occ=wcount>=3
    # 墙与地面冲突时以墙为准（墙根附近的斜射地面点会落进墙格）
    free &= ~occ
    # 去掉零星墙噪点：只保留有 >=2 个 8 邻域墙格的连通片
    lbl,n=ndimage.label(occ, structure=np.ones((3,3)))
    if n:
        sizes=ndimage.sum(occ,lbl,range(1,n+1))
        keep=np.isin(lbl, np.flatnonzero(sizes>=4)+1)
        occ=keep
    # 补掉自由区里的小洞（门框/立柱投影造成的 1~2 格孔）
    free=ndimage.binary_closing(free, structure=np.ones((3,3)), iterations=1) & ~occ

    a=np.full((H,W),205,np.uint8)
    a[free]=254
    a[occ]=0

    write_pgm(args.out_prefix+'.pgm', a)
    with open(args.out_prefix+'.yaml','w',encoding='utf-8') as f:
        f.write('image: %s.pgm\nmode: trinary\nresolution: %.3f\norigin: [%.3f, %.3f, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n'
                % (args.out_prefix.rsplit('/',1)[-1], res, x0, y0))
    print('grid %dx%d @ %.3f m origin (%.2f, %.2f)'%(W,H,res,x0,y0))
    print('free %.1f%%  occupied %.1f%%  unknown %.1f%%'%(100*free.mean(),100*occ.mean(),100*((~free)&(~occ)).mean()))
    print('wrote', args.out_prefix+'.pgm/.yaml')

if __name__=='__main__':
    main()
