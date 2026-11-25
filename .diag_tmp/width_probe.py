import numpy as np, yaml, sys
from scipy import ndimage
sys.path.insert(0,'.diag_tmp')
from map_clearance_report import load_pgm
for name in ['227to219_1125/227to219_1125','227_1113pro/227_1113pro','219_1121/219_1121']:
    m=yaml.safe_load(open(f'maps/{name}.yaml'))
    import os; a=load_pgm(os.path.join(os.path.dirname(f'maps/{name}.yaml'),m['image'])); res=float(m['resolution'])
    free=a>=250; occ=a<=100; unk=~free&~occ
    dt=ndimage.distance_transform_edt(~(occ|unk))*res
    # 走廊宽度 = 中轴处 2*dt；用局部极大近似中轴
    mx=ndimage.maximum_filter(dt,size=11)
    axis=free&(dt>=mx-1e-6)&(dt>0.2)
    w=2*dt[axis]
    print(f"{name:32s} free {free.mean()*100:5.1f}%  occ {occ.mean()*100:5.1f}%  unk {unk.mean()*100:5.1f}%  走廊宽 p10 {np.percentile(w,10):.2f}  p50 {np.percentile(w,50):.2f}  p90 {np.percentile(w,90):.2f} m  最窄 {w.min():.2f}")
