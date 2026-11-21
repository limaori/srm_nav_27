"""Continuous Euclidean clearance from polyline segments to axis-aligned cells."""
import numpy as np


def segment_boxes(a, b, centers, half_size):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    delta = b-a
    low, high = centers-half_size, centers+half_size
    enter, leave = np.zeros(len(centers)), np.ones(len(centers))
    for axis in (0, 1):
        if abs(delta[axis]) < 1e-15:
            outside = (a[axis]<low[:,axis]) | (a[axis]>high[:,axis])
            enter[outside], leave[outside] = np.inf, -np.inf
        else:
            t0, t1 = (low[:,axis]-a[axis])/delta[axis], (high[:,axis]-a[axis])/delta[axis]
            enter = np.maximum(enter,np.minimum(t0,t1))
            leave = np.minimum(leave,np.maximum(t0,t1))
    hit = np.flatnonzero(enter<=leave)
    if len(hit):
        i=int(hit[0])
        return 0.0, (a+enter[i]*delta).tolist(), centers[i].tolist()
    d0=np.sum(np.maximum(np.abs(centers-a)-half_size,0)**2,axis=1)
    d1=np.sum(np.maximum(np.abs(centers-b)-half_size,0)**2,axis=1)
    best=np.minimum(d0,d1)
    parameter=np.where(d0<=d1,0.0,1.0)
    norm=float(delta@delta)
    if norm>1e-30:
        for sx,sy in [(-1,-1),(-1,1),(1,-1),(1,1)]:
            corners=centers+np.array([sx,sy])*half_size
            t=np.clip((corners-a)@delta/norm,0,1)
            dist=np.sum((corners-(a+t[:,None]*delta))**2,axis=1)
            better=dist<best
            best[better],parameter[better]=dist[better],t[better]
    i=int(np.argmin(best))
    return float(np.sqrt(best[i])),(a+parameter[i]*delta).tolist(),centers[i].tolist()


def path_clearance(path, costs, resolution, origin):
    yy,xx=np.nonzero(costs>=100)
    if not len(xx) or len(path)<2:
        raise ValueError('Clearance requires a path and lethal cells')
    centers=np.column_stack((origin[0]+(xx+.5)*resolution,origin[1]+(yy+.5)*resolution))
    # An endpoint supplies an upper bound. Cells farther than that bound from
    # the whole path bounding box cannot be the nearest obstacle.
    points=np.asarray(path,dtype=float)
    upper=float(np.linalg.norm(np.maximum(np.abs(centers-points[0])-resolution/2,0),axis=1).min())
    low=points.min(axis=0)-upper-resolution/2-1e-9
    high=points.max(axis=0)+upper+resolution/2+1e-9
    centers=centers[np.all((centers>=low)&(centers<=high),axis=1)]
    best=(float('inf'),None,None)
    width,height=costs.shape[1]*resolution,costs.shape[0]*resolution
    for p in path:
        if not (origin[0]<=p[0]<origin[0]+width and origin[1]<=p[1]<origin[1]+height):
            return {'min_m':0.0,'xy':list(p),'reason':'out of map'}
    for a,b in zip(path,path[1:]):
        current=segment_boxes(a,b,centers,resolution/2)
        if current[0]<best[0]: best=current
    return {'min_m':best[0],'xy':best[1],'nearest_lethal_center':best[2]}
