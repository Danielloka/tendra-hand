import numpy as np, math, itertools
AX=np.array([10.0236,11.3764])
def rot(v,q):
    c,s=math.cos(-q),math.sin(-q); return np.array([v[0]*c-v[1]*s, v[0]*s+v[1]*c])
S=np.linspace(0,1,121)
def curve(p0,t0,p1,t1,k0,k1):
    L=np.linalg.norm(p1-p0); m0=t0*L*k0; m1=t1*L*k1
    s=S[:,None]
    pts=(2*s**3-3*s**2+1)*p0+(s**3-2*s**2+s)*m0+(-2*s**3+3*s**2)*p1+(s**3-s**2)*m1
    d1=(6*s*s-6*s)*p0+(3*s*s-4*s+1)*m0+(-6*s*s+6*s)*p1+(3*s*s-2*s)*m1
    d2=(12*s-6)*p0+(6*s-4)*m0+(-12*s+6)*p1+(6*s-2)*m1
    k=np.abs(d1[:,0]*d2[:,1]-d1[:,1]*d2[:,0])/np.linalg.norm(d1,axis=1)**3
    return 1/k.max(), pts
def free(pts, zJ):  # stay in the free region behind/under metacarpal: y<=27, z<=29, and not below plate top
    return pts[:,0].max()<=27 and pts[:,1].max()<=29 and pts[:,1].min()>=zJ-0.01
best={}
for dz in (0,4,6,8,10):
  zJ=2.5-dz
  for py in range(4,17):
    for pz in range(2,22):
      rel=np.array([py,pz])-AX
      if np.linalg.norm(rel)<3.5 or np.linalg.norm(rel)>9: continue
      for ang in range(-180,180,15):
        a=math.radians(ang); d0=np.array([-math.sin(a),math.cos(a)])
        worst=1e9
        for qd in (-13,10,35,60,80):
          q=math.radians(qd); P=AX+rot(rel,q); d=rot(d0,q)
          bq=0
          for k0,k1 in itertools.product((1,1.5,2.5),(1,1.5,2.5)):
            R,pts=curve(np.array([15.5,zJ]),np.array([0,1.0]),P,d,k0,k1)
            if free(pts,zJ): bq=max(bq,R)
          worst=min(worst,bq)
          if worst<best.get(dz,(0,))[0]: break
        if worst>best.get(dz,(0,))[0]: best[dz]=(worst,py,pz,ang)
for dz,b in best.items(): print('lower %2d: R %.1f socket y%d z%d dir %d'%(dz,*b))
