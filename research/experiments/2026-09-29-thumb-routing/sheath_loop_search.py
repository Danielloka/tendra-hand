import numpy as np, math, itertools
exec(open(__file__.replace('sheath_loop_search.py', 'sheath_loop_search_base.py')).read().split('best={}')[0])
TUBE_R=1.0
def meta_hit(pts,q,P):
    # metacarpal proximal block in its own (q=0) frame: y in [-40,11], z in [4,18] (grown by tube radius); skip points within 2 mm of the socket
    rel=pts-AX; back=np.array([rot(r,-q) for r in rel])+AX
    near=np.linalg.norm(pts-P,axis=1)<2.0
    inside=(back[:,0]<=11+TUBE_R)&(back[:,1]>=4-TUBE_R)&(back[:,1]<=18+TUBE_R)
    return bool((inside&~near).any())
best={}
for dz in (0,4,8):
  zJ=2.5-dz
  for py in range(0,12):
    for pz in range(12,24):
      rel=np.array([py,pz])-AX
      for ang in range(-90,91,15):
        a=math.radians(ang); d0=np.array([-math.sin(a),math.cos(a)])
        worst=1e9
        for qd in (-13,10,35,60,80):
          q=math.radians(qd); P=AX+rot(rel,q); d=rot(d0,q); bq=0
          for k0,k1 in itertools.product((1,1.5,2,3),(1,1.5,2,3)):
            R,pts=curve(np.array([15.5,zJ]),np.array([0,1.0]),P,d,k0,k1)
            if free(pts,zJ) and not meta_hit(pts,q,P): bq=max(bq,R)
          worst=min(worst,bq)
          if worst<best.get(dz,(0,))[0]: break
        if worst>best.get(dz,(0,))[0]: best[dz]=(worst,py,pz,ang)
for dz,b in best.items(): print('lower %2d: R %.1f socket y%d z%d dir %d'%(dz,*b))
