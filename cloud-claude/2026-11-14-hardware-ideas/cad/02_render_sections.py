import sys, trimesh, numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from pathlib import Path
o=Path(sys.argv[1])
fig,axs=plt.subplots(1,3,figsize=(15,5))
cols={"spool_02":"tab:orange","driver_02":"tab:gray","keeper_02":"tab:blue"}
for ax,(orig,norm,title,ix) in zip(axs,[((0,0,0),(0,1,0),"section y=0 (x right, z up)",(0,2)),((8.6,0,0),(1,0,0),"section x=8.6 hub: tensioner (y right, z up)",(1,2)),((1.6,0,0),(1,0,0),"section x=1.6 ext land (y, z)",(1,2))]):
    for n,c in cols.items():
        m=trimesh.load(o/f"{n}.stl")
        sec=m.section(plane_origin=orig,plane_normal=norm)
        if sec is None: continue
        for e in sec.discrete:
            ax.plot(e[:,ix[0]],e[:,ix[1]],color=c,lw=1)
    ax.set_aspect("equal");ax.set_title(title);ax.grid(alpha=.3)
plt.tight_layout();plt.savefig(o/"spool_cartridge_sections.png",dpi=110)
