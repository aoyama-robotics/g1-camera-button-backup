from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import json

root=Path(__file__).resolve().parents[1]
b=(root/'sources/head_link.STL').read_bytes()
dtype=np.dtype([('normal','<f4',3),('v','<f4',(3,3)),('attr','<u2')])
t=np.frombuffer(b, dtype=dtype, offset=84)['v'].astype(float)*1000
print('bounds mm',t.min((0,1)),t.max((0,1)))
fig,axs=plt.subplots(1,3,figsize=(18,7))
for ax,(i,j,k,title) in zip(axs,[(0,2,1,'Left view (+Y)'),(1,2,0,'Front view (+X)'),(0,1,2,'Top (+Z)')]):
    order=np.argsort(t[:,:,k].mean(1))
    ax.add_collection(PolyCollection(t[order][:,:,[i,j]],facecolors='#ced6df',edgecolors='#4b6073',linewidths=.13))
    ax.autoscale();ax.set_aspect('equal');ax.set_title(title);ax.set_xlabel('XYZ'[i]+' mm');ax.set_ylabel('XYZ'[j]+' mm');ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(root/'output/head_inspection.png',dpi=170)
np.save(root/'output/head_triangles_mm.npy',t)
