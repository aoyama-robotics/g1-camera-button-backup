from pathlib import Path
import json,zipfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'output/v1'
def mesh(file):
    data=file.read_bytes();dt=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')])
    return np.frombuffer(data,dtype=dt,offset=84)['v'].astype(float)
def draw(ax,t,color):
    n=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]);n/=np.maximum(np.linalg.norm(n,axis=1)[:,None],1e-12)
    from matplotlib.colors import to_rgb
    shading=.6+.4*np.maximum(0,n@np.array([.3,-.4,.866]))
    ax.add_collection3d(Poly3DCollection(t,facecolors=shading[:,None]*np.array(to_rgb(color)),edgecolors='none'))
    ax.set(xlim=(-40,40),ylim=(-18,71),zlim=(-15,67));ax.set_box_aspect((80,89,82));ax.view_init(elev=24,azim=-55);ax.set_axis_off()
bracket=mesh(OUT/'L_bracket_53mm_20mm_FIT_PROTOTYPE.stl');carrier=mesh(OUT/'tri_rail_carrier_v1.stl')
fig=plt.figure(figsize=(15,7),facecolor='#f6f8fa')
ax=fig.add_subplot(131,projection='3d');ax.set_facecolor('#f6f8fa');draw(ax,bracket,'#dda045');draw(ax,carrier,'#258f92');ax.set_title('ASSEMBLY | underside L mount')
ax=fig.add_subplot(132,projection='3d');ax.set_facecolor('#f6f8fa');draw(ax,bracket,'#dda045');ax.set_title('L BRACKET | two upward screws')
for x in [-26.5,26.5]:ax.quiver(x,0,-15,0,0,9,color='#ad3325',arrow_length_ratio=.3,linewidth=2)
ax=fig.add_subplot(133)
ax.add_patch(Polygon([(-9,-5),(26,-5),(26,55),(20,55),(20,0),(-9,0)],facecolor='#e7b05a',edgecolor='#81501f'))
ax.plot([0,0],[-17,9],'--',color='#ad3325');ax.annotate('',xy=(0,-5),xytext=(0,-16),arrowprops=dict(arrowstyle='->',color='#ad3325',lw=2))
ax.annotate('',xy=(0,18),xytext=(20,18),arrowprops=dict(arrowstyle='<->'));ax.text(10,21,'20 mm',ha='center',fontsize=13)
ax.plot([-9,20],[0,0],color='#355260',lw=2);ax.text(-8,3,'Contact plane',fontsize=10)
ax.text(-8,-22,'5 mm flange\n2 x assumed M4 clearance, dia. 4.5\n53 mm pitch along X',fontsize=10,linespacing=1.6)
ax.set(xlim=(-13,45),ylim=(-27,63),xlabel='Y: outward (mm)',ylabel='Z: up (mm)',title='SECTION | hole axis to INNER face');ax.set_aspect('equal');ax.grid(alpha=.16)
fig.text(.05,.045,'v1 FIT PROTOTYPE — 53 mm user measurement; 20 mm proposed reach. Seats, thread and load rating remain unverified.',fontsize=11,color='#864423')
fig.subplots_adjust(left=.01,right=.97,bottom=.14,top=.9,wspace=.15);fig.savefig(OUT/'L_mount_preview.png',dpi=155);plt.close(fig)
print('Rendered v1 actual CAD and dimension section.')
