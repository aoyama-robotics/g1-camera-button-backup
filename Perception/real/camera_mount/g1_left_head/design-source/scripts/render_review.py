"""Render actual exported meshes, never an AI-generated representation of CAD."""
from pathlib import Path
import json, hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.collections import PolyCollection

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'output/v0'
plt.rcParams['font.family']='DejaVu Sans'
def read_stl(path,scale=1):
    b=path.read_bytes();dt=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')])
    return np.frombuffer(b,dtype=dt,offset=84)['v'].astype(float)*scale

def add_mesh(ax,t,color,alpha=1):
    n=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]); n/=np.maximum(np.linalg.norm(n,axis=1)[:,None],1e-12)
    light=np.array([.3,.6,.75]);light/=np.linalg.norm(light)
    shade=.5+.5*np.clip(n@light,0,1)
    from matplotlib.colors import to_rgb
    colors=shade[:,None]*np.array(to_rgb(color))[None,:]
    ax.add_collection3d(Poly3DCollection(t,facecolors=colors,edgecolor='none',alpha=alpha,zsort='average'))

head=read_stl(ROOT/'sources/head_link.STL',1000)
carrier=read_stl(OUT/'tri_rail_carrier_CONCEPT.stl')
adapter=read_stl(OUT/'head_adapter_BLANK_NOT_FIT.stl')
cfg=json.loads((ROOT/'config/mount_v0.json').read_text());shift=np.array(cfg['preview']['translation_in_head_mesh_mm'])
fig=plt.figure(figsize=(14,8),facecolor='#f5f7fa')
ax=fig.add_subplot(121,projection='3d');ax.set_facecolor('#f5f7fa')
add_mesh(ax,head,'#b4c0cb');add_mesh(ax,carrier+shift,'#278f92');add_mesh(ax,adapter+shift,'#e2a047')
ax.set(xlim=(-90,105),ylim=(-90,115),zlim=(325,535));ax.set_box_aspect((195,205,210));ax.view_init(elev=18,azim=42);ax.set_axis_off()
ax.set_title('G1 LEFT SIDE | placement study',fontsize=14,pad=3)
ax=fig.add_subplot(122,projection='3d');ax.set_facecolor('#f5f7fa')
add_mesh(ax,carrier,'#278f92');add_mesh(ax,adapter,'#e2a047')
ax.set(xlim=(-48,48),ylim=(-8,70),zlim=(-38,38));ax.set_box_aspect((96,78,76));ax.view_init(elev=24,azim=38);ax.set_axis_off()
ax.set_title('70 mm rails | top / outboard / bottom',fontsize=14,pad=3)
fig.text(.055,.085,'TEAL: three-rail carrier     AMBER: replaceable blank adapter',fontsize=12,color='#204551')
fig.text(.055,.05,'CONCEPT v0 — Robot holes, contact seats, payload and sensor clearance are NOT verified.',fontsize=11,color='#9b451f')
fig.subplots_adjust(top=.94,bottom=.13,left=0,right=1,wspace=.02)
fig.savefig(OUT/'cad_preview.png',dpi=160);plt.close(fig)

fig,axes=plt.subplots(1,2,figsize=(13,7),facecolor='white')
for ax,(i,j,k,title) in zip(axes,[(0,2,1,'OUTBOARD VIEW (X / Z)'),(1,2,0,'END VIEW (Y / Z)')]):
    all_t=np.concatenate([carrier,adapter]);color=np.array(['#5ea8a8']*len(carrier)+['#e4b56e']*len(adapter))
    idx=np.argsort(all_t[:,:,k].mean(1));ax.add_collection(PolyCollection(all_t[idx][:,:,[i,j]],facecolors=color[idx],edgecolors='#34505c',linewidths=.2))
    ax.autoscale();ax.set_aspect('equal');ax.set_title(title);ax.set_xlabel('XYZ'[i]+' (mm)');ax.set_ylabel('XYZ'[j]+' (mm)');ax.grid(alpha=.2)
axes[0].annotate('',xy=(-45,38),xytext=(45,38),arrowprops=dict(arrowstyle='<->'));axes[0].text(0,40,'90 mm adapter',ha='center')
axes[0].set_ylim(-38,47)
fig.suptitle('G1 LEFT TRI-RAIL | CONCEPT DIMENSIONS, NOT A ROBOT DRILLING TEMPLATE',fontsize=13)
fig.text(.06,.025,'Adapter-to-carrier fastening: four M3 holes (hidden in these views). Robot mounting holes are absent.',fontsize=11)
fig.tight_layout(rect=(0,.065,1,.95));fig.savefig(OUT/'dimension_review.png',dpi=150);plt.close(fig)

# Mesh envelope separation: conservative and limited to this chosen illustrative pose.
m=(head[:,:,0].max(1)>=-45)&(head[:,:,0].min(1)<=45)&(head[:,:,2].max(1)>=463)&(head[:,:,2].min(1)<=527)
local_max_y=float(head[m,:,1].max())
evidence={'method':'Axis-aligned separation of all source triangles overlapping accessory X/Z bounding interval',
          'illustrative_pose_only':True,'local_source_max_y_mm':local_max_y,'accessory_min_y_mm':float(shift[1]),
          'minimum_Y_envelope_gap_mm':float(shift[1]-local_max_y),'contact_surface_fit_verified':False,
          'sensor_FOV_tested':False,'arm_swept_volume_tested':False}
(OUT/'preview_clearance.json').write_text(json.dumps(evidence,indent=2))

# Traceability manifest: source URLs, repository snapshots and actual bytes used.
ros=json.loads((ROOT/'sources/ros_commit.json').read_text())['sha'];xr=json.loads((ROOT/'sources/xr_tree.json').read_text())['sha']
urls={'head_link.STL':f'https://raw.githubusercontent.com/unitreerobotics/unitree_ros/{ros}/robots/g1_description/meshes/head_link.STL',
      'g1_29dof.urdf':f'https://raw.githubusercontent.com/unitreerobotics/unitree_ros/{ros}/robots/g1_description/g1_29dof.urdf',
      'Device.md':'https://github.com/unitreerobotics/xr_teleoperate/blob/main/Device.md',
      'classic_head_parts.zip':'https://oss-global-cdn.unitree.com/static/477103c571dc46f99ec6e0b57b3b3be6.zip',
      'renewed_head_parts.zip':'https://oss-global-cdn.unitree.com/static/950f53b95d5943589e278241b59c86ff.zip',
      'MIL_STD_1913.pdf':'https://combatrifle.net/pdfs/picatinny_1913_specs.pdf'}
sources=[]
for filename,url in urls.items():
    p=ROOT/'sources'/filename
    sources.append({'file':str(p.relative_to(ROOT)),'url':url,'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size})
v=head.reshape(-1,3)
report={'retrieved':'2026-09-26','ros_commit':ros,'xr_tree':xr,'sources':sources,
        'head_mesh_measurement':{'unit_conversion':'URDF metres to CAD mm: multiply by 1000','bounds_mm':[v.min(0).tolist(),v.max(0).tolist()],
                                 'extent_mm':(v.max(0)-v.min(0)).tolist(),'scope':'entire supplied head_link including neck skirt; NOT crown-only dimensions'},
        'official_camera_bracket_holes':{'diameter_mm':4.5,'classic_transverse_pitch_mm':98.0,'renewed_transverse_pitch_mm':88.0,
              'axis_in_STEP':[-.040057173190701366,0,.9991973893460542],
              'warning':'Left/right camera bracket pair; NOT two holes on the left head side. STEP and head STL coordinate systems not registered.'}}
(ROOT/'sources/provenance.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(evidence,indent=2));print('Rendered CAD preview and dimension review; wrote source provenance.')
