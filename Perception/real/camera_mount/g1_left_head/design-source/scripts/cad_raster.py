"""Small orthographic Z-buffer renderer for exact exported CAD triangles.

Avoids painter-order artifacts in flat CAD faces. No geometry simplification.
"""
import numpy as np
from matplotlib.colors import to_rgb

def render(parts,elev=20,azim=40,width=650,height=650,background='#f4f7f9'):
    triangles=np.concatenate([t for t,_ in parts]);base=np.concatenate([np.tile(to_rgb(c),(len(t),1)) for t,c in parts])
    e,a=np.deg2rad([elev,azim]);view=np.array([np.cos(e)*np.cos(a),np.cos(e)*np.sin(a),np.sin(e)])
    right=np.array([-np.sin(a),np.cos(a),0]);up=np.cross(view,right)
    xyz=triangles@np.stack([right,up,view],axis=1)
    low=xyz[:,:,:2].min((0,1));high=xyz[:,:,:2].max((0,1));center=(low+high)/2
    scale=.88*min(width/(high[0]-low[0]),height/(high[1]-low[1]))
    xy=xyz[:,:,:2].copy();xy[:,:,0]=(xy[:,:,0]-center[0])*scale+width/2;xy[:,:,1]=height/2-(xy[:,:,1]-center[1])*scale
    normal=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]);normal/=np.maximum(np.linalg.norm(normal,axis=1)[:,None],1e-14)
    normal*=np.where(normal@view>=0,1,-1)[:,None]
    light=(view+.8*up-.5*right);light/=np.linalg.norm(light)
    shade=.44+.56*np.clip(normal@light,0,1);colors=np.clip(base*shade[:,None]*255,0,255).astype(np.uint8)
    pixels=np.tile((np.array(to_rgb(background))*255).astype(np.uint8),(height,width,1));depth=np.full((height,width),-np.inf)
    for q,z,color in zip(xy,xyz[:,:,2],colors):
        xmin=max(0,int(np.floor(q[:,0].min())));xmax=min(width-1,int(np.ceil(q[:,0].max())))
        ymin=max(0,int(np.floor(q[:,1].min())));ymax=min(height-1,int(np.ceil(q[:,1].max())))
        if xmax<xmin or ymax<ymin:continue
        den=(q[1,1]-q[2,1])*(q[0,0]-q[2,0])+(q[2,0]-q[1,0])*(q[0,1]-q[2,1])
        if abs(den)<1e-10:continue
        xx,yy=np.meshgrid(np.arange(xmin,xmax+1)+.5,np.arange(ymin,ymax+1)+.5)
        aa=((q[1,1]-q[2,1])*(xx-q[2,0])+(q[2,0]-q[1,0])*(yy-q[2,1]))/den
        bb=((q[2,1]-q[0,1])*(xx-q[2,0])+(q[0,0]-q[2,0])*(yy-q[2,1]))/den;cc=1-aa-bb
        zz=aa*z[0]+bb*z[1]+cc*z[2];patch=depth[ymin:ymax+1,xmin:xmax+1]
        mask=(aa>=-1e-8)&(bb>=-1e-8)&(cc>=-1e-8)&(zz>patch)
        patch[mask]=zz[mask];pixels[ymin:ymax+1,xmin:xmax+1][mask]=color
    return pixels
