"""STEP-boundary quadratic tetrahedral linear elasticity.

Adapted from SANDRONE step_tetra_precision.py: actual tetra meshing, P2 vector
elasticity, exact force/moment mapping, residual/reaction and mesh-volume checks.
Units: mm, N, MPa. Bonded joint and ideal clamped seat model, not certification.
"""
from pathlib import Path
import argparse,json,time,hashlib,os,sys,traceback
import gmsh
import numpy as np
from scipy.sparse.linalg import splu
from scipy.sparse.csgraph import connected_components
from skfem import MeshTet,Basis,FacetBasis,ElementTetP2,ElementVector,LinearForm,asm
from skfem.models.elasticity import linear_elasticity,lame_parameters
from skfem.helpers import dot
ROOT=Path(__file__).resolve().parents[1]

def write(p,data):
    p.write_text(json.dumps(data,indent=2,allow_nan=False,default=lambda x:x.item() if isinstance(x,np.generic) else x.tolist()),encoding='utf-8')

def weighted_percentile(v,w,q):
    order=np.argsort(v);a=v[order];b=np.cumsum(w[order]);return float(np.interp(q*b[-1],b,a))

def solve(candidate,h,version='v2'):
    start=time.perf_counter();cfg=json.loads((ROOT/f'config/structural_{version}.json').read_text())
    folder=ROOT/f'output/{version}/candidates'/candidate;geo=json.loads((folder/'geometry.json').read_text());p=geo['parameters']
    out=ROOT/f'output/{version}/fea'/f'{candidate}_h{h:g}';out.mkdir(parents=True,exist_ok=True)
    step=ROOT/geo['analysis_step']['path']
    print('MESH',candidate,h,flush=True)
    gmsh.initialize()
    try:
        gmsh.option.setNumber('General.Terminal',0);gmsh.option.setNumber('General.NumThreads',1)
        gmsh.model.add(candidate);gmsh.model.occ.importShapes(str(step));gmsh.model.occ.synchronize()
        assert len(gmsh.model.getEntities(3))==1
        gmsh.option.setNumber('Mesh.MeshSizeMin',.5);gmsh.option.setNumber('Mesh.MeshSizeMax',h)
        gmsh.option.setNumber('Mesh.MeshSizeFromPoints',0);gmsh.option.setNumber('Mesh.MeshSizeFromCurvature',10)
        gmsh.option.setNumber('Mesh.MeshSizeExtendFromBoundary',0)
        gmsh.option.setNumber('Mesh.Algorithm3D',1)
        # Fixed refinement around supports across all candidates and global levels.
        boxes=[]
        for x in [-26.5,26.5]:
            tag=gmsh.model.mesh.field.add('Box');boxes.append(tag)
            for key,value in dict(VIn=1.2,VOut=h,XMin=x-7,XMax=x+7,YMin=-7,YMax=7,ZMin=-6,ZMax=1,Thickness=2).items():
                gmsh.model.mesh.field.setNumber(tag,key,value)
        field=gmsh.model.mesh.field.add('Min');gmsh.model.mesh.field.setNumbers(field,'FieldsList',boxes);gmsh.model.mesh.field.setAsBackgroundMesh(field)
        gmsh.model.mesh.generate(3);gmsh.model.mesh.optimize('Netgen')
        gmsh.write(str(out/'mesh.msh'))
        tags,xyz,_=gmsh.model.mesh.getNodes();_,node_tags=gmsh.model.mesh.getElementsByType(4)
        order=np.argsort(tags);tets=order[np.searchsorted(tags[order],node_tags)].reshape(-1,4)
        pts=xyz.reshape(-1,3)
    finally:gmsh.finalize()
    mesh=MeshTet(pts.T,tets.T);elem=ElementVector(ElementTetP2());basis=Basis(mesh,elem,intorder=4)
    print('ASSEMBLE',candidate,'tet',mesh.nelements,'dofs',basis.N,flush=True)
    assert basis.N<280000,'DOF limit reached; explicitly revise mesh rather than exhausting host memory'
    comp=basis.split_indices();coords=basis.doflocs[:,comp[0]].T
    boundary=basis.get_dofs(mesh.boundary_facets()).all();onboundary=np.isin(comp[0],boundary)
    radius=cfg['support_patch_radius_mm']
    fixmask=(np.abs(coords[:,2])<1e-6)&onboundary&(((coords[:,0]-26.5)**2+coords[:,1]**2<radius**2)|((coords[:,0]+26.5)**2+coords[:,1]**2<radius**2))
    fixed_nodes=np.flatnonzero(fixmask);assert len(fixed_nodes)>30
    fixed=np.concatenate([c[fixed_nodes] for c in comp]);free=np.setdiff1d(np.arange(basis.N),fixed)
    facets=mesh.boundary_facets();centers=mesh.p[:,mesh.facets[:,facets]].mean(axis=1).T
    maxy=60+p['wall'];shift=p['shift']
    loadfacets=facets[(centers[:,0]>shift+10)&(centers[:,0]<shift+33)&(centers[:,1]>maxy-4.5)&(np.abs(centers[:,2]-30)<11)]
    assert len(loadfacets)>10
    fb=FacetBasis(mesh,elem,facets=loadfacets,intorder=4)
    load_nodes=np.flatnonzero(np.isin(comp[0],fb.get_dofs(loadfacets).all()))
    ref=np.array(cfg['payload_reference_local_mm'])+np.array([shift,p['wall']-6,0])
    # Six distributed traction fields, mixed to reproduce the remote COM wrench.
    Q=[]
    for j in range(6):
        @LinearForm
        def traction(v,w):
            if j<3:return v[j]
            r=(w.x-ref[:,None,None])/30.0
            a=np.eye(3)[j-3]
            vec=np.stack([a[1]*r[2]-a[2]*r[1],a[2]*r[0]-a[0]*r[2],a[0]*r[1]-a[1]*r[0]])
            return dot(vec,v)
        Q.append(asm(traction,fb))
    Q=np.column_stack(Q)
    def wrench(f):
        nod=np.column_stack([f[c] for c in comp])
        return np.r_[nod.sum(0),np.cross(coords-ref,nod).sum(0)/30]
    W=np.column_stack([wrench(Q[:,j]) for j in range(6)])
    assert np.linalg.matrix_rank(W)==6
    body=[]
    rho=cfg['density_kg_m3']/1e9
    for j in range(3):
        @LinearForm
        def mass(v,w):return rho*v[j]
        body.append(asm(mass,basis))
    body=np.column_stack(body)
    F=[];cases=[]
    for name,direction in cfg['load_cases'].items():
        direction=np.array(direction);force=direction*cfg['user_accepted_payload_kg']*cfg['acceleration_m_s2']
        target=np.r_[force,[0,0,0]];payload=Q@np.linalg.solve(W,target)
        error=np.linalg.norm(wrench(payload)-target);assert error<1e-8
        F.append(payload+body@(direction*cfg['acceleration_m_s2']))
        cases.append(dict(id=name,payload_force_N=force.tolist(),remote_COM_mm=ref.tolist(),wrench_error_scaled=error))
    F=np.column_stack(F)
    lam,mu=lame_parameters(cfg['elastic_modulus_MPa'],cfg['poisson_ratio'])
    K=asm(linear_elasticity(lam,mu),basis).tocsc()
    assert connected_components(K,directed=False,return_labels=False)==1
    print('SOLVE',candidate,flush=True)
    factor=splu(K[free][:,free]);U=np.zeros_like(F);U[free]=factor.solve(F[free]);residual=K@U-F
    rel=float(np.linalg.norm(residual[free])/np.linalg.norm(F[free]));assert rel<1e-7
    # Differential rigid fit at the loaded rail patch estimates lens/COM translation
    # and rotation. Does not create artificial rigid constraints in the FE solve.
    rr=coords[load_nodes]-ref;B=np.zeros((3*len(rr),6))
    for k,(x,y,z) in enumerate(rr):
        B[3*k:3*k+3,:3]=np.eye(3);B[3*k:3*k+3,3:]=[[0,z,-y],[-z,0,x],[y,-x,0]]
    volume_weights=basis.dx.sum(axis=1);vm_maps=[]
    for i,row in enumerate(cases):
        du=np.column_stack([U[c,i] for c in comp]);fit=np.linalg.lstsq(B,du[load_nodes].reshape(-1),rcond=None)[0]
        reactions=residual[:,i].copy();reactions[free]=0;total=reactions+F[:,i]
        wn=wrench(total);assert np.linalg.norm(wn[:3])<1e-5 and np.linalg.norm(wn[3:]*30)<1e-4
        grad=basis.interpolate(U[:,i]).grad;strain=(grad+grad.swapaxes(0,1))*.5
        stress=2*mu*strain;trace=np.einsum('iieq->eq',strain)
        for j in range(3):stress[j,j]+=lam*trace
        a,b,c=[stress[j,j] for j in range(3)]
        vm=np.sqrt(.5*((a-b)**2+(b-c)**2+(c-a)**2)+3*(stress[0,1]**2+stress[1,2]**2+stress[0,2]**2))
        vm_elem=(vm*basis.dx).sum(1)/volume_weights;vm_maps.append(vm_elem)
        peak_index=np.unravel_index(np.argmax(vm),vm.shape)
        peak_location=np.asarray(basis.global_coordinates())[:,peak_index[0],peak_index[1]].tolist()
        row.update(max_displacement_mm=float(np.linalg.norm(du,axis=1).max()),COM_displacement_mm=float(np.linalg.norm(fit[:3])),
                   rail_rotation_deg=float(np.linalg.norm(fit[3:])*180/np.pi),compliance_Nmm=float(F[:,i]@U[:,i]),
                   peak_vm_MPa=float(vm.max()),peak_vm_location_mm=peak_location,volume_weighted_p99_vm_MPa=weighted_percentile(vm_elem,volume_weights,.99),
                   reaction_force_error_N=float(np.linalg.norm(wn[:3])),reaction_moment_error_Nmm=float(np.linalg.norm(wn[3:]*30)))
    result=dict(candidate=candidate,mesh_target_mm=h,tetrahedra=mesh.nelements,dofs=basis.N,fixed_nodes=len(fixed_nodes),load_facets=len(loadfacets),
                element='P2 displacement / linear boundary tetra geometry',material=cfg['elastic_modulus_MPa'],relative_residual=rel,
                volume_mm3=float(volume_weights.sum()),CAD_volume_mm3=geo['total_volume_mm3'],
                volume_error_fraction=float(abs(volume_weights.sum()-geo['total_volume_mm3'])/geo['total_volume_mm3']),
                support_radius_mm=radius,cases=cases,worst_displacement_mm=max(r['max_displacement_mm'] for r in cases),
                worst_COM_displacement_mm=max(r['COM_displacement_mm'] for r in cases),worst_p99_vm_MPa=max(r['volume_weighted_p99_vm_MPa'] for r in cases),
                seconds=time.perf_counter()-start,step_sha256=hashlib.sha256(step.read_bytes()).hexdigest(),physical_ready=False)
    assert result['volume_error_fraction']<.01
    np.savez_compressed(out/'fields.npz',points_mm=mesh.p.T,tets=mesh.t.T,surface_faces=mesh.facets[:,mesh.boundary_facets()].T,
                        vertex_displacements_mm=np.stack([U[basis.nodal_dofs,i].T for i in range(len(cases))]),
                        element_vm_MPa=np.array(vm_maps),fixed_nodes_mm=coords[fixed_nodes],load_nodes_mm=coords[load_nodes])
    write(out/'result.json',result)
    print('RESULT',candidate,h,'max_u_mm',round(result['worst_displacement_mm'],5),'p99_MPa',round(result['worst_p99_vm_MPa'],3),'seconds',round(result['seconds'],1),flush=True)
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('candidate');parser.add_argument('--h',type=float,default=5.0);parser.add_argument('--version',choices=['v2','v3'],default='v2');args=parser.parse_args()
    code=0
    try:solve(args.candidate,args.h,args.version)
    except BaseException:traceback.print_exc();code=1
    sys.stdout.flush();sys.stderr.flush();os._exit(code)
