"""Independent beam-scale check of units and the elasticity assembly path."""
from pathlib import Path
import numpy as np,json
from skfem import MeshTet,Basis,FacetBasis,ElementVector,ElementTetP2,LinearForm,asm
from skfem.models.elasticity import linear_elasticity,lame_parameters
from scipy.sparse.linalg import spsolve
ROOT=Path(__file__).resolve().parents[1]
E=2000.;nu=.35;length=20.;width=2.;height=2.;force=1.
m=MeshTet.init_tensor(np.linspace(0,length,21),np.linspace(-1,1,4),np.linspace(-1,1,4))
e=ElementVector(ElementTetP2());b=Basis(m,e,intorder=4)
fb=FacetBasis(m,e,facets=m.facets_satisfying(lambda x:np.isclose(x[0],length)),intorder=4)
@LinearForm
def end_load(v,w):return -v[2]*force/(width*height)
f=asm(end_load,fb);lam,mu=lame_parameters(E,nu);K=asm(linear_elasticity(lam,mu),b).tocsr()
fixed=b.get_dofs(lambda x:np.isclose(x[0],0)).all();free=np.setdiff1d(np.arange(b.N),fixed)
u=np.zeros(b.N);u[free]=spsolve(K[free][:,free],f[free]);tip=abs(float(f@u)/force)
beam=force*length**3/(3*E*(width*height**3/12));relative=abs(tip-beam)/beam
assert abs(f[b.split_indices()[2]].sum()+force)<1e-9
assert relative<.04
result={'test':'20 x 2 x 2 mm clamped beam, uniformly loaded tip, F=1N, E=2000MPa','Euler_Bernoulli_tip_mm':beam,'FEA_work_average_tip_mm':tip,'relative_difference':relative,'passed':True,'note':'Small shear/3D support differences allowed; checks unit scale and known-solution accuracy, not robot physical validation.'}
(ROOT/'output/v2/beam_check.json').write_text(json.dumps(result,indent=2));print(result)
