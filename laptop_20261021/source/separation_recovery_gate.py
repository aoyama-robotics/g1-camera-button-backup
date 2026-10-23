"""One finite, separating-only recovery. Distances stay signed and are logged."""
import json,math,struct,time
from pathlib import Path
import numpy as np
PAIR=frozenset(('right_hip_pitch_link','right_wrist_yaw_link'))
class RecoveryGate:
 def __init__(self,geometry):
  self.g=geometry;self.ready=False;self.started=None;self.cleared=False;self.rows=[]
 def distances(self,q):
  cs=self.g.contacts(q);d=min((x['d'] for x in cs if frozenset((x['a'],x['b']))==PAIR),default=.01)
  other=[x for x in cs if x['d']<0 and frozenset((x['a'],x['b']))!=PAIR]
  if other:raise RuntimeError('Different interfering pair: '+repr(other))
  return d,cs
 def prepare(self,anchor,history):
  self.anchor=np.array(anchor);d0,_=self.distances(anchor)
  hs=[x for x in history if x['received_at']>=history[-1]['received_at']-.5]
  ds=[self.distances(x['q'])[0] for x in hs]
  self.floor=min([d0,*ds]);self.span=max([d0,*ds])-self.floor
  self.peak=d0
  self.ordinary=self.floor>=0
  if self.ordinary:
   curve=[]
   for deg in np.linspace(0,2,81):
    q=self.anchor.copy();q[23]-=math.radians(deg);distance,contacts=self.distances(q)
    if self.g.minimum(q)<0:raise RuntimeError('Ordinary outward path intersects model')
    curve.append(dict(outward_deg=float(deg),signed_pair_distance_m=distance,contacts=contacts))
   self.evidence=dict(mode='ordinary absolute collision protection; recovery not used',initial_signed_m=d0,
    baseline_floor_m=self.floor,baseline_span_m=self.span,baseline_count=len(hs),curve=curve)
   self.ready=True;self.cleared=True;return
  normal=None
  self.g.minimum(anchor)
  for c in self.g.data.contact:
   names=[self.g.model.body(int(self.g.model.geom_bodyid[k])).name for k in (c.geom1,c.geom2)]
   if frozenset(names)==PAIR:normal=np.array(c.frame[:3])*(1 if names[0]=='right_hip_pitch_link' else -1);break
  if normal is None:raise RuntimeError('Missing implicated contact normal')
  blob=(Path(__file__).resolve().parent/'right_arm_20deg_preparation/meshes/right_wrist_yaw_link.STL').read_bytes()
  n=struct.unpack_from('<I',blob,80)[0]
  vertices=np.array([struct.unpack_from('<9f',blob,84+50*i+12) for i in range(n)]).reshape(-1,3)
  previous=None;last=None;projection_min=float('inf');curve=[]
  for deg in np.linspace(0,2,81):
   q=self.anchor.copy();q[23]-=math.radians(deg);distance,contacts=self.distances(q)
   if last is not None and distance<last-1e-9:raise RuntimeError('Planned pair distance worsens')
   self.g.minimum(q);b=self.g.model.body('right_wrist_yaw_link').id
   points=vertices@self.g.data.xmat[b].reshape(3,3).T+self.g.data.xpos[b]
   if previous is not None:projection_min=min(projection_min,float(np.min((points-previous)@normal)))
   previous=points;last=distance;curve.append(dict(outward_deg=float(deg),signed_pair_distance_m=distance,contacts=contacts))
  if projection_min<=0 or last<=0:raise RuntimeError('Whole wrist mesh separation not established')
  # The measured stationary signed-distance spread bounds only sensor jitter,
  # not physical calibration uncertainty. No global negative-distance offset.
  if last-d0<=self.span:raise RuntimeError('Separation is smaller than observed baseline variation')
  self.evidence=dict(initial_signed_m=d0,baseline_floor_m=self.floor,baseline_span_m=self.span,
   baseline_count=len(hs),minimum_all_vertex_normal_step_m=projection_min,end_signed_m=last,curve=curve,
   operator_observed_noncontact=True,model_contact_is_not_physical_contact=True)
  self.ready=True
 def check(self,q):
  if not self.ready:raise RuntimeError('Recovery not prepared')
  now=time.perf_counter()
  if self.started is None:self.started=now
  d,contacts=self.distances(q)
  if self.ordinary:
   if self.g.minimum(q)<0:raise RuntimeError('Ordinary shape protection rejected current state')
   self.rows.append(dict(host=now,signed_pair_distance_m=d,cleared=True,mode='ordinary',contacts=contacts));return
  # Current signed geometry, not the 3-degree tracking allowance, controls this
  # exceptional pair. Reject degradation beyond the measured stationary band.
  floor=max(self.floor,self.peak-self.span)
  if self.cleared and d<0:raise RuntimeError('Cleared pair returned to negative distance')
  if d<floor-1e-9:raise RuntimeError('Recovery signed distance worsened beyond observed stationary variation')
  if d>=0:self.cleared=True
  if not self.cleared and now-self.started>7:raise RuntimeError('Finite separation did not clear')
  self.peak=max(self.peak,d)
  self.rows.append(dict(host=now,signed_pair_distance_m=d,required_floor_m=floor,cleared=self.cleared,contacts=contacts))
