"""Conservative nominal-mesh checks; no communication or actuation."""
import math
from pathlib import Path
import numpy as np
import mujoco
from check_sweep import NAMES

class Geometry:
    def __init__(self):
        self.model=mujoco.MjModel.from_xml_path(str(Path(__file__).with_name('collision_only.urdf')))
        self.data=mujoco.MjData(self.model)
        self.address=[int(self.model.joint(n).qposadr[0]) for n in NAMES]
        self.model.geom_margin[:]=.005
    def contacts(self,q):
        self.data.qpos[self.address]=q;mujoco.mj_forward(self.model,self.data)
        result=[]
        for c in self.data.contact:
            a=mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_BODY,int(self.model.geom_bodyid[c.geom1])) or ''
            b=mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_BODY,int(self.model.geom_bodyid[c.geom2])) or ''
            moving=lambda x:x.startswith(('right_shoulder','right_elbow','right_wrist'))
            if not (moving(a) or moving(b)):continue
            # This known positive internal gap is invariant for shoulder-only paths.
            # Never exclude penetration, and actual internal joints remain monitored.
            if set((a,b))==set(('right_elbow_link','right_wrist_pitch_link')) and c.dist>0:continue
            result.append(dict(a=a,b=b,d=float(c.dist)))
        return result
    def minimum(self,q):
        return min((c['d'] for c in self.contacts(q)),default=.010)
    def plan(self,q):
        q=np.array(q,dtype=float);candidates=[]
        for opening in (3.,5.,10.):
            roll_min=.010
            for f in np.linspace(0,1,81):
                a=q.copy();a[23]-=math.radians(opening)*f
                roll_min=min(roll_min,self.minimum(a))
            # Include one degree roll tracking lag toward body, and +/-3 pitch.
            pitch_min=.010
            for pitch in np.linspace(3,-23,105):
                a=q.copy();a[23]-=math.radians(opening-1);a[22]+=math.radians(pitch)
                pitch_min=min(pitch_min,self.minimum(a))
            candidates.append(dict(opening_deg=opening,roll_min_m=roll_min,pitch_envelope_min_m=pitch_min))
            if roll_min>=.0025 and pitch_min>=.010:
                return dict(opening_deg=opening,candidates=candidates,
                    start_contacts=self.contacts(q),assumptions='nominal conservative collision shapes; non-target drift <=0.002rad; actual mesh distance monitored >=2mm; operator reports no physical contact')
        raise RuntimeError('No outward departure/raise path satisfies the bounded geometry plan: '+str(candidates))
