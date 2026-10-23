"""Explicit offline/configurable local effective bias; not full camera extrinsics.

Input/output are torso metres. The correction combines camera/tool/correspondence
bias near a demonstrated contact. It is disabled by default and has no robot APIs.
No stopping conditions or control gains are added or changed.
"""
import json
from pathlib import Path
import numpy as np

class LocalCameraCorrection:
 def __init__(self,config):
  self.config=json.loads(Path(config).read_text(encoding='utf-8'))
  self.enabled=bool(self.config.get('enabled',False))
  self.delta=np.asarray(self.config['translation_torso_m'],dtype=float)
  if self.delta.shape!=(3,) or not np.all(np.isfinite(self.delta)):
   raise ValueError('Invalid correction data')
 def correct_torso(self,nominal_torso_m):
  p=np.asarray(nominal_torso_m,dtype=float)
  return p+self.delta if self.enabled else p.copy()
 def correct_pelvis(self,nominal_pelvis_m,torso_rotation_in_pelvis):
  """Rotate a torso-fixed bias into pelvis; never add fixed pelvis offset."""
  p=np.asarray(nominal_pelvis_m,dtype=float)
  return p+np.asarray(torso_rotation_in_pelvis)@self.delta if self.enabled else p.copy()
