"""Pure diagnostic recording. No actuator or guard behavior."""
from collections import deque
from copy import deepcopy

class Capture:
    def __init__(self,capacity=1024):
        self.history=deque(maxlen=capacity)
        self.last_check=None;self.candidate=None;self.last_sent=None
    def received(self,value):self.history.append(deepcopy(value))
    def checked(self,value):self.last_check=deepcopy(value)
    def preparing(self,value):self.candidate=deepcopy(value)
    def sent(self,value):self.last_sent=deepcopy(value)
    def fault(self,reason,host):
        return deepcopy(dict(reason=reason,host=host,checked_state=self.last_check,
           candidate_not_necessarily_sent=self.candidate,last_successful_write=self.last_sent,
           received_history=list(self.history)))

def command_dict(msg,host,elapsed):
    return dict(prepared_host=host,elapsed=elapsed,mode_pr=msg.mode_pr,
       mode_machine=msg.mode_machine,crc=msg.crc,
       motors=[dict(index=i,mode=c.mode,q=c.q,dq=c.dq,tau=c.tau,kp=c.kp,kd=c.kd)
               for i,c in enumerate(msg.motor_cmd[:29])])
