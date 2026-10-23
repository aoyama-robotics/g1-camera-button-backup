"""Offline injection: capture the exact bad sample without altering guards."""
import json,unittest
from diagnostic_capture import Capture

class TestCapture(unittest.TestCase):
    def test_fault_is_frozen_before_terminal_update(self):
        c=Capture(2)
        c.received(dict(host=1.,tick=10,dq=[0.]*29))
        dq=[0.]*29;dq[17]=.101
        sample=dict(host=1.02,tick=30,dq=dq,motor_modes=[1]*29)
        c.received(sample);c.checked(sample)
        c.preparing(dict(q=[.1]*29,kp=[.001]*29,host=1.02))
        c.sent(dict(q=[.1]*29,kp=[0.]*29,host=1.))
        frozen=c.fault('Fast joint motion',1.021)
        dq[17]=0.;c.received(dict(host=1.5,tick=510,dq=[0.]*29));c.checked({})
        decoded=json.loads(json.dumps(frozen))
        self.assertEqual(decoded['checked_state']['dq'][17],.101)
        self.assertEqual(decoded['checked_state']['tick'],30)
        self.assertEqual(decoded['last_successful_write']['kp'][0],0.)
        self.assertEqual(decoded['candidate_not_necessarily_sent']['kp'][0],.001)
        self.assertEqual(len(decoded['received_history']),2)

if __name__=='__main__':unittest.main()
