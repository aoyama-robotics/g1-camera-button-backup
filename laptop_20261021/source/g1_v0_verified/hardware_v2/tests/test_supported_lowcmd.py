import unittest,math
from test_runtime import message
from supported_vr_lowcmd import fill,WEAK,WRIST
from arm_runtime import SafetyError
class LowcmdTests(unittest.TestCase):
    def test_exact_vr_hold_and_only_target_changes(self):
        q=[.1]*29;m=fill(message(),q,.1-math.radians(1),5)
        for i in range(29):
            x=m.motor_cmd[i]
            self.assertAlmostEqual(x.q,.1-math.radians(1) if i==22 else .1)
            self.assertEqual((x.mode,x.dq,x.tau),(1,0,0))
            self.assertEqual((x.kp,x.kd),(40,1.5) if i in WRIST else ((80,3) if i in WEAK else (300,3)))
        self.assertEqual(m.motor_cmd[29].q,0)
    def test_bad_machine_nan_or_excess_target_rejected(self):
        for q,target,machine in [([0]*29,0,3),([float('nan')]*29,0,5),([0]*29,.1,5)]:
            with self.assertRaises((SafetyError,ValueError)):fill(message(),q,target,machine)
if __name__=='__main__':unittest.main()
