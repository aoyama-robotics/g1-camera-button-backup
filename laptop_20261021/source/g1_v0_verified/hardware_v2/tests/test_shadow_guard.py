import unittest
from shadow_motion_guard import ShadowGuard,ShadowFault

class ShadowTests(unittest.TestCase):
    def sample(self,g,t=0,q=None,dq=None,**kwargs):
        return g.observe(t,t/1000,q or [0.]*29,dq or [0.]*29,[0.]*3,[0.]*3,**kwargs)
    def test_three_ms_dq_spike_without_motion(self):
        g=ShadowGuard()
        for t in range(50):
            dq=[0.]*29;dq[2]=.096 if 10<=t<13 else 0
            self.sample(g,t,dq=dq)
    def test_persistent_dq_stops_by_twenty_ms(self):
        g=ShadowGuard();dq=[.06]+[0.]*28
        for t in range(20):self.sample(g,t,dq=dq)
        with self.assertRaisesRegex(ShadowFault,'persistent_dq'):self.sample(g,20,dq=dq)
    def test_abrupt_position_stops_without_dq(self):
        g=ShadowGuard();self.sample(g)
        q=[.00101]+[0.]*28
        with self.assertRaisesRegex(ShadowFault,'position_envelope'):self.sample(g,1,q=q)
    def test_stale_nan_mode_immediate(self):
        for kwargs in ({'age':.201},{'mode':0},{'age':float('nan')}):
            with self.subTest(kwargs=kwargs),self.assertRaises(ShadowFault):self.sample(ShadowGuard(),**kwargs)
    def test_imu_immediate(self):
        with self.assertRaisesRegex(ShadowFault,'imu'):
            ShadowGuard().observe(0,0,[0.]*29,[0.]*29,[.11,0,0],[0.]*3)
    def test_irregular_time_and_tick_gap_fail_closed(self):
        for tick,host in [(51,.051),(1,.051),(0,.051)]:
            g=ShadowGuard();self.sample(g)
            with self.subTest(tick=tick),self.assertRaises(ShadowFault):
                g.observe(tick,host,[0.]*29,[0.]*29,[0.]*3,[0.]*3)
    def test_duplicate_ticks_do_not_confirm_persistence(self):
        g=ShadowGuard();dq=[.06]+[0.]*28
        self.sample(g,dq=dq)
        for _ in range(10):self.sample(g,dq=dq)
        self.assertEqual(g.pending[0],0)
    def test_steady_motion_position_channel_independent(self):
        g=ShadowGuard()
        with self.assertRaises(ShadowFault):
            for t in range(30):
                q=[t*.00006]+[0.]*28;self.sample(g,t,q=q)

if __name__=='__main__':unittest.main()
