import unittest
from test_runtime import Clock,Transport,PROFILE
from arm_runtime import ArmSession,SafetyError,DT

class TimingTests(unittest.TestCase):
    def test_packet_processing_is_included_in_period(self):
        c=Clock();t=Transport(c);base=t.write
        def write(*args):base(*args);c.t+=.001
        t.write=write
        s=ArmSession(t,PROFILE,clock=c,sleep=c.sleep)
        s.start=s.last=(0.,)*14;s.next_tick=0
        s.tick(s.last,0);s.tick(s.last,0)
        self.assertAlmostEqual(t.timepoints[1]-t.timepoints[0],DT)

    def test_slow_processing_starts_full_new_period_no_burst(self):
        c=Clock();t=Transport(c);base=t.write
        def write(*args):base(*args);c.t+=.006
        t.write=write
        s=ArmSession(t,PROFILE,clock=c,sleep=c.sleep)
        s.start=s.last=(0.,)*14;s.next_tick=0
        s.tick(s.last,0);s.tick(s.last,0)
        self.assertGreaterEqual(t.timepoints[1]-t.timepoints[0],.010-1e-9)

    def test_deadline_still_rejects_before_write(self):
        c=Clock();t=Transport(c);s=ArmSession(t,PROFILE,clock=c,sleep=c.sleep)
        s.start=s.last=(0.,)*14;s.next_tick=0;c.t=.050001
        with self.assertRaises(SafetyError):s.tick(s.last,0)
        self.assertEqual(t.writes,[])

if __name__=='__main__':unittest.main()
