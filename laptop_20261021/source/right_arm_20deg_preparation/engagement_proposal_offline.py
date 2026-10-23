"""Offline proposal only. No DDS, SDK, sockets, or actuator operations.

Not a physical executor and not a certified total-torque bound.
"""
import math

WEAK = {4,10,15,16,17,18,22,23,24,25}
WRIST = {19,20,21,26,27,28}
TOTAL_SECONDS = 3.0
MAX_SCALE = 0.10

def smooth(x):
    x = min(1.0, max(0.0, x))
    return x*x*(3.0-2.0*x)

def scale(t):
    if not math.isfinite(t) or t < 0 or t > TOTAL_SECONDS:
        raise ValueError('outside finite proposal interval')
    if t <= 1.5:
        return MAX_SCALE*smooth(t/1.5)
    if t <= 2.0:
        return MAX_SCALE
    return MAX_SCALE*(1.0-smooth(t-2.0))

def gains(index, t):
    if not 0 <= index < 29:
        raise ValueError('body joint index')
    kp,kd = (40.,1.5) if index in WRIST else ((80.,3.) if index in WEAK else (300.,3.))
    s=scale(t)
    return kp*s,kd*s

if __name__ == '__main__':
    assert scale(0)==0 and scale(3)==0
    assert abs(scale(1.5)-.1)<1e-12
    assert gains(0,1.5)==(30.,.30000000000000004)
    print('OFFLINE ONLY: 1.5s ramp, 0.5s hold, 1s ramp down; no target motion; max 10% VR gains')
