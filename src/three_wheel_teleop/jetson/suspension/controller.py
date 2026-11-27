"""Offline controller core. No serial I/O or hardware activation.

Units: seconds, degrees (nose up positive), millimetres (extension positive).
"""
from dataclasses import dataclass
import math

@dataclass
class Controller:
    low: float = -41.8
    high: float = 28.2
    max_speed: float = 20.0
    acceleration: float = 40.0
    kp: float = 20.0
    kd: float = 3.0
    fast_threshold_deg: float = 0.8
    fast_max_speed: float = 20.0
    deadband: float = 0.2
    speed: float = 0.0

    def update(self, pitch, pitch_rate, position, dt, sensor_age, reference_valid):
        values=(pitch,pitch_rate,position,dt,sensor_age)
        if not all(math.isfinite(v) for v in values):
            self.speed=0; raise ValueError('Nonfinite input')
        if not reference_valid or not 0<dt<=0.1 or not 0<=sensor_age<=0.1:
            self.speed=0; raise ValueError('Reference or timing fault')
        if not self.low<=position<=self.high or abs(pitch)>7:
            self.speed=0; raise ValueError('Commissioning envelope exceeded')
        fast_spd = max(5.0, float(self.fast_max_speed))
        fast = abs(pitch) > self.fast_threshold_deg
        if fast:
            spd_lim = fast_spd
            kp_use = min(35.0, max(self.kp, spd_lim * 0.7))
            kd_use = min(4.2, max(self.kd, spd_lim * 0.07))
            accel_up = max(self.acceleration, spd_lim * 5.0)
        else:
            spd_lim = min(self.max_speed, fast_spd)
            kp_use = self.kp
            kd_use = self.kd
            accel_up = self.acceleration
        brake_accel = max(250.0, fast_spd * 6.0)
        demand=0 if abs(pitch)<self.deadband and abs(pitch_rate)<0.2 else -kp_use*pitch-kd_use*pitch_rate
        demand=max(-spd_lim,min(spd_lim,demand))
        if demand==0 or demand*self.speed<0 or abs(demand)<abs(self.speed):
            slew_accel = brake_accel
        else:
            slew_accel = accel_up
        self.speed+=max(-slew_accel*dt,min(slew_accel*dt,demand-self.speed))
        margin=self.speed*self.speed/(2*brake_accel)+abs(self.speed)*dt
        if (self.speed>0 and position+margin>=self.high) or (self.speed<0 and position-margin<=self.low):
            self.speed=0
        return self.speed

if __name__=='__main__':
    c=Controller(fast_max_speed=50.0)
    assert c.update(1,0,-21,0.02,0,True)<0
    c=Controller(fast_max_speed=50.0)
    assert c.update(-1,0,-21,0.02,0,True)>0
    c=Controller(fast_max_speed=50.0)
    assert c.update(-1,0,28.2,0.02,0,True)==0
    assert c.update(1,0,-41.8,0.02,0,True)==0
    c_slow=Controller(fast_max_speed=50.0)
    v_slow=c_slow.update(0.7,0,-10,0.02,0,True)
    assert abs(v_slow - (-0.8)) < 1e-9, v_slow
    c_fast=Controller(fast_max_speed=50.0)
    v_fast=c_fast.update(0.9,0,-10,0.02,0,True)
    assert abs(v_fast - (-5.0)) < 1e-9, v_fast
    # Verify fast braking when crossing 0.8 deg from 50 mm/s
    c_brake=Controller(fast_max_speed=50.0, speed=-50.0)
    v_brake=c_brake.update(0.5, -2.0, 0.0, 0.02, 0, True)
    assert abs(v_brake) < 44.5, f"Braking too slow across 0.8 deg: {v_brake}"
    # Closed-loop plant simulation (K=0.1675 deg/mm, 60ms delay) across 20..60 mm/s
    for spd in (20.0, 30.0, 50.0, 60.0):
        for init in (-0.75, -0.91, -2.5, 4.0):
            ctrl=Controller(fast_max_speed=spd)
            pos=0.0; hist=[0.0]*3; max_after_2s=0.0
            for k in range(250):
                t=k*0.02
                delayed=hist.pop(0); hist.append(pos)
                err=init+0.1675*delayed
                rate=0.1675*ctrl.speed
                vel=ctrl.update(err,rate,pos,0.02,0,True)
                pos+=vel*0.02
                if t>=2.0: max_after_2s=max(max_after_2s,abs(err))
            assert max_after_2s<=0.25, f"Oscillation spd={spd} init={init} max_after_2s={max_after_2s}"
    print('Offline checks passed: fast braking + oscillation-free across 20-60 mm/s')
