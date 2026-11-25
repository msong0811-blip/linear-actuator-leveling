"""Offline controller core. No serial I/O or hardware activation.

Velocity interface and disturbance-specific validation are still required.
Units: seconds, degrees (nose up positive), millimetres (extension positive).
"""
from dataclasses import dataclass
import math

@dataclass
class Controller:
    low: float = -41.8
    high: float = 28.2
    max_speed: float = 20.0  # Enabled only after staged physical speed checks.
    acceleration: float = 40.0
    kp: float = 20.0  # Candidate commissioning values, not tuned gains.
    kd: float = 3.0
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
        demand=0 if abs(pitch)<self.deadband and abs(pitch_rate)<0.1 else -self.kp*pitch-self.kd*pitch_rate
        demand=max(-self.max_speed,min(self.max_speed,demand))
        self.speed+=max(-self.acceleration*dt,min(self.acceleration*dt,demand-self.speed))
        # Keep braking distance plus one sample's travel inside the boundary.
        margin=self.speed*self.speed/(2*self.acceleration)+abs(self.speed)*dt
        if (self.speed>0 and position+margin>=self.high) or (self.speed<0 and position-margin<=self.low):
            self.speed=0
        return self.speed

if __name__=='__main__':
    c=Controller()
    assert c.update(1,0,-21,0.02,0,True)<0
    c=Controller()
    assert c.update(-1,0,-21,0.02,0,True)>0
    c=Controller()
    assert c.update(-1,0,28.2,0.02,0,True)==0
    assert c.update(1,0,-41.8,0.02,0,True)==0
    for kwargs in ({'sensor_age':0.11},{'reference_valid':False},{'pitch':float('nan')}):
        args=dict(pitch=0,pitch_rate=0,position=-21,dt=0.02,sensor_age=0,reference_valid=True)
        args.update(kwargs)
        try: Controller().update(**args)
        except ValueError: pass
        else: raise AssertionError('Fault not rejected')
    print('Offline checks passed: correction direction, boundaries, stale sensor, lost reference, nonfinite input')
