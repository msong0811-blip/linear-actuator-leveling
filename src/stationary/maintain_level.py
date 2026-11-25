import json, math, os, statistics, time
from datetime import datetime, timezone
from pathlib import Path
R=Path(__file__).resolve().parent
OUT=R/'maintenance_state.json'
if OUT.exists(): raise SystemExit('Refuse duplicate controller')
status={'pid':os.getpid(),'phase':'referencing','duration_seconds':600,'deadband_deg':0.2,'max_step_mm':1,'corrections':0,'travel_mm':0,'active_control':False}
def now(): return datetime.now(timezone.utc)
def publish():
    status['updated_at_utc']=now().isoformat()
    p=R/'maintenance_state.pending'; p.write_text(json.dumps(status,indent=2))
    try: p.replace(OUT)
    except PermissionError: pass
def read():
    s=json.loads((R/'state.json').read_text())
    if (now()-datetime.fromisoformat(s['updated_at_utc'])).total_seconds()>1: raise RuntimeError('Host state stale')
    if s['phase']=='closed' or s.get('error') or s['board']['ref']!=1: raise RuntimeError('Host stopped or reference invalid')
    if (R/'STOP.request').exists() or (R/'MAINTENANCE_STOP.request').exists(): raise RuntimeError('User stop')
    if not all(math.isfinite(v) for v in s['latest_imu']): raise RuntimeError('Invalid sensor data')
    return s
def target_for(pos,error):
    delta=max(-1,min(1,-error/0.165))
    target=round((pos+delta)*160)/160
    if not -65<=target<=-15: raise RuntimeError('Maintenance travel boundary reached')
    return target
assert target_for(-21,0.3)==-22
assert target_for(-21,-0.3)==-20
try:
    target_for(-15,-0.3)
    raise AssertionError('Boundary failed')
except RuntimeError: pass
publish()
try:
    samples=[]
    for _ in range(25):
        s=read()
        if s['board']['phase']!=0: raise RuntimeError('Expected stationary level reference')
        samples.append(s['latest_imu']); time.sleep(0.2)
    reference=[statistics.median(a[i] for a in samples) for i in range(3)]
    if max(a[0] for a in samples)-min(a[0] for a in samples)>0.1: raise RuntimeError('Level reference not stable')
    status.update(phase='maintaining',active_control=True,reference_imu_deg=reference,reference_source='User spirit-level confirmation at -21 mm',started_at_utc=now().isoformat())
    start=time.monotonic(); outside_since=None; pending=None; pending_at=0; settle_until=0
    while time.monotonic()-start<600:
        s=read(); angles=s['latest_imu']; error=angles[0]-reference[0]
        if abs(error)>3 or abs(angles[1]-reference[1])>3: raise RuntimeError('Attitude outside stationary commissioning envelope')
        status.update(body_pitch_error_estimate_deg=error,position_mm=s['board']['pos']/160,elapsed_seconds=round(time.monotonic()-start,1))
        if pending:
            if s.get('last_level_move_id')==pending and s['board']['phase']==0:
                pending=None; settle_until=time.monotonic()+2; outside_since=None
            elif time.monotonic()-pending_at>6: raise RuntimeError('Correction completion timeout')
        elif time.monotonic()>=settle_until:
            if s['board']['phase']!=0: raise RuntimeError('Unexpected motion from another controller')
            if abs(error)<=0.2: outside_since=None
            elif outside_since is None: outside_since=time.monotonic()
            elif time.monotonic()-outside_since>=1:
                if status['corrections']>=50: raise RuntimeError('Correction count limit')
                pos=s['board']['pos']/160; target=target_for(pos,error)
                pending='maintain_'+str(status['corrections']+1)+'_'+str(os.getpid())
                req={'id':pending,'physical_level_reference_confirmed':True,'targets_mm':[target]}
                p=R/'maintenance_command.pending'; p.write_text(json.dumps(req)); p.replace(R/'level_move.json')
                pending_at=time.monotonic(); status['corrections']+=1; status['travel_mm']+=abs(target-pos)
                with (R/'maintenance_actions.jsonl').open('a') as f: f.write(json.dumps({'time':now().isoformat(),'error':error,**req})+'\n')
        publish(); time.sleep(0.2)
    status['stop_reason']='Ten-minute observation complete'
except BaseException as e:
    status['stop_reason']=str(e)
finally:
    # Stop pulses without disabling motor power. No automatic restart.
    (R/'STOP.request').write_text('MAINTENANCE_ENDED')
    status.update(phase='stopped',active_control=False,ended_at_utc=now().isoformat()); publish()
