import csv, json, math, os, re, statistics, sys, time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'pico_wh_setup'/'python_lib'))
import serial
from serial.tools import list_ports
READY='ACTUATOR_LIMITED_READY v=3 ref=0 min=-12000 max=-800 max_move=1600'
PATTERN=re.compile(r'STATUS ref=([01]) phase=([012]) pos=(-?\d+) target=(-?\d+) pulses=(\d+) min=-12000 max=-800')
STATE=ROOT/'state.json'; STOP=ROOT/'STOP.request'; START=ROOT/'START.request'
if STATE.exists() or STOP.exists() or START.exists(): raise SystemExit('Existing run: refuse repeat')
ports=list(list_ports.comports())
motor=next((p for p in ports if p.device=='COM8' and p.vid==0x0403 and p.pid==0x6001 and p.serial_number in ('A5069RR4','A5069RR4A')),None)
imu=next((p for p in ports if p.vid==0x10C4 and p.pid==0xEA60 and p.serial_number=='0001'),None)
if not motor or not imu or imu.device=='COM8': raise SystemExit('Device identity mismatch')
plan=json.loads((ROOT/'plan.json').read_text())
state={'pid':os.getpid(),'phase':'connecting','motor_port':motor.device,'imu_port':imu.device,
       'started_at_utc':datetime.now(timezone.utc).isoformat(),'physical_position_feedback':False,
       'software_limits_mm':[-70,-10],'plateaus':[]}
events=deque(maxlen=200); frames=deque(maxlen=2000); buffers={'motor':b'','imu':b''}
last_imu=0; last_motor=0; baseline=None; latest=None; next_status=0; next_hb=0; next_save=0
mot=None; sensor=None; ready_seen=False; zero_set=False; monitor=False

def save():
    state['updated_at_utc']=datetime.now(timezone.utc).isoformat()
    temp=ROOT/'state.tmp'; temp.write_text(json.dumps(state,indent=2),encoding='utf-8')
    try: temp.replace(STATE)
    except PermissionError:
        # A read-only Windows observer can briefly hold the destination open.
        # Do not stall motor heartbeat delivery; publish on the next pump.
        pass
def log(message):
    with (ROOT/'serial.log').open('a',encoding='utf-8') as f: f.write(datetime.now(timezone.utc).isoformat()+' '+message+'\n')
def send(message):
    mot.write((message+'\n').encode('ascii'))
    if message!='KEEPALIVE': log('TX '+message)
def pump():
    global last_imu,last_motor,baseline,latest,next_status,next_hb,next_save,ready_seen
    now=time.monotonic()
    if STOP.exists(): raise RuntimeError('User stop requested')
    for key,port in [('imu',sensor),('motor',mot)]:
        data=port.read(port.in_waiting or 0)
        buffers[key]+=data
        if len(buffers[key])>8192: raise RuntimeError(key+' receive buffer overflow')
        while b'\n' in buffers[key]:
            raw,buffers[key]=buffers[key].split(b'\n',1)
            line=raw.decode('ascii',errors='replace').strip()
            if key=='imu':
                if not line.startswith('*'): continue
                try: angles=[float(x) for x in line[1:].split(',')]
                except ValueError: continue
                if len(angles)!=3 or not all(math.isfinite(x) for x in angles): continue
                if abs(angles[0])>180 or abs(angles[1])>90 or abs(angles[2])>180: raise RuntimeError('Invalid IMU angle range')
                last_imu=now; latest=angles
                if baseline is None: baseline=angles[:]
                if monitor and max(abs(angles[i]-baseline[i]) for i in (0,1))>20: raise RuntimeError('IMU attitude changed more than 20 degrees from start')
                sample={'t':now,'position_pulses':state.get('board',{}).get('pos'),'angles':angles}
                frames.append(sample)
                writer.writerow([now,sample['position_pulses'],*angles,state['phase'],state.get('target_mm','')])
            else:
                log('RX '+line); events.append(line)
                if line==READY:
                    if ready_seen: raise RuntimeError('Unexpected motor restart; reference lost')
                    ready_seen=True
                if line.startswith('FAULT'): raise RuntimeError(line)
                match=PATTERN.fullmatch(line)
                if match:
                    last_motor=now
                    state['board']=dict(zip(['ref','phase','pos','target','pulses'],map(int,match.groups())))
                    if monitor and state['board']['ref']!=1: raise RuntimeError('Position reference invalidated')
                    if not -12000<=state['board']['pos']<=0: raise RuntimeError('Commanded position outside physical interval')
                if monitor and line.startswith(('STOPPED','REFUSED')): raise RuntimeError(line)
    if ready_seen and now>=next_status:
        send('STATUS'); next_status=now+0.2
    if monitor and now>=next_hb:
        send('KEEPALIVE'); next_hb=now+0.1
    if monitor and now-last_imu>0.3: raise RuntimeError('IMU data stale beyond 0.3 s')
    if monitor and now-last_motor>1.0: raise RuntimeError('Motor status missing beyond 1 s')
    if now>=next_save:
        state['latest_imu']=latest; save(); trace.flush(); next_save=now+0.25
    time.sleep(0.002)
def until(predicate,seconds):
    deadline=time.monotonic()+seconds
    while not predicate():
        if time.monotonic()>deadline: raise TimeoutError('Expected device state/response missing')
        pump()
def dwell(seconds):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline: pump()
def move(target_mm):
    target=round(target_mm*160)
    if abs(target_mm*160-target)>1e-6 or not -11200<=target<=-1600: raise ValueError('Invalid movement target')
    current=state['board']['pos']
    if abs(target-current)>1600: raise ValueError('Move exceeds 10 mm commissioning limit')
    events.clear(); state.update(phase='moving',target_mm=target_mm); save()
    send('MOVE_PULSES '+str(target))
    until(lambda: 'DONE' in events or 'DONE unchanged' in events,12.5)
    until(lambda: state['board']['phase']==0 and state['board']['pos']==target,1)
    state['phase']='settling'; settle_start=time.monotonic(); dwell(1.5)
    selected=[s['angles'] for s in frames if s['t']>=settle_start+0.5]
    if len(selected)<50: raise RuntimeError('Insufficient settled IMU samples')
    point={'position_mm':target/160,'mean_deg':[statistics.mean(a[i] for a in selected) for i in range(3)],
           'stddev_deg':[statistics.stdev(a[i] for a in selected) for i in range(3)],'samples':len(selected)}
    state['plateaus'].append(point); save()

save()
try:
    trace=(ROOT/'trace.csv').open('w',newline='',encoding='utf-8'); writer=csv.writer(trace)
    writer.writerow(['host_monotonic_s','commanded_position_pulses','roll_deg','pitch_deg','yaw_deg','phase','target_mm'])
    sensor=serial.Serial(); sensor.port=imu.device; sensor.baudrate=115200; sensor.timeout=0; sensor.write_timeout=0.2; sensor.dtr=False; sensor.rts=False; sensor.open()
    mot=serial.Serial(motor.device,115200,timeout=0,write_timeout=0.2)
    until(lambda: ready_seen and 'LIMITS_TEST_PASS cases=24014' in events and latest is not None and 'board' in state,6)
    if state['board']!={'ref':0,'phase':0,'pos':0,'target':0,'pulses':0}: raise RuntimeError('Unexpected initial state')
    events.clear(); send('MOVE_PULSES -800')
    until(lambda:'REFUSED target_check=1' in events,1)
    state['unreferenced_move_rejected']=True; dwell(0.4)
    if state['board']['pos']!=0 or state['board']['pulses']!=0: raise RuntimeError('Unreferenced command produced pulses')
    events.clear(); send('REFERENCE_MINUS60_CONFIRMED')
    until(lambda:'REFERENCE pos=-9600 units=pulses' in events and state['board']['ref']==1,1)
    zero_set=True; monitor=True
    state.update(phase='ready_waiting_start',boot_limit_tests_passed=True,reference_basis='User reports 20 mm remaining to retraction stop of original 80 mm stroke; approximate current -60 mm; rear wheels raised'); save()
    until(lambda:START.exists(),120)
    if START.read_text(encoding='utf-8-sig').strip()!='RUN_CALIBRATION_SWEEP_ONCE': raise ValueError('Unexpected start token')
    state['phase']='countdown'; save(); dwell(10)
    stationary=[s['angles'] for s in frames if s['t']>=time.monotonic()-2]
    if len(stationary)<100: raise RuntimeError('Insufficient initial reference samples')
    state['initial_reference']={'position_mm':-60,'body_nose_up_deg':-9.5,
        'source':'User confirmed nose-down 9.5 deg after raising rear wheels',
        'mean_deg':[statistics.mean(a[i] for a in stationary) for i in range(3)],
        'stddev_deg':[statistics.stdev(a[i] for a in stationary) for i in range(3)],
        'samples':len(stationary)}
    save()
    for target in plan['targets_mm']: move(target)
    state.update(phase='calibration_complete_waiting_level',calibration_completed=True); save()
    (ROOT/'calibration_points.json').write_text(json.dumps(state['plateaus'],indent=2),encoding='utf-8')
    # Keep the connection/reference alive for a separately prepared levelling move.
    # No automatic movement while waiting for the spirit-level reference.
    processed=set(); idle_deadline=time.monotonic()+1800
    while time.monotonic()<idle_deadline:
        request=ROOT/'level_move.json'
        if request.exists():
            specification=json.loads(request.read_text(encoding='utf-8-sig'))
            token=specification['id']
            if token not in processed:
                if not specification.get('physical_level_reference_confirmed'): raise ValueError('Missing physical horizontal reference')
                processed.add(token)
                targets=specification['targets_mm']
                if not targets or len(targets)>10: raise ValueError('Invalid levelling sequence')
                for target in targets: move(target)
                state.update(phase='level_move_complete_waiting_measurement',last_level_move_id=token); save()
        pump()
    state['stop_reason']='Idle session timeout; reconnect requires new physical reference'
except BaseException as error:
    state['error']=str(error); log('ERROR '+str(error))
finally:
    monitor=False
    if mot and mot.is_open:
        try:
            mot.write(b'STOP\n'); deadline=time.monotonic()+1
            while time.monotonic()<deadline:
                data=mot.read(mot.in_waiting or 0)
                if data:
                    log('FINAL '+data.decode('ascii',errors='replace').strip())
                    if b'STOPPED reason=command' in data: state['stop_acknowledged']=True
                time.sleep(0.02)
        except Exception as error: state['stop_error']=str(error)
        mot.close()
    if sensor and sensor.is_open: sensor.close()
    if 'trace' in globals(): trace.close()
    state.update(phase='closed',ended_at_utc=datetime.now(timezone.utc).isoformat()); save()
