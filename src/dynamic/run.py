import csv,json,math,os,re,statistics,sys,time
from pathlib import Path
from datetime import datetime,timezone
from collections import deque
from controller import Controller
from calibration import confirmed_position
R=Path(__file__).resolve().parent
sys.path.insert(0,str(R.parent/'pico_wh_setup/python_lib'))
import serial
from serial.tools import list_ports
import msvcrt
lock=(R/'host.lock').open('a+b');lock.seek(0)
try:msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
except OSError:raise SystemExit('Controller already running')
(R/'host.pid').write_text(str(os.getpid()))
archive=R/('session_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'));archive.mkdir()
for name in ('state.json','serial.log','trace.csv'):
    if (R/name).exists():(R/name).replace(archive/name)
for name in ('STOP.request','RESTART.request','CALIBRATE.request','INVALIDATE.request'):(R/name).unlink(missing_ok=True)
settings=json.loads((R/'level_reference.json').read_text()) if (R/'level_reference.json').exists() else {'roll':1.87,'pitch':-1.01}
reference_roll=settings['roll'];reference_pitch=settings['pitch']
ports=list(list_ports.comports())
motor=next((p for p in ports if p.device=='COM8' and p.vid==0x0403 and p.pid==0x6001 and p.serial_number in ('A5069RR4','A5069RR4A')),None)
imu=next((p for p in ports if p.vid==0x10C4 and p.pid==0xEA60 and p.serial_number=='0001'),None)
if not motor or not imu: raise SystemExit('Device identity missing')
state={'pid':os.getpid(),'phase':'connecting','active_control':False,'physical_position_feedback':False,'limits_mm':[-41.8,28.2],'max_speed_mm_s':20,'angle_limit_deg':7,'reference_roll':reference_roll}
mot=sensor=None; buffers=[b'',b'']; events=deque(maxlen=200); samples=deque(maxlen=1000)
ready=False; monitor=False; board=None; angles=None; lastimu=lastmotor=0
nextstatus=nexthb=nextsave=0; rate=0.; previous_roll=None; previous_t=None
pat=re.compile(r'STATUS ref=([01]) phase=([012]) pos=(-?\d+) target=(-?\d+) pulses=(\d+) min=-6688 max=4512')
def log(s):
    with (R/'serial.log').open('a') as f: f.write(datetime.now(timezone.utc).isoformat()+' '+s+'\n')
def save():
    state.update(updated_at_utc=datetime.now(timezone.utc).isoformat(),board=board,imu=angles)
    p=R/'state.pending'; p.write_text(json.dumps(state,indent=2))
    try:p.replace(R/'state.json')
    except PermissionError:pass
def send(s):
    mot.write((s+'\n').encode());
    if s not in ('KEEPALIVE','STATUS'):log('TX '+s)
def pump():
    global buffers,ready,board,angles,lastimu,lastmotor,nextstatus,nexthb,nextsave,rate,previous_roll,previous_t
    now=time.monotonic()
    if (R/'STOP.request').exists():state['pause_requested']=True
    for i,port in enumerate((sensor,mot)):
        buffers[i]+=port.read(port.in_waiting or 0)
        if len(buffers[i])>8192:raise RuntimeError('Receive overflow')
        while b'\n' in buffers[i]:
            raw,buffers[i]=buffers[i].split(b'\n',1); line=raw.decode(errors='replace').strip()
            if i==0:
                if not line.startswith('*'):continue
                try:a=[float(x) for x in line[1:].split(',')]
                except ValueError:continue
                if len(a)!=3 or not all(math.isfinite(x) for x in a):continue
                if previous_t is not None and now-previous_t>=0.005:
                    dt=now-previous_t; derivative=(a[0]-previous_roll)/dt
                    alpha=dt/(0.15+dt); rate+=alpha*(derivative-rate)
                    previous_roll=a[0]; previous_t=now
                elif previous_t is None: previous_roll=a[0]; previous_t=now
                angles=a;lastimu=now;samples.append((now,a))
                writer.writerow([now,board['pos'] if board else '',*a,rate,state['phase']])
            else:
                log('RX '+line);events.append(line)
                if line.startswith('ACTUATOR_LIMITED_READY'):
                    if ready or 'v=9 ' not in line:raise RuntimeError('Unexpected restart/firmware')
                    ready=True
                if line.startswith(('FAULT','REFUSED')) or (line.startswith('STOPPED') and state['phase']!='pausing'):raise RuntimeError(line)
                m=pat.fullmatch(line)
                if m:
                    board=dict(zip(['ref','phase','pos','target','pulses'],map(int,m.groups())));lastmotor=now
                    if monitor and ((state['phase']!='calibration_required' and board['ref']!=1) or (board['ref']==1 and not -6688<=board['pos']<=4512)):raise RuntimeError('Position/reference fault')
    if ready and now>=nextstatus:send('STATUS');nextstatus=now+0.05
    if monitor and now>=nexthb:send('KEEPALIVE');nexthb=now+0.1
    if monitor:
        if now-lastimu>0.1 or now-lastmotor>0.5:raise RuntimeError('Sensor or motor status stale')
    if now>=nextsave:save();trace.flush();nextsave=now+0.2
    time.sleep(0.002)
def until(test,seconds):
    end=time.monotonic()+seconds
    while not test():
        if time.monotonic()>end:raise TimeoutError('Response timeout')
        pump()
def dwell(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:pump()
def speed(hz):
    events.clear();send('SPEED_HZ '+str(hz));until(lambda:'SPEED hz='+str(hz) in events,0.5)
def move(target):
    if not -6688<=target<=4512 or abs(target-board['pos'])>1600:raise ValueError('Target outside envelope')
    events.clear();send('MOVE_PULSES '+str(target));until(lambda:'DONE' in events or 'DONE unchanged' in events,4)
    until(lambda:board['phase']==0 and board['pos']==target,0.5)

def pause(reason):
    # Each micro-move is <=2 mm. Finish the current pulse sequence before STOP
    # so normal operator/angle pauses preserve the commanded-position reference.
    state.update(phase='pausing',active_control=False,pause_reason=reason)
    dwell(0.15)
    until(lambda:board['phase']==0,0.8)
    events.clear();send('STOP');until(lambda:'STOPPED reason=command' in events,0.5)
    send('STATUS');dwell(0.1)
    if board['ref']!=1:raise RuntimeError('Position reference lost: physical re-reference required')
    state.update(phase='paused',pause_requested=False,requested_velocity_mm_s=0)
    (R/'STOP.request').unlink(missing_ok=True);save()

try:
    trace=(R/'trace.csv').open('w',newline='');writer=csv.writer(trace);writer.writerow(['time','position_pulses','roll','pitch','yaw','filtered_roll_rate','phase'])
    sensor=serial.Serial();sensor.port=imu.device;sensor.baudrate=115200;sensor.timeout=0;sensor.dtr=False;sensor.rts=False;sensor.open()
    mot=serial.Serial(motor.device,115200,timeout=0,write_timeout=0.2)
    until(lambda:ready and board is not None and angles is not None and 'LIMITS_TEST_PASS cases=22414' in events,6)
    if board['ref'] or board['pos']!=0:raise RuntimeError('Unexpected boot state')
    monitor=True;state.update(phase='calibration_required',pause_reason='Power-cycle/startup: calibration required');save()
    control=Controller();last=time.monotonic();pending=False;pending_at=0;residual=0;last_hz=160;last_issue=0;start=0;seen=set()
    while True:
        pump();now=time.monotonic()
        state['error_deg']=angles[0]-reference_roll
        if (R/'INVALIDATE.request').exists():
            if state['phase']!='calibration_required':
                pause('Calibration requested')
                state['phase']='calibration_required';send('CLEAR_REFERENCE')
                until(lambda:board['ref']==0,1)
            (R/'INVALIDATE.request').unlink(missing_ok=True)
            state.update(active_control=False,pause_reason='Calibration required');save()
        calibration=R/'CALIBRATE.request'
        if calibration.exists():
            data=json.loads(calibration.read_text())
            if data['id'] not in seen:
                seen.add(data['id'])
                try:
                    if state['phase']!='calibration_required' or board['ref']!=0:raise ValueError('Enter calibration mode first')
                    confirmed=confirmed_position(data['retraction_mm'],data['extension_mm'],data.get('motor_power_on_confirmed'))
                    send('REFERENCE_PULSES '+str(confirmed));until(lambda:board['ref']==1 and board['pos']==confirmed,1)
                    if data.get('physically_level_confirmed'):
                        dwell(1)
                        selected=[a for t,a in samples if t>time.monotonic()-0.8]
                        if len(selected)<40 or max(a[0] for a in selected)-min(a[0] for a in selected)>.15:raise ValueError('Chassis not stable for level reference')
                        reference_roll=statistics.mean(a[0] for a in selected);reference_pitch=statistics.mean(a[1] for a in selected)
                        (R/'level_reference.json').write_text(json.dumps({'roll':reference_roll,'pitch':reference_pitch}))
                    state.update(phase='paused',pause_reason='Calibration complete: press Restart',reference_roll=reference_roll,calibration=data)
                    state.pop('calibration_error',None);save()
                except ValueError as e:
                    if board['ref']==1:
                        state['phase']='calibration_required';send('CLEAR_REFERENCE');until(lambda:board['ref']==0,1)
                    state['calibration_error']=str(e);save()
        request=R/'RESTART.request'
        if request.exists():
            try:token=json.loads(request.read_text())['id']
            except (OSError,ValueError,KeyError):token=None
            if token and token not in seen:
                seen.add(token)
                if state['phase']=='paused':
                    err=angles[0]-reference_roll;pos=board['pos']
                    if abs(err)>7 or abs(angles[1]-reference_pitch)>7:state['pause_reason']='Outside +/-7 degrees: reposition chassis before restart'
                    elif (pos>=4512 and err<-.2) or (pos<=-6688 and err>.2):state['pause_reason']='No travel available in required direction: reposition chassis'
                    else:
                        control=Controller();last=now;start=now;pending=False;residual=0;last_issue=now
                        state.update(phase='maintaining',active_control=True,pause_reason='',duration_seconds=600,elapsed_seconds=0)
                save()
        if state.get('pause_requested'):
            if state['phase']=='calibration_required':
                state['pause_requested']=False;(R/'STOP.request').unlink(missing_ok=True)
            else:pause('Operator stop')
            continue
        if state['phase']!='maintaining':last=now;continue
        if abs(angles[0]-reference_roll)>7 or abs(angles[1]-reference_pitch)>7:
            pause('Angle exceeded +/-7 degrees');continue
        if now-start>=600:pause('Ten-minute test complete; restart available');continue
        if now-last<0.02:continue
        dt=now-last;last=now
        v=control.update(angles[0]-reference_roll,rate,board['pos']/160,dt,now-lastimu,board['ref']==1)
        state.update(filtered_rate_deg_s=rate,requested_velocity_mm_s=v,elapsed_seconds=round(now-start,1))
        if pending:
            if 'DONE' in events and board['phase']==0:pending=False
            elif now-pending_at>0.8:raise RuntimeError('Micro-move completion timeout')
        if not pending and now-last_issue>=0.1:
            last_issue=now;err=angles[0]-reference_roll
            if abs(err)<=0.2 and abs(rate)<0.1:residual=0;continue
            if (board['pos']>=4512 and err<-.2) or (board['pos']<=-6688 and err>.2):pause('Travel boundary reached');continue
            if residual*v<0:residual=0
            residual+=v*.1*160;delta=max(-320,min(320,math.trunc(residual)))
            if delta:
                target=board['pos']+delta
                if not -6688<=target<=4512:pause('Travel boundary reached');continue
                hz=max(160,min(3200,round(abs(v)*160)))
                if hz!=last_hz:speed(hz);last_hz=hz
                events.clear();send('MOVE_PULSES '+str(target));pending=True;pending_at=time.monotonic();residual-=delta
except BaseException as e:state['stop_reason']=str(e);log('ERROR '+str(e))
finally:
    if mot and mot.is_open:
        try:
            mot.write(b'STOP\n');end=time.monotonic()+1;reply=b''
            while time.monotonic()<end:reply+=mot.read(mot.in_waiting or 0);time.sleep(0.01)
            state['stop_acknowledged']=b'STOPPED reason=command' in reply;log('FINAL '+reply.decode(errors='replace'))
        except Exception as e:state['stop_error']=str(e)
        mot.close()
    if sensor and sensor.is_open:sensor.close()
    if 'trace' in globals():trace.close()
    state.update(phase='closed',active_control=False);save()
