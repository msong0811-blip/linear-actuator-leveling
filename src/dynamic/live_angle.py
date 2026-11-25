import json, tkinter as tk, time, math, sys, uuid, subprocess, msvcrt, os
from tkinter import messagebox
from pathlib import Path
from datetime import datetime, timezone
from collections import deque

R=Path(__file__).resolve().parent
ui_lock=(R/'display.lock').open('a+b');ui_lock.seek(0)
try:msvcrt.locking(ui_lock.fileno(),msvcrt.LK_NBLCK,1)
except OSError:raise SystemExit('Display already open')
(R/'display.pid').write_text(str(os.getpid()))
sys.path.insert(0,str(R.parent/'pico_wh_setup/python_lib'))
import serial
from serial.tools import list_ports
readonly_imu=None;imu_buffer=b'';readonly_angle=None;readonly_stamp=0;reconnect_hold_until=0
root=tk.Tk();root.title('액티브 서스펜션 · 초기 캘리브레이션 · 최대 20mm/s');root.geometry('850x800');root.configure(bg='#101827')
root.minsize(700,550)
fg='#edf2f7';muted='#a8b7c9'
tk.Label(root,text='차체 앞뒤 기울기',font=('맑은 고딕',19,'bold'),bg='#101827',fg=fg).pack(pady=(20,2))
tk.Label(root,text='수평계로 확인한 자세 = 0°  ·  앞쪽 들림 + / 앞쪽 내려감 −',font=('맑은 고딕',11),bg='#101827',fg=muted).pack()
angle=tk.Label(root,text='연결 확인 중',font=('맑은 고딕',48,'bold'),bg='#101827',fg='#62ddb0');angle.pack(pady=(8,0))
direction=tk.Label(root,text='',font=('맑은 고딕',12),bg='#101827',fg=fg);direction.pack()
status=tk.Label(root,text='',font=('맑은 고딕',11),bg='#101827',fg=muted);status.pack(pady=6)
canvas=tk.Canvas(root,height=220,bg='#152238',highlightthickness=0);canvas.pack(fill='both',expand=True,padx=24,pady=8)
details=tk.Label(root,text='',font=('맑은 고딕',10),bg='#101827',fg=muted);details.pack(pady=3)
travel=tk.Label(root,text='',font=('맑은 고딕',13,'bold'),bg='#101827',fg=fg);travel.pack(pady=4)
tk.Label(root,text='남은 이동거리: 설정된 제어 범위 기준 · 펄스 추정값',font=('맑은 고딕',10),bg='#101827',fg=muted).pack()
def stop():
    (R/'STOP.request').write_text('User pressed live display STOP')
    status.config(text='정지 요청 전송 · 응답 확인 중',fg='#ffba69')
tk.Button(root,text='모터 구동 정지',command=stop,bg='#b42f45',fg='white',activebackground='#dc425c',font=('맑은 고딕',14,'bold'),relief='flat',padx=26,pady=8).pack(pady=8)
def restart():
    p=R/'RESTART.pending';p.write_text(json.dumps({'id':str(uuid.uuid4())}));p.replace(R/'RESTART.request')
    status.config(text='재시작 조건 확인 중',fg='#62ddb0')
restart_button=tk.Button(root,text='다시 작동 · 최대 20mm/s',command=restart,bg='#21785a',fg='white',font=('맑은 고딕',13,'bold'),relief='flat',padx=24,pady=8)
restart_button.pack(pady=3)
def invalidate():
    (R/'INVALIDATE.request').write_text('Operator requires new physical reference')
    restart_button.config(state='disabled')
def calibrate():
    global readonly_imu,reconnect_hold_until
    try:s=json.loads((R/'state.json').read_text())
    except (OSError,ValueError):s={}
    stale=time.time()-datetime.fromisoformat(s.get('updated_at_utc','2000-01-01T00:00:00+00:00')).timestamp()>2
    if s.get('phase')=='closed' or stale:
        if readonly_imu:readonly_imu.close();readonly_imu=None
        reconnect_hold_until=time.time()+8
        subprocess.Popen([sys.executable,str(R/'run.py')],cwd=R,creationflags=subprocess.CREATE_NO_WINDOW)
    else:invalidate()
    dialog=tk.Toplevel(root);dialog.title('초기 캘리브레이션');dialog.geometry('560x420');dialog.transient(root)
    tk.Label(dialog,text='모터 전원을 켠 뒤 현재 남은 안전 이동거리를 측정하세요.\n끝단 5mm 여유를 제외하고 입력합니다. 합계는 70mm입니다.\n이 과정에서는 모터가 움직이지 않습니다.',justify='left',font=('맑은 고딕',11)).pack(pady=15)
    fields=tk.Frame(dialog);fields.pack()
    tk.Label(fields,text='줄일 수 있는 거리 (mm)').grid(row=0,column=0,padx=8,pady=6)
    retract=tk.Entry(fields,width=12);retract.grid(row=0,column=1)
    tk.Label(fields,text='늘릴 수 있는 거리 (mm)').grid(row=1,column=0,padx=8,pady=6)
    extend=tk.Entry(fields,width=12);extend.grid(row=1,column=1)
    powered=tk.BooleanVar(value=False);level=tk.BooleanVar(value=False)
    tk.Checkbutton(dialog,text='모터 전원을 켰고 현재의 실제 이동 여유를 확인했습니다',variable=powered).pack(pady=8)
    tk.Checkbutton(dialog,text='수평계로 현재 수평을 확인했습니다 → 각도 0°도 다시 저장',variable=level).pack(pady=8)
    tk.Label(dialog,text='각도 보정을 선택하지 않으면 기존 수평 기준을 유지합니다.\nIMU 장착이 바뀌었다면 수평계를 사용해 각도도 보정하세요.',font=('맑은 고딕',10)).pack(pady=5)
    def submit():
        try:
            a=float(retract.get());b=float(extend.get())
            if not all(math.isfinite(v) and 0<=v<=70 for v in (a,b)) or abs(a+b-70)>.02:raise ValueError('두 거리의 합계가 70mm가 되도록 입력해 주세요.')
            if not powered.get():raise ValueError('전원과 실제 이동 여유 확인 항목을 선택해 주세요.')
            current=json.loads((R/'state.json').read_text())
            if current['phase']!='calibration_required':raise ValueError('정지 및 연결 확인 중입니다. 잠시 후 다시 눌러 주세요.')
            data={'id':str(uuid.uuid4()),'retraction_mm':a,'extension_mm':b,'motor_power_on_confirmed':True,'physically_level_confirmed':level.get()}
            p=R/'CALIBRATE.pending';p.write_text(json.dumps(data));p.replace(R/'CALIBRATE.request');dialog.destroy()
        except (OSError,ValueError,KeyError) as e:messagebox.showerror('캘리브레이션 확인',str(e),parent=dialog)
    tk.Button(dialog,text='캘리브레이션 적용 · 이동 없음',command=submit,font=('맑은 고딕',12),padx=20,pady=6).pack(pady=12)
tk.Button(root,text='초기 캘리브레이션',command=calibrate,font=('맑은 고딕',12),bg='#346594',fg='white',relief='flat',padx=22,pady=7).pack(pady=3)
tk.Button(root,text='전원 끄기 전 · 위치 무효화',command=invalidate,font=('맑은 고딕',10),padx=16,pady=4).pack(pady=3)
tk.Label(root,text='이 창을 닫아도 제어는 계속됩니다. 정지하려면 위 버튼을 누르세요.',font=('맑은 고딕',10),bg='#101827',fg=muted).pack(pady=(0,14))
history=deque(maxlen=600);last_stamp=None
def draw(now):
    w=max(canvas.winfo_width(),200);h=max(canvas.winfo_height(),100);left,right,top,bottom=55,w-15,15,h-30
    canvas.delete('all')
    scale=max(7,math.ceil(max((abs(v) for _,v in history),default=7)))
    def y(v):return (top+bottom)/2-v/scale*(bottom-top)/2
    def x(t):return right-(now-t)/60*(right-left)
    canvas.create_rectangle(left,y(.2),right,y(-.2),fill='#174039',outline='')
    for val in (-scale,0,scale):
        yy=y(val);canvas.create_line(left,yy,right,yy,fill='#46546a' if val==0 else '#29374c')
        canvas.create_text(left-6,yy,text=f'{val:+.0f}°',anchor='e',fill=muted)
    for seconds in (60,30,0):canvas.create_text(x(now-seconds),h-13,text=f'−{seconds}s' if seconds else '현재',fill=muted)
    points=[]
    for t,v in history:
        if now-t<=60:points.extend((x(t),y(v)))
    if len(points)>=4:canvas.create_line(*points,fill='#62ddb0',width=2)
    canvas.create_text(right-8,top+5,text='녹색 영역 ±0.2°',anchor='ne',fill=muted)
def update():
    global last_stamp,readonly_imu,imu_buffer,readonly_angle,readonly_stamp
    now=time.time()
    try:
        s=json.loads((R/'state.json').read_text())
        stamp=datetime.fromisoformat(s['updated_at_utc']).timestamp();age=now-stamp
        closed=s.get('phase')=='closed';fresh=age<=1
        restart_button.config(state='normal' if fresh and s.get('phase')=='paused' and s.get('board',{}).get('ref')==1 else 'disabled')
        if closed and now>=reconnect_hold_until:
            if readonly_imu is None:
                device=next((p for p in list_ports.comports() if p.vid==0x10C4 and p.pid==0xEA60 and p.serial_number=='0001'),None)
                if device:
                    port=serial.Serial();port.port=device.device;port.baudrate=115200;port.timeout=0;port.dtr=False;port.rts=False;port.open();readonly_imu=port
            if readonly_imu:
                imu_buffer+=readonly_imu.read(readonly_imu.in_waiting or 0)
                while b'\n' in imu_buffer:
                    raw,imu_buffer=imu_buffer.split(b'\n',1)
                    try:
                        line=raw.decode().strip()
                        if line.startswith('*'):
                            values=[float(v) for v in line[1:].split(',')]
                            if len(values)==3 and all(math.isfinite(v) for v in values):readonly_angle=values;readonly_stamp=now
                    except (ValueError,UnicodeError):pass
                if len(imu_buffer)>8192:imu_buffer=b''
                age=now-readonly_stamp;fresh=age<=1;stamp=readonly_stamp
                if fresh:s['imu']=readonly_angle
        if fresh and s.get('imu'):
            value=s['imu'][0]-s.get('reference_roll',1.87)
            if last_stamp!=stamp:history.append((stamp,value));last_stamp=stamp
            angle.config(text=f'{value:+.2f}°',fg='#62ddb0' if abs(value)<=.2 else '#ffba69')
            direction.config(text='수평 범위' if abs(value)<=.2 else ('앞쪽이 들림' if value>0 else '앞쪽이 내려감'))
        else:
            angle.config(text='—',fg=muted);direction.config(text='실시간 각도 수신 중단')
        if closed:
            reason='±3° 시험 한계 초과' if '3 degree' in s.get('stop_reason','') else str(s.get('stop_reason',''))
            status.config(text='모터 제어 정지 · '+reason+' · 각도만 표시 중',fg='#ffba69')
        elif s.get('phase')=='calibration_required':
            status.config(text='초기 캘리브레이션 필요 · '+str(s.get('calibration_error','거리 입력 후 다시 작동할 수 있습니다')),fg='#ffba69')
        elif s.get('phase')=='paused':
            reasons={'Ready: press Restart':'준비됨 · 다시 작동 버튼을 누르세요',
                     'Calibration complete: press Restart':'캘리브레이션 완료 · 다시 작동 버튼을 누르세요',
                     'Operator stop':'사용자 정지',
                     'Angle exceeded +/-7 degrees':'±7° 초과 · 차체를 범위 안으로 옮긴 뒤 재시작',
                     'Outside +/-7 degrees: reposition chassis before restart':'±7° 밖에서는 재시작할 수 없습니다',
                     'No travel available in required direction: reposition chassis':'필요한 방향의 이동 여유가 없습니다 · 차체 자세를 조정하세요',
                     'Travel boundary reached':'이동 경계 도달 · 차체 자세를 조정하세요',
                     'Ten-minute test complete; restart available':'10분 시험 종료 · 다시 작동 가능'}
            status.config(text='정지 · '+reasons.get(s.get('pause_reason'),str(s.get('pause_reason',''))),fg='#ffba69')
        elif not fresh:status.config(text='통신 확인 필요 · 오래된 값을 현재 각도로 표시하지 않습니다',fg='#ffba69')
        else:
            remaining=max(0,s.get('duration_seconds',600)-s.get('elapsed_seconds',0))
            status.config(text=f'수평 유지 제어 중  ·  남은 시험 {int(remaining)//60:02d}:{int(remaining)%60:02d}  ·  5Hz 표시',fg=muted)
        pos=s.get('board',{}).get('pos',0)/160
        limits=s.get('limits_mm',[-41.8,28.2])
        extension=max(0,limits[1]-pos);retraction=max(0,pos-limits[0])
        valid=s.get('board',{}).get('ref')==1 and not closed and fresh
        travel.config(text=f'늘림 {extension:.1f} mm  |  줄임 {retraction:.1f} mm  |  합계 {extension+retraction:.1f} mm' if valid else '이동 여유: 위치 기준 확인 필요',fg='#ffba69' if min(extension,retraction)<2 else fg)
        details.config(text=f'액추에이터 추정 위치 {pos:.2f} mm  |  보정 속도 명령 {s.get("requested_velocity_mm_s",0):+.2f} mm/s  |  수신 {age:.1f}초 전')
    except (OSError,ValueError,KeyError,TypeError):
        angle.config(text='—',fg=muted);direction.config(text='데이터 확인 중')
    draw(now);root.after(200,update)
root.after(100,update)
root.after(300,lambda:(R/'display_ready.json').write_text(json.dumps({'opened':True,'refresh_hz':5,'serial_ports_opened':False})))
try:root.mainloop()
finally:
    if readonly_imu:readonly_imu.close()
