#!/usr/bin/env python3
"""Mac Tkinter WASD + Active Suspension teleop GUI for the 3-wheeled vehicle on Jetson (`minseong`)."""
import json
import math
import queue
import subprocess
import threading
import tkinter as tk
from tkinter import ttk

SSH_HOST = "minseong"
REMOTE_SCRIPT = "~/three_wheel_teleop_20261127/jetson_worker.py"
RATIO_REAR_TO_FRONT = 260.0 / 120.0

KEY_MAP = {
    "w": "w",
    "a": "a",
    "s": "s",
    "d": "d",
    "c": "c",
    "space": "space",
    "ㅈ": "w",
    "ㅁ": "a",
    "ㄴ": "s",
    "ㅇ": "d",
    "ㅊ": "c",
}


class TeleopApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("3륜 차량 + 액티브 서스펜션 통합 테스트 조종기 (Jetson: minseong)")
        self.root.geometry("720x885")
        self.root.resizable(False, False)

        self.proc = None
        self.rx_queue: queue.Queue = queue.Queue()
        self.pressed_keys: set[str] = set()
        self.release_timers: dict[str, str] = {}
        self.running = False
        self.flashing = False
        self.closing = False

        self.front_limit_var = tk.DoubleVar(value=10.0)
        self.susp_speed_var = tk.DoubleVar(value=30.0)
        self.fw_max_speed = 20.0
        self.last_sent_susp_speed = None
        self.require_susp_var = tk.BooleanVar(value=False)

        self.retract_var = tk.StringVar(value="45.0")
        self.extend_var = tk.StringVar(value="25.0")
        self.sum_var = tk.StringVar(value="합계: 70.0 mm")
        self.motor_power_var = tk.BooleanVar(value=False)
        self.cal_mode_var = tk.StringVar(value="measure")
        self.current_tilt_var = tk.StringVar(value="0.0")

        self.status_var = tk.StringVar(value="대기 중 — '연결 및 모터 활성화'를 누르세요 (바퀴 여유 공간 확인)")
        self.cmd_var = tk.StringVar(value="앞 목표: 0.00 RPM  |  뒤 목표: 0.00 RPM  |  조향: 0.0° (center 3074)")
        self.tilt_main_var = tk.StringVar(value="앞뒤 기울기: 연결 대기 중")
        self.tilt_sub_var = tk.StringVar(value="IMU Roll(앞뒤)= -  |  수평 기준= -  |  IMU Pitch(좌우)= -")
        self.susp_phase_var = tk.StringVar(value="서스펜션 상태: 대기 중")
        self.susp_tel_var = tk.StringVar(value="액추에이터: 연결 전")
        self.susp_err_var = tk.StringVar(value="")

        self.motor_vars = {
            "1": tk.StringVar(value="ID 1 (앞 주행 260mm): -"),
            "2": tk.StringVar(value="ID 2 (앞 조향): -"),
            "3": tk.StringVar(value="ID 3 (우 뒤 120mm): -"),
            "4": tk.StringVar(value="ID 4 (좌 뒤 120mm): -"),
        }

        self._build_ui()
        self._bind_keys()
        self.retract_var.trace_add("write", lambda *_: self._update_sum_label())
        self.extend_var.trace_add("write", lambda *_: self._update_sum_label())
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(50, self._tick_tx)
        self.root.after(50, self._tick_rx)

    def _build_ui(self):
        pad = {"padx": 12, "pady": 5}
        top = ttk.Frame(self.root)
        top.pack(fill="x", **pad)
        ttk.Label(top, textvariable=self.status_var, font=("AppleGothic", 12, "bold")).pack(anchor="w")

        btns = ttk.Frame(self.root)
        btns.pack(fill="x", padx=12, pady=3)
        self.start_btn = ttk.Button(btns, text="연결 및 모터 활성화", command=self.start_worker)
        self.start_btn.pack(side="left", padx=(0, 8))
        self.stop_btn = ttk.Button(btns, text="전체 정지 / 토크 해제 (Esc/Q)", command=self.stop_worker, state="disabled")
        self.stop_btn.pack(side="left", padx=(0, 8))
        self.flash_btn = ttk.Button(btns, text="고속 펌웨어(60mm/s) Arduino 업로드", command=self.flash_firmware)
        self.flash_btn.pack(side="right")

        spd = ttk.LabelFrame(self.root, text="주행 속도 설정 (260mm : 120mm 비율 자동 연동)")
        spd.pack(fill="x", **pad)
        self.spd_label = ttk.Label(spd, text="")
        self.spd_label.pack(anchor="w", padx=8, pady=(4, 0))
        ttk.Scale(
            spd,
            from_=4.0,
            to=16.0,
            variable=self.front_limit_var,
            command=lambda *_: self._update_speed_label(),
        ).pack(fill="x", padx=8, pady=4)
        ttk.Checkbutton(
            spd,
            text="서스펜션 수평 유지(maintaining) 중일 때만 바퀴 주행 허용 (안전 연동)",
            variable=self.require_susp_var,
            command=self._focus_main,
        ).pack(anchor="w", padx=8, pady=(0, 4))
        self._update_speed_label()

        keys_frame = ttk.LabelFrame(self.root, text="키보드 조종 (W/A/S/D, C=직진복귀, Space=즉시정지)")
        keys_frame.pack(fill="x", **pad)
        grid = ttk.Frame(keys_frame)
        grid.pack(pady=6)

        self.key_labels: dict[str, tk.Label] = {}
        layout = [
            ("w", "W\n전진", 0, 1),
            ("a", "A\n좌회전", 1, 0),
            ("s", "S\n후진", 1, 1),
            ("d", "D\n우회전", 1, 2),
            ("c", "C\n직진복귀", 1, 3),
            ("space", "Space\n즉시정지", 1, 4),
        ]
        for k, text, r, c in layout:
            lbl = tk.Label(
                grid,
                text=text,
                width=9,
                height=2,
                relief="ridge",
                bd=2,
                bg="#e6e6e6",
                fg="#111111",
                font=("AppleGothic", 11, "bold"),
            )
            lbl.grid(row=r, column=c, padx=5, pady=3)
            self.key_labels[k] = lbl

        tilt_box = ttk.LabelFrame(self.root, text="실시간 차체 앞뒤 기울기 (IMU 기준: + 앞쪽 들림 / - 앞쪽 숙임)")
        tilt_box.pack(fill="x", **pad)
        tilt_inner = ttk.Frame(tilt_box)
        tilt_inner.pack(fill="x", padx=8, pady=4)
        tilt_text = ttk.Frame(tilt_inner)
        tilt_text.pack(side="left", fill="both", expand=True)
        self.tilt_main_lbl = ttk.Label(tilt_text, textvariable=self.tilt_main_var, font=("AppleGothic", 15, "bold"))
        self.tilt_main_lbl.pack(anchor="w", pady=(2, 2))
        ttk.Label(tilt_text, textvariable=self.tilt_sub_var, font=("Menlo", 10)).pack(anchor="w")
        self.tilt_canvas = tk.Canvas(tilt_inner, width=220, height=62, bg="#f7f7f7", highlightthickness=1, highlightbackground="#cccccc")
        self.tilt_canvas.pack(side="right", padx=(10, 0))
        self._draw_tilt_gauge(0.0, active=False)

        susp = ttk.LabelFrame(self.root, text="액티브 서스펜션 (리니어 액추에이터 + IMU 수평 유지)")
        susp.pack(fill="x", **pad)
        ttk.Label(susp, textvariable=self.susp_phase_var, font=("AppleGothic", 11, "bold")).pack(anchor="w", padx=8, pady=(4, 2))
        ttk.Label(susp, textvariable=self.susp_tel_var, font=("Menlo", 11)).pack(anchor="w", padx=8, pady=2)
        ttk.Label(susp, textvariable=self.susp_err_var, foreground="#c62828", font=("AppleGothic", 10, "bold")).pack(anchor="w", padx=8)

        susp_spd_row = ttk.Frame(susp)
        susp_spd_row.pack(fill="x", padx=8, pady=(3, 2))
        self.susp_spd_label = ttk.Label(susp_spd_row, text="", font=("AppleGothic", 10, "bold"))
        self.susp_spd_label.pack(anchor="w")
        susp_spd_ctrl = ttk.Frame(susp)
        susp_spd_ctrl.pack(fill="x", padx=8, pady=(0, 3))
        ttk.Scale(
            susp_spd_ctrl,
            from_=5.0,
            to=60.0,
            variable=self.susp_speed_var,
            command=lambda *_: self._on_susp_speed_change(),
        ).pack(side="left", fill="x", expand=True, padx=(0, 8))
        for val in (10, 20, 30, 40, 50, 60):
            ttk.Button(
                susp_spd_ctrl,
                text=f"{val}",
                width=3,
                command=lambda v=val: self._set_susp_speed_preset(v),
            ).pack(side="left", padx=1)
        self._update_susp_speed_label()

        cal_row = ttk.Frame(susp)
        cal_row.pack(fill="x", padx=8, pady=4)
        ttk.Label(cal_row, text="줄임여유(mm):").pack(side="left")
        ttk.Entry(cal_row, textvariable=self.retract_var, width=7).pack(side="left", padx=(3, 10))
        ttk.Label(cal_row, text="늘림여유(mm):").pack(side="left")
        ttk.Entry(cal_row, textvariable=self.extend_var, width=7).pack(side="left", padx=(3, 10))
        ttk.Label(cal_row, textvariable=self.sum_var, font=("AppleGothic", 10, "bold")).pack(side="left")

        chk_row = ttk.Frame(susp)
        chk_row.pack(fill="x", padx=8, pady=2)
        ttk.Checkbutton(chk_row, text="액추에이터 전원 ON 확인", variable=self.motor_power_var).pack(side="left", padx=(0, 10))
        ttk.Radiobutton(chk_row, text="현재 앞뒤 기울기(°):", variable=self.cal_mode_var, value="measure").pack(side="left")
        ttk.Entry(chk_row, textvariable=self.current_tilt_var, width=6).pack(side="left", padx=(3, 4))
        ttk.Label(chk_row, text="(+들림 / -숙임)").pack(side="left", padx=(0, 8))
        ttk.Radiobutton(chk_row, text="기존 기준 유지", variable=self.cal_mode_var, value="saved").pack(side="left")

        susp_btns = ttk.Frame(susp)
        susp_btns.pack(fill="x", padx=8, pady=(4, 6))
        self.cal_btn = ttk.Button(susp_btns, text="캘리브레이션 적용", command=self.send_susp_calibrate, state="disabled")
        self.cal_btn.pack(side="left", padx=(0, 6))
        self.susp_start_btn = ttk.Button(susp_btns, text="수평 유지 시작", command=lambda: self._send_susp_cmd("susp_restart"), state="disabled")
        self.susp_start_btn.pack(side="left", padx=(0, 6))
        self.susp_stop_btn = ttk.Button(susp_btns, text="서스펜션 정지", command=lambda: self._send_susp_cmd("susp_stop"), state="disabled")
        self.susp_stop_btn.pack(side="left", padx=(0, 6))
        self.susp_inv_btn = ttk.Button(susp_btns, text="영점 재설정", command=lambda: self._send_susp_cmd("susp_invalidate"), state="disabled")
        self.susp_inv_btn.pack(side="left")

        tel = ttk.LabelFrame(self.root, text="다이나믹셀 4모터 실시간 상태 (2000000 bps)")
        tel.pack(fill="both", expand=True, **pad)
        ttk.Label(tel, textvariable=self.cmd_var, font=("Menlo", 11, "bold")).pack(anchor="w", padx=8, pady=(4, 3))
        for mid in ("1", "2", "3", "4"):
            ttk.Label(tel, textvariable=self.motor_vars[mid], font=("Menlo", 10)).pack(anchor="w", padx=8, pady=1)

    def _focus_main(self):
        self.root.focus_set()

    def _update_speed_label(self):
        f_rpm = round(self.front_limit_var.get(), 1)
        r_rpm = round(f_rpm * RATIO_REAR_TO_FRONT, 1)
        self.spd_label.config(text=f"앞바퀴(260mm): {f_rpm:.1f} RPM   |   뒷바퀴(120mm): {r_rpm:.1f} RPM (모터 상한 38.2 RPM)")

    def _set_susp_speed_preset(self, val: float):
        self._focus_main()
        self.susp_speed_var.set(float(val))
        self._on_susp_speed_change()

    def _on_susp_speed_change(self):
        self._update_susp_speed_label()
        self._send_susp_speed_if_needed()

    def _update_susp_speed_label(self):
        sel = round(self.susp_speed_var.get(), 1)
        eff = min(sel, self.fw_max_speed)
        if sel > self.fw_max_speed + 0.05:
            hint = f"실제 제한: {eff:.0f} mm/s (상단 '고속 펌웨어 업로드' 누르면 {sel:.0f} mm/s 적용)"
        else:
            hint = f"실제 적용: {eff:.0f} mm/s (0.8° 초과 시 고속 작동, 0.8° 이하 기존 정밀 모드)"
        self.susp_spd_label.config(text=f"서스펜션 속도 선택: {sel:.0f} mm/s  |  {hint}")

    def _send_susp_speed_if_needed(self, force: bool = False):
        sel = round(self.susp_speed_var.get(), 1)
        if self.running and (force or self.last_sent_susp_speed != sel):
            self.last_sent_susp_speed = sel
            self._write_json({"type": "susp_speed", "max_speed_mm_s": sel})

    def _update_sum_label(self):
        try:
            total = float(self.retract_var.get()) + float(self.extend_var.get())
            ok = abs(total - 70.0) <= 0.02
            self.sum_var.set(f"합계: {total:.1f} mm ({'정상' if ok else '70.0 mm 맞춰야 함'})")
        except ValueError:
            self.sum_var.set("합계: 숫자 입력 필요")

    def _bind_keys(self):
        self.root.bind_all("<KeyPress>", self._on_key_press)
        self.root.bind_all("<KeyRelease>", self._on_key_release)
        self.root.bind("<FocusOut>", lambda *_: self._clear_keys())

    def _resolve_key(self, event) -> str | None:
        ks = (event.keysym or "").lower()
        ch = (event.char or "").lower()
        if ks in ("escape", "q") or ch in ("q", "ㅂ"):
            return "__quit__"
        if ks in KEY_MAP:
            return KEY_MAP[ks]
        if ch in KEY_MAP:
            return KEY_MAP[ch]
        return None

    def _on_key_press(self, event):
        if isinstance(event.widget, (tk.Entry, ttk.Entry)):
            return
        key = self._resolve_key(event)
        if key == "__quit__":
            self.stop_worker()
            return
        if not key:
            return
        timer_id = self.release_timers.pop(key, None)
        if timer_id:
            self.root.after_cancel(timer_id)
        if key not in self.pressed_keys:
            self.pressed_keys.add(key)
            self._refresh_key_boxes()
            self._send_input_now()

    def _on_key_release(self, event):
        if isinstance(event.widget, (tk.Entry, ttk.Entry)):
            return
        key = self._resolve_key(event)
        if not key or key == "__quit__":
            return
        old = self.release_timers.pop(key, None)
        if old:
            self.root.after_cancel(old)
        self.release_timers[key] = self.root.after(25, lambda k=key: self._finish_release(k))

    def _finish_release(self, key: str):
        self.release_timers.pop(key, None)
        if key in self.pressed_keys:
            self.pressed_keys.discard(key)
            self._refresh_key_boxes()
            self._send_input_now()

    def _clear_keys(self):
        for tid in list(self.release_timers.values()):
            self.root.after_cancel(tid)
        self.release_timers.clear()
        self.pressed_keys.clear()
        self._refresh_key_boxes()
        self._send_input_now()

    def _refresh_key_boxes(self):
        for k, lbl in self.key_labels.items():
            active = k in self.pressed_keys
            if k == "space":
                lbl.config(bg="#e53935" if active else "#e6e6e6", fg="#ffffff" if active else "#111111")
            else:
                lbl.config(bg="#1976d2" if active else "#e6e6e6", fg="#ffffff" if active else "#111111")

    def _set_susp_buttons_enabled(self, enabled: bool):
        st = "normal" if enabled else "disabled"
        for b in (self.cal_btn, self.susp_start_btn, self.susp_stop_btn, self.susp_inv_btn):
            b.config(state=st)

    def start_worker(self):
        if self.running or self.flashing:
            return
        self.status_var.set("Jetson 연결 및 모터/서스펜션 초기화 중...")
        self.start_btn.config(state="disabled")
        self.flash_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        self._set_susp_buttons_enabled(True)
        self.running = True
        self.last_sent_susp_speed = None
        self.proc = subprocess.Popen(
            ["ssh", "-o", "BatchMode=yes", SSH_HOST, f"python3 -u {REMOTE_SCRIPT}"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        threading.Thread(target=self._reader_thread, args=(self.proc.stdout,), daemon=True).start()
        threading.Thread(target=self._stderr_thread, args=(self.proc.stderr,), daemon=True).start()
        self.root.focus_set()

    def flash_firmware(self):
        if self.running or self.flashing:
            return
        self.flashing = True
        self.start_btn.config(state="disabled")
        self.flash_btn.config(state="disabled")
        self.status_var.set("Arduino 고속 펌웨어(최대 60 mm/s) 업로드 중...")
        self.susp_err_var.set("")

        def worker():
            res = subprocess.run(
                ["ssh", "-o", "BatchMode=yes", SSH_HOST, "python3 ~/three_wheel_teleop_20261127/upload_firmware.py"],
                capture_output=True,
                text=True,
            )
            self.rx_queue.put({
                "type": "flash_done",
                "returncode": res.returncode,
                "stdout": res.stdout.strip(),
                "stderr": res.stderr.strip(),
            })

        threading.Thread(target=worker, daemon=True).start()

    def _reader_thread(self, stream):
        try:
            for line in stream:
                line = line.strip()
                if not line:
                    continue
                try:
                    self.rx_queue.put(json.loads(line))
                except json.JSONDecodeError:
                    self.rx_queue.put({"type": "error", "message": line})
        finally:
            self.rx_queue.put({"type": "eof"})

    def _stderr_thread(self, stream):
        for line in stream:
            line = line.strip()
            if line:
                self.rx_queue.put({"type": "stderr", "message": line})

    def stop_worker(self):
        self._clear_keys()
        if self.proc and self.proc.poll() is None:
            try:
                if self.proc.stdin:
                    self.proc.stdin.write(json.dumps({"type": "quit"}) + "\n")
                    self.proc.stdin.flush()
            except Exception:
                pass
            try:
                self.proc.wait(timeout=3.0)
            except Exception:
                self.proc.terminate()
        self.running = False
        self.start_btn.config(state="normal")
        self.flash_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self._set_susp_buttons_enabled(False)
        self.status_var.set("정지됨 (모터 토크 해제 및 서스펜션 정지 완료)")

    def _write_json(self, payload: dict):
        if self.running and self.proc and self.proc.poll() is None and self.proc.stdin:
            try:
                self.proc.stdin.write(json.dumps(payload) + "\n")
                self.proc.stdin.flush()
            except Exception:
                pass

    def _send_input_now(self):
        self._write_json({
            "type": "input",
            "keys": sorted(self.pressed_keys),
            "front_rpm_limit": round(self.front_limit_var.get(), 2),
            "require_susp_for_drive": bool(self.require_susp_var.get()),
        })

    def send_susp_calibrate(self):
        self._focus_main()
        try:
            ret = float(self.retract_var.get())
            ext = float(self.extend_var.get())
        except ValueError:
            self.susp_err_var.set("입력 오류: 줄임여유/늘림여유에 숫자를 입력하세요.")
            return
        payload = {
            "type": "susp_calibrate",
            "retraction_mm": ret,
            "extension_mm": ext,
            "motor_power_on_confirmed": bool(self.motor_power_var.get()),
            "physically_level_confirmed": False,
        }
        if self.cal_mode_var.get() == "measure":
            try:
                tilt = float(self.current_tilt_var.get())
            except ValueError:
                self.susp_err_var.set("입력 오류: 현재 앞뒤 기울기에 숫자를 입력하세요.")
                return
            payload["current_tilt_deg"] = tilt
        self.susp_err_var.set("")
        self._write_json(payload)

    def _send_susp_cmd(self, cmd_type: str):
        self._focus_main()
        self.susp_err_var.set("")
        self._write_json({"type": cmd_type})

    def _tick_tx(self):
        self._send_input_now()
        if not self.closing:
            self.root.after(50, self._tick_tx)

    def _draw_tilt_gauge(self, tilt_deg: float, active: bool = True):
        c = self.tilt_canvas
        c.delete("all")
        w, h = 220, 62
        cx, cy = w // 2, h // 2
        # Horizontal 0-deg reference line
        c.create_line(18, cy, w - 18, cy, fill="#bbbbbb", dash=(3, 3))
        c.create_text(14, cy, text="뒤", font=("AppleGothic", 9), fill="#666666")
        c.create_text(w - 14, cy, text="앞", font=("AppleGothic", 9), fill="#666666")
        if not active:
            c.create_line(cx - 72, cy, cx + 72, cy, width=4, fill="#aaaaaa")
            c.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill="#888888", outline="")
            return
        # Exaggerate angle slightly (x2.5, clamped to +/-30 deg visually) for clarity
        vis_deg = max(-30.0, min(30.0, tilt_deg * 2.5))
        rad = math.radians(vis_deg)
        r = 72
        dx = r * math.cos(rad)
        dy = -r * math.sin(rad)  # positive tilt = front (right side) up -> smaller y
        if abs(tilt_deg) <= 0.2:
            color = "#2e7d32"
        elif abs(tilt_deg) <= 5.0:
            color = "#ef6c00"
        else:
            color = "#c62828"
        c.create_line(cx - dx, cy - dy, cx + dx, cy + dy, width=5, fill=color, capstyle="round")
        c.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill="#333333", outline="")
        c.create_oval(cx + dx - 4, cy + dy - 4, cx + dx + 4, cy + dy + 4, fill=color, outline="")

    def _update_susp_ui(self, s: dict):
        phase = s.get("phase", "unknown")
        reason = s.get("pause_reason") or ""
        phase_map = {
            "connecting": "연결 중 (Arduino/IMU 초기화)",
            "calibration_required": "캘리브레이션 필요 (줄임/늘림여유 입력 후 적용)",
            "paused": "준비/정지됨 (수평 유지 시작 가능)",
            "maintaining": "수평 유지 가동 중 (ACTIVE)",
            "pausing": "정지 중...",
            "exited": "서스펜션 프로세스 종료됨",
        }
        label = phase_map.get(phase, phase)
        self.susp_phase_var.set(f"서스펜션 상태: {label}" + (f" — {reason}" if reason else ""))

        imu = s.get("imu")
        board = s.get("board")
        ref_roll = s.get("reference_roll")
        err_deg = s.get("error_deg")

        if isinstance(imu, list) and len(imu) >= 2:
            tilt_deg = err_deg if err_deg is not None else ((imu[0] - ref_roll) if ref_roll is not None else imu[0])
            if abs(tilt_deg) <= 0.2:
                dir_txt = "수평"
                fg = "#2e7d32"
            elif tilt_deg > 0:
                dir_txt = "앞쪽 들림"
                fg = "#ef6c00" if abs(tilt_deg) <= 5.0 else "#c62828"
            else:
                dir_txt = "앞쪽 숙임"
                fg = "#1565c0" if abs(tilt_deg) <= 5.0 else "#c62828"
            self.tilt_main_var.set(f"{dir_txt}  {tilt_deg:+.2f}°")
            self.tilt_main_lbl.config(foreground=fg)
            ref_txt = f"{ref_roll:+.2f}°" if ref_roll is not None else "-"
            self.tilt_sub_var.set(f"IMU Roll(앞뒤)={imu[0]:+.2f}°  |  수평기준={ref_txt}  |  IMU Pitch(좌우)={imu[1]:+.2f}°")
            self._draw_tilt_gauge(tilt_deg, active=True)
        else:
            self.tilt_main_var.set("앞뒤 기울기: IMU 수신 대기 중")
            self.tilt_main_lbl.config(foreground="#333333")
            self.tilt_sub_var.set("IMU Roll(앞뒤)= -  |  수평 기준= -  |  IMU Pitch(좌우)= -")
            self._draw_tilt_gauge(0.0, active=False)

        fw_cap = s.get("fw_max_speed_mm_s")
        if isinstance(fw_cap, (int, float)) and fw_cap > 0 and abs(float(fw_cap) - self.fw_max_speed) > 0.05:
            self.fw_max_speed = float(fw_cap)
            self._update_susp_speed_label()

        if isinstance(board, dict):
            pos_mm = board.get("pos", 0) / 160.0
            vel = s.get("requested_velocity_mm_s", 0.0)
            board_str = f"액추에이터 위치={pos_mm:+.2f}mm ({board.get('pos', 0)}p, ref={board.get('ref', 0)})  |  명령속도={vel:+.1f}mm/s"
        else:
            board_str = "액추에이터: 연결 대기 중"
        self.susp_tel_var.set(board_str)

        cal_err = s.get("calibration_error") or s.get("stop_reason") or ""
        self.susp_err_var.set(cal_err)

    def _tick_rx(self):
        while True:
            try:
                msg = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            mtype = msg.get("type")
            if mtype == "ready":
                self.status_var.set("활성화 완료! W/S 전후진 · A/D 조향 · C 직진복귀 · Space 정지")
                self._send_susp_speed_if_needed(force=True)
            elif mtype == "flash_done":
                self.flashing = False
                if not self.running:
                    self.start_btn.config(state="normal")
                    self.flash_btn.config(state="normal")
                if msg.get("returncode") == 0:
                    self.fw_max_speed = 60.0
                    self._update_susp_speed_label()
                    self.status_var.set("Arduino 고속 펌웨어(60mm/s) 업로드 완료! 이제 '연결 및 모터 활성화'를 누르세요")
                else:
                    err = msg.get("stderr") or msg.get("stdout") or "업로드 실패"
                    self.status_var.set("Arduino 펌웨어 업로드 실패")
                    self.susp_err_var.set(err.splitlines()[-1][:120])
            elif mtype == "state":
                self.cmd_var.set(
                    f"앞 목표: {msg['cmd_front_rpm']:+.2f} RPM  |  뒤 목표: {msg['cmd_rear_rpm']:+.2f} RPM  |  조향: {msg['steer_deg']:+.1f}° (goal {msg['steer_goal']})"
                )
                names = {
                    "1": "ID 1 (앞 주행 260mm)",
                    "2": "ID 2 (앞 조향)",
                    "3": "ID 3 (우 뒤 120mm)",
                    "4": "ID 4 (좌 뒤 120mm)",
                }
                for mid, info in msg.get("motors", {}).items():
                    self.motor_vars[mid].set(
                        f"{names[mid]:20s} | pos={info['pos']:5d} | {info['rpm']:+6.2f} RPM | {info['cur_mA']:+6.0f} mA | {info['volt_V']:.1f}V | {info['temp_C']}C"
                    )
                if "suspension" in msg:
                    self._update_susp_ui(msg["suspension"])
            elif mtype in ("error", "stderr"):
                self.status_var.set(f"오류/중단: {msg.get('message', '')}")
                self.running = False
                self.start_btn.config(state="normal")
                self.flash_btn.config(state="normal")
                self.stop_btn.config(state="disabled")
                self._set_susp_buttons_enabled(False)
            elif mtype in ("stopped", "eof"):
                if self.running:
                    self.running = False
                    self.start_btn.config(state="normal")
                    self.flash_btn.config(state="normal")
                    self.stop_btn.config(state="disabled")
                    self._set_susp_buttons_enabled(False)
                    if "오류" not in self.status_var.get():
                        self.status_var.set("연결 종료됨 (모터 토크 해제됨)")
        if not self.closing:
            self.root.after(50, self._tick_rx)

    def on_close(self):
        self.closing = True
        self.stop_worker()
        self.root.destroy()


def main():
    root = tk.Tk()
    TeleopApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
