#!/usr/bin/env python3
import json
import math
import secrets
import select
import signal
import subprocess
import sys
import time
from pathlib import Path
from dynamixel_sdk import (
    COMM_SUCCESS,
    DXL_HIBYTE,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_LOWORD,
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)

BASE_DIR = Path(__file__).resolve().parent
SUSP_DIR = BASE_DIR / "suspension"

PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAKRM1V-if00-port0"
BAUD = 2000000

ID_FRONT_DRIVE = 1
ID_STEER = 2
ID_REAR_RIGHT = 3
ID_REAR_LEFT = 4

DRIVE_IDS = (ID_FRONT_DRIVE, ID_REAR_RIGHT, ID_REAR_LEFT)
ALL_IDS = (ID_FRONT_DRIVE, ID_STEER, ID_REAR_RIGHT, ID_REAR_LEFT)

FORWARD_SIGN = {
    ID_FRONT_DRIVE: -1,
    ID_REAR_RIGHT: -1,
    ID_REAR_LEFT: +1,
}

FRONT_DIAMETER_MM = 260.0
REAR_DIAMETER_MM = 120.0
REAR_TO_FRONT_RATIO = FRONT_DIAMETER_MM / REAR_DIAMETER_MM  # 2.166666...

STEER_CENTER = 3074
STEER_HALF_RANGE = round(85.0 * 4096.0 / 360.0)  # 967 ticks -> +/- 85 deg
STEER_MIN = max(0, STEER_CENTER - STEER_HALF_RANGE)
STEER_MAX = min(4095, STEER_CENTER + STEER_HALF_RANGE)
STEER_RATE_RPM = 20.0  # 120 deg/s

VEL_UNIT_RPM = 0.229
MAX_MOTOR_VEL_RAW = 160  # ~36.64 RPM (below 167 / 38.24 RPM hardware limit)


def s32(v):
    return v - 2**32 if v >= 2**31 else v


def s16(v):
    return v - 65536 if v >= 32768 else v


def u32_bytes(val):
    u = val & 0xFFFFFFFF
    return [
        DXL_LOBYTE(DXL_LOWORD(u)),
        DXL_HIBYTE(DXL_LOWORD(u)),
        DXL_LOBYTE(DXL_HIWORD(u)),
        DXL_HIBYTE(DXL_HIWORD(u)),
    ]


def emit(obj):
    sys.stdout.write(json.dumps(obj) + "\n")
    sys.stdout.flush()


def write_susp_request(name, payload):
    pending = SUSP_DIR / f"{name}.pending"
    target = SUSP_DIR / f"{name}.request"
    pending.write_text(json.dumps(payload))
    pending.replace(target)


def read_suspension(susp_proc):
    state_path = SUSP_DIR / "state.json"
    data = None
    try:
        data = json.loads(state_path.read_text())
    except (OSError, ValueError):
        data = None
    if susp_proc is not None and susp_proc.poll() is not None:
        if isinstance(data, dict):
            data["phase"] = "exited"
            if not data.get("pause_reason"):
                data["pause_reason"] = f"Suspension exited ({susp_proc.returncode})"
            return data
        return {
            "phase": "exited",
            "returncode": susp_proc.returncode,
            "pause_reason": f"Suspension process exited ({susp_proc.returncode})",
        }
    return data if isinstance(data, dict) else {"phase": "connecting", "pause_reason": "Connecting to Arduino/IMU..."}


def main():
    port = PortHandler(PORT)
    ph = PacketHandler(2.0)
    if not port.openPort() or not port.setBaudRate(BAUD):
        emit({"type": "error", "message": f"Cannot open {PORT} @ {BAUD}"})
        return 2
    if hasattr(port, "ser"):
        port.ser.exclusive = True

    def rd(i, addr, n):
        v, c, e = {1: ph.read1ByteTxRx, 2: ph.read2ByteTxRx, 4: ph.read4ByteTxRx}[n](port, i, addr)
        if c or e:
            raise RuntimeError(f"rd fault id={i} addr={addr} c={c} e={e}")
        if n == 4:
            return s32(v)
        if n == 2 and addr in (124, 126):
            return s16(v)
        return v

    def wr(i, addr, n, val):
        c, e = {1: ph.write1ByteTxRx, 4: ph.write4ByteTxRx}[n](
            port, i, addr, val & (0xFFFFFFFF if n == 4 else 0xFF)
        )
        if c or e:
            raise RuntimeError(f"wr fault id={i} addr={addr} c={c} e={e}")

    susp_proc = None
    susp_out = None
    susp_err = None

    def safe_shutdown():
        for i in DRIVE_IDS:
            try:
                ph.write4ByteTxRx(port, i, 104, 0)
            except Exception:
                pass
        for i in ALL_IDS:
            try:
                ph.write1ByteTxRx(port, i, 98, 0)
            except Exception:
                pass
            try:
                ph.write1ByteTxRx(port, i, 64, 0)
            except Exception:
                pass
        if susp_proc is not None and susp_proc.poll() is None:
            try:
                write_susp_request("STOP", {})
            except Exception:
                pass
            time.sleep(0.4)
            if susp_proc.poll() is None:
                susp_proc.terminate()
                try:
                    susp_proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    susp_proc.kill()
        for f in (susp_out, susp_err):
            if f:
                try:
                    f.close()
                except Exception:
                    pass

    stop_flag = False

    def handle_sig(*_):
        nonlocal stop_flag
        stop_flag = True

    signal.signal(signal.SIGINT, handle_sig)
    signal.signal(signal.SIGTERM, handle_sig)

    try:
        for i in ALL_IDS:
            model, c, e = ph.ping(port, i)
            if c != COMM_SUCCESS or e or model != 1101:
                raise RuntimeError(f"Motor id={i} missing or mismatch (model={model}, c={c}, e={e})")
            if rd(i, 70, 1) != 0:
                raise RuntimeError(f"Motor id={i} has hardware error")

        if rd(ID_FRONT_DRIVE, 11, 1) != 1 or rd(ID_REAR_RIGHT, 11, 1) != 1 or rd(ID_REAR_LEFT, 11, 1) != 1:
            raise RuntimeError("Drive motor not in velocity mode (1)")
        if rd(ID_STEER, 11, 1) != 3:
            raise RuntimeError("Steering motor not in position mode (3)")

        steer_pos = rd(ID_STEER, 132, 4)
        if not (STEER_MIN <= steer_pos <= STEER_MAX):
            raise RuntimeError(f"Steering pos {steer_pos} outside safe limits [{STEER_MIN}, {STEER_MAX}]")

        if "--check" in sys.argv:
            emit({"type": "check_ok", "steer_pos": steer_pos, "steer_deg": round((steer_pos - STEER_CENTER) * 360.0 / 4096.0, 2)})
            return 0

        # Launch suspension controller subprocess
        susp_out = (BASE_DIR / "suspension_stdout.log").open("ab")
        susp_err = (BASE_DIR / "suspension_stderr.log").open("ab")
        susp_proc = subprocess.Popen(
            [sys.executable, str(SUSP_DIR / "run.py")],
            cwd=SUSP_DIR,
            stdout=susp_out,
            stderr=susp_err,
        )

        # Arm all 4 Dynamixel motors cleanly
        for i in DRIVE_IDS:
            wr(i, 98, 1, 0)
            wr(i, 104, 4, 0)
            wr(i, 108, 4, 45)
            wr(i, 64, 1, 1)
            wr(i, 98, 1, 25)    # 500 ms bus watchdog

        wr(ID_STEER, 98, 1, 0)
        wr(ID_STEER, 108, 4, 40)
        wr(ID_STEER, 112, 4, 100)
        wr(ID_STEER, 116, 4, steer_pos)
        wr(ID_STEER, 64, 1, 1)
        wr(ID_STEER, 98, 1, 25)

        gsw_drive = GroupSyncWrite(port, ph, 104, 4)
        gsr_telem = GroupSyncRead(port, ph, 126, 21)
        gsr_err = GroupSyncRead(port, ph, 70, 1)
        for i in ALL_IDS:
            gsr_telem.addParam(i)
            gsr_err.addParam(i)

        emit({"type": "ready", "steer_center": STEER_CENTER, "steer_pos": steer_pos})

        keys = set()
        front_rpm_limit = 10.0
        require_susp_for_drive = False
        front_rpm = 0.0
        steering = float(steer_pos)
        last_cmd_time = time.monotonic()
        last_vel = {i: None for i in DRIVE_IDS}
        last_steer_goal = steer_pos
        last_watchdog_keepalive = time.monotonic()
        prev_t = time.monotonic()
        telem_t = 0.0
        susp_state = {"phase": "connecting"}

        while not stop_flag:
            now = time.monotonic()
            dt = min(0.1, max(0.001, now - prev_t))
            prev_t = now

            while True:
                rlist, _, _ = select.select([sys.stdin], [], [], 0)
                if not rlist:
                    break
                chunk = sys.stdin.readline()
                if chunk == "":
                    stop_flag = True
                    break
                buf = chunk.strip()
                if not buf:
                    continue
                msg = json.loads(buf)
                mtype = msg.get("type")
                if mtype == "quit":
                    stop_flag = True
                    break
                elif mtype == "input":
                    keys = set(msg.get("keys", []))
                    if "front_rpm_limit" in msg:
                        front_rpm_limit = max(3.0, min(16.0, float(msg["front_rpm_limit"])))
                    if "require_susp_for_drive" in msg:
                        require_susp_for_drive = bool(msg["require_susp_for_drive"])
                    last_cmd_time = now
                elif mtype == "susp_calibrate":
                    payload = {
                        "id": secrets.token_hex(12),
                        "retraction_mm": float(msg["retraction_mm"]),
                        "extension_mm": float(msg["extension_mm"]),
                        "motor_power_on_confirmed": bool(msg.get("motor_power_on_confirmed")),
                        "physically_level_confirmed": bool(msg.get("physically_level_confirmed")),
                    }
                    if msg.get("current_tilt_deg") is not None:
                        payload["current_tilt_deg"] = float(msg["current_tilt_deg"])
                    write_susp_request("CALIBRATE", payload)
                elif mtype == "susp_restart":
                    write_susp_request("RESTART", {"id": secrets.token_hex(12)})
                elif mtype == "susp_stop":
                    write_susp_request("STOP", {})
                elif mtype == "susp_invalidate":
                    write_susp_request("INVALIDATE", {})
                elif mtype == "susp_speed":
                    spd = max(5.0, min(60.0, float(msg["max_speed_mm_s"])))
                    write_susp_request("SPEED", {"max_speed_mm_s": spd})

            if stop_flag:
                break

            active_keys = set() if (now - last_cmd_time > 0.35) else keys

            if "space" in active_keys or (require_susp_for_drive and susp_state.get("phase") != "maintaining"):
                front_rpm = 0.0
            else:
                drive_dir = (1 if "w" in active_keys else 0) - (1 if "s" in active_keys else 0)
                desired_front = drive_dir * front_rpm_limit
                if desired_front == 0.0 or desired_front * front_rpm < 0:
                    front_rpm = 0.0
                else:
                    step_rpm = 80.0 * dt
                    front_rpm += max(-step_rpm, min(step_rpm, desired_front - front_rpm))

            turn_dir = (1 if "d" in active_keys else 0) - (1 if "a" in active_keys else 0)
            if "c" in active_keys:
                diff = STEER_CENTER - steering
                max_step = (4096.0 * STEER_RATE_RPM / 60.0) * dt
                if abs(diff) <= max_step:
                    steering = float(STEER_CENTER)
                else:
                    steering += math.copysign(max_step, diff)
            else:
                steering = max(
                    STEER_MIN,
                    min(STEER_MAX, steering + turn_dir * (4096.0 * STEER_RATE_RPM / 60.0) * dt),
                )

            rear_rpm = front_rpm * REAR_TO_FRONT_RATIO

            raw_cmds = {
                ID_FRONT_DRIVE: max(-MAX_MOTOR_VEL_RAW, min(MAX_MOTOR_VEL_RAW, round((front_rpm * FORWARD_SIGN[ID_FRONT_DRIVE]) / VEL_UNIT_RPM))),
                ID_REAR_RIGHT: max(-MAX_MOTOR_VEL_RAW, min(MAX_MOTOR_VEL_RAW, round((rear_rpm * FORWARD_SIGN[ID_REAR_RIGHT]) / VEL_UNIT_RPM))),
                ID_REAR_LEFT: max(-MAX_MOTOR_VEL_RAW, min(MAX_MOTOR_VEL_RAW, round((rear_rpm * FORWARD_SIGN[ID_REAR_LEFT]) / VEL_UNIT_RPM))),
            }

            drive_changed = any(raw_cmds[i] != last_vel[i] for i in DRIVE_IDS)
            if drive_changed or (now - last_watchdog_keepalive >= 0.15):
                gsw_drive.clearParam()
                for i in DRIVE_IDS:
                    gsw_drive.addParam(i, u32_bytes(raw_cmds[i]))
                gsw_drive.txPacket()
                last_vel.update(raw_cmds)

            steer_goal = round(steering)
            if steer_goal != last_steer_goal or (now - last_watchdog_keepalive >= 0.15):
                ph.write4ByteTxOnly(port, ID_STEER, 116, steer_goal & 0xFFFFFFFF)
                last_steer_goal = steer_goal
                last_watchdog_keepalive = now

            if now >= telem_t:
                susp_state = read_suspension(susp_proc)
                if gsr_err.txRxPacket() == COMM_SUCCESS:
                    for i in ALL_IDS:
                        if gsr_err.isAvailable(i, 70, 1) and gsr_err.getData(i, 70, 1) != 0:
                            raise RuntimeError(f"Hardware error on motor id={i}")
                if gsr_telem.txRxPacket() == COMM_SUCCESS:
                    motors = {}
                    for i in ALL_IDS:
                        if gsr_telem.isAvailable(i, 126, 21):
                            temp = gsr_telem.getData(i, 146, 1)
                            if temp >= 65:
                                raise RuntimeError(f"Motor id={i} reached {temp} C limit")
                            motors[str(i)] = {
                                "pos": s32(gsr_telem.getData(i, 132, 4)),
                                "rpm": round(s32(gsr_telem.getData(i, 128, 4)) * VEL_UNIT_RPM, 2),
                                "cur_mA": round(s16(gsr_telem.getData(i, 126, 2)) * 2.69, 0),
                                "volt_V": round(gsr_telem.getData(i, 144, 2) / 10.0, 1),
                                "temp_C": temp,
                            }
                    if len(motors) == 4:
                        emit({
                            "type": "state",
                            "cmd_front_rpm": round(front_rpm, 2),
                            "cmd_rear_rpm": round(rear_rpm, 2),
                            "steer_deg": round((steer_goal - STEER_CENTER) * 360.0 / 4096.0, 1),
                            "steer_goal": steer_goal,
                            "motors": motors,
                            "suspension": susp_state,
                        })
                telem_t = now + 0.20

            time.sleep(0.015)
    except Exception as exc:
        emit({"type": "error", "message": str(exc)})
        return 1
    finally:
        safe_shutdown()
        port.closePort()
        emit({"type": "stopped"})
    return 0


if __name__ == "__main__":
    sys.exit(main())
