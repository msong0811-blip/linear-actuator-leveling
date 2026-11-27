#!/usr/bin/env python3
import subprocess
import sys
from pathlib import Path

ARDUINO_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0"
SKETCH_DIR = Path.home() / "three_wheel_teleop_20261127" / "firmware" / "ActuatorSweep"
CLI = Path.home() / ".local" / "bin" / "arduino-cli"

def main():
    chk = subprocess.run(["pgrep", "-f", "three_wheel_teleop_20261127/suspension/run.py"], capture_output=True, text=True)
    if chk.returncode == 0 and chk.stdout.strip():
        print("ERROR: suspension controller is currently running. Stop/disconnect first.", file=sys.stderr)
        return 2
    cmd = [
        str(CLI), "compile", "--upload",
        "-p", ARDUINO_PORT,
        "--fqbn", "arduino:avr:uno",
        str(SKETCH_DIR),
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.stdout:
        print(res.stdout)
    if res.stderr:
        print(res.stderr, file=sys.stderr)
    return res.returncode

if __name__ == "__main__":
    sys.exit(main())
