from pathlib import Path
import subprocess,sys,time,json,ctypes
R=Path(__file__).resolve().parent
def alive(pid):
    kernel=ctypes.windll.kernel32
    kernel.OpenProcess.restype=ctypes.c_void_p
    kernel.GetExitCodeProcess.argtypes=[ctypes.c_void_p,ctypes.POINTER(ctypes.c_ulong)]
    kernel.CloseHandle.argtypes=[ctypes.c_void_p]
    handle=kernel.OpenProcess(0x1000,False,int(pid))
    if not handle:return False
    code=ctypes.c_ulong();ok=kernel.GetExitCodeProcess(handle,ctypes.byref(code));kernel.CloseHandle(handle)
    return bool(ok and code.value==259)
try:state=json.loads((R/'state.json').read_text())
except (OSError,ValueError):state={}
if not alive(state.get('pid',0)):
    subprocess.Popen([sys.executable,str(R/'run.py')],cwd=R,creationflags=subprocess.CREATE_NO_WINDOW)
    time.sleep(3)
subprocess.Popen([sys.executable,str(R/'live_angle.py')],cwd=R,creationflags=subprocess.CREATE_NO_WINDOW)
