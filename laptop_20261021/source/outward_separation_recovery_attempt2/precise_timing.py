"""Process-owned high-resolution waiting; no global timer resolution changes."""
import ctypes
import math
import os
import time

class TimingError(RuntimeError): pass

class PreciseSleeper:
    def __init__(self):
        self.handle=None
        if os.name!='nt':
            self.backend='time.sleep';return
        from ctypes import wintypes
        self.kernel=ctypes.WinDLL('kernel32',use_last_error=True)
        self.kernel.CreateWaitableTimerExW.argtypes=[ctypes.c_void_p,wintypes.LPCWSTR,wintypes.DWORD,wintypes.DWORD]
        self.kernel.CreateWaitableTimerExW.restype=wintypes.HANDLE
        self.kernel.SetWaitableTimer.argtypes=[wintypes.HANDLE,ctypes.POINTER(ctypes.c_longlong),wintypes.LONG,ctypes.c_void_p,ctypes.c_void_p,wintypes.BOOL]
        self.kernel.SetWaitableTimer.restype=wintypes.BOOL
        self.kernel.WaitForSingleObject.argtypes=[wintypes.HANDLE,wintypes.DWORD]
        self.kernel.WaitForSingleObject.restype=wintypes.DWORD
        self.kernel.CloseHandle.argtypes=[wintypes.HANDLE]
        self.handle=self.kernel.CreateWaitableTimerExW(None,None,0x2,0x1F0003)
        if not self.handle:raise TimingError('High resolution waitable timer unavailable: '+str(ctypes.get_last_error()))
        self.backend='CREATE_WAITABLE_TIMER_HIGH_RESOLUTION'

    def __call__(self,seconds):
        if not math.isfinite(seconds) or seconds<0:raise TimingError('Invalid wait duration')
        if seconds==0:return
        if self.handle is None:
            time.sleep(seconds);return
        due=ctypes.c_longlong(-max(1,math.ceil(seconds*10_000_000)))
        if not self.kernel.SetWaitableTimer(self.handle,ctypes.byref(due),0,None,None,False):
            raise TimingError('SetWaitableTimer failed')
        if self.kernel.WaitForSingleObject(self.handle,math.ceil(seconds*1000)+100)!=0:
            raise TimingError('Waitable timer did not signal')

    def close(self):
        if self.handle:
            self.kernel.CloseHandle(self.handle);self.handle=None

    def __del__(self):self.close()
