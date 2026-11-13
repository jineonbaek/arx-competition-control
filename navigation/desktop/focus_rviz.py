"""Activate the existing RViz window in the NUC's X11 desktop."""
import ctypes
import subprocess
import sys


def main():
    root = subprocess.check_output(["xprop", "-root", "_NET_CLIENT_LIST"], text=True)
    window = None
    for entry in root.partition("#")[2].replace(",", " ").split():
        owner = subprocess.check_output(["xprop", "-id", entry, "_NET_WM_PID"], text=True)
        if owner.rpartition("=")[2].strip() in sys.argv[1:]:
            window = int(entry, 16)
            break
    if window is None:
        return 1

    class ClientMessage(ctypes.Structure):
        _fields_ = [("type", ctypes.c_int), ("serial", ctypes.c_ulong),
                    ("send_event", ctypes.c_int), ("display", ctypes.c_void_p),
                    ("window", ctypes.c_ulong), ("message_type", ctypes.c_ulong),
                    ("format", ctypes.c_int), ("data", ctypes.c_long * 5)]

    class Event(ctypes.Union):
        _fields_ = [("message", ClientMessage), ("pad", ctypes.c_long * 24)]

    x11 = ctypes.CDLL("libX11.so.6")
    x11.XOpenDisplay.restype = ctypes.c_void_p
    display = x11.XOpenDisplay(None)
    if not display:
        return 1
    d = ctypes.c_void_p(display)
    x11.XDefaultRootWindow.restype = ctypes.c_ulong
    x11.XInternAtom.restype = ctypes.c_ulong
    event = Event()
    event.message = ClientMessage(33, 0, 1, display, window,
                                  x11.XInternAtom(d, b"_NET_ACTIVE_WINDOW", 0),
                                  32, (ctypes.c_long * 5)(2, 0, 0, 0, 0))
    target = ctypes.c_ulong(x11.XDefaultRootWindow(d))
    x11.XSendEvent(d, target, 0, ctypes.c_long((1 << 19) | (1 << 20)), ctypes.byref(event))
    x11.XCloseDisplay(d)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
