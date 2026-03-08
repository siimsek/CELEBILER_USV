import builtins
import os
import sys
import time


def _enable_windows_virtual_terminal():
    if os.name != "nt":
        return
    try:
        import ctypes

        kernel32 = ctypes.windll.kernel32
        enable_vt = 0x0004
        for std_handle in (-11, -12):  # STD_OUTPUT_HANDLE, STD_ERROR_HANDLE
            handle = kernel32.GetStdHandle(std_handle)
            if handle in (0, -1):
                continue
            mode = ctypes.c_uint()
            if kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
                kernel32.SetConsoleMode(handle, mode.value | enable_vt)
    except Exception:
        # ANSI acilamazsa normal cikisla devam et.
        pass


def setup_utf8_console():
    if os.name == "nt":
        try:
            os.system("chcp 65001 >NUL")
        except Exception:
            pass
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass
    _enable_windows_virtual_terminal()


def _pick_color(message):
    if "[ESTOP]" in message or "❌" in message or "🚨" in message:
        return "\x1b[91m"
    if "[WARN]" in message or "⚠️" in message:
        return "\x1b[93m"
    if "[OK]" in message or "✅" in message:
        return "\x1b[92m"
    if "[START]" in message or "[DONE]" in message:
        return "\x1b[96m"
    return "\x1b[37m"


def make_console_printer(component):
    setup_utf8_console()
    raw_print = builtins.print

    def console_print(*args, **kwargs):
        sep = kwargs.pop("sep", " ")
        end = kwargs.pop("end", "\n")
        file = kwargs.pop("file", sys.stdout)
        flush = kwargs.pop("flush", False)
        message = sep.join(str(arg) for arg in args)
        stamp = time.strftime("%H:%M:%S")
        prefix = f"[{stamp}] [{component}]"

        if os.name == "nt" and file in (sys.stdout, sys.stderr):
            color = _pick_color(message)
            raw_print(
                f"{color}{prefix} {message}\x1b[0m",
                end=end,
                file=file,
                flush=flush,
                **kwargs,
            )
            return

        raw_print(
            f"{prefix} {message}",
            end=end,
            file=file,
            flush=flush,
            **kwargs,
        )

    return console_print

