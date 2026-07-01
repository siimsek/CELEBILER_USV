import builtins
import os
import sys
import time


# ANSI renk kodları
COLORS = {
    "RESET": "\x1b[0m",
    "RED": "\x1b[31m",
    "GREEN": "\x1b[32m",
    "YELLOW": "\x1b[33m",
    "BLUE": "\x1b[34m",
    "MAGENTA": "\x1b[35m",
    "CYAN": "\x1b[36m",
    "WHITE": "\x1b[37m",
    "BRIGHT_RED": "\x1b[91m",
    "BRIGHT_GREEN": "\x1b[92m",
    "BRIGHT_YELLOW": "\x1b[93m",
    "BRIGHT_BLUE": "\x1b[94m",
    "BRIGHT_MAGENTA": "\x1b[95m",
    "BRIGHT_CYAN": "\x1b[96m",
    "BRIGHT_WHITE": "\x1b[97m",
}

# Component bazlı renk eşlemesi
COMPONENT_COLORS = {
    "GAZEBO": COLORS["CYAN"],
    "ROS-GZ": COLORS["BLUE"],
    "SITL": COLORS["MAGENTA"],
    "MAVPROXY": COLORS["YELLOW"],
    "CAM": COLORS["GREEN"],
    "LIDAR": COLORS["BRIGHT_BLUE"],
    "USV": COLORS["BRIGHT_WHITE"],
    "TELEM": COLORS["BRIGHT_GREEN"],
    "TEST": COLORS["RED"],
    "COMPLIANCE": COLORS["RED"],
    "SIM": COLORS["CYAN"],
    "WARN": COLORS["BRIGHT_YELLOW"],
    "ERROR": COLORS["BRIGHT_RED"],
}


def _colors_enabled():
    """Renkli çıktı aktif mi kontrol et."""
    # NO_COLOR environment variable varsa renkleri kapat
    if os.environ.get("NO_COLOR", "").lower() in ("1", "true", "yes"):
        return False
    # CI ortamında renkleri kapat
    if os.environ.get("CI", "").lower() in ("1", "true", "yes"):
        return False
    # TTY değilse renkleri kapat
    if not hasattr(sys.stdout, "isatty") or not sys.stdout.isatty():
        return False
    return True


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
            stream.reconfigure(
                encoding="utf-8",
                errors="replace",
                line_buffering=True,
                write_through=True,
            )
        except TypeError:
            try:
                stream.reconfigure(encoding="utf-8", errors="replace")
            except Exception:
                pass
        except Exception:
            pass
    _enable_windows_virtual_terminal()


def _pick_color(component, message):
    """Mesaj içeriğine ve component'a göre renk seç."""
    if not _colors_enabled():
        return None

    # Önce mesaj içeriğine göre kontrol et
    if "[ESTOP]" in message or "❌" in message or "🚨" in message:
        return COLORS["BRIGHT_RED"]
    if "[WARN]" in message or "⚠️" in message:
        return COLORS["BRIGHT_YELLOW"]
    if "[OK]" in message or "✅" in message:
        return COLORS["BRIGHT_GREEN"]
    if "[START]" in message or "[DONE]" in message:
        return COLORS["CYAN"]

    # Sonra component bazlı renk
    return COMPONENT_COLORS.get(component, COLORS["WHITE"])


def make_console_printer(component):
    setup_utf8_console()
    raw_print = builtins.print

    def console_print(*args, **kwargs):
        sep = kwargs.pop("sep", " ")
        end = kwargs.pop("end", "\n")
        file = kwargs.pop("file", sys.stdout)
        flush = kwargs.pop("flush", True)
        message = sep.join(str(arg) for arg in args)
        stamp = time.strftime("%H:%M:%S")
        prefix = f"[{stamp}] [{component}]"

        # Renkli çıktı (sadece terminal için, dosya logları için değil)
        color = _pick_color(component, message)
        if color and file in (sys.stdout, sys.stderr) and _colors_enabled():
            raw_print(
                f"{color}{prefix} {message}{COLORS['RESET']}",
                end=end,
                file=file,
                flush=flush,
                **kwargs,
            )
            return

        # Renksiz çıktı (dosya logları veya NO_COLOR aktif)
        raw_print(
            f"{prefix} {message}",
            end=end,
            file=file,
            flush=flush,
            **kwargs,
        )

    return console_print
