"""
Merkezi debug dosya loglama: LOG_DIR / SIM_LOG_DIR altinda {component}.debug.log.
Ayrica makine-okunur olaylar: {component}.jsonl (satir basina bir JSON, AI/grep icin).
Simulasyonda opsiyonel fonksiyon trace: giris/cikis/hata ve sure bilgisi.

Cikti yonlendirmesi (tee) ile birlikte calisir; tekrarlayan handler eklenmez.
"""
from __future__ import annotations

import functools
import inspect
import json
import logging
import os
import sys
import threading
import time
import traceback
from itertools import count
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any, Optional

_DEFAULT_BYTES = int(os.environ.get("USV_DEBUG_LOG_MAX_BYTES", str(50 * 1024 * 1024)))
_DEFAULT_BACKUP = int(os.environ.get("USV_DEBUG_LOG_BACKUPS", "10"))
_JSONL_BYTES = int(os.environ.get("USV_JSONL_MAX_BYTES", str(100 * 1024 * 1024)))
_JSONL_BACKUP = int(os.environ.get("USV_JSONL_BACKUPS", "15"))
_TRACE_SLOW_MS = max(0.0, float(os.environ.get("USV_TRACE_SLOW_MS", "75")))
_TRACE_MAX_ARG_CHARS = max(32, int(os.environ.get("USV_TRACE_MAX_ARG_CHARS", "160")))
_TRACE_MAX_RETURN_CHARS = max(32, int(os.environ.get("USV_TRACE_MAX_RETURN_CHARS", "160")))
_TRACE_JSONL_EVERY_CALL = os.environ.get("USV_TRACE_JSONL_EVERY_CALL", "0").strip().lower() in ("1", "true", "yes")
_TRACE_INCLUDE_PRIVATE_DEFAULT = os.environ.get("USV_TRACE_INCLUDE_PRIVATE", "1").strip().lower() in ("1", "true", "yes")

_jsonl_loggers: dict[str, logging.Logger] = {}
_jsonl_lock = threading.Lock()
_trace_call_counter = count(1)


def _project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _env_truthy(raw: str) -> bool:
    return raw.strip().lower() in ("1", "true", "yes", "on")


def _trace_enabled(prefer_simulation: bool = False) -> bool:
    raw = os.environ.get("USV_TRACE_FUNCTIONS", "").strip()
    if raw:
        return _env_truthy(raw)
    return bool(prefer_simulation or os.environ.get("USV_SIM") == "1")


def _compact_repr(value: Any, max_chars: int) -> str:
    try:
        text = repr(value)
    except Exception:
        text = f"<{type(value).__name__}>"
    text = " ".join(text.split())
    if len(text) > max_chars:
        return f"{text[: max_chars - 3]}..."
    return text


def _format_arguments(signature: Optional[inspect.Signature], args: tuple[Any, ...], kwargs: dict[str, Any]) -> str:
    if signature is not None:
        try:
            bound = signature.bind_partial(*args, **kwargs)
            parts = []
            for idx, (name, value) in enumerate(bound.arguments.items()):
                if idx >= 8:
                    parts.append("...")
                    break
                if name == "self":
                    parts.append(f"{name}=<{type(value).__name__}>")
                elif name == "cls":
                    parts.append(f"{name}=<{getattr(value, '__name__', type(value).__name__)}>")
                else:
                    parts.append(f"{name}={_compact_repr(value, _TRACE_MAX_ARG_CHARS)}")
            return ", ".join(parts) if parts else "-"
        except Exception:
            pass
    parts = []
    if args:
        parts.append(f"args={_compact_repr(args, _TRACE_MAX_ARG_CHARS)}")
    if kwargs:
        parts.append(f"kwargs={_compact_repr(kwargs, _TRACE_MAX_ARG_CHARS)}")
    return ", ".join(parts) if parts else "-"


def _trace_name_allowed(name: str, include_private: bool) -> bool:
    if name == "__classcell__":
        return False
    if name.startswith("__") and name != "__init__":
        return False
    if not include_private and name.startswith("_") and name != "__init__":
        return False
    return True


def resolve_log_directory(prefer_simulation: bool = False) -> Path:
    """
    prefer_simulation=True: once SIM_LOG_DIR (sim kopruleri icin).
    False: once LOG_DIR (docker_workspace servisleri).
    """
    if prefer_simulation:
        order = ("SIM_LOG_DIR", "LOG_DIR", "USV_LOG_ROOT")
    else:
        order = ("LOG_DIR", "SIM_LOG_DIR", "USV_LOG_ROOT")
    for key in order:
        raw = os.environ.get(key, "").strip()
        if raw:
            p = Path(raw)
            p.mkdir(parents=True, exist_ok=True)
            return p
    fallback = _project_root() / "logs" / ("simulation" if prefer_simulation else "system")
    fallback.mkdir(parents=True, exist_ok=True)
    return fallback


def setup_component_logger(
    component: str,
    *,
    prefer_simulation: bool = False,
    level: int = logging.DEBUG,
    log_dir: Optional[Path] = None,
) -> logging.Logger:
    """
    Tek bir RotatingFileHandler ile usv.{component} logger'i yapilandirir.
    """
    name = f"usv.{component}"
    logger = logging.getLogger(name)
    if logger.handlers:
        return logger

    base = log_dir if log_dir is not None else resolve_log_directory(prefer_simulation=prefer_simulation)
    base.mkdir(parents=True, exist_ok=True)
    path = base / f"{component}.debug.log"

    fh = RotatingFileHandler(
        path,
        maxBytes=_DEFAULT_BYTES,
        backupCount=_DEFAULT_BACKUP,
        encoding="utf-8",
    )
    fh.setLevel(level)
    fmt = logging.Formatter(
        "%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    fh.setFormatter(fmt)
    logger.setLevel(level)
    logger.addHandler(fh)
    logger.propagate = False

    logger.debug(
        "logger_started path=%s pid=%s argv=%s",
        path,
        os.getpid(),
        " ".join(sys.argv[:6]),
    )
    log_jsonl(
        component,
        prefer_simulation,
        event="logger_started",
        path=str(path),
        pid=os.getpid(),
        trace_enabled=_trace_enabled(prefer_simulation=prefer_simulation),
    )

    if os.environ.get("USV_LOG_STDERR", "").strip() in ("1", "true", "TRUE", "yes"):
        sh = logging.StreamHandler(sys.stderr)
        sh.setLevel(logging.INFO)
        sh.setFormatter(fmt)
        logger.addHandler(sh)

    return logger


def setup_jsonl_logger(
    component: str,
    *,
    prefer_simulation: bool = False,
    log_dir: Optional[Path] = None,
) -> logging.Logger:
    """
    Tek satir = tek JSON (.jsonl). Yuksek hacim; sorun izi icin.
    """
    name = f"usv.jsonl.{component}"
    with _jsonl_lock:
        if name in _jsonl_loggers:
            return _jsonl_loggers[name]

        base = log_dir if log_dir is not None else resolve_log_directory(prefer_simulation=prefer_simulation)
        base.mkdir(parents=True, exist_ok=True)
        path = base / f"{component}.jsonl"

        lg = logging.getLogger(name)
        lg.setLevel(logging.INFO)
        lg.handlers.clear()
        fh = RotatingFileHandler(
            path,
            maxBytes=_JSONL_BYTES,
            backupCount=_JSONL_BACKUP,
            encoding="utf-8",
        )
        fh.setFormatter(logging.Formatter("%(message)s"))
        fh.setLevel(logging.INFO)
        lg.addHandler(fh)
        lg.propagate = False
        _jsonl_loggers[name] = lg
        return lg


def log_jsonl(
    component: str,
    prefer_simulation: bool,
    *,
    event: str,
    **fields: Any,
) -> None:
    """Yapilandirilmis tek satir JSON (ts + event + alanlar). USV_LOG_JSONL=0 ile kapatilir."""
    if os.environ.get("USV_LOG_JSONL", "1").strip() in ("0", "false", "no"):
        return
    try:
        lg = setup_jsonl_logger(component, prefer_simulation=prefer_simulation)
        payload = {
            "ts": time.time(),
            "event": event,
            **fields,
        }
        lg.info(json.dumps(payload, ensure_ascii=False, default=str))
    except Exception:
        pass


def log_exception(
    logger: logging.Logger,
    msg: str,
    exc: BaseException,
    **extra: Any,
) -> None:
    """Dosyaya tam traceback + ek alanlar."""
    try:
        tb = traceback.format_exc()
    except Exception:
        tb = ""
    logger.error("%s | exc=%s | extra=%s\n%s", msg, repr(exc), extra, tb)


def log_debug_every(
    logger: logging.Logger,
    counter: list[int],
    every: int,
    fmt: str,
    *args: Any,
) -> None:
    """counter[0] += 1 sonra every mesajda bir debug."""
    counter[0] = counter[0] + 1
    if counter[0] <= 3 or counter[0] % every == 0:
        logger.debug(fmt, *args)


def trace_function_calls(
    func: Any,
    *,
    component: str,
    logger: logging.Logger,
    prefer_simulation: bool = False,
    qualname: Optional[str] = None,
) -> Any:
    """Fonksiyon/method cagrilarini debug.log'a ve opsiyonel olarak jsonl'a yazar."""
    if getattr(func, "__usv_trace_wrapped__", False):
        return func

    try:
        signature = inspect.signature(func)
    except (TypeError, ValueError):
        signature = None

    label = qualname or getattr(func, "__qualname__", getattr(func, "__name__", "callable"))

    @functools.wraps(func)
    def wrapped(*args: Any, **kwargs: Any) -> Any:
        call_id = next(_trace_call_counter)
        arg_text = _format_arguments(signature, args, kwargs)
        logger.debug("[TRACE] enter fn=%s call_id=%s args=%s", label, call_id, arg_text)
        if _TRACE_JSONL_EVERY_CALL:
            log_jsonl(
                component,
                prefer_simulation,
                event="function_enter",
                fn=label,
                call_id=call_id,
                args=arg_text,
            )
        started = time.monotonic()
        try:
            result = func(*args, **kwargs)
        except Exception as exc:
            dt_ms = round((time.monotonic() - started) * 1000.0, 3)
            logger.exception("[TRACE] error fn=%s call_id=%s dt_ms=%.3f", label, call_id, dt_ms)
            log_jsonl(
                component,
                prefer_simulation,
                event="function_error",
                fn=label,
                call_id=call_id,
                dt_ms=dt_ms,
                exc=repr(exc),
            )
            raise
        dt_ms = round((time.monotonic() - started) * 1000.0, 3)
        ret_text = _compact_repr(result, _TRACE_MAX_RETURN_CHARS)
        if dt_ms >= _TRACE_SLOW_MS:
            logger.info("[TRACE] exit fn=%s call_id=%s dt_ms=%.3f return=%s", label, call_id, dt_ms, ret_text)
            log_jsonl(
                component,
                prefer_simulation,
                event="function_slow",
                fn=label,
                call_id=call_id,
                dt_ms=dt_ms,
                return_value=ret_text,
            )
        else:
            logger.debug("[TRACE] exit fn=%s call_id=%s dt_ms=%.3f return=%s", label, call_id, dt_ms, ret_text)
            if _TRACE_JSONL_EVERY_CALL:
                log_jsonl(
                    component,
                    prefer_simulation,
                    event="function_exit",
                    fn=label,
                    call_id=call_id,
                    dt_ms=dt_ms,
                    return_value=ret_text,
                )
        return result

    setattr(wrapped, "__usv_trace_wrapped__", True)
    return wrapped


def _wrap_class_methods(
    cls: type[Any],
    *,
    component: str,
    logger: logging.Logger,
    prefer_simulation: bool,
    include_private: bool,
) -> int:
    wrapped_count = 0
    for attr_name, attr_value in list(vars(cls).items()):
        if not _trace_name_allowed(attr_name, include_private=include_private):
            continue
        descriptor_kind = None
        target = None
        if isinstance(attr_value, staticmethod):
            descriptor_kind = "staticmethod"
            target = attr_value.__func__
        elif isinstance(attr_value, classmethod):
            descriptor_kind = "classmethod"
            target = attr_value.__func__
        elif inspect.isfunction(attr_value):
            descriptor_kind = "function"
            target = attr_value
        if target is None or getattr(target, "__module__", None) != cls.__module__:
            continue
        wrapped = trace_function_calls(
            target,
            component=component,
            logger=logger,
            prefer_simulation=prefer_simulation,
            qualname=f"{cls.__name__}.{attr_name}",
        )
        if wrapped is target:
            continue
        if descriptor_kind == "staticmethod":
            setattr(cls, attr_name, staticmethod(wrapped))
        elif descriptor_kind == "classmethod":
            setattr(cls, attr_name, classmethod(wrapped))
        else:
            setattr(cls, attr_name, wrapped)
        wrapped_count += 1
    return wrapped_count


def install_module_function_tracing(
    module_globals: dict[str, Any],
    *,
    component: str,
    logger: Optional[logging.Logger] = None,
    prefer_simulation: bool = False,
    include_private: Optional[bool] = None,
) -> int:
    """Module-level fonksiyonlar ve sinif method'larini otomatik trace eder."""
    lg = logger or setup_component_logger(component, prefer_simulation=prefer_simulation)
    enabled = _trace_enabled(prefer_simulation=prefer_simulation)
    include_private = _TRACE_INCLUDE_PRIVATE_DEFAULT if include_private is None else include_private
    if not enabled:
        lg.info("function_trace disabled component=%s", component)
        log_jsonl(
            component,
            prefer_simulation,
            event="function_trace_init",
            enabled=False,
            include_private=bool(include_private),
        )
        return 0

    module_name = str(module_globals.get("__name__", ""))
    wrapped_count = 0
    replacements: dict[Any, Any] = {}
    for name, obj in list(module_globals.items()):
        if inspect.isfunction(obj) and getattr(obj, "__module__", None) == module_name:
            if not _trace_name_allowed(name, include_private=include_private):
                continue
            wrapped = trace_function_calls(
                obj,
                component=component,
                logger=lg,
                prefer_simulation=prefer_simulation,
                qualname=name,
            )
            if wrapped is obj:
                continue
            module_globals[name] = wrapped
            replacements[obj] = wrapped
            wrapped_count += 1
        elif inspect.isclass(obj) and getattr(obj, "__module__", None) == module_name:
            wrapped_count += _wrap_class_methods(
                obj,
                component=component,
                logger=lg,
                prefer_simulation=prefer_simulation,
                include_private=include_private,
            )

    for obj in module_globals.values():
        try:
            view_functions = getattr(obj, "view_functions", None)
        except Exception:
            continue
        if isinstance(view_functions, dict):
            for endpoint, fn in list(view_functions.items()):
                if fn in replacements:
                    view_functions[endpoint] = replacements[fn]

    lg.info(
        "function_trace enabled component=%s wrapped=%s include_private=%s slow_ms=%.1f jsonl_every=%s",
        component,
        wrapped_count,
        include_private,
        _TRACE_SLOW_MS,
        _TRACE_JSONL_EVERY_CALL,
    )
    log_jsonl(
        component,
        prefer_simulation,
        event="function_trace_init",
        enabled=True,
        wrapped=wrapped_count,
        include_private=bool(include_private),
        slow_ms=_TRACE_SLOW_MS,
        jsonl_every=bool(_TRACE_JSONL_EVERY_CALL),
    )
    return wrapped_count
