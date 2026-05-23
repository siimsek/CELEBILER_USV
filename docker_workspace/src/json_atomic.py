"""Atomic JSON file I/O for inter-process shared state files."""

from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path
from typing import Any


def atomic_write_json(
    path: os.PathLike | str,
    payload: Any,
    *,
    indent: int | None = None,
    ensure_ascii: bool = False,
    sort_keys: bool = False,
) -> bool:
    """Write JSON via temp file, flush, fsync, and atomic replace.

    Returns True on success. On failure the previous file (if any) is preserved.
    """
    target = Path(path)
    directory = target.parent
    try:
        directory.mkdir(parents=True, exist_ok=True)
    except OSError:
        return False

    tmp_path: str | None = None
    fd: int | None = None
    try:
        fd, tmp_path = tempfile.mkstemp(
            dir=str(directory),
            prefix=f".tmp_{target.stem}_",
            suffix=".json",
        )
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            fd = None
            json.dump(
                payload,
                handle,
                indent=indent,
                ensure_ascii=ensure_ascii,
                sort_keys=sort_keys,
            )
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp_path, target)
        tmp_path = None
        return True
    except Exception:
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass
        if tmp_path:
            try:
                os.remove(tmp_path)
            except OSError:
                pass
        return False


def atomic_read_json(path: os.PathLike | str, default: Any = None) -> Any:
    """Read JSON file; return default when missing or invalid."""
    target = Path(path)
    if not target.is_file():
        return default
    try:
        with open(target, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError, json.JSONDecodeError):
        return default
