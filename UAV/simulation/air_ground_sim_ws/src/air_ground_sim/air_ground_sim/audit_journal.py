"""Small, dependency-free, append-only JSONL journal with bounded rotation."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import threading
from typing import Any, Mapping


SENSITIVE_KEYS = {
    "authorization",
    "auth_token",
    "password",
    "secret",
    "token",
}


def redact(value: Any) -> Any:
    """Recursively remove common credentials before an event reaches disk."""

    if isinstance(value, Mapping):
        return {
            str(key): "[REDACTED]" if str(key).lower() in SENSITIVE_KEYS else redact(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [redact(item) for item in value]
    return value


class AuditJournal:
    """Write one durable JSON object per line and rotate at a fixed size."""

    def __init__(
        self,
        path: str,
        *,
        max_bytes: int = 10 * 1024 * 1024,
        backup_count: int = 5,
        required: bool = False,
    ) -> None:
        self.path = Path(os.path.expandvars(os.path.expanduser(path))) if path else None
        self.max_bytes = max(int(max_bytes), 4096)
        self.backup_count = max(int(backup_count), 1)
        self.required = bool(required)
        self.lock = threading.RLock()
        self.available = False
        self.last_error = ""
        if self.path is not None:
            self._prepare()
        elif self.required:
            raise RuntimeError("an audit journal path is required")

    def _prepare(self) -> None:
        try:
            assert self.path is not None
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8"):
                pass
            self.available = True
        except OSError as error:
            self.last_error = str(error)
            if self.required:
                raise RuntimeError(f"unable to initialize audit journal: {error}") from error

    def _rotate(self) -> None:
        assert self.path is not None
        if not self.path.exists() or self.path.stat().st_size < self.max_bytes:
            return
        oldest = self.path.with_name(f"{self.path.name}.{self.backup_count}")
        if oldest.exists():
            oldest.unlink()
        for index in range(self.backup_count - 1, 0, -1):
            source = self.path.with_name(f"{self.path.name}.{index}")
            if source.exists():
                source.replace(self.path.with_name(f"{self.path.name}.{index + 1}"))
        self.path.replace(self.path.with_name(f"{self.path.name}.1"))

    def write(self, event: Mapping[str, Any]) -> bool:
        """Append an event. Return False only for an optional unavailable journal."""

        if self.path is None or not self.available:
            return False
        record = {
            "schema_version": "1.0",
            "recorded_at": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            **redact(dict(event)),
        }
        encoded = json.dumps(record, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        try:
            with self.lock:
                self._rotate()
                with self.path.open("a", encoding="utf-8", newline="\n") as stream:
                    stream.write(encoded + "\n")
                    stream.flush()
                    os.fsync(stream.fileno())
            return True
        except OSError as error:
            self.available = False
            self.last_error = str(error)
            if self.required:
                raise RuntimeError(f"audit journal write failed: {error}") from error
            return False

