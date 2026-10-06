"""Durable flight-inhibit record. Clearing it is a caller-authorized ground action."""
import json
import os
from pathlib import Path
import time


class VerticalFaultLatch:
    def __init__(self, path):
        self.path = Path(path).expanduser().absolute()
        self.latched = False
        self.reason = ""
        self.latched_wall_s = None
        self.storage_error = None
        self.session_active = False
        try:
            if self.path.exists():
                data = json.loads(self.path.read_text(encoding="utf-8"))
                if (not isinstance(data, dict) or data.get("version") != 1
                        or type(data.get("latched")) is not bool):
                    raise ValueError("invalid vertical latch record")
                self.latched = data["latched"]
                self.reason = str(data.get("reason", ""))
                self.latched_wall_s = data.get("latched_wall_s")
                self.session_active = data.get("session_active", False)
                if type(self.session_active) is not bool:
                    raise ValueError("invalid active session record")
                if self.session_active and not self.latched:
                    self.latched = True
                    self.reason = "UNFINISHED_AUTOMATIC_SESSION_RESTART"
            else:
                self._write()
        except (OSError, ValueError, TypeError) as exc:
            self.latched = True
            self.reason = "VERTICAL_LATCH_STORAGE_UNAVAILABLE"
            self.storage_error = type(exc).__name__

    def _write(self):
        # The single executor writer lock serializes this atomic replacement.
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(self.path.name + ".tmp")
        data = dict(version=1, latched=self.latched, reason=self.reason,
                    latched_wall_s=self.latched_wall_s, session_active=self.session_active)
        with temporary.open("w", encoding="utf-8") as handle:
            json.dump(data, handle, allow_nan=False)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, self.path)
        if os.name != "nt":
            descriptor = os.open(str(self.path.parent), os.O_RDONLY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)

    def trip(self, reason):
        if self.latched:
            return
        self.latched, self.reason, self.latched_wall_s = True, str(reason), time.time()
        try:
            self._write()
            self.storage_error = None
        except OSError as exc:
            # Runtime inhibit remains true; status makes persistence failure explicit.
            self.storage_error = type(exc).__name__

    def begin_session(self):
        """Durably inhibit an unclean restart BEFORE the first automatic write."""
        if self.latched:
            return False
        if self.session_active:
            return True
        self.session_active = True
        try:
            self._write()
            self.storage_error = None
            return True
        except OSError as exc:
            self.latched = True
            self.reason = "VERTICAL_SESSION_PERSISTENCE_FAILED"
            self.storage_error = type(exc).__name__
            return False

    def end_ground_session(self):
        """Caller must establish continuous fresh healthy/disarmed ground evidence."""
        if self.latched:
            return False
        if not self.session_active:
            return True
        return self.clear()

    def clear(self):
        previous = self.reason, self.latched_wall_s, self.session_active
        self.latched, self.reason, self.latched_wall_s = False, "", None
        self.session_active = False
        try:
            self._write()
            self.storage_error = None
            return True
        except OSError as exc:
            self.latched = True
            self.reason, self.latched_wall_s, self.session_active = previous
            self.storage_error = type(exc).__name__
            try:
                self._write()
            except OSError:
                pass
            return False
