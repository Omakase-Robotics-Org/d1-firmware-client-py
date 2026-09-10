"""Nonblocking latest-target gripper channel over independent REST connections."""
from __future__ import annotations
import math
import threading
import time
from . import FirmwareClient, Grip, GripperState, side_name

class FirmwareGripper:
    """Continuous latest-target worker, with independently polled feedback.

    No retry after a failed/lost stroke response. The failure is raised to the
    next caller. close/release discard queued commands and join the worker;
    an already accepted daemon stroke can finish, but no new stroke follows.
    """
    def __init__(self, base_url: str, side: str, grip: Grip | None = None):
        """``grip`` is the hold-force preset sent with every closing target
        (soft/firm/strong, the daemon's per-robot presets); None leaves the
        daemon's configured default preload."""
        if grip not in (None, "soft", "firm", "strong"):
            raise ValueError("grip must be soft, firm, strong or None")
        self.side = side_name(side)
        self.grip: Grip | None = grip
        self._command = FirmwareClient(base_url, timeout=40)
        self._reader = FirmwareClient(base_url, timeout=1)
        self._condition = threading.Condition()
        self._io_lock = threading.Lock()
        self._stop = threading.Event()
        self._desired = None
        self._last_sent: float | None = None
        self._busy = False
        self._error = None
        self._report: GripperState | None = None
        self._sample_at = 0.0
        self._command_thread = threading.Thread(target=self._run, daemon=True)
        self._read_thread = threading.Thread(target=self._poll, daemon=True)
        self._command_thread.start()
        self._read_thread.start()

    def set_target(self, closedness: float, *, force: bool = False) -> None:
        """Queue a closedness target (0 open .. 1 closed).

        A repeat of the target already sent (and not superseded) is a no-op —
        a caller that re-asserts "closed" every tick must not re-stroke the
        daemon every tick. ``force=True`` sends it anyway: a deliberate
        re-stroke, such as re-opening after a stroke came back ``blind``.
        """
        if not math.isfinite(closedness) or not 0 <= closedness <= 1:
            raise ValueError("closedness must be finite in [0, 1]")
        with self._condition:
            if self._stop.is_set():
                raise RuntimeError("gripper is closed")
            self.check_error()
            target = float(closedness)
            if (not force and self._desired is None and self._last_sent is not None
                    and abs(target - self._last_sent) < 1e-9):
                return
            self._desired = target
            self._condition.notify()

    def check_error(self) -> None:
        if self._error is not None:
            raise RuntimeError(f"firmware gripper {self.side} failed") from self._error

    def _run(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._stop.is_set() or self._desired is not None)
                if self._stop.is_set():
                    return
                target, self._desired = self._desired, None
                self._busy = True
            try:
                with self._io_lock:
                    if self._stop.is_set():
                        return
                    self._command.gripper_set(self.side, target, grip=self.grip)
                with self._condition:
                    self._last_sent = target
            except Exception as exc:
                with self._condition:
                    self._error = exc
                    self._desired = None
                return
            finally:
                with self._condition:
                    self._busy = False
                    self._condition.notify_all()

    def wait_idle(self) -> None:
        with self._condition:
            if not self._condition.wait_for(
                    lambda: not self._busy and self._desired is None, timeout=45):
                raise TimeoutError("firmware gripper did not finish")
            self.check_error()

    def _poll(self) -> None:
        while not self._stop.is_set():
            try:
                with self._io_lock:
                    if self._stop.is_set():
                        return
                    report = self._reader.gripper_state(self.side)
                self._absorb(report)
            except Exception:
                with self._condition:
                    self._report = None
            self._stop.wait(0.1)

    def refresh(self) -> GripperState | None:
        """One synchronous state read, so a caller that just finished a stroke
        sees THAT stroke's outcome rather than whatever the 10 Hz poll last
        stored. Returns None when the daemon could not be read."""
        try:
            with self._io_lock:
                report = self._reader.gripper_state(self.side)
        except Exception:
            return None
        self._absorb(report)
        return report

    def _absorb(self, report: GripperState) -> None:
        with self._condition:
            self._report = report
            # ``blind`` is a stroke that got no CAN feedback and swept
            # single-shot; on the D1 passthrough that is a dropped reply, not
            # a failed motor, so it is reported (kind + live=False) and left
            # for the caller to retry or reject.
            if report.kind in ("timeout", "fault", "lost"):
                self._error = RuntimeError(f"gripper stroke outcome: {report.kind}")
                self._desired = None
                self._condition.notify_all()
            # Busy stroke reports have no measurement timestamp. A
            # successful HTTP GET must not make old jaws look fresh.
            if report.live:
                self._sample_at = time.monotonic()

    def measured_rad(self) -> float | None:
        self.check_error()
        with self._condition:
            if (self._busy or self._desired is not None or self._report is None or not self._report.live
                    or time.monotonic() - self._sample_at > 0.5):
                return None
            return self._report.jaw_rad

    @property
    def last_outcome(self):
        return self._report

    def jaw_age_s(self) -> float | None:
        return None if not self._sample_at else time.monotonic() - self._sample_at

    def release(self) -> None:
        with self._condition:
            self._stop.set()
            self._desired = None
            self._condition.notify_all()
        self._command_thread.join()
        self._read_thread.join()
        self._command.close()
        self._reader.close()
