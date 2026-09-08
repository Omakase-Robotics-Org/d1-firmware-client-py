"""Persistent REST access to the daemon. No vendor SDK or hardware sockets.

Each client serializes requests on one keep-alive connection. Use separate
clients for slow operations (recovery, gripper strokes) and the arm tick.
Mutations are never retried: a lost response does not mean the write failed.
Closing a client closes HTTP only; it does not stop or release the robot.
"""
from __future__ import annotations

from dataclasses import dataclass
import http.client
import json
import math
import socket
import threading
from typing import Any, Literal, Sequence
from urllib.parse import urlsplit

Side = Literal["a", "b"]
Grip = Literal["soft", "firm", "strong"]
Joints7 = tuple[float, float, float, float, float, float, float]


class FirmwareError(RuntimeError):
    """HTTP or daemon envelope failure, with the original server message."""

    def __init__(self, method: str, path: str, status: int, message: str):
        self.method, self.path, self.status = method, path, status
        super().__init__(f"{method} {path} (HTTP {status}): {message}")


class ProtocolError(RuntimeError):
    """The server did not return the advertised response shape."""


def side_name(side: str) -> Side:
    value = side.lower()
    if value not in ("a", "b"):
        raise ValueError("side must be a (left) or b (right)")
    return value


def joints7(values: Sequence[float]) -> Joints7:
    result = tuple(float(v) for v in values)
    if len(result) != 7 or not all(math.isfinite(v) for v in result):
        raise ValueError("values must contain exactly seven finite numbers")
    return result


@dataclass(frozen=True)
class ArmState:
    mode: str | dict[str, str]
    error_code: int
    feedback_joints: Joints7
    command_joints: Joints7
    feedback_velocity: Joints7
    feedback_torque: Joints7
    feedback_temperature: Joints7
    frame_serial: int
    stationary: bool

    @classmethod
    def parse(cls, value: Any) -> ArmState:
        try:
            arrays = {key: joints7(value[key]) for key in (
                "feedback_joints", "command_joints", "feedback_velocity",
                "feedback_torque", "feedback_temperature")}
            if not isinstance(value["stationary"], bool):
                raise ValueError("stationary must be boolean")
            mode = value["mode"]
            if not (mode in ("idle", "position", "pvt", "torque", "release", "error")
                    or isinstance(mode, dict) and set(mode) == {"unknown"} and isinstance(mode["unknown"], str)):
                raise ValueError("invalid arm mode")
            if type(value["error_code"]) is not int or type(value["frame_serial"]) is not int:
                raise ValueError("error_code and frame_serial must be integers")
            return cls(mode=mode, error_code=value["error_code"],
                       frame_serial=int(value["frame_serial"]),
                       stationary=value["stationary"], **arrays)
        except (KeyError, TypeError, ValueError) as exc:
            raise ProtocolError(f"invalid arm state: {exc}") from exc


@dataclass(frozen=True)
class GripperState:
    kind: str
    jaw_rad: float
    torque_nm: float
    holding: bool
    grip_preload_rad: float
    live: bool
    open_rad: float | None = None
    coil_c: int | None = None

    @classmethod
    def parse(cls, value: Any) -> GripperState:
        try:
            fields = {key: float(value[key]) for key in (
                "jaw_rad", "torque_nm", "grip_preload_rad")}
            if not all(math.isfinite(v) for v in fields.values()):
                raise ValueError("non-finite gripper reading")
            if not isinstance(value["kind"], str):
                raise ValueError("kind must be a string")
            if type(value["holding"]) is not bool or type(value.get("live", False)) is not bool:
                raise ValueError("holding and live must be boolean")
            for key in ("open_rad", "coil_c"):
                if value.get(key) is not None and not math.isfinite(value[key]):
                    raise ValueError(f"{key} must be finite when available")
            return cls(kind=value["kind"], holding=value["holding"],
                       live=value.get("live", False),
                       open_rad=value.get("open_rad"), coil_c=value.get("coil_c"),
                       **fields)
        except (KeyError, TypeError, ValueError) as exc:
            raise ProtocolError(f"invalid gripper state: {exc}") from exc


class FirmwareClient:
    def __init__(self, base_url: str = "http://127.0.0.1:4750", *,
                 timeout: float = 2.0):
        url = urlsplit(base_url)
        if (url.scheme not in ("http", "https") or not url.hostname
                or url.username or url.password or url.query or url.fragment
                or url.path not in ("", "/")):
            raise ValueError("base_url must be an http(s) origin without credentials")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be positive and finite")
        self.base_url = base_url.rstrip("/")
        connection = (http.client.HTTPSConnection if url.scheme == "https"
                      else http.client.HTTPConnection)
        self._connection = connection(url.hostname, url.port, timeout=timeout)
        self._lock = threading.Lock()
        self._closed = False

    def request(self, method: str, path: str, body: Any = None) -> Any:
        payload = None if body is None else json.dumps(body, allow_nan=False)
        with self._lock:
            if self._closed:
                raise RuntimeError("firmware client is closed")
            try:
                if self._connection.sock is None:
                    self._connection.connect()
                    self._connection.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self._connection.request(method, path, payload,
                                         {"Content-Type": "application/json"})
                response = self._connection.getresponse()
                raw = response.read()
                status = response.status
            except (OSError, http.client.HTTPException):
                self._connection.close()
                raise
            try:
                envelope = json.loads(raw)
            except (ValueError, UnicodeError) as exc:
                raise ProtocolError(f"{method} {path}: HTTP {status}, invalid JSON") from exc
            if not isinstance(envelope, dict) or envelope.get("status") not in ("ok", "error"):
                raise ProtocolError(f"{method} {path}: invalid envelope")
            if not 200 <= status < 300 or envelope["status"] != "ok":
                raise FirmwareError(method, path, status, str(envelope.get("message")))
            if "data" not in envelope:
                raise ProtocolError(f"{method} {path}: missing data")
            return envelope["data"]

    def arm_state(self, side: str) -> ArmState:
        return ArmState.parse(self.request("GET", f"/v1/arm/{side_name(side)}/state"))

    def move_joints_both(self, a: Sequence[float], b: Sequence[float], *,
                         wait: bool = False) -> None:
        self.request("POST", "/v1/arm/move_joints_both",
                     {"a": joints7(a), "b": joints7(b), "wait": wait})

    def arm_mode(self, side: str, mode: str, **parameters: Any) -> None:
        self.request("POST", f"/v1/arm/{side_name(side)}/mode",
                     {"mode": mode, **parameters})

    def preflight(self, side: str) -> dict[str, Any]:
        return self.request("GET", f"/v1/arm/{side_name(side)}/preflight")

    def recover(self, side: str, **parameters: Any) -> dict[str, Any]:
        """Explicit recovery; construct this client with timeout >= 65 seconds."""
        return self.request("POST", f"/v1/arm/{side_name(side)}/recover", parameters)

    def gripper_set(self, side: str, closedness: float, *, grip: Grip | None = None) -> None:
        if not math.isfinite(closedness) or not 0 <= closedness <= 1:
            raise ValueError("closedness must be finite in [0, 1]")
        if grip not in (None, "soft", "firm", "strong"):
            raise ValueError("grip must be soft, firm, or strong")
        body = {"closedness": closedness}
        if grip is not None:
            body["grip"] = grip
        self.request("POST", f"/v1/gripper/{side_name(side)}/set", body)

    def gripper_state(self, side: str) -> GripperState:
        return GripperState.parse(self.request("GET", f"/v1/gripper/{side_name(side)}/state"))

    def close(self) -> None:
        with self._lock:
            self._closed = True
            self._connection.close()

    def __enter__(self) -> FirmwareClient:
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()
