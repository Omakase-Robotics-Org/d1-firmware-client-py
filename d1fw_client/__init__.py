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
from typing import Any, ClassVar, Literal, Sequence
from urllib.parse import urlsplit

from .spec import SPEC_PATH, spec_document, spec_operations, spec_version

Side = Literal["a", "b"]
Grip = Literal["soft", "firm", "strong"]
HandUnit = Literal["frac", "wire"]
Joints7 = tuple[float, float, float, float, float, float, float]


class FirmwareError(RuntimeError):
    """HTTP or daemon envelope failure, with the original server message."""

    def __init__(self, method: str, path: str, status: int, message: str):
        self.method, self.path, self.status = method, path, status
        super().__init__(f"{method} {path} (HTTP {status}): {message}")


class ProtocolError(RuntimeError):
    """The server did not return the advertised response shape."""


class DeviceUnavailable(FirmwareError):
    """The daemon answered, but could not read that device.

    A state slot never fails the whole response: `/v1/state` reads six devices
    concurrently and `/v1/slider/state` is one of its slots, so a device the
    daemon could not reach degrades to ``{"error": "..."}`` in its own slot
    while the request still succeeds. That is a device failure reported
    correctly, not a server that broke its contract, so it is raised as a
    `FirmwareError` carrying the daemon's own explanation -- the same class a
    caller already catches for a device failure on any other verb -- rather
    than as a `ProtocolError` about a missing field.
    """


def _reject_degraded_slot(value: Any, endpoint: str) -> None:
    """Raise `DeviceUnavailable` when a slot is the degraded error form."""
    if (isinstance(value, dict) and set(value) == {"error"}
            and isinstance(value["error"], str)):
        raise DeviceUnavailable("GET", endpoint, 200, value["error"])


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

    endpoint: ClassVar[str] = "/v1/arm/{side}/state"

    @classmethod
    def parse(cls, value: Any) -> ArmState:
        _reject_degraded_slot(value, cls.endpoint)
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

    endpoint: ClassVar[str] = "/v1/gripper/{side}/state"

    @classmethod
    def parse(cls, value: Any) -> GripperState:
        _reject_degraded_slot(value, cls.endpoint)
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


@dataclass(frozen=True)
class HandState:
    """One sample of a dexterous hand.

    ``positions`` is per axis in fractions of that axis's range; the axis
    ORDER is the model's own and is published by ``hand_capabilities``, so a
    consumer reads it rather than hardcoding a per-model list. ``live`` is
    false when the daemon answered from its last sample because the hand's
    bus was busy with a motion command — the same meaning it has on
    ``GripperState``.
    """
    model: str
    side: Side
    positions: tuple[float, ...]
    positions_wire: tuple[int, ...]
    enabled: tuple[bool, ...]
    all_enabled: bool
    error_code: int
    faults: tuple[str, ...]
    live: bool
    features: tuple[str, ...]

    @classmethod
    def parse(cls, value: Any) -> HandState:
        try:
            positions = tuple(float(v) for v in value["positions"])
            if not all(math.isfinite(v) for v in positions):
                raise ValueError("non-finite hand position")
            wire = tuple(int(v) for v in value["positions_wire"])
            enabled = tuple(value["enabled"])
            if not all(type(flag) is bool for flag in enabled):
                raise ValueError("enabled must be a list of booleans")
            if len(positions) != len(wire) or len(positions) != len(enabled):
                raise ValueError("hand arrays disagree on the axis count")
            if type(value["all_enabled"]) is not bool or type(value["live"]) is not bool:
                raise ValueError("all_enabled and live must be boolean")
            if type(value["error_code"]) is not int:
                raise ValueError("error_code must be an integer")
            return cls(model=str(value["model"]), side=side_name(value["side"]),
                       positions=positions, positions_wire=wire, enabled=enabled,
                       all_enabled=value["all_enabled"], error_code=value["error_code"],
                       faults=tuple(str(f) for f in value["faults"]),
                       live=value["live"],
                       features=tuple(str(f) for f in value["features"]))
        except (KeyError, TypeError, ValueError) as exc:
            raise ProtocolError(f"invalid hand state: {exc}") from exc


@dataclass(frozen=True)
class NeckState:
    """`GET /v1/neck/state`: pitch/yaw in radians (positive pitch = look up,
    positive yaw = turn right), velocities in rad/s, torques in N·m."""
    pitch: float
    yaw: float
    pitch_velocity: float
    yaw_velocity: float
    pitch_torque: float
    yaw_torque: float
    enabled: bool

    endpoint: ClassVar[str] = "/v1/neck/state"

    @classmethod
    def parse(cls, value: Any) -> NeckState:
        _reject_degraded_slot(value, cls.endpoint)
        try:
            fields = {key: float(value[key]) for key in (
                "pitch", "yaw", "pitch_velocity", "yaw_velocity", "pitch_torque", "yaw_torque")}
            if not all(math.isfinite(v) for v in fields.values()):
                raise ValueError("non-finite neck reading")
            if type(value["enabled"]) is not bool:
                raise ValueError("enabled must be boolean")
            return cls(enabled=value["enabled"], **fields)
        except (KeyError, TypeError, ValueError) as exc:
            raise ProtocolError(f"invalid neck state: {exc}") from exc


@dataclass(frozen=True)
class SliderState:
    """`GET /v1/slider/state`: the torso lift. `height_m` is metres above the
    lower stop."""
    comms_ok: bool
    height_m: float
    moving: bool
    alarm: bool
    alarm_text: str | None = None

    endpoint: ClassVar[str] = "/v1/slider/state"

    @classmethod
    def parse(cls, value: Any) -> SliderState:
        _reject_degraded_slot(value, cls.endpoint)
        try:
            height = float(value["height_m"])
            if not math.isfinite(height):
                raise ValueError("non-finite height")
            for key in ("comms_ok", "moving", "alarm"):
                if type(value[key]) is not bool:
                    raise ValueError(f"{key} must be boolean")
            text = value.get("alarm_text")
            return cls(comms_ok=value["comms_ok"], height_m=height, moving=value["moving"],
                       alarm=value["alarm"], alarm_text=None if text is None else str(text))
        except (KeyError, TypeError, ValueError) as exc:
            raise ProtocolError(f"invalid slider state: {exc}") from exc


@dataclass(frozen=True)
class EyesState:
    """`GET /v1/eyes/state`."""
    mode: str
    initialized: bool
    last_effect: str | None = None

    endpoint: ClassVar[str] = "/v1/eyes/state"

    @classmethod
    def parse(cls, value: Any) -> EyesState:
        _reject_degraded_slot(value, cls.endpoint)
        try:
            if not isinstance(value["mode"], str) or type(value["initialized"]) is not bool:
                raise ValueError("mode must be a string and initialized a boolean")
            effect = value.get("last_effect")
            return cls(mode=value["mode"], initialized=value["initialized"],
                       last_effect=None if effect is None else str(effect))
        except (KeyError, TypeError, ValueError) as exc:
            raise ProtocolError(f"invalid eyes state: {exc}") from exc


EyeTarget = Literal["both", "left", "right"]
ConversationState = Literal["standby", "starting", "conversing"]


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

    def _send(self, method: str, path: str, body: Any = None) -> tuple[int, bytes]:
        """One request on the locked connection. No envelope, no retry."""
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
                return response.status, response.read()
            except (OSError, http.client.HTTPException):
                self._connection.close()
                raise

    def request(self, method: str, path: str, body: Any = None) -> Any:
        status, raw = self._send(method, path, body)
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

    def daemon_spec_version(self) -> str:
        """``info.version`` of the OpenAPI document THIS daemon serves.

        ``GET /openapi.json`` is the one route outside the `/v1` surface and
        the one response that is NOT enveloped -- it is the document itself --
        so it does not go through :meth:`request`. Compare the result with
        :func:`spec_version`, the version this client was built against.
        """
        status, raw = self._send("GET", "/openapi.json")
        try:
            document = json.loads(raw)
            return str(document["info"]["version"])
        except (ValueError, UnicodeError, KeyError, TypeError) as exc:
            raise ProtocolError(
                f"GET /openapi.json: HTTP {status}, not an OpenAPI document") from exc

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

    def hand_set(self, side: str, axes: Sequence[float], *,
                 unit: HandUnit = "frac") -> None:
        """Command every axis of a dexterous hand.

        ``axes`` is one value per axis in the model's own order (see
        ``hand_capabilities()["axis_names"]``). ``unit="frac"`` is 0.0-1.0 of
        each axis's range; ``unit="wire"`` is the raw vendor unit (0-255 on
        the LinkerHand O30, 0-10000 on the Leadshine DH116S). There is no
        degree unit: the O30's per-joint range of motion is unpublished and
        the direction of zero differs per joint type.
        """
        if unit not in ("frac", "wire"):
            raise ValueError("unit must be frac or wire")
        values = [float(v) for v in axes]
        if not values or not all(math.isfinite(v) for v in values):
            raise ValueError("axes must be a non-empty list of finite numbers")
        if unit == "frac" and not all(0.0 <= v <= 1.0 for v in values):
            raise ValueError("fractional axis values must be within [0, 1]")
        self.request("POST", f"/v1/hand/{side_name(side)}/set",
                     {"axes": values, "unit": unit})

    def hand_enable(self, side: str) -> None:
        self.request("POST", f"/v1/hand/{side_name(side)}/enable")

    def hand_disable(self, side: str) -> None:
        """Disable the joints. This does NOT open the hand: both models hold
        their pose with the motors off, so opening would drop what is held."""
        self.request("POST", f"/v1/hand/{side_name(side)}/disable")

    def hand_open(self, side: str) -> None:
        self.request("POST", f"/v1/hand/{side_name(side)}/open")

    def hand_fist(self, side: str) -> None:
        self.request("POST", f"/v1/hand/{side_name(side)}/fist")

    def hand_state(self, side: str) -> HandState:
        return HandState.parse(self.request("GET", f"/v1/hand/{side_name(side)}/state"))

    def hand_capabilities(self, side: str) -> dict[str, Any]:
        """Axis names and order, wire range, presets and feature tokens.

        Negotiate on the ``features`` tokens rather than on a version string:
        a token is a promise that something works, and tokens are added but
        never renamed or removed.
        """
        return self.request("GET", f"/v1/hand/{side_name(side)}/capabilities")

    def hand_release_soft_kill(self) -> None:
        self.request("POST", "/v1/hand/release_soft_kill")

    # -- neck (pan/tilt) ------------------------------------------------------- #
    def neck_state(self) -> NeckState:
        return NeckState.parse(self.request("GET", "/v1/neck/state"))

    def neck_cmd(self, *, pitch: float | None = None, yaw: float | None = None,
                 relative: bool = False, velocity: float | None = None) -> None:
        """Pose (or delta with ``relative=True``) in radians; ``velocity`` in rad/s
        (None = the daemon's default)."""
        body: dict[str, Any] = {"relative": bool(relative)}
        for name, value in (("pitch", pitch), ("yaw", yaw), ("velocity", velocity)):
            if value is not None:
                value = float(value)
                if not math.isfinite(value):
                    raise ValueError(f"neck {name} must be finite")
                body[name] = value
        if "pitch" not in body and "yaw" not in body:
            raise ValueError("neck_cmd needs pitch and/or yaw")
        self.request("POST", "/v1/neck/cmd", body)

    def neck_enable(self) -> None:
        self.request("POST", "/v1/neck/enable", {})

    def neck_disable(self) -> None:
        self.request("POST", "/v1/neck/disable", {})

    def neck_home(self) -> None:
        self.request("POST", "/v1/neck/home", {})

    def neck_set_zero(self) -> None:
        self.request("POST", "/v1/neck/set_zero", {})

    # -- eyes (LED strips) ------------------------------------------------------ #
    def eyes_state(self) -> EyesState:
        return EyesState.parse(self.request("GET", "/v1/eyes/state"))

    def eyes_effect(self, target: EyeTarget, effect: str, **params: Any) -> None:
        """One LED effect on ``target`` (both/left/right): ``effect`` is the
        daemon's tag (breathe, running, solid, clear, stop, ...) and ``params``
        its fields (r, g, b, step, delay_ms, ...)."""
        if target not in ("both", "left", "right"):
            raise ValueError("target must be both, left or right")
        self.request("POST", "/v1/eyes/cmd", {"target": target, "effect": effect, **params})

    def eyes_set_expression(self, name: str, *, speed: float | None = None,
                            loops: int | None = None) -> None:
        body: dict[str, Any] = {"name": str(name)}
        if speed is not None:
            body["speed"] = float(speed)
        if loops is not None:
            body["loops"] = int(loops)
        self.request("POST", "/v1/eyes/set_expression", body)

    def eyes_list_expressions(self) -> Any:
        return self.request("GET", "/v1/eyes/list_expressions")

    def eyes_conversation_state(self, state: ConversationState) -> None:
        state = str(state).lower()
        if state not in ("standby", "starting", "conversing"):
            raise ValueError("conversation state must be standby, starting or conversing")
        self.request("POST", "/v1/eyes/conversation_state", {"state": state})

    def eyes_battery_level(self, percent: float | None) -> None:
        self.request("POST", "/v1/eyes/battery_level",
                     {"percent": None if percent is None else float(percent)})

    def eyes_pixel(self, target: EyeTarget, index: int, r: int, g: int, b: int, *,
                   delay_ms: int | None = None) -> None:
        if target not in ("left", "right"):
            raise ValueError("pixel target must be left or right")
        body: dict[str, Any] = {"target": target, "index": int(index), "r": int(r), "g": int(g), "b": int(b)}
        if delay_ms is not None:
            body["delay_ms"] = int(delay_ms)
        self.request("POST", "/v1/eyes/pixel", body)

    # -- slider (torso lift) ---------------------------------------------------- #
    def slider_state(self) -> SliderState:
        return SliderState.parse(self.request("GET", "/v1/slider/state"))

    def slider_set_height(self, height_m: float, *, wait: bool = False) -> None:
        """Absolute height in metres. ``wait=True`` returns after the daemon
        reports arrival — construct the client with a timeout that covers the
        travel (the lift moves ~0.3 m in a few seconds)."""
        height_m = float(height_m)
        if not math.isfinite(height_m) or height_m < 0.0:
            raise ValueError("height_m must be finite and non-negative")
        self.request("POST", "/v1/slider/set_height", {"height_m": height_m, "wait": bool(wait)})

    def slider_stop(self) -> None:
        self.request("POST", "/v1/slider/stop", {})

    def slider_home(self) -> None:
        self.request("POST", "/v1/slider/home", {})

    def slider_reset_alarm(self) -> None:
        self.request("POST", "/v1/slider/reset_alarm", {})

    def close(self) -> None:
        with self._lock:
            self._closed = True
            self._connection.close()

    def __enter__(self) -> FirmwareClient:
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()
