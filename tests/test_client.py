"""Exercise actual HTTP framing, keepalive and mutation ambiguity."""
import json
import socket
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from d1fw_client import (ArmState, DeviceUnavailable, EyesState, FirmwareClient,
                         FirmwareError, GripperState, NeckState, ProtocolError, SliderState)


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):
        pass

    def do_GET(self):
        self.do_POST()

    def do_POST(self):
        body = self.rfile.read(int(self.headers.get("Content-Length", 0)))
        self.server.calls.append((self.path, body, self.client_address))
        if self.path == "/lost":
            self.connection.shutdown(socket.SHUT_RDWR)
            self.connection.close()
            return
        status, data = self.server.response
        payload = json.dumps(data).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


class ClientTests(unittest.TestCase):
    def setUp(self):
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.server.calls = []
        self.server.response = (200, {"status": "ok", "data": None, "message": None})
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.client = FirmwareClient(f"http://127.0.0.1:{self.server.server_port}")

    def tearDown(self):
        self.client.close()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def test_both_arms_one_request_and_keepalive(self):
        for _ in range(3):
            self.client.move_joints_both([1]*7, [2]*7)
        self.assertEqual(len(self.server.calls), 3)
        self.assertEqual(len({call[2] for call in self.server.calls}), 1)
        self.assertEqual(json.loads(self.server.calls[0][1]),
                         {"a": [1]*7, "b": [2]*7, "wait": False})

    def test_error_envelope_even_http_200(self):
        for status in (200, 409, 502, 504):
            self.server.response = (status, {"status": "error", "data": None,
                                            "message": "arm operation in progress"})
            with self.assertRaisesRegex(FirmwareError, "arm operation in progress"):
                self.client.move_joints_both([0]*7, [0]*7)

    def test_no_retry_after_lost_response(self):
        with self.assertRaises(Exception):
            self.client.request("POST", "/lost", {"execute": True})
        self.assertEqual(len(self.server.calls), 1)
        self.client.request("GET", "/next")
        self.assertEqual(len(self.server.calls), 2)

    def test_validate_before_sending(self):
        for values in ([0]*6, [float("nan")]*7, [float("inf")]*7):
            with self.assertRaises(ValueError):
                self.client.move_joints_both(values, [0]*7)
        with self.assertRaises(ValueError):
            self.client.gripper_set("a", 1.1)
        self.assertEqual(self.server.calls, [])

    def test_unreadable_device_slot_is_a_device_failure_not_a_protocol_error(self):
        """A state slot never fails the whole response: `/v1/slider/state` is
        one slot of the six-device snapshot, so a slider the daemon cannot
        reach comes back as `200 ok` with `{"error": ...}` in `data`. That is
        a device failure reported correctly, and it must read as one -- the
        old behaviour raised `ProtocolError("invalid slider state: 'height_m'")`,
        which sends the reader looking for a broken daemon instead of a
        disconnected lift."""
        self.server.response = (200, {"status": "ok", "message": None,
                                      "data": {"error": "device error: slider port closed"}})
        with self.assertRaises(DeviceUnavailable) as caught:
            self.client.slider_state()
        self.assertIn("device error: slider port closed", str(caught.exception))
        self.assertIn("/v1/slider/state", str(caught.exception))
        # It is a device failure, so it is catchable exactly like one on any
        # other verb rather than needing its own except clause.
        self.assertIsInstance(caught.exception, FirmwareError)
        self.assertNotIsInstance(caught.exception, ProtocolError)

        # Every state slot degrades the same way in `/v1/state`, so every
        # parser has to read it the same way.
        for parser in (ArmState, GripperState, NeckState, SliderState, EyesState):
            with self.assertRaises(DeviceUnavailable):
                parser.parse({"error": "device error: unreachable"})

        # A genuinely malformed payload is still a protocol error, and an
        # `error` key alongside real fields is not the degraded form.
        with self.assertRaises(ProtocolError):
            SliderState.parse({"error": 17})
        with self.assertRaises(ProtocolError):
            SliderState.parse({"error": "x", "height_m": 0.1})

    def test_bad_shape(self):
        self.server.response = (200, {"status": "ok", "data": {}})
        with self.assertRaises(ProtocolError):
            self.client.arm_state("a")

    def test_close_cannot_reopen(self):
        self.client.close()
        with self.assertRaises(RuntimeError):
            self.client.request("GET", "/v1/health")
        self.assertEqual(self.server.calls, [])


if __name__ == "__main__":
    unittest.main()


def test_gripper_repeat_target_is_not_resent():
    """A caller that re-asserts the same closedness every tick must not
    re-stroke the daemon every tick (teleop asserts the button state
    continuously)."""
    import threading, time
    from d1fw_client.gripper import FirmwareGripper
    from d1fw_client import GripperState

    class Client:
        sets = []
        def __init__(self, *a, **k): pass
        def gripper_set(self, side, value, grip=None): Client.sets.append(value)
        def gripper_state(self, side): return GripperState("open", 1.0, 0.0, False, 0.0, True, 1.35)
        def close(self): pass

    import d1fw_client.gripper as g
    saved = g.FirmwareClient
    g.FirmwareClient = Client
    try:
        w = FirmwareGripper("http://x", "b")
        for _ in range(5):
            w.set_target(1.0)
            w.wait_idle()
        w.set_target(0.0); w.wait_idle()
        w.set_target(1.0); w.wait_idle()
        assert Client.sets == [1.0, 0.0, 1.0]
        w.set_target(1.0, force=True); w.wait_idle()   # a deliberate re-stroke goes out
        assert Client.sets == [1.0, 0.0, 1.0, 1.0]
        w.release()
    finally:
        g.FirmwareClient = saved


class BodyDeviceTests(ClientTests):
    def _last(self):
        path, body, _ = self.server.calls[-1]
        return path, json.loads(body)

    def test_neck_cmd_posts_radians_and_relative_flag(self):
        self.client.neck_cmd(pitch=-0.6, yaw=0.1, relative=True, velocity=0.5)
        path, body = self._last()
        self.assertEqual(path, "/v1/neck/cmd")
        self.assertEqual(body, {"relative": True, "pitch": -0.6, "yaw": 0.1, "velocity": 0.5})
        with self.assertRaises(ValueError):
            self.client.neck_cmd()

    def test_neck_state_parses(self):
        self.server.response = (200, {"status": "ok", "message": None, "data": {
            "pitch": -0.18, "yaw": 0.0, "pitch_velocity": 0.0, "yaw_velocity": 0.0,
            "pitch_torque": 0.1, "yaw_torque": 0.0, "enabled": True}})
        state = self.client.neck_state()
        self.assertEqual((state.pitch, state.enabled), (-0.18, True))

    def test_slider_set_height_and_state(self):
        self.client.slider_set_height(0.2, wait=True)
        self.assertEqual(self._last(), ("/v1/slider/set_height", {"height_m": 0.2, "wait": True}))
        self.server.response = (200, {"status": "ok", "message": None, "data": {
            "comms_ok": True, "height_m": 0.201, "moving": False, "alarm": False, "alarm_text": None}})
        self.assertEqual(self.client.slider_state().height_m, 0.201)
        with self.assertRaises(ValueError):
            self.client.slider_set_height(-0.1)

    def test_eyes_expression_conversation_and_effect(self):
        self.client.eyes_set_expression("blink", speed=1.0, loops=2)
        self.assertEqual(self._last(), ("/v1/eyes/set_expression", {"name": "blink", "speed": 1.0, "loops": 2}))
        self.client.eyes_conversation_state("conversing")
        self.assertEqual(self._last(), ("/v1/eyes/conversation_state", {"state": "conversing"}))
        self.client.eyes_effect("both", "solid", r=0, g=128, b=255)
        self.assertEqual(self._last(), ("/v1/eyes/cmd", {"target": "both", "effect": "solid", "r": 0, "g": 128, "b": 255}))
        with self.assertRaises(ValueError):
            self.client.eyes_conversation_state("dancing")
