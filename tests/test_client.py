"""Exercise actual HTTP framing, keepalive and mutation ambiguity."""
import json
import socket
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from d1fw_client import FirmwareClient, FirmwareError, ProtocolError


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
