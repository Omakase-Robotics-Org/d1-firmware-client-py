# d1fw-client

Install this checkout in each consumer's Python environment:

```sh
python -m pip install -e /path/to/exp--d1-firmware/python
```

`FirmwareClient` uses one locked keep-alive HTTP connection. Arm joint targets
and feedback positions are **degrees**; `a` is physical left, `b` physical right.
`move_joints_both(a, b)` sends both seven-element arrays in one request with
`wait=false`. `arm_state()` returns typed feedback, command, velocity (deg/s),
torque, temperatures, mode, error and frame serial. Gripper `closedness` is
0=open, 1=closed; optional `grip` is soft/firm/strong.

```python
from d1fw_client import FirmwareClient
with FirmwareClient("http://127.0.0.1:4750") as client:
    state = client.arm_state("a")  # read only
```

Non-success HTTP or envelope status raises `FirmwareError`; malformed replies
raise `ProtocolError`. Transport errors propagate. No request is retried:
a lost mutation response has an unknown outcome. `close()` only closes the
connection. It neither changes arm modes nor opens a held gripper.
Recovery is explicit; use a separate client with `timeout=65` for `recover()`.
The client does not acquire leases: callers retain their existing control-plane
ownership and execution gates. The endpoint must be reachable only by permitted
controllers; do not expose an unauthenticated daemon publicly.

`FirmwareGripper` gives consumers a latest-target worker independent of arm
requests. It serializes stroke and state requests on each side, latches failures,
and never retries an uncertain stroke. Only fresh `live=true` state is exposed
as measured jaw position. Measurement is unavailable during a blocking stroke;
consumers must distinguish command echo from feedback.

## REST trajectory jobs

`POST /v1/arm/trajectory/start` accepts
`{"waypoints":[{"t":0,"a":[...7 degrees...],"b":[...7 degrees...]}, ...]}`.
Times are absolute seconds, begin at zero and strictly increase. Limits are
2–10000 points and 120 seconds. Mode/tool changes are separate explicit verbs.
Both arms must be in clean position/torque mode within 3 degrees of the first
sample. The whole Catmull-Rom path is checked at 1 ms intervals for MotionGuard
and 350 deg/s speed limits before acceptance; it is not automatically retimed.

The response is `{id, phase, elapsed_ms, message}`. Poll
`GET /v1/arm/trajectory/{id}/status`; request cancellation with
`POST /v1/arm/trajectory/{id}/cancel {}`. Only the most recent job is retained.
Cancellation is asynchronous: poll until a terminal phase (`cancelled`, `completed`, or `failed`). It stops new targets,
holding the last accepted command without an estop or mode change. `completed`
means the final target was submitted, not that measured arrival was confirmed.
Faults, mode changes, frozen feedback serial (200 ms), soft-kill and arm stop
terminate playback. Other arm mutations are excluded for the job lifetime.
The 1 kHz task skips missed ticks; it is not a hard real-time guarantee.

Joint impedance uses `POST /v1/arm/{side}/mode` with `mode:"torque"`, explicit
`force_compliance` and `torque` both report state mode `torque`.
Joint torque uses seven-element `stiffness`, `damping` (0–1), and `vel_ratio`, `acc_ratio` (0–1).
Register the actual mounted tool before entry. It accepts subsequent joint
commands in torque mode.

Trajectory verbs and the new joint-torque mode are REST-only in this change.
WS binary streaming, its watchdog and metrics are not implemented.

HTTP 409 is a refused/busy operation, 400 invalid input and 404 an unknown job.
Do not blindly retry a refused motion: inspect state and ownership first.
Use a dedicated 40-second client for blocking gripper strokes; the default
2-second client is for state/tick operations. Successful gripper submission
alone does not establish an object grasp. State `kind` is grasp, contact,
empty, open, timeout, fault, blind or lost; `holding` is true for grasp/contact.
Timeout/fault/blind/lost are latched as worker errors. `release()` means worker
shutdown, never jaw opening. `wait_idle()` means the local queue drained.
Unknown arm modes have JSON shape `{"unknown":"..."}` and cannot follow targets.
Arm torque is nominal N m and temperature degrees Celsius; velocity stays deg/s
in the inference adapter to match its existing vendor-velocity contract.

`preflight()` and `recover()` return the daemon reports unchanged; their schema
and hardware sequencing remain defined by the merged recovery API. Recovery can
change modes and is never implied by a read or connection retry.
