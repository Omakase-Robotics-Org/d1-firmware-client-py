# d1fw-client

The Python client for **`d1-firmwared`**, the D1's firmware daemon: one guarded
REST surface over both arms, the grippers, dexterous hands, the pan/tilt neck,
the eye LEDs and the vertical slider.

This package lives in its own repository on purpose. The firmware ships as a
**binary plus an OpenAPI document** — consumers must never depend on the
firmware git repository, and after 2026-09-11 they cannot: `d1-firmware` has no
`python/` directory. What a consumer installs is this package, pinned by
commit; what defines the contract is `d1fw_client/openapi/d1-firmwared.v1.json`,
a byte-for-byte copy of the document the daemon serves at `GET /openapi.json`.

## Install

Pin it by commit, the same way `manipulation-kit` is pinned:

```toml
# pyproject.toml
dependencies = [
    "d1fw-client @ git+ssh://git@github.com/Omakase-Robotics-Org/d1-firmware-client-py.git@<sha>",
]
```

```sh
# requirements.txt / one-off
pip install "d1fw-client @ git+ssh://git@github.com/Omakase-Robotics-Org/d1-firmware-client-py.git@<sha>"
```

**Moving a pin is not enough on its own.** pip decides whether to reinstall by
VERSION, not by commit, so a moved pin at an unchanged version gets
"Requirement already satisfied" and keeps the old client — a successful install
that shipped nothing (this is exactly how `manipulation-kit` bricked three
checkouts on d1-2 on 2026-09-09). Two defences:

- every change to `d1fw_client/` bumps `project.version`, enforced by
  `tools/check_version_bump.py` in CI;
- when reinstalling a moved pin by hand, name it and force it:

  ```sh
  .venv/bin/python -m pip install --force-reinstall --no-deps \
    "d1fw-client @ git+ssh://git@github.com/Omakase-Robotics-Org/d1-firmware-client-py.git@<sha>"
  ```

There are **no runtime dependencies**, and that is a requirement rather than an
accident: this client is imported inside an arm tick and by pin-checks that run
before anything else in a venv is trusted. `http.client`, `json` and
`threading` cover the whole surface.

## The contract with the daemon

`d1fw_client/openapi/d1-firmwared.v1.json` is vendored from the firmware
repository; `d1fw_client/openapi/SPEC_SOURCE` records which commit and which
daemon version it came from. Two things use it:

```python
from d1fw_client import FirmwareClient, spec_version

spec_version()                      # the API version this client was built against
with FirmwareClient() as client:
    client.daemon_spec_version()    # what the daemon in front of you serves
```

Compare them before trusting a robot you did not deploy yourself — a mismatch
means the client and the daemon disagree about the API, which on this hardware
is not a thing to discover mid-motion. `GET /openapi.json` is the one route
outside the `/v1` surface and the one reply that is not enveloped, because it
*is* the document.

CI checks the other direction: `tests/test_spec_contract.py` reads this
package's own source, collects every `(method, path)` it can send, and asserts
each one exists in the vendored spec. A refreshed spec that renamed or dropped
a route fails there, not on a robot.

To refresh the spec: copy `openapi/d1-firmwared.v1.json` from `d1-firmware` at
the commit you want, update `SPEC_SOURCE`, bump `project.version`, and let the
contract test tell you what moved.

## A generated client may replace this one

The firmware repository's position is that *clients are generated, not
shipped*: the document carries `x-timeout-seconds` on every blocking call and
`x-ws-method` on every verb reachable over WebSocket, precisely so a generator
can size timeouts and emit the WS surface. When a generated client lands, it
lands **here**, under the same distribution name `d1fw-client` and the same
import name `d1fw_client` — consumers move their pin and change nothing else.
That is the reason this repository exists rather than a `python/` directory
inside the firmware.

What would go at that point is `tests/test_spec_contract.py` (a generator makes
the same guarantee structurally) and the hand-written verb list. The
behavioural contract below is what a generated client would have to keep.

## Using it

```python
from d1fw_client import FirmwareClient
with FirmwareClient("http://127.0.0.1:4750") as client:
    state = client.arm_state("a")  # read only
```

`FirmwareClient` uses one locked keep-alive HTTP connection. Arm joint targets
and feedback positions are **degrees**; `a` is physical left, `b` physical right.
`move_joints_both(a, b)` sends both seven-element arrays in one request with
`wait=false`. `arm_state()` returns typed feedback, command, velocity (deg/s),
torque, temperatures, mode, error and frame serial. Gripper `closedness` is
0=open, 1=closed; optional `grip` is soft/firm/strong.

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

Trajectory verbs and the joint-torque mode are REST-only in this client.
WS binary streaming, its watchdog and metrics are not implemented here.

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

## Ratios are fractions, not percentages

Every `vel_ratio` and `acc_ratio` the daemon accepts is a fraction in
`0.0..=1.0`. `0.1` is 10% of full speed; `100` is not 100%, it is out of range
and is refused with a 400. This has caused a real full-speed-arm incident on a
robot, which is why the daemon rejects rather than clamps.

## Development

```sh
pip install -e '.[dev]'
pytest -q
```

`tests/test_client.py` runs a real `ThreadingHTTPServer` and exercises actual
HTTP framing, keep-alive and mutation ambiguity — not mocks.
