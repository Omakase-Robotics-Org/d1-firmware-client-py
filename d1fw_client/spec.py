"""The vendored OpenAPI description of the daemon this client speaks to.

The firmware ships as a BINARY plus this document; consumers must never depend
on the firmware git repository. The copy under ``openapi/`` is therefore part
of the installed package, not a developer convenience: it is what lets a robot
answer "is the daemon in front of me the one this client was built against?"
without a firmware checkout. See ``openapi/SPEC_SOURCE`` for the exact firmware
commit it came from.

The daemon serves the identical document at ``GET /openapi.json``, so the
comparison is one line::

    from d1fw_client import FirmwareClient, spec_version
    with FirmwareClient() as client:
        if client.daemon_spec_version() != spec_version():
            raise SystemExit("d1fw-client and d1-firmwared disagree on the API")
"""
from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path
from typing import Any

#: The vendored document, inside the installed package.
SPEC_PATH = Path(__file__).resolve().parent / "openapi" / "d1-firmwared.v1.json"


@lru_cache(maxsize=1)
def spec_document() -> dict[str, Any]:
    """The whole vendored OpenAPI document (parsed once, then cached).

    260 kB of JSON, so it is read lazily: importing ``d1fw_client`` on an arm
    tick must not pay for a document only tooling reads.
    """
    with SPEC_PATH.open(encoding="utf-8") as handle:
        return json.load(handle)


def spec_version() -> str:
    """``info.version`` of the API this client was built against.

    Compare it with the daemon's own — :meth:`FirmwareClient.daemon_spec_version`
    — before trusting a robot you did not deploy yourself.
    """
    return str(spec_document()["info"]["version"])


def spec_operations() -> set[tuple[str, str]]:
    """Every ``(METHOD, path)`` the daemon publishes, methods upper-cased.

    Paths keep their OpenAPI templating (``/v1/arm/{side}/state``).
    """
    return {(method.upper(), path)
            for path, item in spec_document()["paths"].items()
            for method in item
            if method.lower() in ("get", "put", "post", "delete", "patch",
                                  "head", "options", "trace")}
