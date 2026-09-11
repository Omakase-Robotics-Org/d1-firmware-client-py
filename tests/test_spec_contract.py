"""Every route this client calls exists in the vendored OpenAPI document.

The firmware ships as a binary plus `openapi/d1-firmwared.v1.json`; this
repository holds a copy of that document and a hand-written client. Nothing
generates one from the other, so the only thing keeping them honest is this
test: it reads the client's OWN SOURCE, collects the `(method, path)` pairs it
can send, and checks each one against the spec.

That catches the failure this split makes likelier -- the daemon renames or
drops a route, the vendored spec is refreshed, and a client verb quietly keeps
posting to a path that now 404s. It does NOT check request or response bodies;
that is what `tests/test_client.py` and the daemon's own tests do.

When a generated client replaces the hand-written one, this test is what is
deleted: the generator makes the same guarantee structurally.
"""
from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest

from d1fw_client import spec_operations, spec_version
from d1fw_client.spec import SPEC_PATH

PACKAGE = Path(__file__).resolve().parents[1] / "d1fw_client"
_TEMPLATE = re.compile(r"\{[^{}]*\}")

#: Paths the client builds but the spec templates, e.g. the client's
#: f"/v1/arm/{side_name(side)}/state" and the spec's "/v1/arm/{side}/state".
#: Comparing on the template SHAPE rather than the parameter name keeps this
#: from failing over what the daemon's author called a path parameter.
def _shape(path: str) -> str:
    return _TEMPLATE.sub("{}", path)


def _path_of(node: ast.AST) -> str | None:
    """The path a `request(...)` call sends to, templated, or None if dynamic.

    A literal is itself; an f-string becomes its shape, with each interpolation
    standing in for one path parameter. Anything else (a variable, a call) is
    not statically knowable and is skipped rather than guessed at -- see
    `test_no_route_is_built_too_dynamically_to_check`, which keeps that escape
    hatch from quietly swallowing the whole surface.
    """
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, ast.JoinedStr):
        out = []
        for part in node.values:
            if isinstance(part, ast.Constant) and isinstance(part.value, str):
                out.append(part.value)
            elif isinstance(part, ast.FormattedValue):
                out.append("{}")
            else:  # pragma: no cover - no other node kind exists in an f-string
                return None
        return "".join(out)
    return None


def client_calls() -> list[tuple[str, str, str, int]]:
    """`(METHOD, path-shape, file, line)` for every request the package makes."""
    found: list[tuple[str, str, str, int]] = []
    for source in sorted(PACKAGE.rglob("*.py")):
        tree = ast.parse(source.read_text(encoding="utf-8"), str(source))
        for node in ast.walk(tree):
            if not (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr in ("request", "_send")
                    and len(node.args) >= 2):
                continue
            method, path = node.args[0], node.args[1]
            if not (isinstance(method, ast.Constant) and isinstance(method.value, str)):
                continue
            shape = _path_of(path)
            if shape is not None:
                found.append((method.value.upper(), _shape(shape),
                              source.name, node.lineno))
    return found


#: `GET /openapi.json` is deliberately NOT in the document's own `paths`: it is
#: the one route outside the `/v1` surface and the one reply that is not
#: enveloped, because it IS the document. The daemon asserts that separately
#: (`the_openapi_document_is_served_verbatim_outside_the_v1_surface`), so a
#: self-describing route being absent from the self-description is correct, not
#: drift. It is the only exemption, and it is listed rather than pattern-matched.
UNPUBLISHED = {("GET", "/openapi.json")}

CALLS = [call for call in client_calls() if (call[0], call[1]) not in UNPUBLISHED]


def test_the_vendored_spec_is_loadable_and_versioned():
    assert SPEC_PATH.is_file(), f"{SPEC_PATH} is missing from the package"
    assert (SPEC_PATH.parent / "SPEC_SOURCE").is_file(), (
        "the vendored spec has no SPEC_SOURCE note saying which firmware commit "
        "it came from")
    assert re.fullmatch(r"\d+\.\d+\.\d+", spec_version()), spec_version()


def test_the_client_actually_calls_something():
    """A refactor that moves the request out of reach of the AST walk would
    otherwise turn this whole file green by checking nothing."""
    assert len(CALLS) >= 20, CALLS


@pytest.mark.parametrize("method,path", sorted({(m, p) for m, p, _, _ in CALLS}))
def test_every_route_the_client_calls_is_in_the_spec(method, path):
    published = {(m, _shape(p)) for m, p in spec_operations()}
    assert (method, path) in published, (
        f"{method} {path} is not in openapi/d1-firmwared.v1.json "
        f"(spec {spec_version()}). Either the client calls a route the daemon "
        f"does not serve, or the vendored spec is stale -- refresh it from the "
        f"firmware repository and update SPEC_SOURCE.")


def test_no_route_is_built_too_dynamically_to_check():
    """Every `request(...)` in the package has a statically readable path.

    `_path_of` returns None for a computed path, and a None is skipped. If a
    verb ever builds its path from a variable, this fails and says so, instead
    of that verb silently dropping out of the contract check above.
    """
    dynamic = []
    for source in sorted(PACKAGE.rglob("*.py")):
        tree = ast.parse(source.read_text(encoding="utf-8"), str(source))
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                    and node.func.attr in ("request", "_send")
                    and len(node.args) >= 2
                    and isinstance(node.args[0], ast.Constant)
                    and _path_of(node.args[1]) is None):
                dynamic.append(f"{source.name}:{node.lineno}")
    assert not dynamic, (
        "these calls build their path dynamically, so the spec check cannot "
        f"see them: {dynamic}")


def test_spec_version_is_what_a_daemon_would_be_compared_against():
    """`spec_version()` reads the same field a daemon reports at
    `GET /openapi.json`, which is where the comparison happens on a robot."""
    import json
    with SPEC_PATH.open(encoding="utf-8") as handle:
        assert spec_version() == json.load(handle)["info"]["version"]
