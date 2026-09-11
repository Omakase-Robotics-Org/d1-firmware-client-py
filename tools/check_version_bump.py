#!/usr/bin/env python3
"""Fail when consumers are handed a new commit under an unchanged version.

This package publishes no releases. Every consumer pins a COMMIT
(``d1fw-client @ git+ssh://...@<sha>``) and pip decides whether to reinstall by
VERSION, not by commit. That combination has one specific, silent failure mode:
a consumer moves its pin, runs ``pip install -e .``, and pip answers
"Requirement already satisfied" and keeps the OLD client installed. The install
prints success. The robot then talks to the daemon with the wrong verbs, or
dies at import, from a checkout that ``git status`` calls clean.

That is not hypothetical. It happened to this repository's sibling
``manipulation-kit`` on d1-2, 2026-09-09: three repositories pulled a new pin,
reinstalled "successfully", and came up with ``ModuleNotFoundError`` because
pip had reinstalled nothing. The same trap is worse here, because a stale
client fails against MOVING HARDWARE rather than at import.

So: if a change alters what consumers IMPORT (anything under ``d1fw_client/``,
including the vendored OpenAPI document) or what they must install ALONGSIDE it
(the dependency lists in ``pyproject.toml``), ``project.version`` has to move
too. Nothing can make pip notice a commit; a version that changes is the only
thing pip reliably acts on.

Usage
-----
    python tools/check_version_bump.py [--base REF]

``--base`` defaults to the pull request's base branch in GitHub Actions
(``GITHUB_BASE_REF``), else the first parent of HEAD. On the initial commit
there is no base to compare against and the check SKIPS -- it is a diff gate,
not a policy on a lone commit.
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys

PYPROJECT = "pyproject.toml"
# What a consumer actually receives from this repository: the importable code
# and the vendored spec that ships inside it (d1fw_client/), plus the
# dependency lists it resolves alongside them (pyproject.toml). tests/, tools/
# and .github/ change nothing a consumer's venv holds.
WATCHED_PREFIXES = ("d1fw_client/",)

_VERSION_RE = re.compile(
    r"^\s*version\s*=\s*[\"'](?P<v>[^\"']+)[\"']", re.MULTILINE)


def _git(*args: str) -> str:
    return subprocess.run(["git", *args], check=True, capture_output=True,
                          text=True).stdout


def parse_version(pyproject_text: str) -> str | None:
    """``project.version``.

    Regex rather than a TOML parse, so this gate keeps working on any Python
    the runner happens to have, including one without ``tomllib``. The first
    ``version = "..."`` in the
    file is ``[project].version``: ``[build-system]`` above it has no version
    key, and every later one lives inside a dependency string, which this
    pattern (anchored at the start of a line) cannot reach.
    """
    m = _VERSION_RE.search(pyproject_text)
    return m.group("v") if m else None


def parse_dependencies(pyproject_text: str) -> list[str]:
    """Every requirement string in the file, in file order.

    Deliberately crude and deliberately over-broad: it collects the quoted
    entries of ``dependencies``/``optional-dependencies`` arrays without
    understanding TOML, so a reformat counts as a change. A false "bump the
    version" is a five-second edit; a false pass is a robot that will not
    import.
    """
    deps: list[str] = []
    in_array = False
    for line in pyproject_text.splitlines():
        stripped = line.strip()
        if not in_array:
            # `dependencies = [`, `dev = [`, ... — an array opened inside the
            # dependency tables. We accept any `key = [` and then keep only the
            # ones that look like requirements, which is what matters here.
            if re.match(r"^[A-Za-z0-9_.\"'-]+\s*=\s*\[", stripped):
                in_array = True
                stripped = stripped.split("[", 1)[1]
            else:
                continue
        if "]" in stripped:
            stripped, in_array = stripped.split("]", 1)[0], False
        deps += re.findall(r"[\"']([^\"']+)[\"']", stripped.split("#", 1)[0])
    return deps


def needs_bump(changed: list[str], before: str, after: str) -> tuple[bool, str]:
    """Does this diff oblige a version bump, and why."""
    touched_src = sorted(p for p in changed
                         if p.startswith(WATCHED_PREFIXES))
    if touched_src:
        return True, (f"{len(touched_src)} file(s) under d1fw_client/ changed, "
                      f"starting with {touched_src[0]}")
    if PYPROJECT in changed and parse_dependencies(before) != parse_dependencies(after):
        return True, "the dependency lists in pyproject.toml changed"
    return False, ""


def resolve_base(explicit: str | None) -> str | None:
    """The ref to diff against, or None when there is nothing to diff."""
    if explicit:
        return explicit
    base_ref = os.environ.get("GITHUB_BASE_REF")
    if base_ref:
        # actions/checkout fetches the base branch as a remote ref.
        for candidate in (f"origin/{base_ref}", base_ref):
            if subprocess.run(["git", "rev-parse", "--verify", "--quiet",
                               f"{candidate}^{{commit}}"],
                              capture_output=True).returncode == 0:
                return candidate
    # A push build, or a local run: the previous commit. On the INITIAL commit
    # HEAD has no parent and this check has nothing to say.
    if subprocess.run(["git", "rev-parse", "--verify", "--quiet", "HEAD^{commit}"],
                      capture_output=True).returncode != 0:
        return None  # no commits at all
    if subprocess.run(["git", "rev-parse", "--verify", "--quiet", "HEAD~1^{commit}"],
                      capture_output=True).returncode != 0:
        return None  # initial commit
    return "HEAD~1"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", default=None,
                    help="ref to diff against (default: PR base branch, else HEAD~1)")
    args = ap.parse_args()

    base = resolve_base(args.base)
    if base is None:
        print("[version-bump] no base commit to compare against "
              "(initial commit / shallow clone) — skipping")
        return 0

    merge_base = _git("merge-base", base, "HEAD").strip() or base
    changed = [p for p in _git("diff", "--name-only", merge_base, "HEAD").splitlines() if p]
    if not changed:
        print(f"[version-bump] no changes against {base} — nothing to check")
        return 0

    before = _git("show", f"{merge_base}:{PYPROJECT}")
    after = _git("show", f"HEAD:{PYPROJECT}")
    required, why = needs_bump(changed, before, after)
    old, new = parse_version(before), parse_version(after)

    if not required:
        print(f"[version-bump] nothing consumers install changed "
              f"(no d1fw_client/ and no dependency edits) — version {new} may stand")
        return 0
    if old != new:
        print(f"[version-bump] OK: {why}, and version moved {old} → {new}")
        return 0

    print(f"[version-bump] FAIL: {why}, but project.version is still {new}.")
    print("[version-bump]")
    print("[version-bump] Consumers pin this repository by COMMIT, and pip decides whether to")
    print("[version-bump] reinstall by VERSION. Leaving the version alone means every consumer")
    print("[version-bump] that moves its pin gets 'Requirement already satisfied' and keeps the")
    print("[version-bump] OLD code — a successful install that ships nothing, here against")
    print("[version-bump] moving hardware (manipulation-kit hit exactly this on d1-2, 2026-09-09).")
    print("[version-bump]")
    print(f"[version-bump] Fix: bump project.version in {PYPROJECT} (e.g. {new} → "
          f"{_suggest(new)}) in this same change.")
    return 1


def _suggest(version: str | None) -> str:
    """The obvious next version, for the error message only."""
    if not version:
        return "0.1.1"
    parts = version.split(".")
    if parts[-1].isdigit():
        parts[-1] = str(int(parts[-1]) + 1)
        return ".".join(parts)
    return version + ".1"


if __name__ == "__main__":
    sys.exit(main())
