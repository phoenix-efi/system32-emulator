#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# System32 emulator -- C static analysis gate (cppcheck) on the emulator's sources (emu/).

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if ! command -v cppcheck >/dev/null 2>&1; then
    case "${CI:-}" in true|1) strict=1 ;; *) strict="${PINNED:-0}" ;; esac
    if [ "$strict" = 1 ]; then echo "FAIL: cppcheck not found in PATH" >&2; exit 1; fi
    echo "WARN: cppcheck not found in PATH; skipping local cppcheck (nix develop provides it; an error with CI=true or PINNED=1)."
    exit 0
fi

# -US32_BUILD_ID: the Makefiles pass the build stamp as a string literal; cppcheck would define it as 1 and stop on
# "s32pair src " 1 (emu/pair/s32pair.c BUILD_STAMP). The configuration without it is checked.
echo "=== [Static Analysis] Running cppcheck on emu/ ==="
cppcheck \
    --enable=warning,performance,portability \
    --error-exitcode=1 \
    --inline-suppr \
    --suppress=missingIncludeSystem \
    --suppress=unusedFunction \
    --suppress=unmatchedSuppression \
    --suppress=normalCheckLevelMaxBranches \
    --std=c11 \
    -US32_BUILD_ID \
    -Iemu/common \
    -Iemu/pic24/include \
    -Iemu/controller/include \
    -Iemu/ecu/include \
    -Iemu/ecu/shim/include \
    emu/pic24/src/ \
    emu/controller/src/ \
    emu/ecu/src/ \
    emu/ecu/shim/ \
    emu/common/ \
    emu/pair/s32pair.c \
    emu/wasm/src/

echo "PASS: cppcheck static analysis clean."
