#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# The content identity of the emulator's sources, used as the front end's build stamp (BUILD_STAMP, emu/pair/s32pair.c).
#
#   sh emu/common/build_id.sh <repository root> <compiler> [flags...]
#
# Prints 32 hex digits: a SHA-256 over every source and header the front ends compile, the generators of the headers
# made at build time and the register models they read, the Makefiles, the compiler's version line and the flags. Two
# builds of the same sources with the same compiler and flags get the same stamp, wherever the checkout lies (paths are
# relative to the repository); a change to any of these gives another stamp. Snapshot files (emu/common/s32snap.h)
# restore only into a build with the same stamp, and the stimulus signature (S32_STIMULUS) carries it to identify the
# exact build, so the stamp must be both reproducible and sensitive to every input.
set -eu
repo=$1
cc=$2
shift 2
if command -v sha256sum >/dev/null 2>&1; then
  hash() { sha256sum "$@"; }
else
  hash() { shasum -a 256 "$@"; }
fi
cd "$repo"
files=$(
  {
    find emu/pair emu/common emu/ecu/src emu/ecu/include emu/ecu/shim emu/controller/src emu/controller/include \
         emu/pic24/src emu/pic24/include emu/wasm/src emu/wasm/node \
         -type f \( -name '*.c' -o -name '*.h' -o -name '*.js' -o -name '*.sh' \) \
         -not -path '*/build/*' -not -path '*/test/*' -not -path '*/tests/*'
    ls emu/pair/Makefile emu/ecu/Makefile emu/controller/Makefile emu/pic24/Makefile emu/wasm/Makefile \
       emu/ecu/tools/gen_*.py emu/pic24/tools/gen_*.py model/pic32mx_sfr.json model/pic24fj256da210_sfr.json
  } | LC_ALL=C sort
)
{
  # shellcheck disable=SC2086  # the compiler may be a command with arguments
  printf 'compiler %s\n' "$($cc --version 2>&1 | head -n 1)"
  printf 'flags %s\n' "$*"
  # shellcheck disable=SC2086  # one path per word; no path holds a space
  hash $files
} | hash | cut -c1-32
