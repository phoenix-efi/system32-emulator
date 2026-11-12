#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# System32 emulator -- are the tools on PATH the pinned ones? (`make tool-versions`, run first by `make check`)
#
# Compares python3, uv, node, emcc and cppcheck with
# nix/tool-versions.txt, which the flake enforces for `nix develop`. A missing tool counts as a mismatch: without emcc
# the WebAssembly builds would drop out of the conformance gate, without cppcheck the static analysis would not run.
#
# A mismatch is a loud WARNING (exit 0) by default, so `make check` still works on a machine without Nix, and an
# ERROR (exit 1) when CI=true / CI=1 (the variable continuous-integration services commonly set) or PINNED=1 (set by
# `nix develop`):
#   PINNED=1           fail on any mismatch (the dev shell sets it; an automated run fails through CI=true)
# Exit 0 = every tool matches, or mismatches outside CI=true / the pinned shell; 1 = a mismatch with CI=true or in the
# pinned shell.
set -uo pipefail
REPO_ROOT="$(CDPATH= cd -- "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PINS="$REPO_ROOT/nix/tool-versions.txt"

pin() { awk -v t="$1" '$1 == t { print $2 }' "$PINS"; }

# The version each tool reports, or "missing".
have() {
  local out
  case "$1" in
    python)     out="$(python3 --version 2>&1)"     && sed -E 's/^Python ([0-9.]+).*/\1/' <<<"$out" ;;
    uv)         out="$(uv --version 2>&1)"          && sed -E 's/^uv ([0-9.]+).*/\1/' <<<"$out" ;;
    node)       out="$(node --version 2>&1)"        && sed -E 's/^v([0-9.]+).*/\1/' <<<"$out" ;;
    emscripten) out="$(emcc --version 2>/dev/null | head -n 1)" && [ -n "$out" ] &&
                  sed -E 's/.*\) ([0-9]+\.[0-9]+\.[0-9]+).*/\1/' <<<"$out" ;;
    cppcheck)   out="$(cppcheck --version 2>&1)"    && sed -E 's/^Cppcheck ([0-9.]+).*/\1/' <<<"$out" ;;
  esac || echo missing
}

tools=(python uv node emscripten cppcheck)

bad=0
echo "=== [tool versions] tools on PATH vs nix/tool-versions.txt ==="
for t in "${tools[@]}"; do
  want="$(pin "$t")"; got="$(have "$t")"; got="${got:-missing}"
  if [ "$got" = "$want" ]; then
    printf '    %-11s %s\n' "$t" "$got"
  else
    printf '    %-11s %s  (pinned %s)  MISMATCH\n' "$t" "$got" "$want"
    bad=$((bad + 1))
  fi
done
if [ "$bad" -eq 0 ]; then
  echo "PASS: every tool is the pinned version"
  exit 0
fi
case "${CI:-}" in true|1) strict=1 ;; *) strict="${PINNED:-0}" ;; esac
if [ "$strict" = 1 ]; then
  echo "FAIL: $bad tool(s) differ from the pins (CI=true or PINNED=1). The pinned environment: nix develop -c make check" >&2
  exit 1
fi
echo "*****************************************************************************************************"
echo "WARNING: $bad tool(s) differ from the pins. These results are NOT the pinned environment's. The pinned"
echo "         environment: nix develop -c make check (README.md, 'Build and test'); in it (PINNED=1) or with"
echo "         CI=true this is an error."
echo "*****************************************************************************************************"
exit 0
