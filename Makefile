# SPDX-License-Identifier: Apache-2.0
# System32 emulator -- build, test and conformance.
#
#   make check          everything below, in the pinned environment (nix develop): tool versions, the Python lock,
#                       generated sources, cppcheck, native build + unit tests, WebAssembly build +
#                       smoke test, conformance on generated images and on the default firmware (native = WebAssembly)
#   make check-nofw     the same without any firmware (the firmware cases are not run)
#   make firmware       fetch and verify the default firmware into .firmware-cache/v86 (S32_V86_SRC=<dir> for a
#                       local copy of the update files instead of the release assets)
#   make bench-site     the browser bench with its default firmware in build/bench-site (make serve-bench serves it)
#   make dist           the versioned WebAssembly bundle for other sites (dist/, scripts/make_dist.py)

.PHONY: all emu test test-c test-scripts wasm-emcc wasm wasm-test wasm-host-test firmware conformance-synthetic \
        conformance conformance-browser bench bench-site serve-bench dist io-coverage check-generated \
        cppcheck tool-versions python-lock check check-nofw clean

all: emu

PYRUN := scripts/pyrun
# The firmware directory the firmware-dependent targets use: yours (S32_FIRMWARE_DIR), else the default firmware's
# cache that `make firmware` fills.
FW_CACHE := .firmware-cache/v86
FW_DIR ?= $(if $(S32_FIRMWARE_DIR),$(S32_FIRMWARE_DIR),$(CURDIR)/$(FW_CACHE))

emu:
	$(MAKE) -C emu/pic24 B=build all
	$(MAKE) -C emu/controller B=build all
	$(MAKE) -C emu/ecu B=build DEV=../controller/src/s32dev.c all
	$(MAKE) -C emu/pair B=build DEV=../controller/src/s32dev.c all

# The emulator's unit tests (C, and the shell tests of the native front end). Firmware-dependent checks in them run
# when S32_FIRMWARE_DIR is set (make check sets it to the default firmware) and SKIP otherwise.
test-c: emu
	$(MAKE) -C emu/common test
	$(MAKE) -C emu/pic24 test
	$(MAKE) -C emu/controller test
	$(MAKE) -C emu/ecu test
	$(MAKE) -C emu/pair test
	$(MAKE) -C emu/wasm test-host

# Unit tests of the tooling (tests/scripts, Python standard library unittest).
test-scripts: emu
	python3 -I -m unittest discover -s tests/scripts -t .

test: test-c test-scripts

wasm-host-test:
	$(MAKE) -C emu/wasm test-host

# emu/wasm/build/emcc-version.txt: the `emcc --version` line that built the WebAssembly files. A different emcc on PATH
# cleans emu/wasm first, so a build is never compared under another compiler's name.
wasm-emcc:
	@v="$$(emcc --version | head -n 1)"; f=emu/wasm/build/emcc-version.txt; \
	if [ ! -f "$$f" ] || [ "$$(cat "$$f")" != "$$v" ]; then \
	  echo "emu/wasm: building with $$v"; $(MAKE) -s -C emu/wasm clean; mkdir -p emu/wasm/build; echo "$$v" > "$$f"; \
	fi

wasm: emu wasm-emcc
	$(MAKE) -C emu/wasm all node

# The WebAssembly smoke test (emu/wasm/test/smoke.mjs) in Node: firmware-dependent checks run on S32_FIRMWARE_DIR.
wasm-test: wasm
	$(MAKE) -C emu/wasm test

firmware: emu
	python3 -I scripts/fetch_firmware.py --cache $(FW_CACHE)

# Conformance (scripts/conformance.py, docs/conformance.md): the same stimulus must give a byte-identical canonical
# trace (emu/common/s32trace.h) on every build. conformance-synthetic needs no firmware (generated images);
# conformance runs the emulator's cases (tests/conformance/cases.json) and the benchmark scenarios on FW_DIR, native
# against WebAssembly (in Node, both the front-end build and the browser bench's module). CONFORMANCE_ARGS adds options
# (--provider, --remote user@host, --builds ...).
CONFORMANCE_ARGS ?=
CONFORMANCE_JOBS ?= 2
conformance-synthetic: wasm
	python3 scripts/conformance.py --synthetic --builds native,native-again,native-serial,native-scribble,wasm-node,wasm-bench,snapshot,snapshot-self,snapshot-wasm

conformance: wasm
	@if [ ! -d "$(FW_DIR)" ]; then echo "conformance: no firmware directory $(FW_DIR) (make firmware, or set S32_FIRMWARE_DIR)"; exit 1; fi
	python3 scripts/conformance.py --firmware-dir "$(FW_DIR)" --jobs $(CONFORMANCE_JOBS) \
	  --builds native,native-again,native-serial,wasm-node,wasm-bench,snapshot $(CONFORMANCE_ARGS)

# The browser bench's module in headless Chromium (Playwright from uv.lock) against the native build.
conformance-browser: wasm
	python3 scripts/conformance.py --synthetic --builds native,browser

BENCH_ARGS ?=
bench: wasm
	python3 scripts/bench.py --firmware-dir "$(FW_DIR)" $(BENCH_ARGS)

# The bench page with the default firmware next to it (firmware/manifest.json; the page checks each file's SHA-256
# before it loads anything). build/bench-site is git-ignored and is never published from here: the firmware files are
# release assets with their own licence (firmware/LICENSE-FIRMWARE).
BENCH_SITE := build/bench-site
bench-site: wasm firmware
	rm -rf $(BENCH_SITE) && mkdir -p $(BENCH_SITE)
	for f in index.html s32_wasm.js s32_wasm.wasm; do cp "emu/wasm/build/$$f" $(BENCH_SITE)/; done
	python3 -I scripts/fetch_firmware.py --cache $(FW_CACHE) --no-derive --bench-dir $(BENCH_SITE)/firmware

SERVE_PORT ?= 8787
serve-bench: bench-site
	python3 -m http.server $(SERVE_PORT) --bind 127.0.0.1 --directory $(BENCH_SITE)

# The versioned WebAssembly bundle a site embeds: the module, the bench page, the firmware
# manifest (no firmware) and a SHA-256 list. VERSION defaults to `git describe`.
VERSION ?= $(shell git describe --tags --always --dirty 2>/dev/null || echo unknown)
dist: wasm
	python3 -I scripts/make_dist.py --version "$(VERSION)" --out dist

# The I/O coverage gate (scripts/io_coverage.py, docs/io-ledger.md): runs the firmware in FW_DIR over a scripted run set
# with the I/O coverage recorder on and compares what it touched with model/io_ledger.json. Not part of make check.
io-coverage: emu
	python3 -I scripts/io_coverage.py --firmware-dir "$(FW_DIR)"

# Regenerate the generated sources in memory and compare byte for byte: the SFR and pin models from Microchip's public
# DFPs (fetched once by pinned URL + SHA-256 into ~/.cache/system32-emulator/dfp, never committed), the I/O ledger page and
# the bench's synthetic demo datalog.
check-generated:
	python3 -I scripts/gen_pic24_sfr.py --check
	python3 -I scripts/gen_pic32mx_sfr.py --check
	python3 -I scripts/gen_mcu_io.py --check
	python3 -I scripts/gen_io_ledger_doc.py --check
	python3 -I scripts/gen_sample_datalog.py --check

cppcheck:
	./scripts/cppcheck.sh

tool-versions:
	./scripts/tool_versions.sh

python-lock:
	uv lock --locked --offline

check-nofw: tool-versions python-lock check-generated cppcheck test conformance-synthetic wasm-test

# make check: the firmware-free gates, then the unit and smoke tests again and the conformance gate on the default
# firmware (S32_FIRMWARE_DIR set for them, so their firmware-dependent checks run).
check: check-nofw firmware
	S32_FIRMWARE_DIR="$(FW_DIR)" $(MAKE) -C emu/pair test
	S32_FIRMWARE_DIR="$(FW_DIR)" $(MAKE) -C emu/wasm test
	S32_FIRMWARE_DIR="$(FW_DIR)" $(MAKE) conformance

clean:
	for d in common pic24 controller ecu pair wasm; do $(MAKE) -C emu/$$d clean; done
	rm -rf build dist
