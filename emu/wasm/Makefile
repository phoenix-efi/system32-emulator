# System32 Emulator — WebAssembly 3-MCU Co-Simulator & Browser Avionics Bench (emu/wasm).
# Compiles emu/pic24 + emu/controller + emu/ecu + emu/ecu/shim/uc_mips32_shim.c + src/s32_wasm.c into build/s32_wasm.{js,wasm}.
EMCC    ?= emcc
REPO     = ../..
B        = build

EXPORTED_FNS = ["_malloc","_free","_s32_wasm_load_ct_firmware","_s32_wasm_load_ec_firmware","_s32_wasm_load_ec_bflash","_s32_wasm_load_ec_pflash","_s32_wasm_load_fram","_s32_wasm_get_fram_ptr","_s32_wasm_set_map_cell","_s32_wasm_get_map_json","_s32_wasm_set_engine","_s32_wasm_set_link","_s32_wasm_clear_fram","_s32_wasm_load_sd","_s32_wasm_set_tps_fault","_s32_wasm_set_an_fault","_s32_wasm_set_sensors","_s32_wasm_set_ecu_an","_s32_wasm_set_ct_an","_s32_wasm_set_ct_pin","_s32_wasm_set_rpm","_s32_wasm_set_cylinders","_s32_wasm_set_switches","_s32_wasm_button","_s32_wasm_button_pulse","_s32_wasm_encoder_turn","_s32_wasm_boot","_s32_wasm_step_quanta","_s32_wasm_render_rgba","_s32_wasm_get_telemetry_json","_s32_wasm_get_ecu_console","_s32_wasm_knob","_s32_wasm_knob_error","_s32_wasm_knob_names","_s32_wasm_boot_plain","_s32_wasm_trace_enable","_s32_wasm_trace_ptr","_s32_wasm_trace_len","_s32_wasm_trace_clear","_s32_wasm_trace_finish","_s32_wasm_clear_ec","_s32_wasm_clear_ct","_s32_wasm_ecu_app_start","_s32_wasm_have_ct","_s32_wasm_home_seen"]

all: $(B)/s32_wasm.js $(B)/index.html $(B)/cockpit.html $(B)/s32_bench.js

# The native front end (emu/pair/s32pair.c) compiled to WebAssembly for Node, single-threaded (S32_SERIAL), the same
# knobs from the environment and the same files: the cross-build conformance gate (scripts/conformance.py) runs it beside
# the native binary. Not part of the published site.
node: $(B)/s32pair_node.js

$(B):
	mkdir -p $(B)

$(B)/sfr_headers:
	$(MAKE) -C ../pic24 build/sfr_reset.h
	$(MAKE) -C ../ecu build/sfr_names.h build/irq_map.h

$(B)/s32_wasm.js: src/s32_wasm.c ../common/link_decode.c ../common/fram_format.c ../common/bench_engine.c ../common/s32board.c ../common/s32trace.c ../common/s32trace.h ../common/s32panel.c ../common/s32panel.h ../common/s32snap.c ../common/s32snap.h ../common/s32board.h ../common/s32_iocov.h ../common/link_decode.h ../common/fram_format.h ../common/bench_engine.h ../ecu/shim/uc_mips32_shim.c ../ecu/shim/include/unicorn/unicorn.h ../controller/src/s32ct.c ../controller/src/s32dev.c ../controller/src/s32gfx.c ../pic24/src/pic24_core.c ../ecu/src/ecu.c $(B)/sfr_headers | $(B)
	$(EMCC) -std=gnu11 -O3 -flto -ffp-contract=off \
	  -Wall -Wextra -Wno-unused-parameter \
	  -I../ecu/shim/include \
	  -I../common \
	  -I../pic24/include \
	  -I../pic24/build \
	  -I../controller/include \
	  -I../ecu/include \
	  -I../ecu/build \
	  src/s32_wasm.c \
	  ../common/link_decode.c \
	  ../common/fram_format.c \
	  ../common/bench_engine.c \
	  ../common/s32board.c \
	  ../common/s32trace.c \
	  ../common/s32panel.c \
	  ../common/s32snap.c \
	  ../ecu/shim/uc_mips32_shim.c \
	  ../controller/src/s32ct.c \
	  ../controller/src/s32dev.c \
	  ../controller/src/s32gfx.c \
	  ../pic24/src/pic24_core.c \
	  \
	  ../ecu/src/ecu.c \
	  -sUSE_ZLIB=1 \
	  -sALLOW_MEMORY_GROWTH=1 \
	  -sINITIAL_MEMORY=33554432 \
	  -sSTACK_SIZE=4194304 \
	  -sMODULARIZE=1 \
	  -sEXPORT_NAME="createS32Module" \
	  -sEXPORTED_FUNCTIONS='$(EXPORTED_FNS)' \
	  -sEXPORTED_RUNTIME_METHODS='["ccall","cwrap","UTF8ToString","HEAPU8"]' \
	  -o $(B)/s32_wasm.js

PAIR_SRC = ../pair/s32pair.c ../pair/xp_rref.c ../common/link_decode.c ../common/fram_format.c ../common/bench_engine.c ../common/s32board.c ../common/s32trace.c ../common/s32panel.c ../common/s32snap.c ../ecu/shim/uc_mips32_shim.c ../controller/src/s32ct.c ../controller/src/s32dev.c ../controller/src/s32gfx.c ../pic24/src/pic24_core.c ../ecu/src/ecu.c
PAIR_FLAGS = -std=gnu11 -O3 -flto -ffp-contract=off -Wall -Wextra -Wno-unused-parameter \
	  -I../ecu/shim/include -I../common -I../pic24/include -I../pic24/build -I../controller/include -I../ecu/include -I../ecu/build -I../pair \
	  -sUSE_ZLIB=1 -sALLOW_MEMORY_GROWTH=1 -sINITIAL_MEMORY=67108864 -sSTACK_SIZE=4194304 \
	  -sENVIRONMENT=node -sNODERAWFS=1 -sEXIT_RUNTIME=1 --pre-js node/env_pre.js
# The front end's build stamp: the content identity of the sources, compiler and flags (emu/pair/Makefile); a new id
# is written a second later, so that make, which compares times to the second on macOS, relinks.
$(B)/build_id: FORCE | $(B)
	@sh ../common/build_id.sh $(REPO) "$(EMCC)" $(PAIR_FLAGS) > $@.tmp && \
	  { cmp -s $@.tmp $@ || { sleep 1; cat $@.tmp > $@; }; }; rm -f $@.tmp

$(B)/s32pair_node.js: $(PAIR_SRC) ../common/s32board.h ../common/s32trace.h ../common/s32panel.h ../common/s32snap.h node/env_pre.js $(B)/sfr_headers $(B)/build_id | $(B)
	$(EMCC) $(PAIR_FLAGS) -DS32_BUILD_ID='"$(shell cat $(B)/build_id)-node"' $(PAIR_SRC) -o $@

# The pages: the bench (index.html), the cockpit view (cockpit.html) and the script they share (s32_bench.js).
$(B)/index.html $(B)/cockpit.html $(B)/s32_bench.js: $(B)/%: web/% | $(B)
	cp web/$* $@

# No firmware is built, copied or published from here. The browser sim is bring-your-own
# firmware: the user drops a Controller .bin/.hex, ECU .bin/.hex and optional .fram files on the page.
# The smoke test takes firmware from S32_FIRMWARE_DIR and SKIPs firmware-dependent checks without it.
NODE := $(shell which -a node 2>/dev/null | grep -v emsdk | head -n 1)
ifeq ($(NODE),)
NODE := node
endif

test: all
	$(NODE) test/smoke.mjs

# Host test of the knob-name JSON (test/test_knob_names.c), no emcc needed: src/s32_wasm.c is compiled alone with
# s32board_knob_names renamed to the test's own provider, whose name list is far longer than the bench's real one,
# so the JSON must be built to the names' size to come back complete. The provider stays the real one everywhere
# else, so no other file is compiled with the rename.
HOSTCC ?= cc
KNOBTEST_INC = -I../ecu/shim/include -I../common -I../pic24/include -I../pic24/build -I../controller/include -I../ecu/include -I../ecu/build
KNOBTEST_SRC = ../common/link_decode.c ../common/fram_format.c ../common/bench_engine.c ../common/s32board.c ../common/s32trace.c ../common/s32panel.c ../common/s32snap.c ../ecu/shim/uc_mips32_shim.c ../controller/src/s32ct.c ../controller/src/s32dev.c ../controller/src/s32gfx.c ../pic24/src/pic24_core.c ../ecu/src/ecu.c

$(B)/s32_wasm_knobtest.o: src/s32_wasm.c $(B)/sfr_headers | $(B)
	$(HOSTCC) -std=gnu11 -O2 -ffp-contract=off -Wall -Wextra -Wno-unused-parameter $(KNOBTEST_INC) -Ds32board_knob_names=test_knob_names -c src/s32_wasm.c -o $@

$(B)/test_knob_names: test/test_knob_names.c $(B)/s32_wasm_knobtest.o $(KNOBTEST_SRC) | $(B)
	$(HOSTCC) -std=gnu11 -O2 -ffp-contract=off -Wall -Wextra -Wno-unused-parameter $(KNOBTEST_INC) test/test_knob_names.c $(B)/s32_wasm_knobtest.o $(KNOBTEST_SRC) -lz -lm -o $@

test-host: $(B)/test_knob_names
	$(B)/test_knob_names

serve: all
	python3 -m http.server 8787 --bind 127.0.0.1 --directory $(B)

clean:
	rm -rf $(B)

.PHONY: all node test test-host serve clean FORCE
FORCE:
