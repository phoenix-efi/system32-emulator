# System32 Emulator — 3-MCU Lockstep Co-Simulator (Cockpit Controller + ECU1 + ECU2 + Engine Plant).
CC       ?= cc
CFLAGS   ?= -std=gnu11 -O2 -g -Wall -Wextra -Werror -Wno-unused-parameter -Wshadow -Wstrict-prototypes
# No fused multiply-add contraction: a*b+c rounds the same on every CPU (arm64 clang fuses by default, x86-64 gcc and
# WebAssembly do not), so every build computes bit-identical results (docs/conformance.md).
CFLAGS   += -ffp-contract=off
REPO      = ../..
SHIM_INC  = -I../common -I../controller/include -I../pic24/include -I../pic24/build -I../ecu/include -I../ecu/build -I../ecu/shim/include
CPPFLAGS  = $(SHIM_INC)
LIBS      = ../controller/build/libs32ct.a ../ecu/build/libecu.a ../pic24/build/libpic24.a
LDFLAGS   = -lz -lm
B         = build

all: $(B)/s32pair $(B)/s32pair_shim $(B)/test_board $(B)/test_trace

$(B):
	mkdir -p $(B)

# The libraries are made by their own Makefiles (always asked, FORCE); a binary relinks only when a library file
# actually changed, so an unchanged tree rebuilds nothing.
$(LIBS): FORCE
	$(MAKE) -C $(patsubst %/build/,%,$(dir $@)) all

libs: $(LIBS)

FORCE:

COMMON    = ../common/link_decode.c ../common/fram_format.c ../common/s32board.c ../common/bench_engine.c ../common/s32trace.c ../common/s32panel.c ../common/s32snap.c
COMMON_H  = ../common/link_decode.h ../common/fram_format.h ../common/s32board.h ../common/bench_engine.h ../common/s32_iocov.h ../common/s32trace.h ../common/s32panel.h ../common/s32snap.h

# BUILD_STAMP (s32pair.c) is the content identity of the sources, generators, register models, Makefiles, compiler and
# flags (../common/build_id.sh), never the build time: two builds of the same tree load the same bytes and carry the
# same stamp, which identifies the exact build in the stimulus signature (S32_STIMULUS); a snapshot restores only into
# a build of the same sources. build/build_id is rewritten only when that identity changes, and then one second after
# the recipe starts: make compares file times to the second on macOS (GNU make 4.4.1 and /usr/bin/make), so an id
# written in the same second as the last link would not relink the binaries (removing them instead leaves them
# missing: make has already looked at them). The stamp hashes the pair's own flags only: a library made by its own
# make with other flags keeps the stamp.
$(B)/build_id: FORCE | $(B)
	@sh ../common/build_id.sh $(REPO) "$(CC)" $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) > $@.tmp && \
	  { cmp -s $@.tmp $@ || { sleep 1; cat $@.tmp > $@; }; }; rm -f $@.tmp
STAMP = -DS32_BUILD_ID='"$(shell cat $(B)/build_id)-$(1)"'

$(B)/s32pair: s32pair.c xp_rref.c xp_rref.h $(COMMON) $(COMMON_H) $(LIBS) $(B)/build_id | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(call STAMP,uc) s32pair.c xp_rref.c $(COMMON) $(LIBS) $(LDFLAGS) -o $@

$(B)/s32pair_shim: s32pair.c xp_rref.c xp_rref.h $(COMMON) $(COMMON_H) ../ecu/src/ecu.c ../ecu/shim/uc_mips32_shim.c $(LIBS) $(B)/build_id | $(B)
	$(CC) $(SHIM_INC) $(CFLAGS) $(call STAMP,shim) s32pair.c xp_rref.c $(COMMON) ../ecu/src/ecu.c ../ecu/shim/uc_mips32_shim.c ../controller/build/libs32ct.a ../pic24/build/libpic24.a -lz -lm -o $@

# Harness model unit tests (no firmware): every knob family against the SoC pin / ADC / UART / SPI models.
$(B)/test_board: test/test_board.c $(COMMON) $(COMMON_H) ../ecu/src/ecu.c ../ecu/shim/uc_mips32_shim.c $(LIBS) | $(B)
	$(CC) $(SHIM_INC) -I../pic24/tests $(CFLAGS) test/test_board.c $(COMMON) ../ecu/src/ecu.c ../ecu/shim/uc_mips32_shim.c ../controller/build/libs32ct.a ../pic24/build/libpic24.a -lz -lm -o $@

# Conformance trace unit tests (emu/common/s32trace.c), no firmware.
$(B)/test_trace: test/test_trace.c $(COMMON) $(COMMON_H) ../ecu/src/ecu.c ../ecu/shim/uc_mips32_shim.c $(LIBS) | $(B)
	$(CC) $(SHIM_INC) -I../pic24/tests $(CFLAGS) test/test_trace.c $(COMMON) ../ecu/src/ecu.c ../ecu/shim/uc_mips32_shim.c ../controller/build/libs32ct.a ../pic24/build/libpic24.a -lz -lm -o $@

test: all
	$(B)/test_board
	$(B)/test_trace
	bash test/smoke.sh
	bash test/power_sense.sh
	bash test/io_faults.sh
	bash test/log_order.sh
	bash test/snapshot.sh
	bash test/table_write_report.sh

clean:
	rm -rf $(B)

.PHONY: all libs test clean FORCE
