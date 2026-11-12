# System32 Emulator — Cockpit Controller SoC model (PIC24FJ256DA210 peripherals and graphics controller + FM25V10 FRAM +
# PCF2127 RTC + SD card on SPI).
CC      ?= cc
CFLAGS  ?= -std=gnu11 -O2 -g -Wall -Wextra -Werror -Wno-unused-parameter -Wshadow -Wstrict-prototypes
# No fused multiply-add contraction: a*b+c rounds the same on every CPU (arm64 clang fuses by default, x86-64 gcc and
# WebAssembly do not), so every build computes bit-identical results (docs/conformance.md).
CFLAGS   += -ffp-contract=off
CORE     = ../pic24
CPPFLAGS = -Iinclude -I../common -I$(CORE)/include -I$(CORE)/build
B        = build
LIBCORE  = $(CORE)/build/libpic24.a
LIB      = $(B)/libs32ct.a
REPO     = ../..

all: $(LIB) $(B)/test_s32ct $(B)/test_s32dev $(B)/test_s32gfx

# Always ask the core's own make, which rebuilds libpic24.a when pic24.h or a core source changed, so this directory's
# objects never run against a library with a different pic24_cpu layout (tests/stale_core.sh).
$(LIBCORE) $(CORE)/build/sfr_reset.h: FORCE
	$(MAKE) -C $(CORE) all
FORCE:

$(B):
	mkdir -p $(B)

$(B)/s32ct.o: src/s32ct.c include/s32ct.h include/s32gfx.h $(CORE)/include/pic24.h $(CORE)/build/sfr_reset.h | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(B)/s32dev.o: src/s32dev.c include/s32dev.h ../common/s32_spidev.h | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(B)/s32gfx.o: src/s32gfx.c include/s32gfx.h | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(LIB): $(B)/s32ct.o $(B)/s32dev.o $(B)/s32gfx.o
	ar rcs $@ $^

$(B)/test_s32ct: tests/test_s32ct.c $(LIB) $(LIBCORE)
	$(CC) $(CPPFLAGS) $(CFLAGS) -I$(CORE)/tests $< $(LIB) $(LIBCORE) -lz -o $@

$(B)/test_s32dev: tests/test_s32dev.c $(LIB) $(LIBCORE)
	$(CC) $(CPPFLAGS) $(CFLAGS) -I$(CORE)/tests $< $(LIB) $(LIBCORE) -lz -o $@

$(B)/test_s32gfx: tests/test_s32gfx.c $(LIB) $(LIBCORE)
	$(CC) $(CPPFLAGS) $(CFLAGS) -I$(CORE)/tests $< $(LIB) $(LIBCORE) -lz -o $@

test: all
	$(B)/test_s32ct
	$(B)/test_s32dev
	$(B)/test_s32gfx
	bash tests/stale_core.sh

clean:
	rm -rf $(B)

.PHONY: all test clean FORCE
