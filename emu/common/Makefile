# System32 Emulator — shared emulator front-end observers: link_decode (wire protocol), fram_format (FRAM layout),
# bench_engine (injector meter, air model, wideband sensor). Built into emu/pair and emu/wasm from source.
CC      ?= cc
CFLAGS  ?= -std=gnu11 -O2 -g -Wall -Wextra -Werror -Wno-unused-parameter -Wshadow -Wstrict-prototypes
# No fused multiply-add contraction: a*b+c rounds the same on every CPU (arm64 clang fuses by default, x86-64 gcc and
# WebAssembly do not), so every build computes bit-identical results (docs/conformance.md).
CFLAGS   += -ffp-contract=off
CPPFLAGS = -I. -I../pic24/tests -DFIXTURES='"tests/fixtures"'
B        = build
SRC      = link_decode.c fram_format.c bench_engine.c
HDR      = link_decode.h fram_format.h bench_engine.h

all: $(B)/test_common

$(B):
	mkdir -p $(B)

$(B)/test_common: tests/test_common.c $(SRC) $(HDR) | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) tests/test_common.c $(SRC) -lm -o $@

test: all
	$(B)/test_common

clean:
	rm -rf $(B)

.PHONY: all test clean
