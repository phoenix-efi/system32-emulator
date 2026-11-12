# System32 Emulator — PIC24FJ256DA210 CPU Core Emulator (`libpic24.a` + unit tests).
CC      ?= cc
CFLAGS  ?= -std=gnu11 -O2 -g -Wall -Wextra -Werror -Wno-unused-parameter -Wshadow -Wstrict-prototypes
# No fused multiply-add contraction: a*b+c rounds the same on every CPU (arm64 clang fuses by default, x86-64 gcc and
# WebAssembly do not), so every build computes bit-identical results (docs/conformance.md).
CFLAGS   += -ffp-contract=off
CPPFLAGS = -Iinclude
B        = build
LIB      = $(B)/libpic24.a
SRC      = src/pic24_core.c
OBJ      = $(SRC:src/%.c=$(B)/%.o)
TESTS    = $(B)/test_core
REPO     = ../..
SFRJSON ?= $(REPO)/model/pic24fj256da210_sfr.json

# sfr_reset.h: the SFR reset values from the register model, used by the Controller model (emu/controller).
all: $(LIB) $(TESTS) $(B)/sfr_reset.h

$(B):
	mkdir -p $(B)

$(B)/%.o: src/%.c include/pic24.h | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(LIB): $(OBJ)
	ar rcs $@ $^

$(B)/sfr_reset.h: tools/gen_sfr_reset.py $(SFRJSON) | $(B)
	python3 -I tools/gen_sfr_reset.py $(SFRJSON) $@

$(B)/test_core: tests/test_core.c tests/minitest.h $(LIB)
	$(CC) $(CPPFLAGS) $(CFLAGS) -Itests $< $(LIB) -o $@

test: all
	$(B)/test_core

clean:
	rm -rf $(B)

.PHONY: all test clean
