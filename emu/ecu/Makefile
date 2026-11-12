# System32 Emulator — target-system ECU SoC Emulator (PIC32MX795F512L on pure-C11 MIPS32 M4K interpreter shim).
CC       ?= cc
CFLAGS   ?= -std=gnu11 -O2 -g -Wall -Wextra -Werror -Wno-unused-parameter -Wshadow -Wstrict-prototypes
# No fused multiply-add contraction: a*b+c rounds the same on every CPU (arm64 clang fuses by default, x86-64 gcc and
# WebAssembly do not), so every build computes bit-identical results (docs/conformance.md).
CFLAGS   += -ffp-contract=off
REPO      = ../..
B         = build
PART     ?= PIC32MX795F512L
USE_SHIM ?= 1
SHIM_INC  = -Iinclude -I$(B) -Ishim/include -I../common -I../controller/include
CPPFLAGS  = $(SHIM_INC)
LDFLAGS   =
ECU_OBJS  = $(B)/ecu.o $(B)/s32dev.o $(B)/uc_mips32_shim.o

all: $(B)/libecu.a $(B)/test_ecu $(B)/test_ecu_shim

$(B):
	mkdir -p $(B)

$(B)/sfr_names.h: tools/gen_sfr_names.py $(REPO)/model/pic32mx_sfr.json | $(B)
	python3 -I tools/gen_sfr_names.py $(REPO)/model/pic32mx_sfr.json $(PART) $@

$(B)/irq_map.h: tools/gen_irq_map.py $(REPO)/model/pic32mx_sfr.json | $(B)
	python3 -I tools/gen_irq_map.py $(REPO)/model/pic32mx_sfr.json $(PART) $@

DEV = ../controller/src/s32dev.c

$(B)/ecu.o: src/ecu.c include/ecu.h $(B)/sfr_names.h $(B)/irq_map.h | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(B)/s32dev.o: $(DEV) | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(B)/uc_mips32_shim.o: shim/uc_mips32_shim.c shim/include/unicorn/unicorn.h | $(B)
	$(CC) $(SHIM_INC) $(CFLAGS) -c $< -o $@

$(B)/libecu.a: $(ECU_OBJS)
	ar rcs $@ $^

$(B)/test_ecu: tests/test_ecu.c $(B)/libecu.a
	$(CC) $(CPPFLAGS) -I../pic24/tests $(CFLAGS) $< $(B)/libecu.a $(LDFLAGS) -o $@

$(B)/test_ecu_shim: tests/test_ecu.c src/ecu.c $(DEV) $(B)/uc_mips32_shim.o $(B)/sfr_names.h $(B)/irq_map.h
	$(CC) $(SHIM_INC) -I../pic24/tests $(CFLAGS) tests/test_ecu.c src/ecu.c $(DEV) $(B)/uc_mips32_shim.o -o $@

test: $(B)/test_ecu $(B)/test_ecu_shim
	$(B)/test_ecu
	$(B)/test_ecu_shim

clean:
	rm -rf $(B)

.PHONY: all test clean
