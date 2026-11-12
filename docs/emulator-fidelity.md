# Emulator fidelity

What the emulator models, how closely, how that is checked, and the known limits. Evidence is labelled **READ** (from
a Microchip data sheet or Family Reference Manual, cited), **OBSERVED** (seen on the emulated system; a command anyone
can rerun), **MEASURED** (on hardware) and **INFERRED** (reasoning over the others). Every external signal, with its
model, faults, knob, test and evidence, is in [io-ledger.md](io-ledger.md); build-to-build agreement (native = WebAssembly = browser) is in
[conformance.md](conformance.md).

## What is modelled

* **Cockpit Controller** (PIC24FJ256DA210): the CPU core with the instruction timings of the Programmer's Reference
  Manual, interrupts, timers, ADC, UARTs, SPI, the flash controller, the graphics controller and the 320 x 240 panel,
  the FRAM, the real-time clock, the SD card, the bezel buttons and the trim knob.
* **ECUs** (PIC32MX795F512L, two of them): a MIPS32 M4K core at one instruction per SYSCLK cycle, the interrupt
  controller, timers, UARTs, SPI, ADC, output compares, the program and boot flash, the watchdog, the FRAM and the SD
  card.
* **The link** between them: bytes delivered at the programmed baud over the two cat5 cables, decoded by a passive
  analyser (`emu/common/link_decode.c`).
* **The harness and engine**: crank and cam sensors, coils, injectors, analogue sensors, switches, supplies and an
  engine model, with injectable faults (`emu/pair/s32pair.c --knobs`).

The emulator observes the system only the way a bench could: pins, wires, the panel and the chips' contents
([simulator.md](simulator.md)).

## Controller UART timing against the data sheets (READ)

Each row compares the silicon document with the model; "match" means no behavioural difference that could move a byte
across the FIFO boundary.

| aspect | silicon (READ) | emulator | agreement |
|---|---|---|---|
| baud formula | baud = Fp / (16 x (UxBRG+1)) with BRGH = 0, Fp / (4 x (UxBRG+1)) with BRGH = 1 (FRM UART, DS70000582E, section 3, Equations 3-1 / 3-3) | `uart_byte_tcy` = 10 x (BRGH ? 4 : 16) x (UxBRG+1) Tcy per 8N1 byte | match |
| baud as programmed | link physical rate 38,462 Bd | OBSERVED: U3/U4 BRG = 25, BRGH = 0 -> 38,461.5 Bd at Fp = 16 MHz; byte = 4160 Tcy | match |
| delivery rate into the FIFO | wire bytes arrive at the sender's baud | host-fed bytes re-pace into the FIFO at the Controller's own BRG (`s32ct_uart_feed` / `uart_rx_land`); OBSERVED land spacing 4158-4162 Tcy, mean exactly 4160 (the -2 is instruction-granularity rounding at slice stops and never accumulates) | never faster than the programmed baud |
| receive FIFO depth, OERR trip | 4-deep receive FIFO; a fifth word fully received into UxRSR with the FIFO full sets OERR and is held there (DS70000582E section 7.2; DS39969B Register 17-2). Figure 7-3's note ("start of the 6th character") conflicts with the 7.2 text; the text is implemented | 4-deep FIFO + held fifth word; OERR at the fifth (unit test `overrun_trips_on_the_fifth_unread_word`) | match |
| after OERR | "further transfers to the receive FIFO and UART interrupts are inhibited" while OERR is set; clearing OERR resets the FIFO and RSR to empty (DS70000582E 7.2 and its note; Register 17-2 bit 1 is R/C-0, HS) | same, unit-tested (`reception_inhibited_while_oerr_set`, `oerr_clear_resets_the_fifo_and_writes_cannot_set_it`) | match |
| receive interrupt select | URXISEL<1:0>: 0x = flag on any transfer into the buffer, 10 = the transfer making 3 characters, 11 = making 4; an event per transfer (DS39969B Register 17-2 bits 7:6; the FRM 7.3 "three or four" wording for 10 conflicts with the data sheet's "has 3 data characters" — the data sheet is implemented) | same | match |
| interrupt priority | per-source IPC nibble, level must exceed IPL to preempt, natural order breaks ties (PIC24F FRM section 8, DS39707A; DS39969B Table 7-2: U3RX 82, U4RX 88, U3ER 81, U4ER 87) | same | match |
| interrupt latency | current instruction completes, then FNOP / vector fetch / FNOP, first ISR instruction in the fifth cycle (DS39707A, Fig 8-3) | fixed 3 Tcy entry after the current instruction | silicon's per-instruction jitter (about ±1 Tcy) is not modelled; immaterial against a 4160-Tcy byte |
| instruction timing | 1 Tcy base; 2 for a taken branch / call / goto / table op / MOV.D; 3 for a return (2 with an exception pending); skips +1/+2; DIV as 18 REPEAT iterations (Programmer's Reference Manual, DS70000157G) | same, plus the RAW pipeline stall and the PSV/EDS read wait states; PIC24F flash needs no wait states at 16 MHz | match |
| Fcy | config words decode to 8 MHz XT x 96 MHz PLL, Fosc 32 MHz, Fcy 16 MHz (DS39969B section 8) | the emulator's Controller clock is 16 MHz and checks the images' config words at startup (`s32board_clock_check` prints the decode and warns on any mismatch; the stock images agree) | match |

Bytes are never delivered faster than the programmed baud: the observed land spacing is the programmed byte time, and
the producers (`ecu_uart`'s transmit into the quantum outbox, the barrier's `s32board_link` pass-through, the
Controller model's ring pacing) leave no path that could beat it. The Controller CPU does not run slower than Fcy: the
cycle is the time base, the instruction costs follow the PRM, and the image is fused for the clock the emulator runs.

## Known limits

* Interrupt entry takes a fixed 3 Tcy after the current instruction; silicon's per-instruction jitter (about ±1 Tcy)
  is not modelled.
* ECU timing is one instruction per SYSCLK cycle (INFERRED); cache and flash wait-state timing is not modelled.
* Idle loops are not executed instruction by instruction: once a loop has gone round with nothing changed, time moves
  on to the next event (`s32ct_idle_skip`, `ecu_idle_skip`), and a repeated read of an unchanged UART or SPI status
  moves on to when it changes. The loop is watched once round again after every event, so what it does is unchanged;
  where inside the loop an interrupt finds the CPU can differ from silicon by up to one time round.
  `S32_NOIDLESKIP=1` executes every instruction.
* Where the UART Family Reference Manual's figures and text disagree (the OERR point, URXISEL = 10), the text and the
  data sheet are implemented (table above).
* At power-on the emulated Controller drops some link bytes while the link starts (the end-of-run report counts them
  per UART); not confirmed on hardware.
* Panel-controller register contents are not decoded; panel faults (dead pixels, no backlight) are not modelled.
* Injector and coil driver-stage faults are not modelled; which driver pin reaches which cylinder is not documented and
  is modelled as RAn -> injector n+1.
* Snapshots restore only into the build that wrote them; the browser bench's module cannot load ECU SD cards or
  per-ECU program-flash images ([conformance.md](conformance.md)).
* Per-signal limits and evidence: [io-ledger.md](io-ledger.md).
