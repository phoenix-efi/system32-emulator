/* s32ct.h -- Cockpit Controller SoC model (PIC24FJ256DA210 peripherals around emu/pic24).
 *
 * The core owns W regs, SR, CORCON, the interrupt controller and RAM. This library owns every other SFR in
 * 0x0000-0x07FF: storage initialised from the POR table (model/pic24fj256da210_sfr.json), plus behaviour for the
 * peripherals modelled so far. Each peripheral is added as one small step with its own tests.
 */
#ifndef S32CT_H
#define S32CT_H
#include "pic24.h"
#include "s32gfx.h"
#include <stdio.h>

#include "s32_spidev.h"   /* the SPI slave interface (emu/common) */
#include "s32_iocov.h"    /* I/O coverage recorder (emu/common) */

/* A chip-select: a port latch bit (port index 0 = A .. 6 = G). */
typedef struct { uint8_t port, bit; } s32_pin;

#define S32_MAX_SPI_DEVS 4

typedef struct {
  uint16_t stat, con1, con2;
  uint8_t rxbuf;
  bool rbf;               /* SPIRBF: a received byte is waiting */
  uint64_t busy_until;    /* Tcy at which the byte in flight completes */
  uint8_t pending_rx;
  struct { s32_pin cs; s32_spi_dev dev; bool asserted; } devs[S32_MAX_SPI_DEVS];
  unsigned ndevs;
} s32_spi;

/* Timer1..5 (16-bit modes; T32 pairing not modelled yet). Counted lazily: TMR = count0 + (cycles - base) / prescale. */
typedef struct {
  uint16_t con, pr;
  uint64_t base;          /* Tcy at which `count0` was valid */
  uint32_t count0;
} s32_timer;

/* UART1..4: TX paced at 10 bit-times per byte to a sink; RX bytes fed by the host enter a 4-deep FIFO at line rate.
 * Receive semantics per DS39969B (this chip's data sheet) Register 17-2 and FRM DS70000582E 4.2, 7.2, 7.3: with the
 * FIFO full the fifth word completing into UxRSR sets OERR and is held there; while OERR is set transfers to the
 * FIFO and UART interrupts are inhibited (later words are lost); clearing OERR in software resets the FIFO and the
 * RSR to empty; the RX interrupt flag is set on an RSR-to-FIFO transfer as URXISEL<1:0> selects. */
#define S32_UART_RING 8192
typedef struct {
  void (*tx)(void *ctx, unsigned uart, uint8_t b);   /* sink for transmitted bytes (NULL = dropped) */
  void *tx_ctx;
  uint64_t tx_busy_until;
  uint8_t ring[S32_UART_RING]; unsigned rh, rt;      /* host -> pin, not yet on the wire */
  uint8_t fifo[4]; unsigned fn, fn_peak;             /* hardware receive FIFO; peak occupancy */
  uint8_t rsr; bool rsr_full;                        /* UxRSR: the word received into a full FIFO (OERR) is held here */
  uint64_t rx_next;                                  /* Tcy when the next ring byte lands in the FIFO (0 = idle) */
  uint64_t tx_bytes, rx_bytes, rx_overruns;          /* rx_bytes: fully received (FIFO or held UxRSR); rx_overruns: lost */
} s32_uart;

/* The idle-loop watch (s32ct.c, "idle loops"): the instruction it waits to come back to and the core registers there. */
#define S32CT_IDLE_REGS 80
typedef struct {
  bool active;
  uint32_t head, resume;                 /* resume: a head an exception interrupted (watched again on return) */
  unsigned steps, cooldown;
  uint64_t io0;
  uint32_t exc0;
  uint16_t regs[S32CT_IDLE_REGS];
} s32ct_idle_watch;

typedef struct s32ct {
  pic24_cpu cpu;
  uint16_t sfr[0x400];    /* word storage for non-core SFRs, index = addr/2 */
  s32_spi spi[3];         /* SPI1..SPI3 */
  s32_timer tmr[5];       /* Timer1..Timer5 */
  s32_gfx gfx;            /* graphics controller G1 */
  uint64_t next_frame;    /* Tcy of the next vblank (0 = display not running) */
  s32_uart uart[4];       /* UART1..UART4 */
  uint16_t pin_in[7];     /* external level on each port's INPUT pins (A..G); default all high (pull-ups) */
  uint16_t an[32];        /* ADC inputs, raw 10-bit counts per ANx (the pins' voltage x 1023 / AVdd) */
  uint64_t adc_done;      /* Tcy at which the current scan sequence completes (0 = idle) */
  uint64_t adc_irqs;
  bool idle_on;           /* idle-loop skipping (s32ct_idle_skip; s32ct.c, "idle loops") */
  s32ct_idle_watch idle_watch;
  uint64_t io_accesses;   /* peripheral accesses by the core (SFR reads / writes, table writes) since power-on */
  uint64_t idle_skipped;
  uint64_t slice_end;
  uint16_t last_sfr_addr, last_sfr_val;
  unsigned same_sfr_reads;
  uint64_t timer_irqs[5]; /* period matches raised, per timer */
  FILE *log;              /* optional event log (SPI transactions, CS edges) */
  uint64_t spi_bytes;
  uint32_t nvm_latch[64]; /* 64-instruction row write latches (24-bit words, reset to 0xFFFFFF) */
  uint32_t nvm_addr;      /* 24-bit PC address latched by most recent TBLWTL/TBLWTH */
  uint8_t  nvm_key_step;  /* NVMKEY unlock state: 0=idle, 1=saw 0x55, 2=saw 0xAA */
  uint32_t nvm_erases;    /* flash page erases completed (NVMCON 0x4042) */
  uint32_t nvm_writes;    /* flash row/word writes completed (NVMCON 0x4001/0x4003) */
  uint32_t illegal_resets;/* device resets on an illegal opcode / address mode / BREAK without debugger (RCON.IOPUWR) */
  uint32_t sw_resets;     /* RESET-instruction resets (RCON.SWR) */
  uint32_t trap_resets;   /* hard-trap-conflict resets (RCON.TRAPR) */
  s32_iocov *cov;         /* optional I/O coverage recorder (s32ct_iocov_enable; NULL = off) */
  /* every driven-pin change on a port (0 = A .. 6 = G; LAT where TRIS = 0), as a logic analyser on the pins sees it */
  void (*lat_sink)(void *ctx, struct s32ct *s, uint64_t cyc, unsigned port, uint16_t old_pins, uint16_t new_pins);
  void *lat_ctx;
} s32ct;

/* RCON reset-cause flags (DS39969 Register 6-1). */
#define S32CT_RCON_POR    0x0001u
#define S32CT_RCON_BOR    0x0002u
#define S32CT_RCON_SWR    0x0040u
#define S32CT_RCON_EXTR   0x0080u
#define S32CT_RCON_IOPUWR 0x4000u
#define S32CT_RCON_TRAPR  0x8000u

/* Power-on: everything at POR values (RCON = POR | BOR). PIC24_ILLEGAL_STOPS=1 / PIC24_DEBUGGER=1 in the environment set
 * the core's debug switches (pic24_config.illegal_stops / .debugger); unset = silicon behaviour. */
void s32ct_init(s32ct *s, const uint8_t *prog, size_t nprog);
/* MCLR: warm-reset the Controller CPU and SFRs while preserving current program flash (s->cpu.prog) and attached
 * devices. RCON keeps its flags and sets EXTR. Resets the core performs itself (RESET instruction, illegal opcode,
 * BREAK without a debugger, trap conflict) take the same peripheral path inside s32ct_run and set SWR / IOPUWR / TRAPR. */
void s32ct_reset(s32ct *s);
/* Attach a device to SPIn (n = 1..3) selected by a latch bit. */
void s32ct_spi_attach(s32ct *s, unsigned n, s32_pin cs, s32_spi_dev dev);
/* Run the core for `cycles` Tcy, stopping at each timer period match to raise its interrupt flag. Use this instead of
 * pic24_run() so timers fire on time without per-instruction polling. */
pic24_status s32ct_run(s32ct *s, uint64_t cycles);
/* Run until cpu.cycles reaches `target` (absolute; no accumulating overshoot across co-simulation quanta). */
pic24_status s32ct_run_to(s32ct *s, uint64_t target);
/* UARTn (n = 1..4): attach a transmit sink; feed received bytes (queued, delivered at line rate). */
void s32ct_uart_sink(s32ct *s, unsigned n, void (*tx)(void *ctx, unsigned uart, uint8_t b), void *ctx);
void s32ct_uart_feed(s32ct *s, unsigned n, const uint8_t *b, size_t len);
/* Drive the external level of pin port.bit (an input; e.g. a button: 0 = pressed for an active-low switch). */
void s32ct_set_pin(s32ct *s, unsigned port, unsigned bit, bool level);
/* Idle-loop skipping (on = true) or none. When the CPU goes once round a loop and comes back to the same instruction
 * with every register as it was, no data-memory value changed, no peripheral accessed and no exception taken, every
 * later time round is the same until a peripheral event changes something: the model then moves time on to the next
 * event instead of executing, and checks the loop again after it. It needs no knowledge of the program. Also enables
 * the status-poll skip (a repeated read of an unchanged UART / SPI status that only time will change). */
void s32ct_idle_skip(s32ct *s, bool on);
/* Set analog input ANch (raw 10-bit count). */
void s32ct_set_an(s32ct *s, unsigned ch, uint16_t raw);
/* Current value of an SFR (data address < 0x0800; 0 otherwise), as a register-level bus monitor reads it back. */
uint16_t s32ct_sfr_value(s32ct *s, uint16_t addr);
/* Start recording I/O coverage (every SFR word incl. core-owned ones, sampled input pins, output pins, ADC channels). */
void s32ct_iocov_enable(s32ct *s);
/* Latch value of port p (0 = A .. 6 = G). */
uint16_t s32ct_lat(const s32ct *s, unsigned p);
/* Program flash contents now (erases and programming since power-on included), packed 3 bytes per 24-bit instruction
 * word (little-endian, same layout as s32ct_init's input): what an ICSP readback gives. Copies min(cap, S32CT_PFL_BYTES)
 * bytes; returns the count. */
#define S32CT_PFL_BYTES (PIC24_PROG_WORDS * 3u)
size_t s32ct_pflash_read(const s32ct *s, uint8_t *dst, size_t cap);

#endif
