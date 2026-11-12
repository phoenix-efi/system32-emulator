/* ecu.h -- ECU SoC model (PIC32MX795F512L class, MIPS32 M4K) on the pure-C MIPS32 core in emu/ecu/shim.
 *
 * One ecu_t per emulated ECU: several can run in one process (ECU1 + ECU2 + the Controller). Time is SYSCLK cycles at
 * 80 MHz, 1 instruction = 1 cycle (INFERRED); PBCLK = SYSCLK / 2^OSCCON.PBDIV, loaded at reset from the image's DEVCFG1
 * (the stock images select /2: 40 MHz). Modelled: memory map, SFR storage with CLR/SET/INV
 * portals, GPIO (inputs idle high), the interrupt controller in external (EIC) mode, Timer1-5 (+T32), SPI1-4 masters
 * with chip-select-routed s32_spi_dev slaves, UART1-6 (line-rate RX FIFO with OERR, paced TX to a sink).
 */
#ifndef ECU_H
#define ECU_H
#include "s32_spidev.h"
#include "s32_iocov.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define ECU_SYSCLK_HZ 80000000.0
#define ECU_SFR_SIZE  0x00100000u
#define ECU_UART_RING 8192
#define ECU_PFL_BYTES 0x00080000u   /* 512 KB program flash (PIC32MX795F512L) */
#define ECU_BFL_BYTES 0x00003000u   /* 12 KB boot flash, configuration words in its last 16 bytes */
/* The application window of program flash: physical 0x1D006000 up to 0x1D080000 (observed: the stock boot code hands
 * over to window + 0x1000). An application-only image covers it from its first byte for ECU_APP_FLAT_BYTES (499,700
 * bytes, up to 0x1D07FFF4). Everything below the window and the boot flash are the boot region. */
#define ECU_APP_OFF        0x00006000u
#define ECU_APP_FLAT_BYTES 0x00079FF4u
/* Time from power-up with no card to the first instruction fetch in the application window (observed with the stock
 * images: 204.7 us at 80 MHz). Application start holds the core this long, so the application starts when it would on a
 * device with that boot code. The CPU model sees every instruction fetch; the hand-off point, time and state were
 * measured that way (docs/simulator.md, "Application start"). */
#define ECU_APP_BOOT_CYCLES 16379u
/* The same with a card in the ECU slot (card detect RB3 low at power-up) holding no update file. Observed on several
 * cards: 647,850 to 648,558 cycles (8.10 ms), depending on what the card holds; this is the middle of that range. */
#define ECU_APP_CARD_BOOT_CYCLES 648210u

typedef struct ecu ecu_t;

typedef struct {
  void (*tx)(void *ctx, ecu_t *e, unsigned uart, uint8_t b);   /* transmitted byte sink (NULL = dropped) */
  void *tx_ctx;
  uint64_t tx_busy_until;
  uint8_t ring[ECU_UART_RING]; unsigned rh, rt;                /* host -> pin, not yet on the wire */
  uint8_t fifo[8]; unsigned fn, fn_peak;                       /* PIC32 receive FIFO (8 deep); peak occupancy */
  uint64_t rx_next;                                            /* cycle the next ring byte lands (0 = idle) */
  uint64_t tx_bytes, rx_bytes, rx_overruns;                    /* rx_bytes: into the FIFO; rx_overruns: lost */
} ecu_uart;

typedef struct { s32_spi_dev dev; int port, bit; bool asserted; } ecu_spi_slave;

typedef struct { uint64_t at; uint8_t port, bit, level; } ecu_pin_ev;
#define ECU_PIN_Q 256

/* The idle-loop watch (ecu.c, "idle loops"): the block it waits to come back to and the core registers there. */
#define ECU_IDLE_REGS 78
typedef struct {
  bool active;
  uint32_t head, prev;                         /* prev: the last block's address (a backward jump starts a watch) */
  uint32_t resume, exc0;                       /* resume: a head an interrupt interrupted (watched again on return) */
  unsigned blocks;
  uint64_t io0, deadline, cooldown_until;
  uint32_t regs[ECU_IDLE_REGS];
  uint32_t ifs[3];                             /* IFS0-2 at the start (the model sets flags at CP0 sites) */
} ecu_idle_watch;

struct ecu {
  void *uc;                                    /* uc_engine* */
  uint32_t sfr[ECU_SFR_SIZE / 4];
  uint8_t portal[ECU_SFR_SIZE / 16];
  uint16_t pin_in[7];
  uint64_t cyc;                                /* SYSCLK cycles since reset */
  struct { uint64_t base, count0, irqs; } tm[5];
  struct { ecu_spi_slave dev[4]; unsigned n; uint8_t rx; bool rbf; uint64_t bytes; } spi[4];
  ecu_uart uart[6];
  uint32_t pc;
  uint64_t sfr_reads, sfr_writes, slices;
  uint64_t budget_used, budget_limit;      /* block-hook instruction meter (see ecu_run) */
  uint64_t slice_start, next_event;        /* cycle base of the running slice; cycle of the next timer/UART event */
  uint64_t events_inline;                  /* events serviced inside the block hook */
  uint32_t last_off, last_v; unsigned same_reads;   /* spin detection for the fast-forward */
  uint64_t warped_cycles, warps;
  bool idle_on;                                /* idle-loop skipping (ecu_idle_skip) */
  ecu_idle_watch idle_watch;
  uint64_t io_accesses;                        /* peripheral accesses by the core (SFRs, the core timer) */
  uint64_t idle_skipped, idle_blocks;            /* time skipped in idle loops; how many skips */
  uint32_t *read_hist, *write_hist;
  ecu_pin_ev pinq[ECU_PIN_Q]; unsigned pinq_n;   /* scheduled external pin changes, sorted by time */
  uint64_t int_edges[5];                         /* INT0..4 edges that set their flag */
  /* outputs: coil pulses (OC1-4 dual-compare single pulse) and PORTA latch changes (injectors) */
  void (*oc_sink)(void *ctx, ecu_t *e, unsigned oc, uint64_t on_cyc, uint64_t off_cyc);
  void (*porta_sink)(void *ctx, ecu_t *e, uint64_t cyc, uint16_t old_lat, uint16_t new_lat);
  /* every output-latch change on any port (0 = A .. 6 = G): what a logic analyser on the pins sees (TRIS applied:
   * only bits configured as outputs are reported as driven) */
  void (*lat_sink)(void *ctx, ecu_t *e, uint64_t cyc, unsigned port, uint16_t old_pins, uint16_t new_pins);
  void *lat_ctx;
  double hz;                                     /* SYSCLK cycles per emulated second (ECU_SYSCLK_HZ x crystal error) */
  void *out_ctx;
  uint64_t oc_pulses[4];
  struct { bool armed, risen; uint64_t on_cyc; uint32_t ref; } oc[4];   /* pending dual-compare single pulse per OC1-4;
                                                                       ref = TMR2 count up to which matches were checked */
  uint16_t an[32];                               /* ADC inputs: raw 10-bit count per ANx */
  uint16_t an_noise[32]; uint32_t rng;           /* per-sample uniform noise +-an_noise counts (deterministic) */
  uint64_t adc_done, adc_irqs;                   /* cycle the running scan completes (0 = idle) */
  uint8_t  nvm_key_step;                         /* NVMKEY unlock state: 0=idle, 1=saw 0xAA996655, 2=saw 0x556699AA */
  uint32_t nvm_erases, nvm_writes;               /* flash page erases (0x4004) and word/row writes (0x4001/0x4003) */
  uint64_t cp0_count_base_cyc;                   /* SYSCLK cycle at last WriteCoreTimer (mtc0 rt, $9, 0) */
  uint32_t cp0_count_base_val;                   /* CP0_Count value written at cp0_count_base_cyc */
  int last_err;                                /* uc_err of the last run slice (0 = ok): the core has stopped */
  /* watchdog timer (DS61114 section 9): enabled by DEVCFG1.FWDTEN or WDTCON.ON, period 1 ms x 2^WDTPS (LPRC), cleared
   * by WDTCON.WDTCLR; it keeps counting while the core is stopped on an exception. Expiry resets the core (the front
   * end performs the reset between quanta: ecu_wdt_poll). */
  bool wdt_fuse; uint32_t wdt_ps; uint64_t wdt_clear_cyc;
  /* the program flash chip's contents (what a device programmer reads back): the image at power-on as erased and
   * programmed since through NVMCON; a power cycle re-initialises the core from it (ecu_pflash_read) */
  uint8_t pfl_chip[ECU_PFL_BYTES];
  /* per program-flash word: which coprocessor-0 access the SoC model handles there (if any) and whether its code hook
   * is installed (ecu.c, "CP0 sites in program flash"); rebuilt for every word the chip changes */
  uint8_t cp0_site[ECU_PFL_BYTES / 4];
  s32_iocov *cov;                               /* optional I/O coverage recorder (ecu_iocov_enable; NULL = off) */
  /* application start (ecu.c, "application start"): the boot region is absent, so the core starts at the application's
   * start address in the state the boot code hands it over in; the peripheral part is applied over the bus at the
   * first ecu_run_to, so the pins it drives reach the front end's sinks at that moment */
  bool app_start, handoff_pending;
  uint64_t boot_cycles;                         /* hold the core this long before the hand-off: ECU_APP_BOOT_CYCLES (the
                                                 * boot code's run time with no card); a front end may set 0 = at once */
  uint64_t handoff_at;                          /* cycle of the hand-off (set at the first ecu_run_to) */
  bool handoff_card;                            /* a card was in the slot at power-up: the card variant of the hand-off */
};

/* Create from boot + program flash images; returns 0 on success. With no boot region (nb == 0, or a boot flash whose
 * reset vector is erased) the ECU starts in application-start mode (ecu.c, "application start"): the program flash below
 * the application window is erased, the configuration words are the declared ones (ecu_app_boot_flash) and the core
 * starts at the application's start address with the declared hand-off state. */
int ecu_init(ecu_t *e, const uint8_t *bflash, size_t nb, const uint8_t *pflash, size_t np);
/* An erased boot flash holding only the declared configuration words (ECU_BFL_BYTES): what application-start mode runs
 * with. Front ends pass it to ecu_init (and to their clock check) when no boot flash image is given. */
void ecu_app_boot_flash(uint8_t *bfl, size_t n);
/* Place a program-flash image file into a chip-sized buffer (ECU_PFL_BYTES, filled with 0xFF first): a full image from
 * the start of program flash, an application-only image (exactly ECU_APP_FLAT_BYTES) at the application window.
 * Returns the bytes of the buffer that hold the image's end (what ecu_init's np should be). */
size_t ecu_place_pflash(uint8_t *chip, const uint8_t *img, size_t n);
void ecu_free(ecu_t *e);
/* Run `cycles` SYSCLK cycles (stops early on a CPU error: e->last_err). */
int ecu_run(ecu_t *e, uint64_t cycles);
/* Run until the cycle counter reaches `target` (absolute): co-simulation uses this so block-granular overshoot never
 * accumulates across quanta (it did: ~2 deg/s spark-angle drift against the engine model). */
int ecu_run_to(ecu_t *e, uint64_t target);
void ecu_spi_attach(ecu_t *e, unsigned n, int port, int bit, s32_spi_dev dev);
void ecu_uart_sink(ecu_t *e, unsigned n, void (*tx)(void *, ecu_t *, unsigned, uint8_t), void *ctx);
void ecu_uart_feed(ecu_t *e, unsigned n, const uint8_t *b, size_t len);
void ecu_set_pin(ecu_t *e, unsigned port, unsigned bit, bool level);
uint32_t ecu_reg(ecu_t *e, int uc_reg);        /* read a CPU register (UC_MIPS_REG_*) */
const char *ecu_sfr_name(uint32_t kseg1);
/* Debugger/test access to the SFR bus at a KSEG1 address, with the same semantics as a guest access (portals,
 * peripheral side effects). size = 1, 2 or 4. */
uint32_t ecu_bus_read(ecu_t *e, uint32_t kseg1, unsigned size);
void ecu_bus_write(ecu_t *e, uint32_t kseg1, unsigned size, uint32_t value);
uint64_t ecu_timer_irqs(const ecu_t *e, int t);   /* t = 1..5 */
/* Idle-loop skipping (on = true) or none, as the Controller model's s32ct_idle_skip: when the core goes once round a
 * loop of blocks with every register as it was, no RAM value changed, no peripheral accessed and no interrupt pending,
 * the model moves time on to the next timer / UART / pin event (or the slice end) and checks the loop again after it. */
void ecu_idle_skip(ecu_t *e, bool on);
/* MIPS32 instruction encoders (field layout from the MIPS32 Architecture manual, Vol. II), so instruction patterns are
 * written as instructions, not as byte strings. */
enum { MIPS_ZERO = 0, MIPS_V0 = 2, MIPS_A0 = 4, MIPS_SP = 29, MIPS_FP = 30, MIPS_RA = 31 };
enum { MIPS_OP_J = 0x02, MIPS_OP_JAL = 0x03, MIPS_OP_ADDIU = 0x09, MIPS_OP_ORI = 0x0D,
       MIPS_OP_COP0 = 0x10, MIPS_OP_LW = 0x23, MIPS_OP_SW = 0x2B };
enum { MIPS_FN_SLL = 0x00, MIPS_FN_JR = 0x08, MIPS_FN_ADDU = 0x21, MIPS_COP0_MF = 0x00, MIPS_COP0_MT = 0x04,
       MIPS_CP0_CAUSE = 13 };
#define MIPS_I(op, rs, rt, imm) \
  ((uint32_t)(op) << 26 | (uint32_t)(rs) << 21 | (uint32_t)(rt) << 16 | ((uint32_t)(imm) & 0xFFFFu))
#define MIPS_R(rs, rt, rd, sa, fn) \
  ((uint32_t)(rs) << 21 | (uint32_t)(rt) << 16 | (uint32_t)(rd) << 11 | (uint32_t)(sa) << 6 | (uint32_t)(fn))
#define MIPS_COP0(mt, rt, rd, sel) \
  ((uint32_t)MIPS_OP_COP0 << 26 | (uint32_t)(mt) << 21 | (uint32_t)(rt) << 16 | (uint32_t)(rd) << 11 | (uint32_t)(sel))
/* Schedule an external pin level change at emulated cycle `at` (call between ecu_run calls). INT0..INT4 pins
 * (RF6/RE8/RE9/RA14/RA15 on the 795L) raise their interrupt flag on the edge INTCON.INTxEP selects. */
/* Set analog input ANch (raw 10-bit count = volts / 3.3 * 1023). */
void ecu_set_an(ecu_t *e, unsigned ch, uint16_t raw);
void ecu_schedule_pin(ecu_t *e, uint64_t at, unsigned port, unsigned bit, bool level);
/* Start recording I/O coverage (SFR words, sampled input pins, output pins, ADC channels): s32_iocov.h. */
void ecu_iocov_enable(ecu_t *e);
/* Stored value of the SFR at a KSEG1 address (no side effects; 0 outside the SFR space). */
uint32_t ecu_sfr_value(const ecu_t *e, uint32_t kseg1);
/* True when the watchdog is enabled and has not been cleared for its period (call between quanta; the caller then
 * resets the core, and may mark the cause with ecu_mark_reset_cause(e, ECU_RCON_WDTO) after re-initialising). */
bool ecu_wdt_poll(const ecu_t *e);
#define ECU_RCON_WDTO (1u << 4)
void ecu_mark_reset_cause(ecu_t *e, uint32_t rcon_bits);
/* Program flash contents now (erases and programming since power-on included): what an ICSP readback gives. Copies
 * min(cap, ECU_PFL_BYTES) bytes; returns the count. */
size_t ecu_pflash_read(const ecu_t *e, uint8_t *dst, size_t cap);
/* Peripheral bus clock: e->hz / 2^OSCCON.PBDIV (set from DEVCFG1.FPBDIV at reset, changeable by software). */
double ecu_pbclk_hz(const ecu_t *e);
/* True when `err` (an ecu.last_err / uc_err code) is an internal emulator fault (e.g. UC_ERR_RESOURCE when the hook
 * table overflows) rather than a guest CPU fault (invalid instruction, unmapped/unaligned access, software exception). */
bool ecu_is_emu_err(int err);
const char *ecu_err_str(int err);
/* Snapshots (emu/common/s32snap.h): the ECU's whole state (SoC and core) in ecu_snap_size() bytes, pointers zeroed;
 * restoring keeps the live ECU's pointers (devices, sinks). 0 / -1 when the core engine cannot snapshot. */
size_t ecu_snap_size(void);
int ecu_snap_save(const ecu_t *e, uint8_t *out);
int ecu_snap_restore(ecu_t *e, const uint8_t *in, size_t n);

#endif
