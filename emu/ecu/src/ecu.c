/* ecu.c -- target-system ECU SoC model (see ecu.h). Register semantics: PIC32MX FRM sections 8 (interrupts), 12 (ports),
 * 14 (timers), 21 (UART), 23 (SPI); addresses from model/pic32mx_sfr.json (build/sfr_names.h, build/irq_map.h). */
#include "ecu.h"
#include <unicorn/unicorn.h>
#include <stdlib.h>
#include <string.h>
#include "sfr_names.h"
#include "irq_map.h"

#define RAM_BASE 0x00000000u
#define RAM_SIZE 0x00020000u
#define PFL_BASE 0x1D000000u
#define PFL_SIZE 0x00080000u
#define BFL_BASE 0x1FC00000u
#define BFL_SIZE 0x00003000u
#define SFR_BASE 0x1F800000u

#define UC(e) ((uc_engine *)(e)->uc)
#define REG(e, off) ((e)->sfr[(off) / 4])

/* ---------------------------------------------------------------- names */
const char *ecu_sfr_name(uint32_t kseg1) {
  unsigned lo = 0, hi = sizeof SFR_NAMES / sizeof SFR_NAMES[0];
  while (lo < hi) { unsigned m = (lo + hi) / 2; if (SFR_NAMES[m].addr < kseg1) lo = m + 1; else hi = m; }
  return lo < sizeof SFR_NAMES / sizeof SFR_NAMES[0] && SFR_NAMES[lo].addr == kseg1 ? SFR_NAMES[lo].name : NULL;
}

uint32_t ecu_reg(ecu_t *e, int r) { uint32_t v = 0; uc_reg_read(UC(e), r, &v); return v; }


/* ---------------------------------------------------------------- interrupt controller (EIC) */
#define IC_OFF 0x00081000u
static void set_ifs(ecu_t *e, unsigned irq) { REG(e, IC_OFF + 0x30u + 0x10u * (irq / 32)) |= 1u << (irq % 32); }

static void eic_eval(ecu_t *e) {
  int best_v = -1, best_p = 0, best_s = -1;
  for (unsigned w = 0; w < 3; w++) {
    uint32_t p = REG(e, IC_OFF + 0x30u + 0x10u * w) & REG(e, IC_OFF + 0x60u + 0x10u * w);
    while (p) {
      unsigned b = (unsigned)__builtin_ctz(p);
      p &= p - 1u;
      unsigned irq = w * 32u + b;
      if (irq >= PIC32_NIRQ) break;
      int v = IRQ_VEC[irq];
      if (v < 0) continue;
      uint32_t ipc = REG(e, IC_OFF + 0x90u + 0x10u * (unsigned)(v / 4)) >> ((v % 4) * 8);
      int pr = (int)((ipc >> 2) & 7u), sp = (int)(ipc & 3u);
      if (pr == 0) continue;
      if (pr > best_p || (pr == best_p && (sp > best_s || (sp == best_s && v < best_v)))) { best_p = pr; best_s = sp; best_v = v; }
    }
  }
  uint32_t cause = ecu_reg(e, UC_MIPS_REG_CP0_CAUSE);
  uint32_t nc = (cause & ~0xFC00u) | ((uint32_t)best_p << 10);
  uint32_t vec1 = best_v >= 0 ? (uint32_t)best_v + 1u : 0u;
  uc_reg_write(UC(e), UC_MIPS_REG_CP0_EICVEC, &vec1);
  if (nc != cause) uc_reg_write(UC(e), UC_MIPS_REG_CP0_CAUSE, &nc);
  REG(e, IC_OFF + 0x10u) = best_v >= 0 ? ((uint32_t)best_p << 8) | (uint32_t)best_v : 0;
}

/* ---------------------------------------------------------------- CP0 sites in program flash
 * The CPU core does not model the side effects of three coprocessor-0 accesses, so the SoC
 * model handles them at the instruction itself: `mtc0 rt, Cause` (software interrupt request: sets CS0IF/CS1IF),
 * `mtc0 rt, Count` (core timer write) and `mfc0 rt, Count` (core timer read; the word the core executes is replaced
 * by `addu rt, rt, $0`, a no-op, and the hook writes rt before it runs). Each program-flash word has a site record
 * (cp0_site): its kind, from the word now in the chip, and whether a code hook is already installed there. Hooks are
 * never removed, so a hook acts only while the word at its address is still a site of that kind. The records are
 * rebuilt for every word the chip changes (power-on, page erase, word and row program), so an image the bootloader
 * programs gets the same handling as one present at power-on, and a hook left at an address whose word was
 * reprogrammed does nothing. */
enum { CP0_SITE_NONE = 0, CP0_SITE_MTC0_CAUSE = 1, CP0_SITE_MTC0_COUNT = 2, CP0_SITE_MFC0_COUNT = 3,
       CP0_SITE_KIND = 0x03, CP0_SITE_HOOKED = 0x80 };

static uint32_t pfl_word(const ecu_t *e, uint32_t off) { uint32_t w; memcpy(&w, e->pfl_chip + off, 4); return w; }

static unsigned cp0_site_kind(uint32_t w) {
  if ((w & 0xFFE0FFFFu) == 0x40806800u) return CP0_SITE_MTC0_CAUSE;
  if ((w & 0xFFE0FFFFu) == 0x40804800u) return CP0_SITE_MTC0_COUNT;
  if ((w & 0xFFE0FFFFu) == 0x40004800u) return CP0_SITE_MFC0_COUNT;
  return CP0_SITE_NONE;
}

static void hook_cp0_site(uc_engine *uc, uint64_t addr, uint32_t size, void *ud) {
  (void)size;
  ecu_t *e = ud;
  uint32_t off = (uint32_t)(addr & 0x1FFFFFFFull) - PFL_BASE;   /* KSEG0 -> physical (memory is mapped physically) */
  if (off >= PFL_SIZE) return;
  unsigned kind = e->cp0_site[off / 4u] & CP0_SITE_KIND;
  if (kind == CP0_SITE_NONE) return;                         /* the word was reprogrammed: this hook is stale */
  int rt = (int)((pfl_word(e, off) >> 16) & 31u);
  if (kind == CP0_SITE_MTC0_CAUSE) {
    uint32_t v = 0; uc_reg_read(uc, UC_MIPS_REG_0 + rt, &v);
    if (v & 0x100u) REG(e, IC_OFF + 0x30u) |= 1u << 1;        /* CS0IF */
    if (v & 0x200u) REG(e, IC_OFF + 0x30u) |= 1u << 2;        /* CS1IF */
    /* Not raised here: this hook runs before the mtc0 commits, and taking the interrupt now would leave EPC on the mtc0
     * itself, so it would run again on return and request the interrupt again, without end. A CP0 write ends the block,
     * so evaluating at the next block start is after the mtc0, as on silicon. */
    e->next_event = e->cyc;
  } else if (kind == CP0_SITE_MTC0_COUNT) {
    e->io_accesses++;
    uint32_t v = 0; uc_reg_read(uc, UC_MIPS_REG_0 + rt, &v);
    e->cp0_count_base_cyc = e->cyc;
    e->cp0_count_base_val = v;
  } else {
    e->io_accesses++;
    uint32_t cnt = e->cp0_count_base_val + (uint32_t)((e->cyc - e->cp0_count_base_cyc) / 2u);
    uc_reg_write(uc, UC_MIPS_REG_0 + rt, &cnt);
  }
}

/* Rebuild the site record of the program-flash word at byte offset `off` from the chip's contents; the core's copy of
 * the word must already equal the chip's. Returns 0, or the engine's error when no hook could be installed. */
static int cp0_site_update(ecu_t *e, uint32_t off) {
  uint32_t w = pfl_word(e, off);
  unsigned kind = cp0_site_kind(w);
  uint8_t *site = &e->cp0_site[off / 4u];
  *site = (uint8_t)((*site & CP0_SITE_HOOKED) | kind);
  if (kind == CP0_SITE_NONE) return 0;
  if (kind == CP0_SITE_MFC0_COUNT) {
    uint32_t rt = (w >> 16) & 31u, addu_nop = MIPS_R(rt, MIPS_ZERO, rt, 0, MIPS_FN_ADDU);
    uc_mem_write(UC(e), PFL_BASE + off, &addu_nop, 4);
  }
  if (*site & CP0_SITE_HOOKED) return 0;
  uc_hook h; uint64_t a = 0x9D000000ull + off;
  uc_err err = uc_hook_add(UC(e), &h, UC_HOOK_CODE, hook_cp0_site, e, a, a);
  if (err) return (int)err;
  *site |= CP0_SITE_HOOKED;
  return 0;
}

/* After the chip changed [off, off + n): rebuild those site records. A hook that cannot be installed stops the core
 * (last_err), as an emulation error must not pass silently. */
static void cp0_sites_reprogrammed(ecu_t *e, uint32_t off, uint32_t n) {
  for (uint32_t o = off; o < off + n; o += 4u) {
    int err = cp0_site_update(e, o);
    if (err && !e->last_err) { e->last_err = err; uc_emu_stop(UC(e)); }
  }
}

/* ---------------------------------------------------------------- GPIO */
#define PORT_OFF 0x00086000u
void ecu_set_pin(ecu_t *e, unsigned port, unsigned bit, bool level) {
  if (port < 7 && bit < 16) e->pin_in[port] = (uint16_t)((e->pin_in[port] & ~(1u << bit)) | ((unsigned)level << bit));
}
static inline int port_of(uint32_t off, uint32_t *reg) {
  if (off < PORT_OFF || off >= PORT_OFF + 7 * 0x40) return -1;
  *reg = (off - PORT_OFF) & 0x3Fu;
  return (int)((off - PORT_OFF) / 0x40);
}

/* External interrupts INT0..4 (DS61143 section 12 + FRM 8): pins RF6, RE8, RE9, RA14, RA15 (PIC32MX795F512L pinout);
 * INTCON bit n = INTnEP (1 = rising edge); IRQs 3, 7, 11, 15, 19. */
static const uint8_t INT_PORT[5] = {5, 4, 4, 0, 0}, INT_BIT[5] = {6, 8, 9, 14, 15};
static const unsigned INT_IRQ[5] = {3, 7, 11, 15, 19};

static void pin_change(ecu_t *e, unsigned port, unsigned bit, bool level) {
  bool old = (e->pin_in[port] >> bit) & 1u;
  ecu_set_pin(e, port, bit, level);
  if (old == level) return;
  uint32_t intcon = REG(e, IC_OFF);
  for (int n = 0; n < 5; n++)
    if (INT_PORT[n] == port && INT_BIT[n] == bit && (bool)((intcon >> n) & 1u) == level) { set_ifs(e, INT_IRQ[n]); e->int_edges[n]++; }
}

void ecu_schedule_pin(ecu_t *e, uint64_t at, unsigned port, unsigned bit, bool level) {
  if (e->pinq_n >= ECU_PIN_Q) return;
  unsigned i = e->pinq_n++;
  while (i > 0 && e->pinq[i - 1].at > at) { e->pinq[i] = e->pinq[i - 1]; i--; }
  e->pinq[i] = (ecu_pin_ev){at, (uint8_t)port, (uint8_t)bit, (uint8_t)level};
  if (at < e->next_event) e->next_event = at;
}

static void pins_due(ecu_t *e) {
  unsigned k = 0;
  while (k < e->pinq_n && e->pinq[k].at <= e->cyc) { pin_change(e, e->pinq[k].port, e->pinq[k].bit, e->pinq[k].level); k++; }
  if (k) { memmove(e->pinq, e->pinq + k, (e->pinq_n - k) * sizeof e->pinq[0]); e->pinq_n -= k; }
}

/* ---------------------------------------------------------------- SPI */
static const uint32_t SPI_OFF[4] = {0x5E00u, 0x5A00u, 0x5800u, 0x5C00u};
static inline int spi_of(uint32_t off) {
  if ((off - 0x5800u) >= 0x0640u) return -1;
  for (int i = 0; i < 4; i++) if ((off & ~0x3Fu) == SPI_OFF[i]) return i;
  return -1;
}
void ecu_spi_attach(ecu_t *e, unsigned n, int port, int bit, s32_spi_dev d) {
  if (n < 1 || n > 4 || e->spi[n - 1].n >= 4) return;
  e->spi[n - 1].dev[e->spi[n - 1].n++] = (ecu_spi_slave){d, port, bit, false};
}
static void cs_update(ecu_t *e) {
  for (int i = 0; i < 4; i++)
    for (unsigned k = 0; k < e->spi[i].n; k++) {
      ecu_spi_slave *d = &e->spi[i].dev[k];
      bool low = !((REG(e, PORT_OFF + 0x40u * (unsigned)d->port + 0x20u) >> d->bit) & 1u);
      if (low != d->asserted) { d->asserted = low; if (d->dev.select) d->dev.select(d->dev.ctx, low); }
    }
}

/* ---------------------------------------------------------------- clock tree
 * SYSCLK is the model's time base (e->hz cycles per emulated second, ECU_SYSCLK_HZ nominal). PBCLK = SYSCLK / 2^PBDIV,
 * with PBDIV = OSCCON<20:19>: at reset the hardware loads it from configuration word DEVCFG1.FPBDIV (and COSC/NOSC,
 * PLLMULT, PLLODIV from DEVCFG1/DEVCFG2; PIC32MX FRM section 6), and software may change it after the SYSKEY unlock.
 * Every peripheral clocked from PBCLK (timers, UART baud, SPI, ADC TAD) uses ecu_pbdiv(). */
#define OSC_OFF 0xF000u
static inline unsigned ecu_pbdiv(const ecu_t *e) { return 1u << ((e->sfr[OSC_OFF / 4] >> 19) & 3u); }
static void osc_from_config(ecu_t *e, const uint8_t *bfl, size_t nb) {
  if (nb < 0x3000u) return;                                   /* no configuration words in the image: keep the POR value */
  uint32_t cfg1, cfg2;
  memcpy(&cfg1, bfl + 0x2FF8, 4); memcpy(&cfg2, bfl + 0x2FF4, 4);   /* DEVCFG1 0x1FC02FF8, DEVCFG2 0x1FC02FF4 */
  uint32_t fnosc = cfg1 & 7u, fpbdiv = (cfg1 >> 12) & 3u, mul = (cfg2 >> 4) & 7u, odiv = (cfg2 >> 16) & 7u;
  uint32_t v = e->sfr[OSC_OFF / 4] & ~((7u << 8) | (7u << 12) | (7u << 16) | (3u << 19) | (7u << 27));
  e->sfr[OSC_OFF / 4] = v | (fnosc << 8) | (fnosc << 12) | (mul << 16) | (fpbdiv << 19) | (odiv << 27);
  e->wdt_fuse = (cfg1 >> 23) & 1u;                            /* FWDTEN */
  e->wdt_ps = (cfg1 >> 16) & 0x1Fu;                           /* WDTPS: postscaler 2^WDTPS (1:1 .. 1:1048576) */
}
#define WDTCON_OFF 0x0000u
bool ecu_wdt_poll(const ecu_t *e) {
  if (!e->wdt_fuse && !(e->sfr[WDTCON_OFF / 4] & 0x8000u)) return false;
  double period_s = 1e-3 * (double)(1ull << (e->wdt_ps > 20u ? 20u : e->wdt_ps));   /* LPRC 32 kHz / 32 = 1 ms base */
  return (double)(e->cyc - e->wdt_clear_cyc) >= period_s * ECU_SYSCLK_HZ;
}
void ecu_mark_reset_cause(ecu_t *e, uint32_t rcon_bits) { e->sfr[0xF600u / 4] |= rcon_bits; }
double ecu_pbclk_hz(const ecu_t *e) { return e->hz / ecu_pbdiv(e); }
uint32_t ecu_sfr_value(const ecu_t *e, uint32_t a) {
  uint32_t off = (a & 0x1FFFFFFFu) - 0x1F800000u;
  return off < ECU_SFR_SIZE ? e->sfr[off / 4] : 0u;
}

/* ---------------------------------------------------------------- timers */
static const uint32_t TMR_OFF[5] = {0x0600u, 0x0800u, 0x0A00u, 0x0C00u, 0x0E00u};
static const unsigned TMR_IRQ[5] = {4, 8, 12, 16, 20};
static inline int tmr_of(uint32_t off) {
  uint32_t d = off - 0x0600u;
  return (d < 0x0A00u && (d & 0x1C0u) == 0u) ? (int)(d >> 9) : -1;
}
static bool tmr_slave(ecu_t *e, int i) { return (i == 2 && (REG(e, TMR_OFF[1]) & 8u)) || (i == 4 && (REG(e, TMR_OFF[3]) & 8u)); }
static bool tmr_32(ecu_t *e, int i) { return (i == 1 || i == 3) && (REG(e, TMR_OFF[i]) & 8u); }
static bool tmr_on(ecu_t *e, int i) { return !tmr_slave(e, i) && (REG(e, TMR_OFF[i]) & 0x8000u) && !(REG(e, TMR_OFF[i]) & 2u); }
static unsigned tmr_ps(ecu_t *e, int i) {
  static const unsigned A[4] = {1, 8, 64, 256}, B[8] = {1, 2, 4, 8, 16, 32, 64, 256};
  uint32_t c = REG(e, TMR_OFF[i]);
  return ecu_pbdiv(e) * (i == 0 ? A[(c >> 4) & 3u] : B[(c >> 4) & 7u]);
}
static uint64_t tmr_pr(ecu_t *e, int i) {
  return tmr_32(e, i) ? (uint64_t)REG(e, TMR_OFF[i] + 0x20u) : (REG(e, TMR_OFF[i] + 0x20u) & 0xFFFFu);
}
static void tmr_advance(ecu_t *e, int i) {
  if (!tmr_on(e, i)) { e->tm[i].base = e->cyc; return; }
  uint64_t ps = tmr_ps(e, i), ticks = (e->cyc - e->tm[i].base) / ps;
  if (!ticks) return;
  e->tm[i].base += ticks * ps;
  uint64_t pr = tmr_pr(e, i), period = pr + 1u, c = e->tm[i].count0 + ticks;
  if (e->tm[i].count0 <= pr && c > pr) {
    set_ifs(e, TMR_IRQ[tmr_32(e, i) ? i + 1 : i]);
    e->tm[i].irqs += 1 + (c - period) / period;
    c = (c - period) % period;
  } else if (e->tm[i].count0 > pr) {
    uint64_t top = tmr_32(e, i) ? 0x100000000ull : 0x10000ull;   /* counting above PR: wraps at the counter top first */
    if (c >= top) {                            /* wrapped: the count climbs from 0 again, in the <= PR regime */
      c -= top;                                /* subtract exactly one wrap: later cycles hold more matches */
      if (c > pr) {                            /* the PR match(es) after the wrap fire too (PIC32MX FRM section 14) */
        set_ifs(e, TMR_IRQ[tmr_32(e, i) ? i + 1 : i]);
        e->tm[i].irqs += 1 + (c - period) / period;
        c = (c - period) % period;
      }
    }
  }
  e->tm[i].count0 = c;
}
static uint64_t tmr_next_event(ecu_t *e) {
  uint64_t best = UINT64_MAX;
  for (int i = 0; i < 5; i++) {
    if (!tmr_on(e, i)) continue;
    uint64_t pr = tmr_pr(e, i);
    if (e->tm[i].count0 > pr || pr == 0xFFFFFFFFull) continue;
    uint64_t at = e->tm[i].base + (pr + 1u - e->tm[i].count0) * tmr_ps(e, i);
    uint64_t d = at > e->cyc ? at - e->cyc : 1;
    if (d < best) best = d;
  }
  return best;
}
uint64_t ecu_timer_irqs(const ecu_t *e, int t) { return t >= 1 && t <= 5 ? e->tm[t - 1].irqs : 0; }

/* ---------------------------------------------------------------- UART */
static const uint32_t UART_OFF[6] = {0x6000u, 0x6800u, 0x6400u, 0x6200u, 0x6A00u, 0x6600u};   /* UART1..UART6 */
static const unsigned UART_IRQ[6] = {26, 40, 37, 67, 73, 70};                                  /* ERR; RX +1, TX +2 */
static inline int uart_of(uint32_t off) {
  if ((off - 0x6000u) >= 0x0A50u) return -1;
  for (int i = 0; i < 6; i++) if ((off & ~0x7Fu) == UART_OFF[i] && (off & 0x7Fu) < 0x50) return i;
  return -1;
}
static bool uart_on(ecu_t *e, int i) { return REG(e, UART_OFF[i]) & 0x8000u; }
static uint64_t uart_byte_cyc(ecu_t *e, int i) {
  uint32_t mode = REG(e, UART_OFF[i]), brg = REG(e, UART_OFF[i] + 0x40u) & 0xFFFFu;
  return (uint64_t)ecu_pbdiv(e) * 10ull * ((mode & 0x8u) ? 4u : 16u) * ((uint64_t)brg + 1u);
}
void ecu_uart_sink(ecu_t *e, unsigned n, void (*tx)(void *, ecu_t *, unsigned, uint8_t), void *ctx) {
  if (n >= 1 && n <= 6) { e->uart[n - 1].tx = tx; e->uart[n - 1].tx_ctx = ctx; }
}
void ecu_uart_feed(ecu_t *e, unsigned n, const uint8_t *b, size_t len) {
  if (n < 1 || n > 6) return;
  ecu_uart *u = &e->uart[n - 1];
  for (size_t k = 0; k < len; k++) {
    unsigned nt = (u->rt + 1u) % ECU_UART_RING;
    if (nt == u->rh) break;
    u->ring[u->rt] = b[k]; u->rt = nt;
  }
  if (!u->rx_next && u->rh != u->rt) u->rx_next = e->cyc + uart_byte_cyc(e, (int)n - 1);
  if (u->rx_next && u->rx_next < e->next_event) e->next_event = u->rx_next;
}
/* The receive interrupt is a level condition on the receive buffer, selected by URXISEL<1:0> (UxSTA<7:6>): "Interrupt
 * flag bit is asserted while receive buffer is not empty" for 00 on the 8-level FIFO (DS60001156K, Register 20-2).
 * The UART keeps asserting the flag while the condition holds, so clearing IFSx with enough data still in the FIFO
 * raises it again (DS61107G 21.7.3). */
static void uart_rx_irq_update(ecu_t *e, int i) {
  uint32_t sta = REG(e, UART_OFF[i] + 0x10u);
  if (!uart_on(e, i) || !(sta & 0x1000u)) return;                          /* ON, URXEN */
  unsigned fn = e->uart[i].fn, need = 6;                                   /* 10: 3/4 full; 11 is reserved, as 10 */
  switch ((sta >> 6) & 3u) { case 0: need = 1; break; case 1: need = 4; break; default: break; }
  if (fn >= need) set_ifs(e, UART_IRQ[i] + 1u);
}
static void uart_rx_land(ecu_t *e, int i) {
  ecu_uart *u = &e->uart[i];
  if (u->rh == u->rt) { u->rx_next = 0; return; }
  uint8_t b = u->ring[u->rh]; u->rh = (u->rh + 1u) % ECU_UART_RING;
  if (uart_on(e, i) && (REG(e, UART_OFF[i] + 0x10u) & 0x1000u)) {   /* URXEN */
    uint32_t *sta = &REG(e, UART_OFF[i] + 0x10u);
    if (*sta & 0x2u) u->rx_overruns++;                              /* OERR set: the receiver stops; the byte is lost on the wire */
    else if (u->fn < 8) { u->fifo[u->fn++] = b; u->rx_bytes++; if (u->fn > u->fn_peak) u->fn_peak = u->fn; }
    else { *sta |= 0x2u; u->rx_overruns++;                          /* FIFO full as this byte completes into UxRSR: OERR, */
           set_ifs(e, UART_IRQ[i]); }                               /* the word is not kept (DS61107G 21.7.2); UxEIF (21.14) */
    uart_rx_irq_update(e, i);
  }
  u->rx_next = u->rh != u->rt ? u->rx_next + uart_byte_cyc(e, i) : 0;
}
static uint64_t uart_next_event(ecu_t *e) {
  uint64_t best = UINT64_MAX;
  for (int i = 0; i < 6; i++)
    if (e->uart[i].rx_next) { uint64_t d = e->uart[i].rx_next > e->cyc ? e->uart[i].rx_next - e->cyc : 1; if (d < best) best = d; }
  return best;
}

/* ---------------------------------------------------------------- ADC1 (DS61104 section 17)
 * AD1CON1 +0x9000 (ON 15, SSRC 7:5, ASAM 2, SAMP 1, DONE 0), AD1CON2 +0x9010 (CSCNA 10, SMPI 5:2), AD1CON3 +0x9020
 * (ADRC 15, SAMC 12:8, ADCS 7:0; TAD = 2*(ADCS+1) PBCLK), AD1CSSL +0x9050, ADC1BUFn +0x9070 + 0x10n; IRQ 33.
 * Auto-sample + auto-convert scan: a sequence of SMPI+1 conversions, then the interrupt; ASAM restarts it. */
#define AD_OFF 0x9000u
void ecu_set_an(ecu_t *e, unsigned ch, uint16_t raw) { if (ch < 32) e->an[ch] = raw & 0x3FFu; }
static uint64_t adc_seq_cyc(ecu_t *e) {
  uint32_t c3 = REG(e, AD_OFF + 0x20u), c2 = REG(e, AD_OFF + 0x10u);
  uint64_t tad = (c3 & 0x8000u) ? 16u : (uint64_t)ecu_pbdiv(e) * 2u * ((c3 & 0xFFu) + 1u);   /* SYSCLK cycles; ADRC ~200 ns */
  uint64_t samc = (c3 >> 8) & 0x1Fu, n = ((c2 >> 2) & 0xFu) + 1u;
  return n * (samc + 12u) * tad;
}
static void adc_kick(ecu_t *e) {
  uint32_t c1 = REG(e, AD_OFF);
  if (!(c1 & 0x8000u)) { e->adc_done = 0; return; }
  if ((c1 & 0x4u) && !e->adc_done) { e->adc_done = e->cyc + adc_seq_cyc(e); if (e->adc_done < e->next_event) e->next_event = e->adc_done; }
}
static uint16_t an_sample(ecu_t *e, unsigned ch) {
  int v = e->an[ch];
  if (e->an_noise[ch]) {
    e->rng ^= e->rng << 13; e->rng ^= e->rng >> 17; e->rng ^= e->rng << 5;      /* xorshift32 */
    v += (int)(e->rng % (2u * e->an_noise[ch] + 1u)) - (int)e->an_noise[ch];
  }
  return (uint16_t)(v < 0 ? 0 : v > 1023 ? 1023 : v);
}
static void adc_complete(ecu_t *e) {
  uint32_t c2 = REG(e, AD_OFF + 0x10u), scan = REG(e, AD_OFF + 0x50u);
  unsigned n = ((c2 >> 2) & 0xFu) + 1u, k = 0, ch0 = REG(e, AD_OFF + 0x40u) >> 16 & 0x1Fu;
  bool cscna = c2 & 0x400u;
  /* Scan mode cycles through the AD1CSSL list until SMPI+1 conversions (FRM 17.4.10); otherwise CH0SA each time. */
  for (unsigned ch = 0; k < n && cscna && scan; ch = (ch + 1u) & 31u)
    if ((scan >> ch) & 1u) { REG(e, AD_OFF + 0x70u + 0x10u * k) = an_sample(e, ch); s32_iocov_an(e->cov, ch); k++; }
  for (; k < n; k++) { REG(e, AD_OFF + 0x70u + 0x10u * k) = an_sample(e, ch0); s32_iocov_an(e->cov, ch0); }
  if (REG(e, AD_OFF) & 0x0010u) REG(e, AD_OFF) &= ~0x0004u;                      /* CLRASAM: clear ASAM on interrupt */
  REG(e, AD_OFF) |= 1u;                                                          /* DONE */
  set_ifs(e, 33); e->adc_irqs++;
  e->adc_done = (REG(e, AD_OFF) & 0x8004u) == 0x8004u ? e->adc_done + adc_seq_cyc(e) : 0;
}

/* ---------------------------------------------------------------- SFR bus */
static uint32_t lane(uint64_t off, unsigned size, uint32_t v) {
  return (v >> ((off & 3u) * 8u)) & (size >= 4 ? 0xFFFFFFFFu : ((1u << (size * 8u)) - 1u));
}

static void oc_service(ecu_t *e, uint64_t *next);
static uint64_t sfr_read(uc_engine *uc, uint64_t off64, unsigned size, void *ud) {
  (void)uc;
  ecu_t *e = ud;
  uint32_t off = (uint32_t)off64, v;
  e->sfr_reads++;
  e->io_accesses++;
  if (e->read_hist) e->read_hist[off / 4]++;
  if (e->cov) s32_iocov_sfr(e->cov, ((off & 0xFu) && e->portal[(off & ~0xFu) / 16] ? off & ~0xFu : off) / 4, false);
  int ti = tmr_of(off), si = spi_of(off), ui = uart_of(off);
  uint32_t preg; int p = port_of(off, &preg);
  if (ti >= 0 && (off & 0x30u) == 0x10) {
    tmr_advance(e, ti);
    v = (uint32_t)e->tm[ti].count0;
    if (ti == 2 && tmr_slave(e, 2)) { tmr_advance(e, 1); v = (uint32_t)(e->tm[1].count0 >> 16); }
    if (ti == 4 && tmr_slave(e, 4)) { tmr_advance(e, 3); v = (uint32_t)(e->tm[3].count0 >> 16); }
  } else if (si >= 0 && (off & 0x30u) == 0x10) {
    v = (REG(e, SPI_OFF[si] + 0x10u) & ~0x6Bu) | (e->spi[si].rbf ? 1u : 0x20u) | 0x8u;
  } else if (si >= 0 && (off & 0x30u) == 0x20) {
    e->spi[si].rbf = false; v = e->spi[si].rx;
  } else if (ui >= 0 && (off & 0x70u) == 0x10) {
    ecu_uart *u = &e->uart[ui];
    uint64_t bt = uart_byte_cyc(e, ui);
    bool busy = u->tx_busy_until > e->cyc, full = u->tx_busy_until > e->cyc + 7 * bt;
    v = (REG(e, UART_OFF[ui] + 0x10u) & ~0x301u) | (u->fn ? 1u : 0u) | (busy ? 0u : 0x100u) | (full ? 0x200u : 0u);
    /* Spin fast-forward: a second identical read of this status with no other SFR access in between is a pure poll
     * loop (a firmware polling the TX status bits). Skip emulated time to when the value would next change -- the TX
     * FIFO draining -- bounded by the next timer/UART event and the slice end, so interrupts still land on time. */
    if (busy && e->last_off == off && e->last_v == v && ++e->same_reads >= 1) {
      uint64_t change = full ? u->tx_busy_until - 7 * bt : u->tx_busy_until;
      uint64_t lim = e->slice_start + e->budget_limit;
      if (e->next_event < change) change = e->next_event;
      if (lim < change) change = lim;
      if (change > e->cyc) {
        uint64_t w = change - e->cyc;
        e->budget_used += w; e->cyc += w; e->warped_cycles += w; e->warps++;
        full = u->tx_busy_until > e->cyc + 7 * bt; busy = u->tx_busy_until > e->cyc;
        v = (REG(e, UART_OFF[ui] + 0x10u) & ~0x301u) | (u->fn ? 1u : 0u) | (busy ? 0u : 0x100u) | (full ? 0x200u : 0u);
      }
    }
  } else if (ui >= 0 && (off & 0x70u) == 0x30) {
    ecu_uart *u = &e->uart[ui];
    v = 0;
    if (u->fn) { v = u->fifo[0]; memmove(u->fifo, u->fifo + 1, --u->fn); }
  } else if (p >= 0 && (preg & ~3u) == 0x10) {
    uint32_t tris = REG(e, PORT_OFF + 0x40u * (unsigned)p), lat = REG(e, PORT_OFF + 0x40u * (unsigned)p + 0x20u);
    /* Port B pins are also the ADC inputs AN0-15: a pin left analog (AD1PCFG bit clear, the reset state) reads 0 on
     * the digital port (DS61156 section 12.1, "pins configured as analog inputs read as 0"). */
    uint32_t analog = p == 1 ? ~REG(e, AD_OFF + 0x60u) & 0xFFFFu : 0u;
    v = ((lat & ~tris) | (e->pin_in[p] & tris & ~analog)) & 0xFFFFu;
    if (e->cov) e->cov->pin_in[p] |= (uint16_t)(tris & ~analog);
  } else {
    v = REG(e, off & ~3u);
  }
  if (e->last_off != off || e->last_v != v) { e->last_off = off; e->last_v = v; e->same_reads = 0; }
  return lane(off, size, v);
}

static void sfr_write(uc_engine *uc, uint64_t off64, unsigned size, uint64_t value, void *ud) {
  (void)uc;
  ecu_t *e = ud;
  uint32_t off = (uint32_t)off64;
  e->sfr_writes++;
  e->io_accesses++;
  if (e->write_hist) e->write_hist[off / 4]++;
  if (e->cov) s32_iocov_sfr(e->cov, ((off & 0xFu) && e->portal[(off & ~0xFu) / 16] ? off & ~0xFu : off) / 4, true);
  e->last_off = 0xFFFFFFFFu;                                   /* any write breaks a poll sequence */
  if (off >= 0x80000u) {
    uint16_t lata_before = (uint16_t)REG(e, PORT_OFF + 0x20u);
    uint16_t pins_before[7] = {0};                               /* driven pin levels: LAT where TRIS = 0 */
    bool port_reg = off >= PORT_OFF && off < PORT_OFF + 7 * 0x40;
    if (port_reg && e->lat_sink)
      for (int p = 0; p < 7; p++) pins_before[p] = (uint16_t)(REG(e, PORT_OFF + 0x40u * p + 0x20u) & ~REG(e, PORT_OFF + 0x40u * p));
    { uint32_t preg; int p = port_of(off, &preg); if (p >= 0 && (preg & ~0xFu) == 0x10) off += 0x10; }   /* PORTx -> LATx */
    uint32_t slot = off & 0xFu, base = off & ~0xFu, v = (uint32_t)value;
    if (size < 4) v <<= (off & 3u) * 8u;
    uint32_t *r, prev;
    if (slot && e->portal[base / 16] && size == 4) {
      r = &REG(e, base);
      prev = *r;
      if (slot == 4) *r = prev & ~v; else if (slot == 8) *r = prev | v; else *r = prev ^ v;
    } else {
      uint32_t m = size >= 4 ? 0xFFFFFFFFu : (((1u << (size * 8u)) - 1u) << ((off & 3u) * 8u));
      r = &REG(e, off & ~3u);
      prev = *r;
      *r = (prev & ~m) | (v & m);
    }
    if (*r != prev) {
      if (off >= IC_OFF && off < IC_OFF + 0x200u) {
        /* IFS0..IFS2: a cleared UART RX flag re-asserts while its level condition still holds (uart_rx_irq_update) */
        if (base == IC_OFF + 0x30u || base == IC_OFF + 0x40u || base == IC_OFF + 0x50u)
          for (int i = 0; i < 6; i++) uart_rx_irq_update(e, i);
        eic_eval(e);
      }
      else if (off >= PORT_OFF && off < PORT_OFF + 7 * 0x40) {
        if (e->cov) for (int p = 0; p < 7; p++) e->cov->pin_out[p] |= (uint16_t)~REG(e, PORT_OFF + 0x40u * (unsigned)p);
        cs_update(e);
        uint16_t lata = (uint16_t)REG(e, PORT_OFF + 0x20u);
        if (lata != lata_before && e->porta_sink) e->porta_sink(e->out_ctx, e, e->cyc, lata_before, lata);
        if (e->lat_sink)
          for (int p = 0; p < 7; p++) {
            uint16_t now = (uint16_t)(REG(e, PORT_OFF + 0x40u * p + 0x20u) & ~REG(e, PORT_OFF + 0x40u * p));
            if (now != pins_before[p]) e->lat_sink(e->lat_ctx, e, e->cyc, (unsigned)p, pins_before[p], now);
          }
      }
    }
    return;
  }
  int si = spi_of(off), ui = uart_of(off), ti = tmr_of(off);
  if (si >= 0 && (off & 0x3Fu) == 0x20) {
    uint8_t miso = 0xFF;
    if (REG(e, SPI_OFF[si]) & 0x8000u) {
      for (unsigned k = 0; k < e->spi[si].n; k++)
        if (e->spi[si].dev[k].asserted) miso = e->spi[si].dev[k].dev.exchange(e->spi[si].dev[k].dev.ctx, (uint8_t)value);
      if (e->spi[si].rbf) REG(e, SPI_OFF[si] + 0x10u) |= 0x40u;
      e->spi[si].rx = miso; e->spi[si].rbf = true; e->spi[si].bytes++;
    }
    return;
  }
  if (ui >= 0 && (off & 0x7Fu) == 0x20) {
    ecu_uart *u = &e->uart[ui];
    if (!uart_on(e, ui)) return;
    uint64_t bt = uart_byte_cyc(e, ui);
    u->tx_busy_until = (u->tx_busy_until > e->cyc ? u->tx_busy_until : e->cyc) + bt;
    u->tx_bytes++;
    if (u->tx) u->tx(u->tx_ctx, e, (unsigned)ui + 1u, (uint8_t)value);
    set_ifs(e, UART_IRQ[ui] + 2u);
    return;
  }
  if (ti >= 0) tmr_advance(e, ti);
  bool oc_reg = off >= 0x3000u && off < 0x3800u && (off & 0x1F0u) <= 0x020u;
  if (oc_reg) { uint64_t ignore = UINT64_MAX; oc_service(e, &ignore); }   /* edges due before this write fire first */
  uint32_t slot = off & 0xFu, base = off & ~0xFu, v = (uint32_t)value;
  if (size < 4) v <<= (off & 3u) * 8u;
  uint32_t sta_before = (ui >= 0 && (off & 0x70u) == 0x10) ? REG(e, UART_OFF[ui] + 0x10u) : 0;   /* UxSTA pre-write */
  if (base == 0xF410u) { /* NVMKEY: 0xAA996655 followed by 0x556699AA unlocks WR in NVMCON (PIC32 FRM Section 5) */
    if (v == 0xAA996655u) e->nvm_key_step = 1;
    else if (v == 0x556699AAu && e->nvm_key_step == 1) e->nvm_key_step = 2;
    else e->nvm_key_step = 0;
    REG(e, 0xF410u) = 0;
    return;
  }
  uint32_t nvmcon_before = (base == 0xF400u) ? REG(e, 0xF400u) : 0;
  uint32_t wdt_on_before = REG(e, WDTCON_OFF) & 0x8000u;
  if (slot && e->portal[base / 16] && size == 4) {
    uint32_t *r = &REG(e, base);
    if (slot == 4) *r &= ~v; else if (slot == 8) *r |= v; else *r ^= v;
  } else {
    uint32_t m = size >= 4 ? 0xFFFFFFFFu : (((1u << (size * 8u)) - 1u) << ((off & 3u) * 8u));
    uint32_t *r = &REG(e, off & ~3u);
    *r = (*r & ~m) | (v & m);
  }
  if (ui >= 0 && (off & 0x70u) == 0x10) {                 /* UxSTA (or its CLR/SET/INV portals) */
    ecu_uart *u = &e->uart[ui];
    uint32_t *stap = &REG(e, UART_OFF[ui] + 0x10u);
    if (!(sta_before & 0x2u)) *stap &= ~0x2u;             /* OERR is set by hardware only; a write can only clear it */
    else if (!(*stap & 0x2u)) {                           /* cleared in software: receive FIFO and UxRSR reset empty */
      u->fn = 0;                                          /* (DS60001156K, Register 20-2 bit 1) */
    }
    uart_rx_irq_update(e, ui);
  }
  if (base == WDTCON_OFF) {   /* WDTCON: ON (bit 15) starts the count; WDTCLR (bit 0) clears it and reads back 0 */
    if ((REG(e, WDTCON_OFF) & 1u) || (!wdt_on_before && (REG(e, WDTCON_OFF) & 0x8000u))) e->wdt_clear_cyc = e->cyc;
    REG(e, WDTCON_OFF) &= ~1u;
  }
  if (base == 0xF400u) { /* NVMCON: bit15 WR, bit14 WREN, bits3:0 NVMOP */
    uint32_t nvmcon_after = REG(e, 0xF400u);
    if (!(nvmcon_before & 0x8000u) && (nvmcon_after & 0x8000u)) {
      if (e->nvm_key_step == 2 && (nvmcon_after & 0x4000u)) {
        uint32_t op = nvmcon_after & 0x000Fu;
        uint32_t pa = REG(e, 0xF420u) & 0x1FFFFFFFu;
        if (op == 0x4u) { /* Page Erase: 4096 bytes (0x1000) */
          uint32_t page_pa = pa & ~0xFFFu;
          if (page_pa >= PFL_BASE && page_pa + 0x1000u <= PFL_BASE + PFL_SIZE) {
            uint32_t pfl_off = page_pa - PFL_BASE;
            uint8_t ff[4096];
            memset(ff, 0xFF, sizeof ff);
            uc_mem_write(UC(e), page_pa, ff, sizeof ff);
            memset(e->pfl_chip + pfl_off, 0xFF, sizeof ff);
            cp0_sites_reprogrammed(e, pfl_off, sizeof ff);
            e->nvm_erases++;
          }
        } else if (op == 0x1u) { /* Word Program: 4 bytes */
          uint32_t word_pa = pa & ~3u;
          if (word_pa >= PFL_BASE && word_pa + 4u <= PFL_BASE + PFL_SIZE) {
            uint32_t pfl_off = word_pa - PFL_BASE, cur = 0xFFFFFFFFu;
            memcpy(&cur, e->pfl_chip + pfl_off, 4);
            cur &= REG(e, 0xF430u);
            uc_mem_write(UC(e), word_pa, &cur, 4);
            memcpy(e->pfl_chip + pfl_off, &cur, 4);
            cp0_sites_reprogrammed(e, pfl_off, 4);
            e->nvm_writes++;
          }
        } else if (op == 0x3u) { /* Row Program: 512 bytes from NVMSRCADDR */
          uint32_t row_pa = pa & ~0x1FFu;
          uint32_t src_pa = REG(e, 0xF440u) & 0x1FFFFFCu;
          if (row_pa >= PFL_BASE && row_pa + 512u <= PFL_BASE + PFL_SIZE && src_pa + 512u <= RAM_SIZE) {
            uint32_t pfl_off = row_pa - PFL_BASE;
            uint8_t *dst = e->pfl_chip + pfl_off, src[512];
            uc_mem_read(UC(e), src_pa, src, 512);
            for (unsigned k = 0; k < 512u; k++) dst[k] &= src[k];
            uc_mem_write(UC(e), row_pa, dst, 512);
            cp0_sites_reprogrammed(e, pfl_off, 512);
            e->nvm_writes++;
          }
        }
      }
      REG(e, 0xF400u) &= ~0x8000u; /* WR self-clears when operation completes */
    } else {
      REG(e, 0xF400u) &= ~0x8000u;
    }
    e->nvm_key_step = 0;
  }
  if (ti >= 0) {
    if ((off & 0x30u) == 0x10) e->tm[ti].count0 = REG(e, TMR_OFF[ti] + 0x10u);
    e->tm[ti].base = e->cyc;
  }
  if ((off & ~0xFu) == AD_OFF) adc_kick(e);
  if (ti >= 0 || ui >= 0) e->next_event = e->cyc;          /* re-plan timer/UART events at the next block */
  /* OC1-4 (DS61111): OCxCON +0, OCxR +0x10, OCxRS +0x20 at 0xBF803000 + 0x200n. Dual-compare single pulse (OCM = 100,
   * OC32 on Timer2:3, the coil drivers' mode): turning ON arms one pulse, high when TMR2 reaches OCxR, low when it
   * reaches OCxRS. Rewriting R/RS while armed moves the pending edges; a target already passed is reached after the
   * 32-bit counter wraps, as on silicon.
   * A match is TMR2 == target at ANY tick since the last check: the block hook can land several ticks after an edge. */
  if (oc_reg) {
    unsigned n = (off - 0x3000u) / 0x200u;
    uint32_t con = REG(e, 0x3000u + 0x200u * n);
    if ((off & 0x1F0u) == 0x000u) {
      bool on = (con & 0x8000u) && (con & 7u) == 4u;
      static const bool rearm_same = false;                     /* silicon: rewriting ON with the same OCM does nothing */
      if (!on) e->oc[n].armed = false;
      else if (!e->oc[n].armed || rearm_same) { e->oc[n].armed = true; e->oc[n].risen = false; }
    }
    tmr_advance(e, 1);
    e->oc[n].ref = (uint32_t)e->tm[1].count0;                    /* a target equal to the count at the write waits */
    if (e->oc[n].armed) e->next_event = e->cyc;                  /* re-plan the pending edges */
  }
}

static void oc_service(ecu_t *e, uint64_t *next) {
  tmr_advance(e, 1);
  uint32_t now = (uint32_t)e->tm[1].count0;
  uint64_t ps = tmr_ps(e, 1), base = e->tm[1].base;
  for (unsigned n = 0; n < 4; n++) {
    if (!e->oc[n].armed) continue;
    uint32_t r = REG(e, 0x3010u + 0x200u * n), rs = REG(e, 0x3020u + 0x200u * n), ref = e->oc[n].ref;
    if (!e->oc[n].risen && r - ref - 1u < now - ref) {          /* r in (ref, now] */
      e->oc[n].risen = true; e->oc[n].on_cyc = base - (uint64_t)(now - r) * ps; ref = r;
    }
    if (e->oc[n].risen && rs - ref - 1u < now - ref) {          /* rs in (ref, now] */
      e->oc_pulses[n]++;
      if (e->oc_sink) e->oc_sink(e->out_ctx, e, n + 1, e->oc[n].on_cyc, base - (uint64_t)(now - rs) * ps);
      e->oc[n].armed = false;
      continue;
    }
    e->oc[n].ref = now;
    uint64_t nx = base + (uint64_t)((e->oc[n].risen ? rs : r) - now) * ps;
    if (nx > e->cyc && nx < *next) *next = nx;
  }
}

/* ---------------------------------------------------------------- instruction meter
 * Unicorn's uc_emu_start(count=N) counts with a code hook on EVERY instruction (~21 MIPS here). Instead a block hook adds
 * each translated block's instruction count and stops emulation once the slice budget is spent: one callback per block,
 * overshoot at most one block (a few instructions of extra interrupt latency). MIPS32: 4 bytes per instruction. */
static void oc_service(ecu_t *e, uint64_t *next);
static void service_events(ecu_t *e) {
  pins_due(e);
  if (e->adc_done && e->cyc >= e->adc_done) adc_complete(e);
  for (int i = 0; i < 5; i++) tmr_advance(e, i);
  for (int i = 0; i < 6; i++) if (e->uart[i].rx_next && e->cyc >= e->uart[i].rx_next) uart_rx_land(e, i);
  eic_eval(e);
  uint64_t ev = tmr_next_event(e), ue = uart_next_event(e);
  if (ue < ev) ev = ue;
  e->next_event = ev == UINT64_MAX ? UINT64_MAX : e->cyc + ev;
  if (e->pinq_n && e->pinq[0].at < e->next_event) e->next_event = e->pinq[0].at > e->cyc ? e->pinq[0].at : e->cyc + 1;
  if (e->adc_done && e->adc_done < e->next_event) e->next_event = e->adc_done > e->cyc ? e->adc_done : e->cyc + 1;
  oc_service(e, &e->next_event);
}

bool ecu_is_emu_err(int err) {
  switch (err) {
    case UC_ERR_OK:
    case UC_ERR_READ_UNMAPPED:
    case UC_ERR_WRITE_UNMAPPED:
    case UC_ERR_FETCH_UNMAPPED:
    case UC_ERR_INSN_INVALID:
    case UC_ERR_WRITE_PROT:
    case UC_ERR_READ_PROT:
    case UC_ERR_FETCH_PROT:
    case UC_ERR_READ_UNALIGNED:
    case UC_ERR_WRITE_UNALIGNED:
    case UC_ERR_FETCH_UNALIGNED:
    case UC_ERR_EXCEPTION:
      return false;
    default:
      return true;
  }
}
const char *ecu_err_str(int err) { return uc_strerror((uc_err)err); }

/* ---------------------------------------------------------------- idle loops
 * When the core goes once round a loop of blocks -- from a block back to the same block -- with every core register
 * (uc_shim_core_regs: general registers, PC, HI/LO, coprocessor 0 except the cycle-driven Count) as it was, every RAM
 * word it stored to holding its value from before again (uc_shim_wlog_start), the interrupt flags as they were, no
 * peripheral access (SFR read or write, core timer read or write) and no interrupt pending, the state is the same as when it set off, so every later time
 * round is the same too until an event changes something. Executing it only passes time; the model moves time on to
 * the next timer / UART / pin event (or the slice end) instead, and then watches the loop once round again before it
 * skips again, so a loop waiting for something an event or an interrupt handler changes leaves as soon as that
 * happens. A watch starts at the first block of a slice, at each backward jump of up to 4 KB and where an interrupt
 * interrupted one, and gives up after 64 blocks or at the next event; after one that came back changed or touched a
 * peripheral, the next starts 64 cycles later at the earliest.
 * It needs no knowledge of the program and never changes what the core does. */
#define IDLE_MAX_BLOCKS 64u
#define IDLE_MAX_BACK 4096u
#define IDLE_COOLDOWN 64u

void ecu_idle_skip(ecu_t *e, bool on) {
  e->idle_on = on;
  memset(&e->idle_watch, 0, sizeof e->idle_watch);
}
static bool irq_pending(ecu_t *e) {
  uint32_t st = ecu_reg(e, UC_MIPS_REG_CP0_STATUS), ca = ecu_reg(e, UC_MIPS_REG_CP0_CAUSE);
  return (st & 1u) && !(st & 6u) && ((ca >> 10) & 0x3Fu) > ((st >> 10) & 0x3Fu);
}
static uint64_t slice_deadline(const ecu_t *e) {
  uint64_t lim = e->slice_start + e->budget_limit;
  return e->next_event < lim ? e->next_event : lim;
}
static void idle_watch_start(ecu_t *e, uint32_t pc) {
  ecu_idle_watch *V = &e->idle_watch;
  V->active = !irq_pending(e) && e->cyc < slice_deadline(e);
  if (!V->active) return;
  V->head = pc;
  V->blocks = 0;
  V->io0 = e->io_accesses;
  V->exc0 = uc_shim_exceptions(UC(e));
  V->deadline = slice_deadline(e);
  for (unsigned i = 0; i < 3; i++) V->ifs[i] = REG(e, IC_OFF + 0x30u + 0x10u * i);
  uc_shim_wlog_start(UC(e));
  (void)uc_shim_core_regs(UC(e), V->regs);
}
static void idle_watch_fail(ecu_t *e) {
  e->idle_watch.active = false;
  e->idle_watch.cooldown_until = e->cyc + IDLE_COOLDOWN;
  uc_shim_wlog_stop(UC(e));
}
/* At the head again: 1 = nothing changed; 0 = the registers are the same but memory or a flag is not (once round more
 * from here may well be unchanged); -1 = the registers changed. */
static int idle_watch_unchanged(ecu_t *e) {
  const ecu_idle_watch *V = &e->idle_watch;
  uint32_t r[ECU_IDLE_REGS];
  (void)uc_shim_core_regs(UC(e), r);
  if (memcmp(r, V->regs, sizeof r)) return -1;
  if (!uc_shim_wlog_restored(UC(e))) return 0;
  for (unsigned i = 0; i < 3; i++) if (REG(e, IC_OFF + 0x30u + 0x10u * i) != V->ifs[i]) return 0;
  return 1;
}
/* At each block start (before the block runs). */
static void idle_watch_block(ecu_t *e, uint32_t pc) {
  ecu_idle_watch *V = &e->idle_watch;
  uint32_t prev = V->prev;
  V->prev = pc;
  if (V->active) {
    if (uc_shim_exceptions(UC(e)) != V->exc0) {                         /* interrupted: watch again when it comes back */
      V->resume = V->head;
      V->active = false;
      uc_shim_wlog_stop(UC(e));
      return;
    }
    if (e->io_accesses != V->io0 || uc_shim_wlog_full(UC(e)) || e->cyc >= V->deadline || ++V->blocks > IDLE_MAX_BLOCKS) {
      idle_watch_fail(e);
      return;
    }
    if (pc != V->head) return;
    if (irq_pending(e)) { idle_watch_fail(e); return; }
    int u = idle_watch_unchanged(e);
    if (u < 0) { idle_watch_fail(e); return; }                          /* a busy loop, for now */
    if (!u) { uc_shim_wlog_stop(UC(e)); idle_watch_start(e, pc); return; }   /* once round more from here */
    uc_shim_wlog_stop(UC(e));
    uint64_t to = slice_deadline(e);                                     /* round and round until the event */
    if (to > e->cyc) {
      uint64_t w = to - e->cyc;
      e->budget_used += w; e->cyc += w; e->idle_skipped += w;
      e->idle_blocks++;
      if (e->cyc >= e->next_event) { service_events(e); e->events_inline++; }
    }
    idle_watch_start(e, pc);                                            /* and once round again after it */
    return;
  }
  if (pc == V->resume) { V->resume = 0; idle_watch_start(e, pc); return; }
  if (e->cyc < V->cooldown_until) return;
  if (!prev || (pc <= prev && prev - pc <= IDLE_MAX_BACK)) idle_watch_start(e, pc);
}

/* Every block: keep the cycle counter live (block-accurate, for TMRx reads and UART pacing inside the slice), service
 * due timer/UART events in place (an interrupt raised here is taken before the block runs), skip time in an idle loop,
 * and stop when the slice budget is spent. */
static void hook_block(uc_engine *uc, uint64_t addr, uint32_t size, void *ud) {
  ecu_t *e = ud;
  (void)size;
  e->budget_used += size / 4u;
  e->cyc = e->slice_start + e->budget_used;
  if (e->cyc >= e->next_event) { service_events(e); e->events_inline++; }
  if (e->idle_on) idle_watch_block(e, (uint32_t)addr);
  if (e->budget_used >= e->budget_limit) uc_emu_stop(uc);
}

uint32_t ecu_bus_read(ecu_t *e, uint32_t a, unsigned size) { return (uint32_t)sfr_read(UC(e), (a & 0x1FFFFFFFu) - SFR_BASE, size, e); }
void ecu_bus_write(ecu_t *e, uint32_t a, unsigned size, uint32_t v) { sfr_write(UC(e), (a & 0x1FFFFFFFu) - SFR_BASE, size, v, e); }

/* ---------------------------------------------------------------- application start (no boot region)
 * An application-only image has no boot region: no boot flash and nothing in program flash below the application
 * window. A full image runs the boot region first and then enters the application; here, when the boot region is
 * absent, the ECU starts the application directly, in the state a full image has there on a normal power-up with no
 * card in the ECU slot (so no update). Like the Controller, whose update image starts with its own reset vector, the
 * application then runs from that state through the same pins and peripherals as always.
 *
 * The state below was observed in this emulator, whose CPU model sees every instruction fetch: the stock images (boot
 * flash + program flash) run from power-on with no ECU card, recorded at the first fetch in the application window
 * (after 16,379 SYSCLK cycles, 204.7 us: ECU_APP_BOOT_CYCLES, the moment of the hand-off here). At that moment:
 *   - execution enters the application window at its start + 0x1000, in KSEG0 (ECU_APP_ENTRY);
 *   - RAM and the general registers are as at power-on;
 *   - coprocessor 0: Status, Cause, IntCtl and Config.K0 as below; SRSCtl and EBase as at reset;
 *   - SFRs: the table below is every SFR whose value differs from its reset value; TRISx / LATx are the pins driven
 *     before the hand-off, and pins the application never configures keep these levels for the whole run;
 *   - the timers, UARTs, SPI ports, ADC, output compares and the flash controller are as at reset; the watchdog is off;
 *   - the configuration words (fuses), which on the device sit at the end of boot flash: only their fields the emulator
 *     models are declared (ECU_APP_CFG below; 8 MHz crystal x 20 / 2 = 80 MHz SYSCLK, PBCLK = SYSCLK / 2, watchdog off,
 *     postscaler 1:1048576); every other bit reads as erased.
 * With a card in the ECU slot (card detect RB3 low at power-up) and no update file on it, the hand-off comes 8.10 ms
 * after power-on (ECU_APP_CARD_BOOT_CYCLES), with SPI4 on, RF4/RF5 swapped in direction and RG15 high in addition
 * (ECU_HANDOFF_SFR_CARD, observed the same way on several cards). Cases that need the boot region itself (an update
 * file on the card) cannot run in this mode (they SKIP with the reason). */
#define ECU_APP_ENTRY (0x80000000u | (PFL_BASE + ECU_APP_OFF + 0x1000u))   /* KSEG0, window + 0x1000: the first fetch after the hand-off */
typedef struct { const char *name; uint32_t kseg1, value; } ecu_handoff_sfr;
static const ecu_handoff_sfr ECU_HANDOFF_SFR[] = {    /* applied in this order: latches before directions (no glitch) */
  {"BMXCON",  0xBF882000u, 0x001F0001u},              /* bus matrix: BMXARB = 1, the rest as at reset */
  {"CHECON",  0xBF884000u, 0x00000032u},              /* PFMWS = 2 wait states, PREFEN = 3 (prefetch for all regions) */
  {"DDPCON",  0xBF80F200u, 0x00000000u},              /* JTAG port off: RA0/1/4/5 are plain I/O */
  {"AD1PCFG", 0xBF809060u, 0x00000008u},              /* AN3 (RB3, card detect) digital */
  {"CNCON",   0xBF8861C0u, 0x00008000u},              /* change notification on */
  {"CNEN",    0xBF8861D0u, 0x00000020u},              /* CN5 */
  {"CNPUE",   0xBF8861E0u, 0x00000020u},              /* pull-up on CN5 */
  {"LATD",    0xBF8860E0u, 0x00000100u},
  {"LATF",    0xBF886160u, 0x0000FFF4u},
  {"LATG",    0xBF8861A0u, 0x00007000u},
  {"TRISA",   0xBF886000u, 0x00000600u},
  {"TRISB",   0xBF886040u, 0x00007FFFu},
  {"TRISC",   0xBF886080u, 0x00002000u},
  {"TRISD",   0xBF8860C0u, 0x00005E40u},
  {"TRISE",   0xBF886100u, 0x00000300u},
  {"TRISF",   0xBF886140u, 0x00000024u},
  {"TRISG",   0xBF886180u, 0x00000C8Cu},
};
static const ecu_handoff_sfr ECU_HANDOFF_SFR_CARD[] = {   /* in addition, with a card in the slot */
  {"SPI4BRG", 0xBF805C30u, 0x00000003u},              /* SCK = PBCLK / 8 */
  {"SPI4CON", 0xBF805C00u, 0x00008260u},              /* on, master, CKE, SMP */
  {"LATG",    0xBF8861A0u, 0x0000F000u},              /* RG15 high */
  {"TRISF",   0xBF886140u, 0x00000014u},              /* RF4 (SDI4) input, RF5 (SDO4) output */
};
/* Coprocessor 0 at the jump (MIPS32 M4K): Status = 0 (BEV, ERL, EXL and IE clear), Cause.IV = 1, IntCtl.VS = 1 (32-byte
 * vector spacing), Config.K0 = 3 (KSEG0 cacheable); SRSCtl as at reset (0). */
#define ECU_HANDOFF_STATUS 0x00000000u
#define ECU_HANDOFF_CAUSE  0x00800000u
#define ECU_HANDOFF_INTCTL 0xE0000020u
#define ECU_HANDOFF_K0     3u
/* The configuration-word fields the emulator models (DEVCFG1 / DEVCFG2, PIC32MX FRM section 32), as measured. */
static const struct { unsigned word, shift, width, value; const char *name; } ECU_APP_CFG[] = {
  {1,  0, 3, 3, "FNOSC: primary oscillator with PLL"},
  {1,  5, 1, 0, "FSOSCEN: secondary oscillator off"},
  {1,  8, 2, 1, "POSCMOD: XT"},
  {1, 12, 2, 1, "FPBDIV: PBCLK = SYSCLK / 2"},
  {1, 16, 5, 31, "WDTPS: 1:1048576"},
  {1, 23, 1, 0, "FWDTEN: watchdog off"},
  {2,  0, 3, 1, "FPLLIDIV: / 2"},
  {2,  4, 3, 5, "FPLLMUL: x 20"},
  {2, 16, 3, 0, "FPLLODIV: / 1"},
};

void ecu_app_boot_flash(uint8_t *bfl, size_t n) {
  memset(bfl, 0xFF, n);
  if (n < ECU_BFL_BYTES) return;
  uint32_t w[3] = {0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};   /* [1] DEVCFG1 at 0x1FC02FF8, [2] DEVCFG2 at 0x1FC02FF4 */
  for (size_t i = 0; i < sizeof ECU_APP_CFG / sizeof ECU_APP_CFG[0]; i++) {
    uint32_t m = ((1u << ECU_APP_CFG[i].width) - 1u) << ECU_APP_CFG[i].shift;
    w[ECU_APP_CFG[i].word] = (w[ECU_APP_CFG[i].word] & ~m) | (ECU_APP_CFG[i].value << ECU_APP_CFG[i].shift);
  }
  memcpy(bfl + 0x2FF8, &w[1], 4);
  memcpy(bfl + 0x2FF4, &w[2], 4);
}

size_t ecu_place_pflash(uint8_t *chip, const uint8_t *img, size_t n) {
  memset(chip, 0xFF, ECU_PFL_BYTES);
  if (n == ECU_APP_FLAT_BYTES) { memcpy(chip + ECU_APP_OFF, img, n); return ECU_APP_OFF + n; }
  if (n > ECU_PFL_BYTES) n = ECU_PFL_BYTES;
  memcpy(chip, img, n);
  return n;
}

static bool boot_region_absent(const uint8_t *bfl, size_t nb) {
  if (!bfl || nb < 4) return true;
  uint32_t w; memcpy(&w, bfl, 4);
  return w == 0xFFFFFFFFu;                                     /* nothing at the reset vector: no boot code */
}

/* The peripheral half of the hand-off, over the bus (the pins it drives reach the sinks now). Not the application's
 * own I/O: coverage and histograms are paused. */
static void handoff_apply(ecu_t *e) {
  s32_iocov *cov = e->cov; uint32_t *rh = e->read_hist, *wh = e->write_hist;
  uint64_t nw = e->sfr_writes;
  e->cov = NULL; e->read_hist = e->write_hist = NULL;
  for (size_t i = 0; i < sizeof ECU_HANDOFF_SFR / sizeof ECU_HANDOFF_SFR[0]; i++)
    ecu_bus_write(e, ECU_HANDOFF_SFR[i].kseg1, 4, ECU_HANDOFF_SFR[i].value);
  if (e->handoff_card)
    for (size_t i = 0; i < sizeof ECU_HANDOFF_SFR_CARD / sizeof ECU_HANDOFF_SFR_CARD[0]; i++)
      ecu_bus_write(e, ECU_HANDOFF_SFR_CARD[i].kseg1, 4, ECU_HANDOFF_SFR_CARD[i].value);
  e->cov = cov; e->read_hist = rh; e->write_hist = wh; e->sfr_writes = nw;
  e->handoff_pending = false;
}

/* ---------------------------------------------------------------- lifecycle */
int ecu_init(ecu_t *e, const uint8_t *bfl, size_t nb, const uint8_t *pfl, size_t np) {
  s32_iocov *cov = e->cov;
  memset(e, 0, sizeof *e);
  e->cov = cov;
  e->hz = ECU_SYSCLK_HZ;
  for (int p = 0; p < 7; p++) e->pin_in[p] = 0xFFFFu;
  for (int c = 0; c < 32; c++) e->an[c] = 512;
  e->rng = 0x9E3779B9u;                                   /* mid-scale until the host sets sensors */
  for (size_t i = 0; i < sizeof SFR_NAMES / sizeof SFR_NAMES[0]; i++) {   /* power-on reset values (DFP, DOCUMENTED) */
    uint32_t o = (SFR_NAMES[i].addr & 0x1FFFFFFFu) - SFR_BASE;
    REG(e, o) = SFR_NAMES[i].por;                         /* e.g. TRISx = 1 (inputs), PRx = 0xFFFF, BMXPFMSZ = 512 KB */
    if (SFR_NAMES[i].portals) e->portal[o / 16] = 1;
  }
  uint8_t app_bfl[ECU_BFL_BYTES];
  bool app = boot_region_absent(bfl, nb);
  if (app) { ecu_app_boot_flash(app_bfl, sizeof app_bfl); bfl = app_bfl; nb = sizeof app_bfl; }   /* declared fuses */
  osc_from_config(e, bfl, nb);
  memset(e->pfl_chip, 0xFF, sizeof e->pfl_chip);
  memcpy(e->pfl_chip, pfl, np < sizeof e->pfl_chip ? np : sizeof e->pfl_chip);
  if (app) memset(e->pfl_chip, 0xFF, ECU_APP_OFF);            /* the boot region is absent: nothing below the window */
  uc_engine *uc;
  if (uc_open(UC_ARCH_MIPS, UC_MODE_MIPS32 | UC_MODE_LITTLE_ENDIAN, &uc)) return -1;
  e->uc = uc;
  if (uc_ctl_set_cpu_model(uc, UC_CPU_MIPS32_M14K) || uc_mem_map(uc, RAM_BASE, RAM_SIZE, UC_PROT_ALL) ||
      uc_mem_map(uc, PFL_BASE, PFL_SIZE, UC_PROT_READ | UC_PROT_EXEC) ||
      uc_mem_map(uc, BFL_BASE, BFL_SIZE, UC_PROT_READ | UC_PROT_EXEC) ||
      uc_mem_write(uc, PFL_BASE, e->pfl_chip, PFL_SIZE) ||        /* the image, erased (0xFF) beyond its end */
      uc_mem_write(uc, BFL_BASE, bfl, nb < BFL_SIZE ? nb : BFL_SIZE) ||
      uc_mmio_map(uc, SFR_BASE, ECU_SFR_SIZE, sfr_read, e, sfr_write, e))
    return -2;
  uint32_t cfg3 = ecu_reg(e, UC_MIPS_REG_CP0_CONFIG3) | (1u << 6) | (1u << 5);   /* VEIC + VInt */
  uc_reg_write(uc, UC_MIPS_REG_CP0_CONFIG3, &cfg3);
  for (uint32_t off = 0; off < PFL_SIZE; off += 4u)                                /* every CP0 site (see above) */
    if (cp0_site_update(e, off)) return -3;
  { uc_hook h; if (uc_hook_add(uc, &h, UC_HOOK_BLOCK, hook_block, e, 1, 0)) return -4; }
  e->pc = 0xBFC00000u;
  if (app) {                                                   /* application start: the core half of the hand-off */
    uint32_t st = ECU_HANDOFF_STATUS, ca = ECU_HANDOFF_CAUSE, ic = ECU_HANDOFF_INTCTL;
    if (uc_reg_write(uc, UC_MIPS_REG_CP0_STATUS, &st) || uc_reg_write(uc, UC_MIPS_REG_CP0_CAUSE, &ca) ||
        uc_reg_write(uc, UC_MIPS_REG_CP0_INTCTL, &ic)) return -5;
#ifdef UC_SHIM_HAS_CP0_CONFIG0
    uint32_t k0 = ECU_HANDOFF_K0;
    if (uc_reg_write(uc, UC_MIPS_REG_CP0_CONFIG0, &k0)) return -5;
#endif
    e->pc = ECU_APP_ENTRY;
    e->app_start = e->handoff_pending = true;
    e->boot_cycles = ECU_APP_BOOT_CYCLES;
  }
  return 0;
}

size_t ecu_pflash_read(const ecu_t *e, uint8_t *dst, size_t cap) {
  size_t n = cap < sizeof e->pfl_chip ? cap : sizeof e->pfl_chip;
  memcpy(dst, e->pfl_chip, n);
  return n;
}

void ecu_free(ecu_t *e) { if (e->uc) uc_close(UC(e)); e->uc = NULL; }   /* e->cov survives: a power cycle keeps the record */

void ecu_iocov_enable(ecu_t *e) { if (!e->cov) e->cov = s32_iocov_new(0xBF800000u, ECU_SFR_SIZE / 4, 4); }


int ecu_run(ecu_t *e, uint64_t cycles) { return ecu_run_to(e, e->cyc + cycles); }

int ecu_run_to(ecu_t *e, uint64_t end) {
  if (e->handoff_pending) {
    if (!e->handoff_at) {                            /* power-up: is a card in the slot (RB3 low)? */
      e->handoff_card = !((e->pin_in[1] >> 3) & 1u);
      e->handoff_at = e->cyc + (e->handoff_card && e->boot_cycles == ECU_APP_BOOT_CYCLES ? ECU_APP_CARD_BOOT_CYCLES
                                                                                           : e->boot_cycles);
    }
    if (e->cyc < e->handoff_at) {                    /* held while the (absent) boot code would run: nothing executes */
      if (end <= e->handoff_at) { e->cyc = end; return 0; }
      e->cyc = e->handoff_at;
    }
    handoff_apply(e);
  }
  service_events(e);
  while (e->cyc < end) {
    e->slice_start = e->cyc; e->budget_used = 0; e->budget_limit = end - e->cyc;
    uc_err err = uc_emu_start(UC(e), e->pc, 0xFFFFFFFFull, 0, 0);
    if (e->idle_watch.active) idle_watch_fail(e);                /* never across a slice end: events change state */
    e->idle_watch.prev = 0;
    e->cyc = e->slice_start + (e->budget_used ? e->budget_used : e->budget_limit);
    e->slices++;
    e->pc = ecu_reg(e, UC_MIPS_REG_PC);
    service_events(e);
    if (err) { e->last_err = (int)err; return -1; }
    if (e->last_err) return -1;                                /* the SoC model stopped the core (cp0_sites_reprogrammed) */
  }
  return 0;
}

/* ---- snapshots (emu/common/s32snap.h) ----
 * The ECU's state is the ecu_t (pointers zeroed) followed by the MIPS core's state (uc_shim_snap_*, emu/ecu/shim). */
/* Every callback this file gives the engine, so the engine state can name them by index. */
static void *const SNAP_FNS[] = {(void *)hook_cp0_site, (void *)hook_block, (void *)sfr_read, (void *)sfr_write};
#define N_SNAP_FNS (sizeof SNAP_FNS / sizeof SNAP_FNS[0])

size_t ecu_snap_size(void) { return uc_shim_snap_size() ? sizeof(ecu_t) + uc_shim_snap_size() : 0; }

int ecu_snap_save(const ecu_t *e, uint8_t *out) {
  if (!uc_shim_snap_size() || !e->uc) return -1;
  ecu_t *c = (ecu_t *)out;
  memcpy(c, e, sizeof *c);
  c->uc = NULL;
  for (int i = 0; i < 6; i++) { c->uart[i].tx = NULL; c->uart[i].tx_ctx = NULL; }
  for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) memset(&c->spi[i].dev[j].dev, 0, sizeof c->spi[i].dev[j].dev);
  c->read_hist = c->write_hist = NULL;
  c->oc_sink = NULL; c->porta_sink = NULL; c->lat_sink = NULL; c->lat_ctx = NULL; c->out_ctx = NULL;
  c->cov = NULL;
  return uc_shim_snap_save(UC(e), out + sizeof *c, SNAP_FNS, N_SNAP_FNS, e);
}

int ecu_snap_restore(ecu_t *e, const uint8_t *in, size_t n) {
  if (!uc_shim_snap_size() || n != ecu_snap_size() || !e->uc) return -1;
  void *uc = e->uc;
  void (*tx[6])(void *, ecu_t *, unsigned, uint8_t); void *txc[6];
  s32_spi_dev dev[4][4]; unsigned nspi[4];
  for (int i = 0; i < 6; i++) { tx[i] = e->uart[i].tx; txc[i] = e->uart[i].tx_ctx; }
  for (int i = 0; i < 4; i++) { nspi[i] = e->spi[i].n; for (int j = 0; j < 4; j++) dev[i][j] = e->spi[i].dev[j].dev; }
  uint32_t *rh = e->read_hist, *wh = e->write_hist;
  void (*oc)(void *, ecu_t *, unsigned, uint64_t, uint64_t) = e->oc_sink;
  void (*pa)(void *, ecu_t *, uint64_t, uint16_t, uint16_t) = e->porta_sink;
  void (*ls)(void *, ecu_t *, uint64_t, unsigned, uint16_t, uint16_t) = e->lat_sink;
  void *lc = e->lat_ctx, *oc_ctx = e->out_ctx;
  s32_iocov *cov = e->cov;
  memcpy(e, in, sizeof *e);
  e->uc = uc;
  for (int i = 0; i < 4; i++) if (e->spi[i].n != nspi[i]) return -2;   /* the devices on the board differ */
  for (int i = 0; i < 6; i++) { e->uart[i].tx = tx[i]; e->uart[i].tx_ctx = txc[i]; }
  for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) e->spi[i].dev[j].dev = dev[i][j];
  e->read_hist = rh; e->write_hist = wh; e->oc_sink = oc; e->porta_sink = pa; e->lat_sink = ls; e->lat_ctx = lc;
  e->out_ctx = oc_ctx; e->cov = cov;
  return uc_shim_snap_restore(UC(e), in + sizeof *e, SNAP_FNS, N_SNAP_FNS, e);
}
