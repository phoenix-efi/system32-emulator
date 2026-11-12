/* s32ct.c -- Cockpit Controller SoC model: the PIC24FJ256DA210's peripherals around the core (emu/pic24): SFR storage,
 * GPIO, SPI1..3 masters with chip-select-routed devices, UART1..4, Timer1..5, ADC1, the flash controller, the graphics
 * controller (s32gfx.c), and the idle-loop skip. Register semantics: PIC24FJ256DA210 data sheet DS39969 and the
 * Family Reference Manual sections it cites. */
#include "s32ct.h"
#include "sfr_reset.h"

#include <stdlib.h>
#include <string.h>

/* port registers: TRISx = 0x02C0 + 8p, PORTx = +2, LATx = +4, ODCx = +6 (p = 0..6 for A..G) */
#define PORT_BASE 0x02C0u
#define PORT_TOP  (PORT_BASE + 7u * 8u)
/* SPIn: STAT = base, CON1 = +2, CON2 = +4, BUF = +8 */
static const uint16_t SPI_BASE[3] = {0x0240, 0x0260, 0x0280};

static uint16_t *reg(s32ct *s, uint16_t a) { return &s->sfr[a >> 1]; }

/* ---- Timer1..5 (DS39969 sections 12-13, 16-bit modes) ---- */
static const uint16_t TMR_ADDR[5] = {0x0100, 0x0106, 0x010A, 0x0114, 0x0118};
static const uint16_t PR_ADDR[5]  = {0x0102, 0x010C, 0x010E, 0x011A, 0x011C};
static const uint16_t CON_ADDR[5] = {0x0104, 0x0110, 0x0112, 0x011E, 0x0120};
static const unsigned TMR_IRQ[5]  = {3, 7, 8, 27, 28};       /* DS Table 7-2 */
static const unsigned PRESCALE[4] = {1, 8, 64, 256};

static inline int timer_index(uint16_t a, int *which) {        /* which: 0 TMR, 1 PR, 2 CON */
  if ((uint16_t)(a - 0x0100u) > 0x0020u) return -1;
  for (int i = 0; i < 5; i++) {
    if (a == TMR_ADDR[i]) { *which = 0; return i; }
    if (a == PR_ADDR[i]) { *which = 1; return i; }
    if (a == CON_ADDR[i]) { *which = 2; return i; }
  }
  return -1;
}
static bool timer_on(const s32_timer *t) { return (t->con & 0x8000u) && !(t->con & 0x0002u); }   /* TON, internal clock */
static unsigned timer_ps(const s32_timer *t) { return PRESCALE[(t->con >> 4) & 3u]; }

/* Bring the timer's count up to `now`, raising TxIF for every period match passed (period = PR + 1 counts). */
static void timer_advance(s32ct *s, int i, uint64_t now) {
  s32_timer *t = &s->tmr[i];
  if (!timer_on(t)) { t->base = now; return; }
  uint64_t ticks = (now - t->base) / timer_ps(t);
  if (!ticks) return;
  t->base += ticks * timer_ps(t);
  uint64_t period = (uint64_t)t->pr + 1u;
  uint64_t c = t->count0 + ticks;
  if (t->count0 <= t->pr && c > t->pr) {            /* crossed PR at least once */
    pic24_set_irq(&s->cpu, TMR_IRQ[i], true);
    s->timer_irqs[i] += 1 + (c - period) / period;
    c = (c - period) % period;
  } else if (t->count0 > t->pr) {                     /* counting above PR: wraps at 0xFFFF first */
    if (c > 0xFFFFu) {                                /* wrapped: the count climbs from 0 again, in the <= PR regime */
      c -= 0x10000u;                                  /* subtract exactly one wrap: later cycles hold more matches */
      if (c > t->pr) {                                /* the PR match(es) after the wrap fire too (PIC24F FRM, Timers, DS39704) */
        pic24_set_irq(&s->cpu, TMR_IRQ[i], true);
        s->timer_irqs[i] += 1 + (c - period) / period;
        c = (c - period) % period;
      }
    }
  }
  t->count0 = (uint32_t)c;
}

/* Tcy until the next period match of any running timer (UINT64_MAX if none). */
static uint64_t timer_next_event(const s32ct *s) {
  uint64_t best = UINT64_MAX;
  for (int i = 0; i < 5; i++) {
    const s32_timer *t = &s->tmr[i];
    if (!timer_on(t) || t->count0 > t->pr) continue;
    uint64_t left = ((uint64_t)t->pr + 1u - t->count0) * timer_ps(t);
    uint64_t at = t->base + left, now = s->cpu.cycles;
    uint64_t d = at > now ? at - now : 1;
    if (d < best) best = d;
  }
  return best;
}

/* ---- UART1..4 (DS39969 section 17) ---- */
static const uint16_t UART_BASE[4] = {0x0220, 0x0230, 0x0250, 0x02B0};    /* MODE, STA +2, TXREG +4, RXREG +6, BRG +8 */
static const unsigned UART_RX_IRQ[4] = {11, 30, 82, 88}, UART_TX_IRQ[4] = {12, 31, 83, 89};
static const unsigned UART_ERR_IRQ[4] = {65, 66, 81, 87};                 /* UxERIF: DS39969B interrupt table */

static inline int uart_index(uint16_t a) {
  if ((uint16_t)(a - 0x0220u) > 0x0098u) return -1;
  for (int i = 0; i < 4; i++) if (a >= UART_BASE[i] && a <= UART_BASE[i] + 8) return i;
  return -1;
}
static uint64_t uart_byte_tcy(s32ct *s, int i) {
  uint16_t mode = s->sfr[UART_BASE[i] >> 1], brg = s->sfr[(UART_BASE[i] + 8) >> 1];
  return 10ull * ((mode & 0x0008u) ? 4u : 16u) * ((uint64_t)brg + 1u);
}
static bool uart_on(s32ct *s, int i) { return s->sfr[UART_BASE[i] >> 1] & 0x8000u; }

static inline bool any_irq_pending(const s32ct *s) {
  if (s->cpu.intcon1 & 0x001Eu) return true;
  for (int k = 0; k < 7; k++) if (s->cpu.ifs[k] & s->cpu.iec[k]) return true;
  return false;
}

void s32ct_uart_sink(s32ct *s, unsigned n, void (*tx)(void *, unsigned, uint8_t), void *ctx) {
  if (n >= 1 && n <= 4) { s->uart[n - 1].tx = tx; s->uart[n - 1].tx_ctx = ctx; }
}
void s32ct_uart_feed(s32ct *s, unsigned n, const uint8_t *b, size_t len) {
  if (n < 1 || n > 4) return;
  s32_uart *u = &s->uart[n - 1];
  for (size_t k = 0; k < len; k++) {
    unsigned nt = (u->rt + 1u) % S32_UART_RING;
    if (nt == u->rh) break;                                   /* host ring full: drop (host's problem, counted) */
    u->ring[u->rt] = b[k]; u->rt = nt;
  }
  if (!u->rx_next && u->rh != u->rt) u->rx_next = s->cpu.cycles + uart_byte_tcy(s, (int)n - 1);
}
/* A fed byte completes on the wire. With the 4-deep FIFO full, the fifth word fully received into UxRSR sets OERR
 * and is held in the RSR (FRM DS70000582E 7.2; the Figure 7-3 note that puts the overrun at the start of the 6th
 * character conflicts with that text and is not followed). While OERR is set, "further transfers to the receive
 * FIFO and UART interrupts are inhibited" (7.2): later words are lost on the wire and UxRXIF is not raised. The
 * flag is set on an RSR-to-FIFO transfer as URXISEL<1:0> selects (DS39969B Register 17-2 bits 7:6: 0x = any
 * transfer; 10 = the transfer making 3 characters; 11 = the transfer making 4) -- an event, not a level. */
static void uart_rx_land(s32ct *s, int i) {
  s32_uart *u = &s->uart[i];
  if (u->rh == u->rt) { u->rx_next = 0; return; }
  uint8_t b = u->ring[u->rh]; u->rh = (u->rh + 1u) % S32_UART_RING;
  if (uart_on(s, i)) {                                           /* receiver off: the word is lost */
    uint16_t *sta = &s->sfr[(UART_BASE[i] + 2) >> 1];
    if (*sta & 0x0002u) {                                          /* OERR set: the word is lost, no transfer, no flag */
      u->rx_overruns++;
    } else if (u->fn < 4) {
      u->fifo[u->fn++] = b; u->rx_bytes++;
      if (u->fn > u->fn_peak) u->fn_peak = u->fn;
      unsigned isel = (*sta >> 6) & 3u;
      if (isel < 2 || u->fn == (isel == 2 ? 3u : 4u)) pic24_set_irq(&s->cpu, UART_RX_IRQ[i], true);
    } else {
      *sta |= 0x0002u;                                           /* OERR: the fifth word is kept in UxRSR, never */
      u->rsr = b; u->rsr_full = true; u->rx_bytes++;             /* transferred while OERR is set (7.2) */
      pic24_set_irq(&s->cpu, UART_ERR_IRQ[i], true);             /* the error interrupt fires when OERR occurs */
    }                                                            /* (FRM 7.2); while OERR stays set, nothing fires */
  }
  u->rx_next = u->rh != u->rt ? u->rx_next + uart_byte_tcy(s, i) : 0;
}
static uint16_t uart_read(s32ct *s, int i, uint16_t off) {
  s32_uart *u = &s->uart[i];
  uint16_t *sta = &s->sfr[(UART_BASE[i] + 2) >> 1];
  if (off == 6) {                                           /* RXREG: pop */
    if (!u->fn) return 0;
    uint8_t b = u->fifo[0];
    memmove(u->fifo, u->fifo + 1, --u->fn);
    return b;
  }
  if (off == 2) {
    uint64_t now = s->cpu.cycles, bt = uart_byte_tcy(s, i);
    bool busy = u->tx_busy_until > now, full = u->tx_busy_until > now + 3 * bt;
    /* URXDA, TRMT, UTXBF and RIDLE are hardware status bits (read-only, Register 17-2): URXDA = FIFO not empty,
     * RIDLE = UxRSR empty (FRM 7.3) -- nothing on the wire and nothing held. */
    bool ridle = !u->rx_next && !u->rsr_full;
    uint16_t v = (uint16_t)((*sta & ~0x0311u) | (u->fn ? 0x0001u : 0u) | (ridle ? 0x0010u : 0u) |
                            (busy ? 0u : 0x0100u) | (full ? 0x0200u : 0u));
    uint16_t a = (uint16_t)(UART_BASE[i] + 2u);
    if (s->idle_on && busy && s->last_sfr_addr == a && s->last_sfr_val == v && ++s->same_sfr_reads >= 1 && !any_irq_pending(s)) {
      uint64_t change = full ? u->tx_busy_until - 3 * bt : u->tx_busy_until;
      if (s->slice_end && s->slice_end < change) change = s->slice_end;
      if (change > s->cpu.cycles) {
        uint64_t w = change - s->cpu.cycles;
        s->cpu.cycles = change;
        s->idle_skipped += w;
        now = s->cpu.cycles;
        busy = u->tx_busy_until > now;
        full = u->tx_busy_until > now + 3 * bt;
        v = (uint16_t)((*sta & ~0x0311u) | (u->fn ? 0x0001u : 0u) | (ridle ? 0x0010u : 0u) |
                       (busy ? 0u : 0x0100u) | (full ? 0x0200u : 0u));
      }
    }
    return v;
  }
  return s->sfr[(UART_BASE[i] + off) >> 1];
}
static void uart_write(s32ct *s, int i, uint16_t off, uint16_t v, uint16_t m) {
  s32_uart *u = &s->uart[i];
  uint16_t *r = &s->sfr[(UART_BASE[i] + off) >> 1];
  if (off == 4) {                                           /* TXREG */
    if (!uart_on(s, i)) return;
    uint64_t now = s->cpu.cycles, bt = uart_byte_tcy(s, i);
    u->tx_busy_until = (u->tx_busy_until > now ? u->tx_busy_until : now) + bt;
    u->tx_bytes++;
    if (u->tx) u->tx(u->tx_ctx, (unsigned)i + 1u, (uint8_t)(v & m));
    pic24_set_irq(&s->cpu, UART_TX_IRQ[i], true);
    return;
  }
  if (off == 2) {
    uint16_t before = *r;
    /* R/W bits only: UTXISEL1, UTXINV, UTXISEL0, UTXBRK, UTXEN, URXISEL<1:0>, ADDEN (DS39969B Register 17-2;
     * model/pic24fj256da210_sfr.json impl_mask 0xEFFF). OERR is R/C-0, HS: hardware sets it, a write can only
     * clear it, and clearing a set OERR resets the receiver buffer and the RSR to the empty state (Register 17-2
     * bit 1; FRM DS70000582E 7.2: "The FIFO is reset when OERR is cleared"). */
    *r = (uint16_t)((before & ~(m & 0xECE0u)) | (v & m & 0xECE0u));
    if (!(before & 0x0002u)) *r &= ~0x0002u;
    else if ((m & 0x0002u) && !(v & 0x0002u)) {
      *r &= ~0x0002u; u->fn = 0; u->rsr_full = false;
    }
    return;
  }
  if (off == 0) {
    uint16_t before = *r;
    *r = (uint16_t)((before & ~m) | (v & m));
    if ((before & 0x8000u) && !(*r & 0x8000u)) {                    /* UARTEN 1 -> 0 (FRM DS70000582E 4.2) */
      uint16_t *sta = &s->sfr[(UART_BASE[i] + 2) >> 1];
      *sta = (uint16_t)((*sta & ~0x0E0Fu) | 0x0110u);               /* UTXBRK/UTXEN/UTXBF/PERR/FERR/OERR/URXDA */
      u->fn = 0; u->rsr_full = false;                               /* cleared, TRMT/RIDLE set; buffers reset empty */
      u->tx_busy_until = 0;                                         /* pending transmissions aborted */
      if (u->rx_next && u->rh != u->rt) {                           /* the reception in flight is aborted too; */
        u->rh = (u->rh + 1u) % S32_UART_RING;                       /* the baud counter resets (4.2), so the wire's */
        u->rx_next = u->rh != u->rt ? s->cpu.cycles + uart_byte_tcy(s, i) : 0;   /* next byte re-paces from now */
      }
    }
    return;
  }
  *r = (uint16_t)((*r & ~m) | (v & m));
}

/* ---- ADC1 (DS39969 section 21): auto-sample + auto-convert scan of AD1CSSL/H, interrupt every SMPI+1 conversions ---- */
#define AD1CON1 0x0320u
#define AD1CON2 0x0322u
#define AD1CON3 0x0324u
#define AD1CSSH 0x032Eu
#define AD1CSSL 0x0330u
#define ADC1BUF0 0x0300u
#define AD1CHS 0x0328u
#define ANSA 0x04E0u                                   /* ANSA..ANSG: 1 = analog (digital read returns 0) */

void s32ct_set_pin(s32ct *s, unsigned port, unsigned bit, bool level) {
  if (port < 7 && bit < 16) s->pin_in[port] = (uint16_t)((s->pin_in[port] & ~(1u << bit)) | ((unsigned)level << bit));
}

void s32ct_set_an(s32ct *s, unsigned ch, uint16_t raw) { if (ch < 32) s->an[ch] = raw & 0x3FFu; }

static uint64_t adc_seq_tcy(s32ct *s) {
  uint16_t c3 = *reg(s, AD1CON3), c2 = *reg(s, AD1CON2);
  unsigned tad = (c3 & 0x8000u) ? 4u : ((c3 & 0xFFu) + 1u);           /* ADRC: internal RC, ~250 ns */
  unsigned samc = (c3 >> 8) & 0x1Fu, n = ((c2 >> 2) & 0x1Fu) + 1u;
  return (uint64_t)n * (samc + 12u) * tad;
}
static void adc_kick(s32ct *s) {
  uint16_t c1 = *reg(s, AD1CON1);
  if ((c1 & 0x8000u) && (c1 & 0x0004u) && !s->adc_done) s->adc_done = s->cpu.cycles + adc_seq_tcy(s);
  if (!(c1 & 0x8000u)) s->adc_done = 0;
}
static void adc_complete(s32ct *s) {
  uint16_t c2 = *reg(s, AD1CON2);
  unsigned n = ((c2 >> 2) & 0x1Fu) + 1u, k = 0;
  uint32_t scan = *reg(s, AD1CSSL) | ((uint32_t)*reg(s, AD1CSSH) << 16);
  bool cscna = c2 & 0x0400u;
  unsigned ch0 = *reg(s, AD1CHS) & 0x1Fu;                                /* CH0SA: the channel when not scanning */
  /* Scan mode cycles through the AD1CSSL/H list until SMPI+1 conversions (DS39969 21.0); otherwise CH0SA each time. */
  for (unsigned ch = 0; k < n && cscna && scan; ch = (ch + 1u) & 31u)
    if ((scan >> ch) & 1u) { s->sfr[(ADC1BUF0 >> 1) + k++] = s->an[ch]; s32_iocov_an(s->cov, ch); }
  for (; k < n; k++) { s->sfr[(ADC1BUF0 >> 1) + k] = s->an[ch0]; s32_iocov_an(s->cov, ch0); }
  *reg(s, AD1CON1) |= 0x0001u;                                          /* DONE */
  pic24_set_irq(&s->cpu, 13, true);                                      /* AD1IF */
  s->adc_irqs++;
  s->adc_done = (*reg(s, AD1CON1) & 0x8004u) == 0x8004u ? s->adc_done + adc_seq_tcy(s) : 0;
}

/* ---- idle loops ----
 * When the CPU goes once round a loop -- from an instruction back to the same instruction -- with every core register
 * (W0-W15, SR, the interrupt controller and the other core registers) as it was, every data-memory word it wrote
 * holding its value from before again (pic24_wlog_start), no peripheral access (SFR read or write, table write) and no exception taken, the state is the same as when it
 * set off, so every later time round is the same too until a peripheral event changes something. Executing it only
 * passes time; the model moves time on to the end of the slice (the next event) instead. After the event it watches the
 * loop once round again before it skips again, so a loop waiting for something an event or an interrupt handler
 * changes leaves as soon as that happens. A watch starts at the beginning of each slice, at each short backward
 * branch and where an exception interrupted one, and gives up after 64 instructions; after one that came back changed
 * or touched a peripheral, the next starts 64 instructions later at the earliest. It needs no knowledge of the program and never changes what the CPU does. */
#define IDLE_MAX_STEPS 64u
#define IDLE_MAX_BACK 512u          /* PC units: a backward branch shorter than this starts a watch */
#define IDLE_COOLDOWN 64u

void s32ct_idle_skip(s32ct *s, bool on) {
  s->idle_on = on;
  memset(&s->idle_watch, 0, sizeof s->idle_watch);
}
static void idle_regs(const s32ct *s, uint16_t *r) {
  const pic24_cpu *c = &s->cpu;
  unsigned n = 0;
  for (unsigned i = 0; i < 16; i++) r[n++] = c->ram[i];
  r[n++] = c->sr; r[n++] = c->ipl3; r[n++] = c->rcount; r[n++] = c->splim; r[n++] = c->splim_armed;
  r[n++] = c->dsrpag; r[n++] = c->dswpag; r[n++] = c->tblpag; r[n++] = c->disicnt;
  r[n++] = c->intcon1; r[n++] = c->intcon2; r[n++] = c->inttreg;
  for (unsigned i = 0; i < 7; i++) r[n++] = c->ifs[i];
  for (unsigned i = 0; i < 7; i++) r[n++] = c->iec[i];
  for (unsigned i = 0; i < 26; i++) r[n++] = c->ipc[i];
  for (unsigned i = 0; i < 4; i++) r[n++] = c->shadow_w[i];
  r[n++] = c->shadow_sr; r[n++] = (uint16_t)c->power; r[n++] = c->raw_wmask;
  r[n++] = c->repeat_first; r[n++] = c->repeat_final; r[n++] = c->repeat_iter;
  r[n++] = (uint16_t)c->pc; r[n++] = (uint16_t)(c->pc >> 16);
  while (n < S32CT_IDLE_REGS) r[n++] = 0;
}
static void idle_watch_start(s32ct *s) {
  s32ct_idle_watch *V = &s->idle_watch;
  V->active = true;
  V->head = s->cpu.pc;
  V->steps = 0;
  V->io0 = s->io_accesses;
  V->exc0 = s->cpu.exceptions;
  pic24_wlog_start(&s->cpu);
  idle_regs(s, V->regs);
}
static void idle_watch_stop(s32ct *s, bool cool) {
  s->idle_watch.active = false;
  if (cool) s->idle_watch.cooldown = IDLE_COOLDOWN;
  pic24_wlog_stop(&s->cpu);
}
/* After each instruction while watching: true when the loop came back unchanged and time may move on. */
static bool idle_watch_step(s32ct *s) {
  s32ct_idle_watch *V = &s->idle_watch;
  if (s->cpu.exceptions != V->exc0) {                            /* interrupted: watch again when it comes back */
    V->resume = V->head;
    idle_watch_stop(s, false);
    return false;
  }
  if (s->io_accesses != V->io0 || s->cpu.wlog_full || ++V->steps > IDLE_MAX_STEPS) {
    idle_watch_stop(s, true);
    return false;
  }
  if (s->cpu.pc != V->head) return false;
  uint16_t r[S32CT_IDLE_REGS];
  idle_regs(s, r);
  if (memcmp(r, V->regs, sizeof r)) { idle_watch_stop(s, true); return false; }   /* a busy loop, for now */
  if (pic24_wlog_restored(&s->cpu)) return true;
  idle_watch_start(s);                       /* same registers, memory written: once round more from here */
  return false;
}

pic24_status s32ct_run_to(s32ct *s, uint64_t target) {
  return target > s->cpu.cycles ? s32ct_run(s, target - s->cpu.cycles) : PIC24_STOP_CYCLES;
}

static void s32ct_reset_periphs(s32ct *s);

pic24_status s32ct_run(s32ct *s, uint64_t cycles) {
  uint64_t end = s->cpu.cycles + cycles;
  pic24_status st = PIC24_STOP_CYCLES;
  while (s->cpu.cycles < end) {
    uint64_t slice = end - s->cpu.cycles, ev = timer_next_event(s);
    if (ev < slice) slice = ev;
    uint64_t ft = s32_gfx_frame_tcy(&s->gfx);
    if (ft && !s->next_frame) s->next_frame = s->cpu.cycles + ft;
    if (s->next_frame && s->next_frame > s->cpu.cycles && s->next_frame - s->cpu.cycles < slice)
      slice = s->next_frame - s->cpu.cycles;
    if (s->adc_done && s->adc_done > s->cpu.cycles && s->adc_done - s->cpu.cycles < slice)
      slice = s->adc_done - s->cpu.cycles;
    for (int u = 0; u < 4; u++)
      if (s->uart[u].rx_next && s->uart[u].rx_next > s->cpu.cycles && s->uart[u].rx_next - s->cpu.cycles < slice)
        slice = s->uart[u].rx_next - s->cpu.cycles;
    uint64_t slice_end = s->cpu.cycles + (slice ? slice : 1);
    s->slice_end = slice_end;
    if (!s->idle_on) {
      st = pic24_run(&s->cpu, slice ? slice : 1);
    } else {
      s32ct_idle_watch *V = &s->idle_watch;
      st = PIC24_STOP_CYCLES;
      V->cooldown = 0;
      if (s->cpu.power == PIC24_RUNNING) idle_watch_start(s);
      while (s->cpu.cycles < slice_end) {
        uint32_t prev = s->cpu.pc;
        pic24_status step_st = pic24_step(&s->cpu);
        if (step_st != PIC24_OK) { st = step_st; break; }
        if (s->cpu.stop_request) { s->cpu.stop_request = false; st = PIC24_STOP_BREAK; break; }
        if (V->active) {
          if (idle_watch_step(s)) {
            idle_watch_stop(s, false);
            if (any_irq_pending(s)) continue;
            uint64_t skip = slice_end - s->cpu.cycles;          /* round the loop again and again until the event */
            s->cpu.cycles = slice_end;
            s->idle_skipped += skip;
            break;
          }
        } else if (s->cpu.pc == V->resume) {
          V->resume = 0;
          idle_watch_start(s);
        } else if (V->cooldown) {
          V->cooldown--;
        } else if (s->cpu.pc < prev && prev - s->cpu.pc <= IDLE_MAX_BACK) {
          idle_watch_start(s);
        }
      }
      idle_watch_stop(s, false);                                 /* never across a slice end: events change state */
    }
    if (st == PIC24_STOP_RESET || st == PIC24_STOP_TRAP_CONFLICT) {   /* peripherals already reset by ct_on_reset */
      st = PIC24_STOP_CYCLES;
      continue;
    }
    for (int i = 0; i < 5; i++) timer_advance(s, i, s->cpu.cycles);
    if (s->adc_done && s->cpu.cycles >= s->adc_done) adc_complete(s);
    for (int u = 0; u < 4; u++) if (s->uart[u].rx_next && s->cpu.cycles >= s->uart[u].rx_next) uart_rx_land(s, u);
    if (s->next_frame && s->cpu.cycles >= s->next_frame) { s32_gfx_vblank(&s->gfx, &s->cpu); s->next_frame += ft ? ft : 246000; }
    if (st != PIC24_STOP_CYCLES) break;
  }
  return st;
}

uint16_t s32ct_lat(const s32ct *s, unsigned p) { return s->sfr[(PORT_BASE + 8u * p + 4u) >> 1]; }

static inline int spi_index(uint16_t a) {
  if ((uint16_t)(a - 0x0240u) > 0x0048u) return -1;
  for (int i = 0; i < 3; i++)
    if (a >= SPI_BASE[i] && a <= SPI_BASE[i] + 8) return i;
  return -1;
}

/* Tcy per bit: SPIxCON1 PPRE<1:0> (bits 1:0: 11=1, 10=4, 01=16, 00=64) x SPRE<2:0> (bits 4:2: 111=1 .. 000=8). */
static unsigned spi_tcy_per_bit(uint16_t con1) {
  static const unsigned ppre[4] = {64, 16, 4, 1};
  unsigned spre = 8u - ((con1 >> 2) & 7u);
  return ppre[con1 & 3u] * spre;
}

/* Re-evaluate chip selects after a latch change: call select() on edges. */
static void spi_update_cs(s32ct *s) {
  for (int i = 0; i < 3; i++) {
    s32_spi *sp = &s->spi[i];
    for (unsigned d = 0; d < sp->ndevs; d++) {
      bool low = !((s32ct_lat(s, sp->devs[d].cs.port) >> sp->devs[d].cs.bit) & 1u);
      if (low != sp->devs[d].asserted) {
        sp->devs[d].asserted = low;
        if (s->log) fprintf(s->log, "%llu SPI%d CS %s %s\n", (unsigned long long)s->cpu.cycles, i + 1,
                            sp->devs[d].dev.name, low ? "LOW" : "high");
        if (sp->devs[d].dev.select) sp->devs[d].dev.select(sp->devs[d].dev.ctx, low);
      }
    }
  }
}

static void spi_complete(s32ct *s, s32_spi *sp) {
  if (sp->busy_until && s->cpu.cycles >= sp->busy_until) {
    if (sp->rbf) sp->stat |= 0x0040; /* SPIROV: previous byte never read */
    sp->rxbuf = sp->pending_rx;
    sp->rbf = true;
    sp->busy_until = 0;
  }
}

static void spi_start(s32ct *s, int i, uint8_t mosi) {
  s32_spi *sp = &s->spi[i];
  if (!(sp->stat & 0x8000)) return; /* SPIEN off: no transfer */
  uint8_t miso = 0xFF;              /* idle line: nothing selected drives MISO */
  int who = -1;
  for (unsigned d = 0; d < sp->ndevs; d++)
    if (sp->devs[d].asserted) {
      if (who >= 0 && s->log) fprintf(s->log, "SPI%d: two devices selected (%s, %s)\n", i + 1,
                                      sp->devs[who].dev.name, sp->devs[d].dev.name);
      who = (int)d;
      miso = sp->devs[d].dev.exchange ? sp->devs[d].dev.exchange(sp->devs[d].dev.ctx, mosi) : 0xFF;
    }
  sp->pending_rx = miso;
  sp->busy_until = s->cpu.cycles + 8u * spi_tcy_per_bit(sp->con1) + 1u;
  s->spi_bytes++;
  if (s->log) {
    fprintf(s->log, "%llu SPI%d %s > %02X < %02X  [LATF=%04X LATG=%04X]\n", (unsigned long long)s->cpu.cycles,
            i + 1, who >= 0 ? sp->devs[who].dev.name : "(none)", mosi, miso, s32ct_lat(s, 5), s32ct_lat(s, 6));
  }
}

static uint16_t sfr_read(void *ctx, pic24_cpu *c, uint16_t a) {
  s32ct *s = ctx;
  uint16_t v = 0;
  s->io_accesses++;
  if (a >= 0x0700 && a <= 0x073E) {
    v = s32_gfx_read(&s->gfx, c, a);
  } else {
    int u = uart_index(a);
    if (u >= 0) {
      v = uart_read(s, u, (uint16_t)(a - UART_BASE[u]));
    } else {
      int which, ti = timer_index(a, &which);
      if (ti >= 0) {
        timer_advance(s, ti, s->cpu.cycles);
        v = which == 0 ? (uint16_t)s->tmr[ti].count0 : which == 1 ? s->tmr[ti].pr : s->tmr[ti].con;
      } else {
        int i = spi_index(a);
        if (i >= 0) {
          s32_spi *sp = &s->spi[i];
          spi_complete(s, sp);
          uint16_t off = (uint16_t)(a - SPI_BASE[i]);
          if (off == 0) {
            v = (uint16_t)((sp->stat & ~0x0003u) | (sp->rbf ? 1u : 0u) | (sp->busy_until ? 2u : 0u));
            if (s->idle_on && sp->busy_until > s->cpu.cycles && s->last_sfr_addr == a && s->last_sfr_val == v &&
                ++s->same_sfr_reads >= 1 && !any_irq_pending(s)) {
              uint64_t change = sp->busy_until;
              if (s->slice_end && s->slice_end < change) change = s->slice_end;
              if (change > s->cpu.cycles) {
                uint64_t w = change - s->cpu.cycles;
                s->cpu.cycles = change;
                s->idle_skipped += w;
                spi_complete(s, sp);
                v = (uint16_t)((sp->stat & ~0x0003u) | (sp->rbf ? 1u : 0u) | (sp->busy_until ? 2u : 0u));
              }
            }
          } else if (off == 2) v = sp->con1;
          else if (off == 4) v = sp->con2;
          else if (off == 8) {
            sp->rbf = false;
            v = sp->rxbuf;
          }
        } else if (a >= PORT_BASE && a < PORT_TOP && ((a - PORT_BASE) & 7u) == 2u) {
          /* PORTx: output pins read their latch; input pins read the external level (default high: pull-ups) */
          uint16_t tris = *reg(s, (uint16_t)(a - 2u)), lat = *reg(s, (uint16_t)(a + 2u));
          unsigned p = (a - PORT_BASE) >> 3;
          uint16_t analog = *reg(s, (uint16_t)(ANSA + 2u * p));            /* DS39969 11.2: analog pins read 0 */
          v = (uint16_t)((lat & ~tris) | (s->pin_in[p] & tris & ~analog));
          if (s->cov) s->cov->pin_in[p] |= (uint16_t)(tris & ~analog);
        } else {
          v = *reg(s, a);
        }
      }
    }
  }
  if (s->last_sfr_addr != a || s->last_sfr_val != v) {
    s->last_sfr_addr = a;
    s->last_sfr_val = v;
    s->same_sfr_reads = 0;
  }
  return v;
}

static void tblwt_cb(void *ctx, pic24_cpu *c, uint32_t addr24, uint16_t val, uint16_t mask, bool high) {
  (void)c;
  s32ct *s = ctx;
  s->io_accesses++;
  s->nvm_addr = addr24 & 0xFFFFFEu;
  unsigned idx = (addr24 >> 1) & 0x3Fu;
  uint32_t cur = s->nvm_latch[idx];
  if (!high) {
    uint16_t lo = (uint16_t)(((cur & 0xFFFFu) & ~mask) | (val & mask));
    s->nvm_latch[idx] = (cur & 0xFF0000u) | lo;
  } else {
    uint8_t hi = (uint8_t)((((cur >> 16) & 0xFFu) & ~(mask & 0xFFu)) | (val & mask & 0xFFu));
    s->nvm_latch[idx] = ((uint32_t)hi << 16) | (cur & 0xFFFFu);
  }
}

static void nvm_reset_latches(s32ct *s) {
  for (int i = 0; i < 64; i++) s->nvm_latch[i] = 0xFFFFFFu;
}

static void sfr_write(void *ctx, pic24_cpu *c, uint16_t a, uint16_t v, uint16_t m) {
  s32ct *s = ctx;
  s->io_accesses++;
  s->last_sfr_addr = 0xFFFFu; /* any SFR write breaks a poll sequence */
  if (a >= 0x0700 && a <= 0x073E) { s32_gfx_write(&s->gfx, c, a, v, m); return; }
  if (a == 0x0766u) { /* NVMKEY: 0x55 followed by 0xAA unlocks WR in NVMCON (DS39969 section 5.2) */
    uint8_t key = (uint8_t)(v & m & 0xFFu);
    if (key == 0x55u) s->nvm_key_step = 1;
    else if (key == 0xAAu && s->nvm_key_step == 1) s->nvm_key_step = 2;
    else s->nvm_key_step = 0;
    *reg(s, a) = 0;
    return;
  }
  if (a == 0x0760u) { /* NVMCON: bit15 WR, bit14 WREN, bit6 ERASE, bits3:0 NVMOP */
    uint16_t prev = *reg(s, a);
    uint16_t next = (uint16_t)((prev & ~m) | (v & m));
    if (!(prev & 0x8000u) && (next & 0x8000u)) {
      if (s->nvm_key_step == 2 && (next & 0x4000u)) {
        uint16_t op = next & 0x004Fu;
        if (op == 0x0042u) { /* Page Erase: 512 instructions (0x400 PC units) */
          uint32_t base_w = (s->nvm_addr & 0xFFFFFC00u) >> 1;
          for (uint32_t k = 0; k < 512u; k++)
            if (base_w + k < PIC24_PROG_WORDS) c->prog[base_w + k] = 0xFFFFFFu;
          s->nvm_erases++;
        } else if (op == 0x0001u) { /* Row Program: 64 instructions (0x80 PC units) */
          uint32_t base_w = (s->nvm_addr & 0xFFFFFF80u) >> 1;
          for (uint32_t k = 0; k < 64u; k++)
            if (base_w + k < PIC24_PROG_WORDS) c->prog[base_w + k] &= s->nvm_latch[k] & 0xFFFFFFu;
          nvm_reset_latches(s);
          s->nvm_writes++;
        } else if (op == 0x0003u) { /* Word Program: 1 instruction (2 PC units) */
          uint32_t w = (s->nvm_addr & 0xFFFFFFFEu) >> 1;
          if (w < PIC24_PROG_WORDS) c->prog[w] &= s->nvm_latch[w & 0x3Fu] & 0xFFFFFFu;
          nvm_reset_latches(s);
          s->nvm_writes++;
        }
      }
      next &= (uint16_t)~0x8000u; /* WR self-clears when flash operation completes */
    } else {
      next &= (uint16_t)~0x8000u;
    }
    s->nvm_key_step = 0;
    *reg(s, a) = next;
    return;
  }
  { int u = uart_index(a); if (u >= 0) { uart_write(s, u, (uint16_t)(a - UART_BASE[u]), v, m); return; } }
  int which, ti = timer_index(a, &which);
  if (ti >= 0) {
    s32_timer *t = &s->tmr[ti];
    timer_advance(s, ti, s->cpu.cycles);
    if (which == 0) t->count0 = (uint16_t)((t->count0 & ~m) | (v & m));
    else if (which == 1) t->pr = (uint16_t)((t->pr & ~m) | (v & m));
    else { bool was = timer_on(t); t->con = (uint16_t)((t->con & ~m) | (v & m)); if (!was) t->base = s->cpu.cycles; }
    t->base = s->cpu.cycles;
    *reg(s, a) = which == 0 ? (uint16_t)t->count0 : which == 1 ? t->pr : t->con;
    return;
  }
  int i = spi_index(a);
  if (i >= 0) {
    s32_spi *sp = &s->spi[i];
    spi_complete(s, sp);
    uint16_t off = (uint16_t)(a - SPI_BASE[i]);
    if (off == 0) {
      uint16_t w = (uint16_t)((sp->stat & ~m) | (v & m));
      /* SPIEN, SPISIDL, SPIROV (clear-only) are writable; buffer flags are read-only */
      sp->stat = (uint16_t)((w & 0xA000u) | (sp->stat & w & 0x0040u));
      if (!(sp->stat & 0x8000)) { sp->rbf = false; sp->busy_until = 0; }
    } else if (off == 2) sp->con1 = (uint16_t)((sp->con1 & ~m) | (v & m));
    else if (off == 4) sp->con2 = (uint16_t)((sp->con2 & ~m) | (v & m));
    else if (off == 8) spi_start(s, i, (uint8_t)(v & m & 0xFF));
    return;
  }
  if (a >= PORT_BASE && a < PORT_TOP && ((a - PORT_BASE) & 7u) == 2u) a = (uint16_t)(a + 2u); /* PORTx write -> LATx */
  unsigned pp = (a - PORT_BASE) >> 3;
  uint16_t pins_before = 0;
  if (a >= PORT_BASE && a < PORT_TOP) pins_before = (uint16_t)(*reg(s, (uint16_t)(PORT_BASE + 8u * pp + 4u)) & ~*reg(s, (uint16_t)(PORT_BASE + 8u * pp)));
  uint16_t *r = reg(s, a);
  *r = (uint16_t)((*r & ~m) | (v & m));
  if (a >= PORT_BASE && a < PORT_TOP && s->lat_sink) {
    uint16_t now = (uint16_t)(*reg(s, (uint16_t)(PORT_BASE + 8u * pp + 4u)) & ~*reg(s, (uint16_t)(PORT_BASE + 8u * pp)));
    if (now != pins_before) s->lat_sink(s->lat_ctx, s, s->cpu.cycles, pp, pins_before, now);
  }
  if (a >= PORT_BASE && a < PORT_TOP) {
    spi_update_cs(s);
    if (s->cov) for (unsigned p = 0; p < 7; p++) s->cov->pin_out[p] |= (uint16_t)~*reg(s, (uint16_t)(PORT_BASE + 8u * p));
  }
  if (a == AD1CON1) adc_kick(s);
}

static void cov_watch(void *ctx, pic24_cpu *c, uint16_t addr, uint16_t value, uint16_t lane_mask, bool write) {
  (void)c; (void)value; (void)lane_mask;
  s32ct *s = ctx;
  s32_iocov_sfr(s->cov, addr >> 1, write);
}
/* An observer's read (panel backlight, reports): the firmware must not be able to tell it happened. The idle-poll
 * detector's last read and the I/O-coverage hook are left as the firmware's own reads set them. */
uint16_t s32ct_sfr_value(s32ct *s, uint16_t addr) {
  if (addr >= 0x0800u) return 0u;
  uint16_t la = s->last_sfr_addr, lv = s->last_sfr_val;
  unsigned same = s->same_sfr_reads;
  void (*watch)(void *, pic24_cpu *, uint16_t, uint16_t, uint16_t, bool) = s->cpu.cfg.sfr_watch;
  s->cpu.cfg.sfr_watch = NULL;
  uint16_t v = pic24_peek16(&s->cpu, addr);
  s->cpu.cfg.sfr_watch = watch;
  s->last_sfr_addr = la; s->last_sfr_val = lv; s->same_sfr_reads = same;
  return v;
}
void s32ct_iocov_enable(s32ct *s) {
  if (!s->cov) s->cov = s32_iocov_new(0, 0x800u / 2u, 2);
  s->cpu.cfg.sfr_watch = cov_watch;
}

/* RCON (DS39969 Register 6-1, Table 6-1): a POR loads POR | BOR (the POR table); every other reset keeps the status
 * flags and sets its own cause bit. Flags kept: TRAPR IOPUWR CM EXTR SWR WDTO SLEEP IDLE BOR POR. SWDTEN and VREGS
 * are controls and take their reset value 0. */
#define RCON_ADDR 0x0740u
#define RCON_FLAGS 0xC2DFu
static void rcon_after_reset(s32ct *s, uint16_t before, uint16_t cause) {
  s->sfr[RCON_ADDR >> 1] = (uint16_t)((before & RCON_FLAGS) | cause);
}
static uint16_t rcon_cause(pic24_status why) {
  switch (why) {
  case PIC24_STOP_ILLEGAL: return S32CT_RCON_IOPUWR;       /* illegal opcode / address mode, BREAK without debugger */
  case PIC24_STOP_TRAP_CONFLICT: return S32CT_RCON_TRAPR;
  default: return S32CT_RCON_SWR;                           /* RESET instruction */
  }
}
/* A reset the core performed (RESET instruction, illegal opcode, trap conflict): peripherals as at MCLR, RCON flag. */
static void ct_on_reset(void *ctx, pic24_cpu *c, pic24_status why) {
  (void)c;
  s32ct *s = ctx;
  uint16_t before = s->sfr[RCON_ADDR >> 1];
  s32ct_reset_periphs(s);
  rcon_after_reset(s, before, rcon_cause(why));
  if (why == PIC24_STOP_ILLEGAL) s->illegal_resets++;
  else if (why == PIC24_STOP_TRAP_CONFLICT) s->trap_resets++;
  else s->sw_resets++;
}

static bool env_flag(const char *name) {
  const char *v = getenv(name);
  return v && atoi(v) != 0;
}

void s32ct_init(s32ct *s, const uint8_t *prog, size_t nprog) {
  if (pic24_cpu_size() != sizeof(pic24_cpu)) {           /* core library and this file built from different pic24.h */
    fprintf(stderr, "s32ct: libpic24 was built from a different pic24.h (sizeof(pic24_cpu) %zu there, %zu here): "
            "rebuild emu/pic24\n", pic24_cpu_size(), sizeof(pic24_cpu));
    abort();
  }
  FILE *log = s->log, *glog = s->gfx.log;
  unsigned gb = s->gfx.log_budget;
  s32_iocov *cov = s->cov;
  void (*ls)(void *, struct s32ct *, uint64_t, unsigned, uint16_t, uint16_t) = s->lat_sink;
  void *lc = s->lat_ctx;
  memset(s, 0, sizeof *s);
  s->log = log; s->gfx.log = glog; s->gfx.log_budget = gb;
  s->lat_sink = ls; s->lat_ctx = lc;
  for (size_t k = 0; k < sizeof SFR_ROWS / sizeof SFR_ROWS[0]; k++) s->sfr[SFR_ROWS[k].addr >> 1] = SFR_ROWS[k].por;
  for (int p = 0; p < 7; p++) s->pin_in[p] = 0xFFFFu;
  for (int i = 0; i < 5; i++) { s->tmr[i].pr = s->sfr[PR_ADDR[i] >> 1]; s->tmr[i].con = s->sfr[CON_ADDR[i] >> 1]; }
  nvm_reset_latches(s);
  pic24_config cfg = {0};
  cfg.ctx = s;
  cfg.sfr_read = sfr_read;
  cfg.sfr_write = sfr_write;
  cfg.tblwt = tblwt_cb;
  cfg.on_reset = ct_on_reset;
  cfg.illegal_stops = env_flag("PIC24_ILLEGAL_STOPS");   /* debug only: stop on an illegal opcode instead of resetting */
  cfg.debugger = env_flag("PIC24_DEBUGGER");             /* BREAK halts (PIC24_STOP_BREAK) instead of resetting */
  pic24_init(&s->cpu, &cfg);
  pic24_load_prog(&s->cpu, prog, nprog);
  if (cov) { s->cov = cov; s32ct_iocov_enable(s); }      /* a power cycle keeps the coverage record */
}

static void s32ct_reset_periphs(s32ct *s) {
  for (size_t k = 0; k < sizeof SFR_ROWS / sizeof SFR_ROWS[0]; k++) s->sfr[SFR_ROWS[k].addr >> 1] = SFR_ROWS[k].por;
  for (int i = 0; i < 5; i++) {
    s->tmr[i].pr = s->sfr[PR_ADDR[i] >> 1];
    s->tmr[i].con = s->sfr[CON_ADDR[i] >> 1];
    s->tmr[i].base = s->cpu.cycles;
    s->tmr[i].count0 = 0;
  }
  for (int i = 0; i < 3; i++) {
    s->spi[i].stat = 0; s->spi[i].con1 = 0; s->spi[i].con2 = 0;
    s->spi[i].rbf = false; s->spi[i].busy_until = 0;
  }
  for (int u = 0; u < 4; u++) {
    s->uart[u].tx_busy_until = 0; s->uart[u].rh = s->uart[u].rt = 0;
    s->uart[u].fn = 0; s->uart[u].rsr_full = false; s->uart[u].rx_next = 0;
  }
  s->adc_done = 0;
  s->next_frame = 0;
  s->nvm_key_step = 0;
  nvm_reset_latches(s);
  spi_update_cs(s);
}

void s32ct_reset(s32ct *s) {
  uint16_t before = s->sfr[RCON_ADDR >> 1];
  pic24_reset(&s->cpu);
  s32ct_reset_periphs(s);
  rcon_after_reset(s, before, S32CT_RCON_EXTR);          /* MCLR pin */
}

void s32ct_spi_attach(s32ct *s, unsigned n, s32_pin cs, s32_spi_dev dev) {
  s32_spi *sp = &s->spi[n - 1];
  if (sp->ndevs >= S32_MAX_SPI_DEVS) return;
  sp->devs[sp->ndevs].cs = cs;
  sp->devs[sp->ndevs].dev = dev;
  sp->devs[sp->ndevs].asserted = !((s32ct_lat(s, cs.port) >> cs.bit) & 1u);
  sp->ndevs++;
}

size_t s32ct_pflash_read(const s32ct *s, uint8_t *dst, size_t cap) {
  size_t n = cap < S32CT_PFL_BYTES ? cap : S32CT_PFL_BYTES;
  for (size_t b = 0; b < n; b++) dst[b] = (uint8_t)(s->cpu.prog[b / 3u] >> (8u * (b % 3u)));
  return n;
}
