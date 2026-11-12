/* Unit tests for the Cockpit Controller SoC model. Peripherals are driven through the core's data-bus access
 * (pic24_peek16 / pic24_poke16 invoke the SFR callbacks), with cpu.cycles advanced by hand. */
#include "minitest.h"
#include "s32ct.h"
#include <string.h>

#define SPI2STAT 0x0260
#define SPI2CON1 0x0262
#define SPI2BUF  0x0268
#define TRISG 0x02F0
#define PORTG 0x02F2
#define LATG  0x02F4
#define ADC1BUF0 0x0300
#define AD1CON1  0x0320
#define AD1CON2  0x0322
#define AD1CON3  0x0324
#define AD1CSSL  0x0330

typedef struct { uint8_t got[16]; unsigned n; int selects, deselects; uint8_t reply; } fake_dev;
static uint8_t fx(void *ctx, uint8_t mosi) { fake_dev *d = ctx; if (d->n < 16) d->got[d->n++] = mosi; return d->reply++; }
static void fsel(void *ctx, bool a) { fake_dev *d = ctx; if (a) d->selects++; else d->deselects++; }

static s32ct S;
static fake_dev D;
static const uint8_t NOPROG[3] = {0, 0, 0};

static void setup(void) {
  memset(&D, 0, sizeof D);
  D.reply = 0xA0;
  s32ct_init(&S, NOPROG, sizeof NOPROG);
  pic24_poke16(&S.cpu, TRISG, 0x0000);          /* all G outputs */
  pic24_poke16(&S.cpu, LATG, 0x0080);           /* G7 high = deselected */
  s32ct_spi_attach(&S, 2, (s32_pin){6, 7}, (s32_spi_dev){"dev", &D, fx, fsel});
  pic24_poke16(&S.cpu, SPI2CON1, 0x0331);       /* PPRE 16:1, SPRE 4:1 => 64 Tcy/bit */
  pic24_poke16(&S.cpu, SPI2STAT, 0x8000);       /* SPIEN */
}

TEST(por_values_come_from_the_table) {
  s32ct_init(&S, NOPROG, sizeof NOPROG);
  CHECK_EQ(pic24_peek16(&S.cpu, TRISG), 0xF3CF); /* TRISG POR in model/pic24fj256da210_sfr.json */
}

TEST(chip_select_edges_follow_the_latch) {
  setup();
  CHECK_EQ(D.selects, 0);
  pic24_poke16(&S.cpu, LATG, 0x0000);
  CHECK_EQ(D.selects, 1);
  pic24_poke16(&S.cpu, PORTG, 0x0080);          /* a PORT write lands in LAT */
  CHECK_EQ(D.deselects, 1);
  CHECK_EQ(pic24_peek16(&S.cpu, LATG), 0x0080);
}

TEST(byte_exchange_timing_and_flags) {
  setup();
  pic24_poke16(&S.cpu, LATG, 0x0000);
  S.cpu.cycles = 1000;
  pic24_poke16(&S.cpu, SPI2BUF, 0x9F);
  CHECK_EQ(D.n, 1);
  CHECK_EQ(D.got[0], 0x9F);
  CHECK_EQ(pic24_peek16(&S.cpu, SPI2STAT) & 3, 2);   /* SPITBF busy, SPIRBF clear */
  S.cpu.cycles = 1000 + 8 * 64;                       /* one bit-time short of done (+1 Tcy load) */
  CHECK_EQ(pic24_peek16(&S.cpu, SPI2STAT) & 1, 0);
  S.cpu.cycles = 1000 + 8 * 64 + 1;
  CHECK_EQ(pic24_peek16(&S.cpu, SPI2STAT) & 3, 1);   /* SPIRBF set, idle */
  CHECK_EQ(pic24_peek16(&S.cpu, SPI2BUF), 0xA0);
  CHECK_EQ(pic24_peek16(&S.cpu, SPI2STAT) & 1, 0);   /* reading BUF clears SPIRBF */
}

TEST(deselected_device_sees_nothing_and_miso_idles_high) {
  setup();                                           /* G7 high */
  pic24_poke16(&S.cpu, SPI2BUF, 0x55);
  S.cpu.cycles += 10000;
  CHECK_EQ(D.n, 0);
  CHECK_EQ(pic24_peek16(&S.cpu, SPI2BUF), 0xFF);
}

TEST(overflow_sets_spirov_and_disable_drops_state) {
  setup();
  pic24_poke16(&S.cpu, LATG, 0x0000);
  pic24_poke16(&S.cpu, SPI2BUF, 0x01); S.cpu.cycles += 10000; (void)pic24_peek16(&S.cpu, SPI2STAT);
  pic24_poke16(&S.cpu, SPI2BUF, 0x02); S.cpu.cycles += 10000;
  CHECK(pic24_peek16(&S.cpu, SPI2STAT) & 0x0040);   /* SPIROV */
  pic24_poke16(&S.cpu, SPI2STAT, 0x0000);            /* SPIEN off */
  CHECK_EQ(pic24_peek16(&S.cpu, SPI2STAT) & 1, 0);
  pic24_poke16(&S.cpu, SPI2BUF, 0x03);               /* ignored while disabled */
  CHECK_EQ(D.n, 2);
}

#define IFS0  0x0084
#define TMR1  0x0100
#define PR1   0x0102
#define T1CON 0x0104

TEST(timer1_period_match_raises_t1if_on_time) {
  s32ct_init(&S, NOPROG, sizeof NOPROG);          /* executes NOPs (opcode 0) */
  pic24_poke16(&S.cpu, PR1, 99);
  pic24_poke16(&S.cpu, T1CON, 0x8000);            /* TON, 1:1, internal clock */
  uint64_t t0 = S.cpu.cycles;
  s32ct_run(&S, 99);
  CHECK_EQ(pic24_peek16(&S.cpu, IFS0) & 0x0008, 0);
  s32ct_run(&S, (t0 + 100) - S.cpu.cycles + 1);
  CHECK_EQ(pic24_peek16(&S.cpu, IFS0) & 0x0008, 0x0008);
  CHECK_EQ(S.timer_irqs[0], 1);
}

TEST(timer_matches_come_pr_plus_1_ticks_apart) {
  /* DS39969 section 11 (Timer1): the period match is at PR+1 counts and the count restarts from 0, so consecutive
   * interrupts are PR+1 ticks apart. */
  s32ct_init(&S, NOPROG, sizeof NOPROG);
  pic24_poke16(&S.cpu, PR1, 99);
  pic24_poke16(&S.cpu, T1CON, 0x8000);
  uint64_t t0 = S.cpu.cycles;
  s32ct_run(&S, 101);
  CHECK_EQ(S.timer_irqs[0], 1);                   /* first match at tick 100 */
  s32ct_run(&S, (t0 + 199) - S.cpu.cycles);
  CHECK_EQ(S.timer_irqs[0], 1);                   /* the second at tick 200, not 199 */
  s32ct_run(&S, (t0 + 201) - S.cpu.cycles);
  CHECK_EQ(S.timer_irqs[0], 2);
}

TEST(timer_count_above_pr_wraps_at_ffff) {
  /* PIC24F FRM, Timers chapter (DS39704): the 16-bit counter counts through 0xFFFF before wrapping, so a period
   * register lowered below the current count is reached only after the wrap, and the match fires there. */
  s32ct_init(&S, NOPROG, sizeof NOPROG);
  pic24_poke16(&S.cpu, PR1, 0xFFFF);
  pic24_poke16(&S.cpu, T1CON, 0x8000);
  s32ct_run(&S, 5000);
  uint16_t c0 = pic24_peek16(&S.cpu, TMR1);
  CHECK(c0 > 999 && c0 < 0xF000);
  pic24_poke16(&S.cpu, PR1, 999);                 /* PR below the count */
  s32ct_run(&S, 400);
  CHECK_EQ(pic24_peek16(&S.cpu, TMR1), c0 + 400); /* keeps climbing toward 0xFFFF: no early wrap at PR */
  s32ct_run(&S, (uint64_t)(0x10000u - (c0 + 400u)) + 1000u + 40);
  CHECK(S.timer_irqs[0] >= 1);                    /* the match after the wrap fires */
  CHECK_EQ(pic24_peek16(&S.cpu, IFS0) & 0x0008, 0x0008);
}

TEST(timer_above_pr_multi_cycle_advance_keeps_every_match) {
  /* DS39704 (Timers): the same lowered-PR corner over one advance spanning several full counter cycles: after the
   * 0xFFFF wrap the count is in the <= PR regime and every PR+1 period matches. */
  s32ct_init(&S, NOPROG, sizeof NOPROG);
  pic24_poke16(&S.cpu, PR1, 0xFFFF);
  pic24_poke16(&S.cpu, T1CON, 0x8000);
  s32ct_run(&S, 5000);                            /* count = 5000 */
  pic24_poke16(&S.cpu, PR1, 999);                 /* PR below the count */
  s32ct_run(&S, 3 * 65536);                       /* one advance over three full counter cycles */
  /* 60536 ticks to the wrap, then 136072 ticks in the <= PR regime: 136 matches, count 72 */
  CHECK_EQ(S.timer_irqs[0], 136);
  CHECK_EQ(pic24_peek16(&S.cpu, TMR1), 72);
}

TEST(adc_scan_fills_smpi_plus_1_buffers) {
  /* DS39969 section 23 (ADC): with CSCNA the converter scans the AD1CSSL list and interrupts after SMPI+1
   * conversions; with SMPI = 1 each sequence writes ADC1BUF0 and ADC1BUF1 (in scan order). */
  s32ct_init(&S, NOPROG, sizeof NOPROG);
  s32ct_set_an(&S, 10, 0x2AA);
  s32ct_set_an(&S, 11, 0x155);
  pic24_poke16(&S.cpu, AD1CSSL, 0x0C00);          /* AN10, AN11 */
  pic24_poke16(&S.cpu, AD1CON2, 0x0404);          /* CSCNA, SMPI 1 */
  pic24_poke16(&S.cpu, AD1CON3, 0x0101);          /* SAMC 1, ADCS 1 (SAMC 0 is not a legal setting) */
  pic24_poke16(&S.cpu, AD1CON1, 0x80E4);          /* ON, auto-convert, ASAM */
  s32ct_run(&S, 400);
  CHECK(S.adc_irqs > 0);
  CHECK_EQ(pic24_peek16(&S.cpu, ADC1BUF0), 0x2AA);
  CHECK_EQ(pic24_peek16(&S.cpu, ADC1BUF0 + 2), 0x155);
}

TEST(timer_prescale_and_count_read) {
  s32ct_init(&S, NOPROG, sizeof NOPROG);
  pic24_poke16(&S.cpu, PR1, 9999);
  pic24_poke16(&S.cpu, T1CON, 0x8010);            /* TON, 1:8 -> 200 Hz at 16 MHz */
  s32ct_run(&S, 8 * 1234);
  uint16_t c = pic24_peek16(&S.cpu, TMR1);
  CHECK(c >= 1233 && c <= 1235);
  s32ct_run(&S, 16000000);                        /* one second */
  CHECK(S.timer_irqs[0] >= 199 && S.timer_irqs[0] <= 201);
}

TEST(nvm_flash_erase_and_program) {
  s32ct_init(&S, NOPROG, sizeof NOPROG);
  S.cpu.prog[0x5A14 >> 1] = 0x00A5A5u; /* an arbitrary programmed word */
  S.cpu.cfg.tblwt(&S, &S.cpu, 0x005A14u, 0x3C3Cu, 0xFFFFu, false);
  S.cpu.cfg.tblwt(&S, &S.cpu, 0x005A14u, 0x005Au, 0x00FFu, true);
  /* Without NVMKEY unlock, setting WR (0x8000) must not erase or write */
  pic24_poke16(&S.cpu, 0x0760, 0x4042);
  pic24_poke16(&S.cpu, 0x0760, 0xC042);
  CHECK_EQ(S.cpu.prog[0x5A14 >> 1], 0x00A5A5u);
  CHECK_EQ(S.nvm_erases, 0u);
  /* Proper NVMKEY 0x55 -> 0xAA unlock + Page Erase (0x4042) */
  pic24_poke16(&S.cpu, 0x0760, 0x4042);
  pic24_poke16(&S.cpu, 0x0766, 0x0055);
  pic24_poke16(&S.cpu, 0x0766, 0x00AA);
  pic24_poke16(&S.cpu, 0x0760, 0xC042);
  CHECK_EQ(pic24_peek16(&S.cpu, 0x0760) & 0x8000u, 0u); /* WR self-clears */
  CHECK_EQ(S.nvm_erases, 1u);
  CHECK_EQ(S.cpu.prog[0x5A14 >> 1], 0xFFFFFFu);
  /* Row Program (0x4001) at row containing 0x5A14 */
  S.cpu.cfg.tblwt(&S, &S.cpu, 0x005A14u, 0x3C3Cu, 0xFFFFu, false);
  S.cpu.cfg.tblwt(&S, &S.cpu, 0x005A14u, 0x005Au, 0x00FFu, true);
  pic24_poke16(&S.cpu, 0x0760, 0x4001);
  pic24_poke16(&S.cpu, 0x0766, 0x0055);
  pic24_poke16(&S.cpu, 0x0766, 0x00AA);
  pic24_poke16(&S.cpu, 0x0760, 0xC001);
  CHECK_EQ(S.nvm_writes, 1u);
  CHECK_EQ(S.cpu.prog[0x5A14 >> 1], 0x5A3C3Cu);
  /* s32ct_reset preserves flash contents; s32ct_pflash_read reads back the packed 3-byte words */
  s32ct_reset(&S);
  CHECK_EQ(S.cpu.prog[0x5A14 >> 1], 0x5A3C3Cu);
  static uint8_t chip[S32CT_PFL_BYTES];
  CHECK_EQ(s32ct_pflash_read(&S, chip, sizeof chip), (size_t)S32CT_PFL_BYTES);
  size_t off = (0x5A14u >> 1) * 3u;
  CHECK_EQ((uint32_t)chip[off] | ((uint32_t)chip[off + 1] << 8) | ((uint32_t)chip[off + 2] << 16), 0x5A3C3Cu);
}

/* Reset causes in RCON (DS39969 Register 6-1): POR loads POR|BOR; RESET instruction sets SWR, an illegal opcode or
 * BREAK without a debugger sets IOPUWR, MCLR sets EXTR; earlier flags are kept. Peripherals take their reset values
 * and the program restarts at the reset vector. */
TEST(reset_causes_set_rcon_and_reset_peripherals) {
  static uint8_t prog[0x400 * 3];
  memset(prog, 0, sizeof prog);
  /* 0x000000: GOTO 0x000200 ; 0x000200: BREAK ; 0x000202: RESET */
  const uint32_t w[] = {0x040200u, 0x000000u};
  for (int i = 0; i < 2; i++) { prog[3 * i] = (uint8_t)w[i]; prog[3 * i + 1] = (uint8_t)(w[i] >> 8); prog[3 * i + 2] = (uint8_t)(w[i] >> 16); }
  const uint32_t b[] = {0xDA4000u, 0xFE0000u};
  for (int i = 0; i < 2; i++) {
    size_t o = (0x200u / 2u + (unsigned)i) * 3u;
    prog[o] = (uint8_t)b[i]; prog[o + 1] = (uint8_t)(b[i] >> 8); prog[o + 2] = (uint8_t)(b[i] >> 16);
  }
  s32ct_init(&S, prog, sizeof prog);
  CHECK_EQ(pic24_peek16(&S.cpu, 0x0740), S32CT_RCON_POR | S32CT_RCON_BOR);
  pic24_poke16(&S.cpu, TRISG, 0x0000);                 /* a peripheral register away from its reset value */
  /* GOTO, then BREAK (no debugger): illegal-opcode reset, IOPUWR */
  CHECK_EQ(pic24_step(&S.cpu), PIC24_OK);
  CHECK_EQ(S.cpu.pc, 0x200u);
  CHECK_EQ(pic24_step(&S.cpu), PIC24_STOP_RESET);
  CHECK_EQ(S.cpu.pc, 0u);
  CHECK_EQ(S.illegal_resets, 1u);
  CHECK_EQ(pic24_peek16(&S.cpu, 0x0740), S32CT_RCON_IOPUWR | S32CT_RCON_POR | S32CT_RCON_BOR);
  CHECK_EQ(pic24_peek16(&S.cpu, TRISG), 0xF3CF);       /* back at its reset value */
  /* software clears RCON; then MCLR sets EXTR */
  pic24_poke16(&S.cpu, 0x0740, 0x0000);
  s32ct_reset(&S);
  CHECK_EQ(pic24_peek16(&S.cpu, 0x0740), S32CT_RCON_EXTR);
  /* the RESET instruction sets SWR, flags kept; s32ct_run goes on after it */
  S.cpu.pc = 0x202;
  CHECK_EQ(pic24_step(&S.cpu), PIC24_STOP_RESET);
  CHECK_EQ(S.sw_resets, 1u);
  CHECK_EQ(pic24_peek16(&S.cpu, 0x0740), S32CT_RCON_EXTR | S32CT_RCON_SWR);
  CHECK_EQ(s32ct_run(&S, 100), PIC24_STOP_CYCLES);     /* GOTO -> BREAK -> reset, repeatedly: never stops the run */
  CHECK(S.illegal_resets > 1u);
  CHECK(pic24_peek16(&S.cpu, 0x0740) & S32CT_RCON_IOPUWR);
}

/* Idle loops: a loop that goes round with nothing changing is skipped to the next event; a loop that changes state,
 * reads a peripheral, or waits for a flag that an event sets is executed, and a waiting loop leaves when the flag is
 * set. */
static void put24(uint8_t *img, uint32_t pc, uint32_t w) {
  size_t b = (pc / 2u) * 3u;
  img[b] = (uint8_t)w; img[b + 1] = (uint8_t)(w >> 8); img[b + 2] = (uint8_t)(w >> 16);
}
/* Run a two-instruction loop at PC 0 (`insn; bra 0`) for 200,000 Tcy. */
static void run_loop(uint32_t insn, bool skip) {
  static uint8_t img[3 * 4];
  memset(img, 0, sizeof img);
  put24(img, 0, insn);
  put24(img, 2, 0x37FFFEu);                                       /* bra 0 */
  s32ct_init(&S, img, sizeof img);
  s32ct_idle_skip(&S, skip);
  CHECK_EQ(s32ct_run(&S, 200000), PIC24_STOP_CYCLES);
  CHECK(S.cpu.cycles == 200000u || S.cpu.cycles == 200001u);     /* a two-cycle branch may end one cycle over */
}
TEST(idle_loop_skipped_only_when_nothing_changes) {
  run_loop(0x000000u, true);                                      /* nop: nothing changes */
  CHECK(S.idle_skipped > 199000u);
  run_loop(0x000000u, false);                                     /* skipping off */
  CHECK_EQ(S.idle_skipped, 0u);
  run_loop(0xE80000u, true);                                      /* inc w0,w0: a register changes every time */
  CHECK_EQ(S.idle_skipped, 0u);
  run_loop(0x800800u, true);                                      /* mov TMR1,w0: a peripheral read */
  CHECK_EQ(S.idle_skipped, 0u);
}
/* `btss IFS0,#3 ; bra 0 ; nop`: wait for the Timer1 flag with its interrupt disabled. The wait is skipped, and the loop
 * leaves at the period match (Timer1 on at Fcy / 1, PR1 = 9999: the flag at 10,000 Tcy). */
TEST(idle_loop_leaves_when_an_event_sets_its_flag) {
  static uint8_t img[3 * 4];
  memset(img, 0, sizeof img);
  put24(img, 0, 0xAE6084u);                                       /* btss IFS0,#3 (T1IF) */
  put24(img, 2, 0x37FFFEu);                                       /* bra 0 */
  put24(img, 4, 0x000000u);                                       /* nop */
  put24(img, 6, 0x37FFFFu);                                       /* bra . (after the wait) */
  s32ct_init(&S, img, sizeof img);
  s32ct_idle_skip(&S, true);
  pic24_poke16(&S.cpu, 0x0102, 9999);                             /* PR1 */
  pic24_poke16(&S.cpu, 0x0104, 0x8000);                           /* T1CON: on, 1:1 */
  s32ct_run(&S, 30000);
  CHECK(S.idle_skipped > 9000u);
  CHECK(S.cpu.pc >= 4u);                                          /* left the wait */
}

#define U1MODE  0x0220
#define U1STA   0x0222
#define U1TXREG 0x0224
#define U1RXREG 0x0226
#define U1BRG   0x0228

typedef struct { uint8_t buf[16]; unsigned n, last_uart; } uart_cap;
static void cap_tx(void *ctx, unsigned uart, uint8_t b) {
  uart_cap *c = ctx;
  c->last_uart = uart;
  if (c->n < sizeof c->buf) c->buf[c->n++] = b;
}

TEST(uart1_rx_pacing_and_tx_sink) {
  uart_cap cap = {0};
  s32ct_init(&S, NOPROG, sizeof NOPROG);
  s32ct_uart_sink(&S, 1, cap_tx, &cap);
  pic24_poke16(&S.cpu, U1BRG, 33);                  /* BRG = 33, BRGH = 1 => 1360 Tcy/byte */
  pic24_poke16(&S.cpu, U1MODE, 0x8008);             /* UARTEN | BRGH */
  pic24_poke16(&S.cpu, U1STA, 0x0400);              /* UTXEN */
  s32ct_uart_feed(&S, 1, (const uint8_t *)"FR1", 3);
  s32ct_run(&S, 1359);
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0001u, 0u);
  CHECK_EQ(pic24_peek16(&S.cpu, IFS0) & (1u << 11), 0u);
  s32ct_run(&S, 1);
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0001u, 1u);
  CHECK_EQ(pic24_peek16(&S.cpu, IFS0) & (1u << 11), 1u << 11);
  CHECK_EQ(pic24_peek16(&S.cpu, U1RXREG), 'F');
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0001u, 0u);
  s32ct_run(&S, 1360);
  CHECK_EQ(pic24_peek16(&S.cpu, U1RXREG), 'R');
  s32ct_run(&S, 1360);
  CHECK_EQ(pic24_peek16(&S.cpu, U1RXREG), '1');
  CHECK_EQ(S.uart[0].rx_bytes, 3u);
  const char *msg = "v86.001";
  for (const char *p = msg; *p; p++) pic24_poke16(&S.cpu, U1TXREG, (uint8_t)*p);
  CHECK_EQ(cap.last_uart, 1u);
  CHECK_EQ(cap.n, 7u);
  CHECK_EQ(memcmp(cap.buf, "v86.001", 7), 0);
  CHECK_EQ(S.uart[0].tx_bytes, 7u);
  CHECK_EQ(pic24_peek16(&S.cpu, IFS0) & (1u << 12), 1u << 12);
}

/* UART1 set up at BRG = 33, UARTEN | BRGH, UTXEN, URXISEL = 00: one byte every 10 x 4 x (33+1) = 1360 Tcy; the RX
 * flag is IFS0<11>. */
static void uart1_setup(void) {
  s32ct_init(&S, NOPROG, sizeof NOPROG);
  pic24_poke16(&S.cpu, U1BRG, 33);
  pic24_poke16(&S.cpu, U1MODE, 0x8008);
  pic24_poke16(&S.cpu, U1STA, 0x0400);
}

/* Overrun (DS70000582E 7.2): with the 4-deep FIFO full, the FIFTH word fully received into UxRSR sets OERR and is
 * held there (never transferred while OERR is set); words completing while OERR is set are lost. Figure 7-3's note
 * ("an overrun error occurs at the start of the 6th character") conflicts with the 7.2 text ("If the FIFO is full
 * (four characters) and a fifth character is fully received into the UxRSR register, the OERR bit (UxSTA<1>) will
 * be set"); the text is implemented, as for the ECU's 8-level FIFO. */
TEST(overrun_trips_on_the_fifth_unread_word) {
  uart1_setup();
  s32ct_uart_feed(&S, 1, (const uint8_t *)"123456", 6);
  s32ct_run(&S, 4 * 1360);
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0003u, 0x0001u);     /* 4 held: URXDA, not yet OERR */
  s32ct_run(&S, 1360);
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0003u, 0x0003u);     /* the 5th trips OERR */
  CHECK_EQ(S.uart[0].rx_bytes, 5u);                             /* 4 in the FIFO + the one held in UxRSR */
  CHECK_EQ(S.uart[0].rx_overruns, 0u);
  pic24_poke16(&S.cpu, IFS0, 0x0000);                           /* the transfers raised U1RXIF; clear it */
  s32ct_run(&S, 1360);                                          /* the 6th completes while OERR is set */
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0002u, 0x0002u);
  CHECK_EQ(S.uart[0].rx_overruns, 1u);                          /* lost on the wire */
  CHECK_EQ(pic24_peek16(&S.cpu, IFS0) & (1u << 11), 0u);        /* interrupts inhibited while OERR (7.2) */
  for (unsigned k = 0; k < 4; k++) CHECK_EQ(pic24_peek16(&S.cpu, U1RXREG), (uint16_t)('1' + k));
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0001u, 0u);          /* the held 5th word is never delivered */
  CHECK_EQ(pic24_peek16(&S.cpu, U1RXREG), 0u);
}

/* While OERR is set, "further transfers to the receive FIFO and UART interrupts are inhibited" (DS70000582E 7.2):
 * reading the FIFO empty without clearing OERR leaves every later byte lost (and raising no flag) until software
 * clears OERR. */
TEST(reception_inhibited_while_oerr_set) {
  uart1_setup();
  s32ct_uart_feed(&S, 1, (const uint8_t *)"12345", 5);
  s32ct_run(&S, 5 * 1360);
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0002u, 0x0002u);
  for (unsigned k = 0; k < 4; k++) CHECK_EQ(pic24_peek16(&S.cpu, U1RXREG), (uint16_t)('1' + k));  /* drain, no OERR clear */
  pic24_poke16(&S.cpu, IFS0, 0x0000);
  s32ct_uart_feed(&S, 1, (const uint8_t *)"X", 1);
  s32ct_run(&S, 1360);
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0001u, 0u);          /* not transferred to the FIFO */
  CHECK_EQ(pic24_peek16(&S.cpu, IFS0) & (1u << 11), 0u);        /* no interrupt either */
  CHECK_EQ(S.uart[0].rx_overruns, 1u);
  pic24_poke16(&S.cpu, U1STA, 0x0400);                          /* software clears OERR: reception resumes */
  s32ct_uart_feed(&S, 1, (const uint8_t *)"Y", 1);
  s32ct_run(&S, 1360);
  CHECK_EQ(pic24_peek16(&S.cpu, U1RXREG), (uint16_t)'Y');
  CHECK_EQ(pic24_peek16(&S.cpu, IFS0) & (1u << 11), 1u << 11);
}

/* Clearing a set OERR resets the receive buffer and the RSR to the empty state (DS39969B Register 17-2 bit 1, a
 * 1 -> 0 transition; DS70000582E 7.2 note: "all data in the buffer is lost"), and OERR is hardware-set only:
 * a write can clear it but never set it (Register 17-2: R/C-0, HS). A read of UxRXREG after the clear returns
 * nothing stale. */
TEST(oerr_clear_resets_the_fifo_and_writes_cannot_set_it) {
  uart1_setup();
  s32ct_uart_feed(&S, 1, (const uint8_t *)"12345", 5);
  s32ct_run(&S, 5 * 1360);
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0002u, 0x0002u);
  pic24_poke16(&S.cpu, U1STA, 0x0400);                          /* a full-word write clears OERR */
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0003u, 0u);          /* OERR and URXDA both clear: the FIFO is empty */
  CHECK_EQ(pic24_peek16(&S.cpu, U1RXREG), 0u);                  /* nothing stale */
  pic24_poke16(&S.cpu, U1STA, 0x0402);                          /* a write of 1 to OERR must not set it */
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0002u, 0u);
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0400u, 0x0400u);     /* UTXEN and the other R/W bits unaffected */
  s32ct_uart_feed(&S, 1, (const uint8_t *)"Z", 1);
  s32ct_run(&S, 1360);
  CHECK_EQ(pic24_peek16(&S.cpu, U1RXREG), (uint16_t)'Z');       /* the next byte received is the new one */
}

/* The RX flag is set on an RSR-to-buffer transfer as URXISEL selects (DS39969B Register 17-2 bits 7:6: 0x = any
 * transfer; 10 = a transfer making the buffer 3/4 full, i.e. 3 characters; 11 = making it full, i.e. 4). An event
 * per transfer, not a level: this chip's register text has no "asserted while" form (contrast the ECU's PIC32MX
 * DS60001156K Register 20-2). FRM 7.3's "three or four characters" for mode 10 conflicts with the datasheet's
 * "has 3 data characters"; the datasheet, which its 17.0 note says supersedes the FRM, is implemented. */
TEST(rx_interrupt_fires_per_urxisel_on_transfer) {
  uart1_setup();                                                /* URXISEL = 00 */
  s32ct_uart_feed(&S, 1, (const uint8_t *)"ab", 2);
  s32ct_run(&S, 1360);
  CHECK_EQ(pic24_peek16(&S.cpu, IFS0) & (1u << 11), 1u << 11);  /* every transfer sets it */
  pic24_poke16(&S.cpu, IFS0, 0x0000);
  s32ct_run(&S, 1360);
  CHECK_EQ(pic24_peek16(&S.cpu, IFS0) & (1u << 11), 1u << 11);
  CHECK_EQ(pic24_peek16(&S.cpu, U1RXREG), (uint16_t)'a');       /* drain before the mode change */
  CHECK_EQ(pic24_peek16(&S.cpu, U1RXREG), (uint16_t)'b');
  pic24_poke16(&S.cpu, U1STA, 0x0480);                          /* URXISEL = 10: the transfer that makes 3 */
  s32ct_uart_feed(&S, 1, (const uint8_t *)"cdef", 4);
  for (unsigned k = 0; k < 4; k++) {
    pic24_poke16(&S.cpu, IFS0, 0x0000);
    s32ct_run(&S, 1360);
    CHECK_EQ(pic24_peek16(&S.cpu, IFS0) & (1u << 11), k == 2 ? (unsigned)(1u << 11) : 0u);
  }
  pic24_poke16(&S.cpu, U1STA, 0x04C0);                          /* URXISEL = 11: the transfer that makes 4 */
  for (unsigned k = 0; k < 4; k++) (void)pic24_peek16(&S.cpu, U1RXREG);
  s32ct_uart_feed(&S, 1, (const uint8_t *)"ghij", 4);
  for (unsigned k = 0; k < 4; k++) {
    pic24_poke16(&S.cpu, IFS0, 0x0000);
    s32ct_run(&S, 1360);
    CHECK_EQ(pic24_peek16(&S.cpu, IFS0) & (1u << 11), k == 3 ? (unsigned)(1u << 11) : 0u);
  }
}

/* Disabling the UART (UARTEN 1 -> 0) resets the buffers to empty and the status flags as at POR -- UTXBRK, UTXEN,
 * UTXBF, PERR, FERR, OERR and URXDA cleared; TRMT and RIDLE set (DS70000582E 4.2) -- and "will abort all pending
 * transmissions and receptions" (4.2): after the UART is set up again no byte from before the disable is readable. */
TEST(uart_disable_resets_buffers_and_flags) {
  uart1_setup();
  s32ct_uart_feed(&S, 1, (const uint8_t *)"12345", 5);
  s32ct_run(&S, 5 * 1360);
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0002u, 0x0002u);     /* OERR set, FIFO full */
  s32ct_uart_feed(&S, 1, (const uint8_t *)"7", 1);             /* on the wire (completes in 1360) ... */
  s32ct_run(&S, 1359);                                         /* ... but not yet complete */
  pic24_poke16(&S.cpu, U1MODE, 0x0000);                         /* UARTEN = 0: aborts the reception in flight */
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0001u, 0u);          /* URXDA clear: buffers empty */
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0002u, 0u);          /* OERR clear */
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0400u, 0u);          /* UTXEN clear */
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0110u, 0x0110u);     /* TRMT and RIDLE set */
  s32ct_run(&S, 2);                                             /* the aborted byte never completes */
  CHECK_EQ(S.uart[0].rx_bytes, 5u);
  CHECK_EQ(S.uart[0].rx_overruns, 0u);
  pic24_poke16(&S.cpu, U1MODE, 0x8008);                         /* set up again */
  pic24_poke16(&S.cpu, U1STA, 0x0400);
  s32ct_uart_feed(&S, 1, (const uint8_t *)"Q", 1);
  s32ct_run(&S, 1360);
  CHECK_EQ(pic24_peek16(&S.cpu, U1RXREG), (uint16_t)'Q');       /* no stale bytes from before the disable */
  CHECK_EQ(S.uart[0].rx_bytes, 6u);
}

/* FRM DS70000582E 7.2: "An interrupt is generated if any of these (OERR, FERR and PERR) errors occur" -- UxERIF is
 * set when OERR occurs, i.e. at the transition; while OERR stays set UART interrupts are inhibited, so the words
 * lost then do not re-raise it. UART1's error flag is IFS4<1> (DS39969B interrupt table). With UxERIE clear the flag
 * vectors nowhere, but it is still hardware-set and software-readable. */
TEST(oerr_raises_the_error_flag_once) {
  uart1_setup();
  s32ct_uart_feed(&S, 1, (const uint8_t *)"123456", 6);
  s32ct_run(&S, 5 * 1360);
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0002u, 0x0002u);     /* OERR set by the fifth word */
  CHECK_EQ(pic24_peek16(&S.cpu, 0x008C) & 0x0002u, 0x0002u);    /* IFS4<1> (U1ERIF) raised at the transition */
  pic24_poke16(&S.cpu, 0x008C, 0x0000);                         /* software clears it */
  s32ct_run(&S, 1360);                                          /* the sixth word completes while OERR is set */
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0002u, 0x0002u);
  CHECK_EQ(pic24_peek16(&S.cpu, 0x008C) & 0x0002u, 0u);         /* inhibited while OERR is set: not raised again */
}

/* An overrun episode end to end: four words held, the fifth trips OERR, the sixth is lost; clearing OERR lets the
 * queued seventh land; a word completing while the UART is disabled is dropped uncounted (FRM DS70000582E 4.2, 7.2). */
TEST(overrun_episode_and_disable) {
  s32ct_init(&S, NOPROG, sizeof NOPROG);
  pic24_poke16(&S.cpu, U1BRG, 33);
  pic24_poke16(&S.cpu, U1MODE, 0x8008);                         /* UARTEN | BRGH, 1360 Tcy/byte */
  pic24_poke16(&S.cpu, U1STA, 0x0400);
  s32ct_uart_feed(&S, 1, (const uint8_t *)"1234567", 7);
  s32ct_run(&S, 6 * 1360);              /* 4 held; the 5th trips OERR at 6800; the 6th is lost at 8160 */
  CHECK_EQ(S.uart[0].rx_bytes, 5u);
  CHECK_EQ(S.uart[0].rx_overruns, 1u);
  CHECK_EQ(pic24_peek16(&S.cpu, U1STA) & 0x0002u, 0x0002u);
  pic24_poke16(&S.cpu, U1STA, 0x0400);                          /* software clears OERR at 8160 */
  s32ct_run(&S, 1360);                                          /* the queued 7th word lands at 9520 and is read */
  CHECK_EQ(pic24_peek16(&S.cpu, U1RXREG), (uint16_t)'7');
  pic24_poke16(&S.cpu, U1MODE, 0x0000);                         /* UARTEN 1 -> 0 */
  s32ct_uart_feed(&S, 1, (const uint8_t *)"Q", 1);              /* completes while disabled: dropped uncounted */
  s32ct_run(&S, 5440);
  CHECK_EQ(S.uart[0].rx_bytes, 6u);                             /* the five, then the 7th; Q lands nowhere */
  CHECK_EQ(S.uart[0].rx_overruns, 1u);
}

int main(void) {
  RUN(por_values_come_from_the_table);
  RUN(chip_select_edges_follow_the_latch);
  RUN(byte_exchange_timing_and_flags);
  RUN(deselected_device_sees_nothing_and_miso_idles_high);
  RUN(overflow_sets_spirov_and_disable_drops_state);
  RUN(timer1_period_match_raises_t1if_on_time);
  RUN(timer_matches_come_pr_plus_1_ticks_apart);
  RUN(timer_count_above_pr_wraps_at_ffff);
  RUN(timer_above_pr_multi_cycle_advance_keeps_every_match);
  RUN(timer_prescale_and_count_read);
  RUN(adc_scan_fills_smpi_plus_1_buffers);
  RUN(nvm_flash_erase_and_program);
  RUN(idle_loop_skipped_only_when_nothing_changes);
  RUN(idle_loop_leaves_when_an_event_sets_its_flag);
  RUN(reset_causes_set_rcon_and_reset_peripherals);
  RUN(uart1_rx_pacing_and_tx_sink);
  RUN(overrun_trips_on_the_fifth_unread_word);
  RUN(reception_inhibited_while_oerr_set);
  RUN(oerr_clear_resets_the_fifo_and_writes_cannot_set_it);
  RUN(rx_interrupt_fires_per_urxisel_on_transfer);
  RUN(uart_disable_resets_buffers_and_flags);
  RUN(oerr_raises_the_error_flag_once);
  RUN(overrun_episode_and_disable);
  MT_REPORT();
}
