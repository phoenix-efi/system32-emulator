/* Unit tests for the ECU SoC model (src/ecu.c). The CPU runs a two-instruction idle loop from boot flash (interrupts
 * stay off after reset: Status.ERL/BEV), so only the peripheral models are under test, driven through ecu_bus_*. */
#include "minitest.h"
#include "ecu.h"
#include <unicorn/unicorn.h>
#include <stdlib.h>
#include <string.h>


static ecu_t E;
static uint8_t BFL[0x3000], PFL[0x1000];

#define IFS0 0xBF881030u
#define IEC0 0xBF881060u
#define IPC1 0xBF8810A0u
#define INTCON 0xBF881000u
#define INTSTAT 0xBF881010u

static void setup_pbdiv(unsigned fpbdiv) {
  memset(BFL, 0, sizeof BFL);
  uint32_t loop[2] = {0x1000FFFFu, 0x00000000u};               /* b . ; nop */
  memcpy(BFL, loop, sizeof loop);
  uint32_t devcfg1 = 0xFFFFCFFFu | (fpbdiv << 12);              /* DEVCFG1 at 0x1FC02FF8: FPBDIV<13:12> */
  memcpy(BFL + 0x2FF8, &devcfg1, 4);
  memset(&E, 0, sizeof E);
  if (ecu_init(&E, BFL, sizeof BFL, PFL, sizeof PFL)) abort();
}
static void setup(void) { setup_pbdiv(1); }                     /* PBCLK = SYSCLK / 2, as the stock images select */

TEST(pbclk_follows_devcfg1_and_osccon) {
  setup_pbdiv(2);                                                /* FPBDIV = /4 */
  CHECK(ecu_pbclk_hz(&E) == ECU_SYSCLK_HZ / 4);
  CHECK_EQ((ecu_bus_read(&E, 0xBF80F000u, 4) >> 19) & 3u, 2);   /* OSCCON.PBDIV loaded from the configuration word */
  ecu_bus_write(&E, 0xBF80F000u, 4, (ecu_bus_read(&E, 0xBF80F000u, 4) & ~(3u << 19)) | (1u << 19));  /* software /2 */
  CHECK(ecu_pbclk_hz(&E) == ECU_SYSCLK_HZ / 2);
  ecu_free(&E);
}

TEST(portals_clr_set_inv) {
  setup();
  ecu_bus_write(&E, 0xBF8860E0u, 4, 0x00F0);                    /* LATD */
  ecu_bus_write(&E, 0xBF8860E8u, 4, 0x0001);                    /* LATDSET */
  ecu_bus_write(&E, 0xBF8860E4u, 4, 0x0010);                    /* LATDCLR */
  ecu_bus_write(&E, 0xBF8860ECu, 4, 0x0100);                    /* LATDINV */
  CHECK_EQ(ecu_bus_read(&E, 0xBF8860E0u, 4), 0x01E1);
  ecu_free(&E);
}

TEST(port_reads_latch_for_outputs_and_pins_for_inputs) {
  setup();
  ecu_bus_write(&E, 0xBF886040u, 4, 0x00FF);                    /* TRISB: low byte inputs */
  ecu_bus_write(&E, 0xBF886050u, 4, 0x0F00);                    /* PORTB write -> LATB */
  CHECK_EQ(ecu_bus_read(&E, 0xBF886060u, 4), 0x0F00);           /* LATB */
  ecu_set_pin(&E, 1, 2, false);                                  /* RB2 grounded */
  CHECK_EQ(ecu_bus_read(&E, 0xBF886050u, 4), 0x0F00);           /* AD1PCFG reset 0: port B inputs are analog, read 0 */
  ecu_bus_write(&E, 0xBF809060u, 4, 0xFFFF);                    /* AD1PCFG: all digital */
  CHECK_EQ(ecu_bus_read(&E, 0xBF886050u, 4), 0x0F00 | (0x00FF & ~0x4u));
  ecu_free(&E);
}

TEST(timer1_period_sets_t1if_on_time) {
  setup();
  ecu_bus_write(&E, 0xBF800620u, 4, 999);                       /* PR1 */
  ecu_bus_write(&E, 0xBF800600u, 4, 0x8000);                    /* T1CON ON, 1:1 -> 2 SYSCLK per tick */
  ecu_run(&E, 1990);
  CHECK_EQ(ecu_bus_read(&E, IFS0, 4) & 0x10, 0);
  ecu_run(&E, 40);
  CHECK_EQ(ecu_bus_read(&E, IFS0, 4) & 0x10, 0x10);              /* T1IF (IRQ 4) after 1000 ticks */
  CHECK_EQ(ecu_timer_irqs(&E, 1), 1);
  ecu_free(&E);
}

TEST(eic_presents_highest_priority_vector) {
  setup();
  ecu_bus_write(&E, 0xBF881090u, 4, (3u << 2) << 0x10);         /* IPC0: vector 2 (CS1) priority 3 */
  ecu_bus_write(&E, 0xBF881090u + 0x10u, 4, 5u << 2);           /* IPC1: vector 4 (T1) priority 5 */
  ecu_bus_write(&E, IEC0, 4, (1u << 2) | (1u << 4));
  ecu_bus_write(&E, IFS0, 4, (1u << 2) | (1u << 4));
  CHECK_EQ(ecu_bus_read(&E, INTSTAT, 4), (5u << 8) | 4u);        /* SRIPL 5, VEC 4 */
  CHECK_EQ((ecu_reg(&E, UC_MIPS_REG_CP0_CAUSE) >> 10) & 0x3F, 5);
  ecu_bus_write(&E, IFS0 + 4, 4, 1u << 4);                       /* IFS0CLR T1 -> CS1 at priority 3 */
  CHECK_EQ(ecu_bus_read(&E, INTSTAT, 4), (3u << 8) | 2u);
  ecu_free(&E);
}

#define U1STA 0xBF806010u
#define U1STACLR 0xBF806014u
#define U1RXREG 0xBF806030u
#define U1_BYTE 41600ull                                        /* SYSCLK cycles per byte: PBDIV 2 x 10 x 16 x 130 */
static void uart1_on(void) {                                    /* URXISEL = 00 */
  ecu_bus_write(&E, 0xBF806040u, 4, 129);                       /* U1BRG: 40 MHz/16/130 = 19,231 Bd */
  ecu_bus_write(&E, U1STA, 4, 0x1400);                          /* U1STA URXEN | UTXEN */
  ecu_bus_write(&E, 0xBF806000u, 4, 0x8000);                    /* U1MODE ON */
}

/* The receive path on silicon (FRM DS61107G 21.7): with the 8-deep FIFO full the next byte may begin shifting into
 * UxRSR (21.7.1); when it is fully received OERR is set and that word is lost ("If the FIFO is full and a new
 * character is fully received into the UxRSR register, the overrun error bit, OERR (UxSTA<1>), is set. The word in
 * UxRSR register is not kept", 21.7.2). */
TEST(uart_rx_line_rate_and_overrun) {
  setup();
  uart1_on();
  uint8_t b[11] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
  ecu_uart_feed(&E, 1, b, 11);
  ecu_run(&E, U1_BYTE - 10);
  CHECK_EQ(ecu_bus_read(&E, U1STA, 4) & 1, 0);                  /* not yet */
  ecu_run(&E, 20);
  CHECK_EQ(ecu_bus_read(&E, U1STA, 4) & 1, 1);                  /* URXDA */
  CHECK_EQ(ecu_bus_read(&E, U1RXREG, 4), 1);
  ecu_run(&E, U1_BYTE * 8);                                       /* bytes 2..9 fill the 8-deep FIFO */
  CHECK_EQ(E.uart[0].fn, 8u);
  CHECK_EQ(ecu_bus_read(&E, U1STA, 4) & 2, 0);                  /* no overrun while the FIFO alone is full */
  ecu_run(&E, U1_BYTE);                                           /* byte 10, the ninth unread, completes with the FIFO full */
  CHECK_EQ(ecu_bus_read(&E, U1STA, 4) & 2, 2);                  /* OERR */
  CHECK_EQ(E.uart[0].rx_overruns, 1);
  CHECK_EQ(E.uart[0].fn, 8u);
  CHECK_EQ(ecu_bus_read(&E, IFS0, 4) & 0x4000000u, 0x4000000u); /* U1EIF: the error interrupt fired (DS61107G 21.14) */
  ecu_free(&E);
}

/* A read that frees a FIFO slot while the ninth byte is still shifting in saves it: the byte completes into a FIFO
 * with room, and with one read per byte the whole burst is received. */
TEST(uart_rx_ninth_byte_saved_by_a_read) {
  setup();
  uart1_on();
  uint8_t b[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  ecu_uart_feed(&E, 1, b, 10);
  ecu_run(&E, 8 * U1_BYTE + 10);                                  /* 8 in the FIFO, byte 9 shifting in */
  CHECK_EQ(ecu_bus_read(&E, U1STA, 4) & 2, 0);
  CHECK_EQ(ecu_bus_read(&E, U1RXREG, 4), 1);                      /* frees a slot before byte 9 completes */
  ecu_run(&E, U1_BYTE);                                           /* byte 9 completes into the FIFO */
  CHECK_EQ(ecu_bus_read(&E, U1STA, 4) & 2, 0);
  CHECK_EQ(ecu_bus_read(&E, U1RXREG, 4), 2);                      /* frees a slot before byte 10 completes */
  ecu_run(&E, U1_BYTE);
  CHECK_EQ(ecu_bus_read(&E, U1STA, 4) & 2, 0);                    /* still no overrun */
  for (uint8_t want = 3; want <= 10; want++) CHECK_EQ(ecu_bus_read(&E, U1RXREG, 4), want);
  CHECK_EQ(ecu_bus_read(&E, U1RXREG, 4), 0);                      /* empty */
  CHECK_EQ(E.uart[0].rx_overruns, 0);
  ecu_free(&E);
}

/* While OERR is set the receiver stops ("the UxRSR stops accepting new data and transfers to the receive FIFO are
 * inhibited as long as the OERR bit is set", DS60001107H 21.7.2 -- the chip has no RUNOVF bit, DS60001156K Register
 * 20-1); clearing OERR in software resets the FIFO and UxRSR to empty ("Clearing a previously set OERR bit resets
 * the receiver buffer and RSR to an empty state", DS60001156K Register 20-2 bit 1). Both clear styles: the CLR
 * portal and a plain U1STA write. */
static void oerr_clear_resets(bool portal) {
  setup();
  uart1_on();
  uint8_t b[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  ecu_uart_feed(&E, 1, b, 12);
  ecu_run(&E, 12 * U1_BYTE + 10);                                 /* 8 in the FIFO, byte 9 trips OERR, 10-12 lost on the wire */
  CHECK_EQ(ecu_bus_read(&E, U1STA, 4) & 2, 2);
  CHECK_EQ(E.uart[0].rx_overruns, 4);
  CHECK_EQ(ecu_bus_read(&E, U1RXREG, 4), 1);                      /* the FIFO contents are still readable */
  CHECK_EQ(ecu_bus_read(&E, U1RXREG, 4), 2);
  uint8_t c[3] = {20, 21, 22};
  ecu_uart_feed(&E, 1, c, 3);
  ecu_run(&E, 3 * U1_BYTE + 10);                                  /* arrive while OERR is set: lost, FIFO unchanged */
  CHECK_EQ(E.uart[0].fn, 6u);
  CHECK_EQ(E.uart[0].rx_overruns, 7);
  if (portal) ecu_bus_write(&E, U1STACLR, 4, 2);                  /* software clears OERR ... */
  else ecu_bus_write(&E, U1STA, 4, 0x1400);                       /* ... with the portal or a plain write */
  CHECK_EQ(ecu_bus_read(&E, U1STA, 4) & 3, 0);                    /* OERR and URXDA both clear: the FIFO is empty */
  CHECK_EQ(ecu_bus_read(&E, U1RXREG, 4), 0);
  ecu_uart_feed(&E, 1, c, 3);
  ecu_run(&E, 3 * U1_BYTE + 10);                                  /* reception resumes into the emptied FIFO */
  CHECK_EQ(ecu_bus_read(&E, U1STA, 4) & 1, 1);
  CHECK_EQ(ecu_bus_read(&E, U1RXREG, 4), 20);                     /* no stale bytes from before the clear */
  CHECK_EQ(ecu_bus_read(&E, U1RXREG, 4), 21);
  CHECK_EQ(ecu_bus_read(&E, U1RXREG, 4), 22);
  ecu_free(&E);
}
TEST(uart_rx_oerr_clear_via_portal_resets_fifo) { oerr_clear_resets(true); }
TEST(uart_rx_oerr_clear_via_sta_write_resets_fifo) { oerr_clear_resets(false); }

/* URXISEL = 00: the RX interrupt flag is asserted while the buffer is not empty, so clearing IFS0 with data still in
 * the FIFO raises it again (DS60001156K Register 20-2 bits 7:6; DS61107G 21.7.3). */
TEST(uart_rx_interrupt_reasserts_while_data_remains) {
  setup();
  uart1_on();
  uint8_t b[3] = {1, 2, 3};
  ecu_uart_feed(&E, 1, b, 3);
  ecu_run(&E, 3 * U1_BYTE + 10);
  CHECK_EQ(ecu_bus_read(&E, IFS0, 4) & 0x8000000u, 0x8000000u);   /* U1RXIF (IRQ 27) */
  for (uint8_t want = 1; want <= 3; want++) {
    CHECK_EQ(ecu_bus_read(&E, U1RXREG, 4), want);                 /* one byte at a time */
    ecu_bus_write(&E, IFS0 + 4, 4, 0x8000000u);                   /* IFS0CLR ... */
    if (want < 3) CHECK_EQ(ecu_bus_read(&E, IFS0, 4) & 0x8000000u, 0x8000000u);   /* ... re-asserts while bytes remain */
    else CHECK_EQ(ecu_bus_read(&E, IFS0, 4) & 0x8000000u, 0u);    /* and stays clear once the FIFO is empty */
  }
  ecu_free(&E);
}

/* The level thresholds of the other URXISEL modes on the 8-level FIFO (DS60001156K Register 20-2 bits 7:6):
 * 01 asserts at 1/2 full (4 characters). */
TEST(uart_rx_interrupt_threshold_urxisel01) {
  setup();
  ecu_bus_write(&E, 0xBF806040u, 4, 129);
  ecu_bus_write(&E, U1STA, 4, 0x1440);                            /* URXEN | UTXEN, URXISEL = 01 */
  ecu_bus_write(&E, 0xBF806000u, 4, 0x8000);
  uint8_t b[4] = {1, 2, 3, 4};
  ecu_uart_feed(&E, 1, b, 4);
  ecu_run(&E, 3 * U1_BYTE + 10);
  CHECK_EQ(ecu_bus_read(&E, IFS0, 4) & 0x8000000u, 0u);           /* 3 of 8: below the threshold */
  ecu_run(&E, U1_BYTE);
  CHECK_EQ(ecu_bus_read(&E, IFS0, 4) & 0x8000000u, 0x8000000u);   /* 4 of 8: asserted */
  CHECK_EQ(ecu_bus_read(&E, U1RXREG, 4), 1);
  ecu_bus_write(&E, IFS0 + 4, 4, 0x8000000u);
  CHECK_EQ(ecu_bus_read(&E, IFS0, 4) & 0x8000000u, 0u);           /* back to 3: a clear now holds */
  ecu_free(&E);
}

TEST(int1_falling_edge_sets_flag) {
  setup();
  ecu_bus_write(&E, INTCON, 4, 0);                                /* INT1EP = 0: falling */
  ecu_schedule_pin(&E, E.cyc + 100, 4, 8, false);                 /* RE8 low */
  ecu_schedule_pin(&E, E.cyc + 200, 4, 8, true);
  ecu_run(&E, 300);
  CHECK_EQ(ecu_bus_read(&E, IFS0, 4) & 0x80, 0x80);              /* INT1IF (IRQ 7) */
  CHECK_EQ(E.int_edges[1], 1);                                    /* only the falling edge counted */
  ecu_free(&E);
}

TEST(timer23_pair_interrupts_on_timer3_flag) {
  /* PIC32MX FRM section 14 (Timers): with T32 set, Timer2 and Timer3 form one 32-bit timer, and the pair interrupts
   * on the ODD timer's flag (T3IF); T2IF stays clear. */
  setup();
  ecu_bus_write(&E, 0xBF800820u, 4, 999);                         /* PR2: the pair's 32-bit period */
  ecu_bus_write(&E, 0xBF800800u, 4, 0x8008);                      /* T2CON: ON + T32 */
  ecu_run(&E, 1990);                                              /* PBCLK = SYSCLK/2: 2 cycles per tick */
  CHECK_EQ(ecu_bus_read(&E, IFS0, 4) & 0x1000, 0);                /* T3IF (IRQ 12) not yet */
  ecu_run(&E, 40);
  CHECK_EQ(ecu_bus_read(&E, IFS0, 4) & 0x1000, 0x1000);           /* T3IF set at the 1000-tick period */
  CHECK_EQ(ecu_bus_read(&E, IFS0, 4) & 0x100, 0);                 /* T2IF (IRQ 8) never */
  ecu_free(&E);
}

TEST(timer_above_pr_wraps_at_ffff_then_matches) {
  /* PIC32MX FRM section 14 (Timers): a 16-bit timer counts through 0xFFFF before wrapping, so a period register
   * lowered below the current count is reached after the wrap, and the match fires there. */
  setup();
  ecu_bus_write(&E, 0xBF800620u, 4, 0xFFFF);                      /* PR1 */
  ecu_bus_write(&E, 0xBF800600u, 4, 0x8000);                      /* T1CON ON, 1:1 (2 SYSCLK per tick here) */
  ecu_run(&E, 2 * 5000);
  uint32_t c0 = ecu_bus_read(&E, 0xBF800610u, 4);                 /* TMR1 */
  CHECK(c0 > 999 && c0 < 0xF000);
  ecu_bus_write(&E, 0xBF800620u, 4, 999);                         /* PR1 below the count */
  ecu_run(&E, 2 * 400);
  CHECK_EQ(ecu_bus_read(&E, 0xBF800610u, 4), c0 + 400);           /* keeps climbing toward 0xFFFF: no early wrap at PR */
  ecu_run(&E, 2 * (0x10000u - (c0 + 400u) + 999u) + 80);
  CHECK(ecu_timer_irqs(&E, 1) >= 1);                              /* the match after the wrap fires */
  CHECK_EQ(ecu_bus_read(&E, IFS0, 4) & 0x10, 0x10);               /* T1IF (IRQ 4) */
  ecu_free(&E);
}

TEST(timer_above_pr_multi_cycle_advance_keeps_every_match) {
  /* PIC32MX FRM section 14 (Timers): the same lowered-PR corner over one advance spanning several full counter
   * cycles: after the 0xFFFF wrap the count is in the <= PR regime and every PR+1 period matches. */
  setup();
  ecu_bus_write(&E, 0xBF800620u, 4, 0xFFFF);                      /* PR1 */
  ecu_bus_write(&E, 0xBF800600u, 4, 0x8000);                      /* T1CON ON, 1:1 (2 SYSCLK per tick) */
  ecu_run(&E, 2 * 5000);                                          /* count = 5000 */
  ecu_bus_write(&E, 0xBF800620u, 4, 999);                         /* PR1 below the count */
  ecu_run(&E, 2 * 3 * 65536);                                     /* one advance over three full counter cycles */
  /* 60536 ticks to the wrap, then 136072 ticks in the <= PR regime: 136 matches, count 72 */
  CHECK_EQ(ecu_timer_irqs(&E, 1), 136);
  CHECK_EQ(ecu_bus_read(&E, 0xBF800610u, 4), 72);                 /* TMR1 */
  ecu_free(&E);
}

static uint64_t ON_C, OFF_C; static unsigned PULSES;
static void sink(void *ctx, ecu_t *e, unsigned oc, uint64_t on, uint64_t off) { (void)ctx; (void)e; (void)oc; ON_C = on; OFF_C = off; PULSES++; }

TEST(oc_single_pulse_with_late_refinement) {
  setup();
  E.oc_sink = sink; PULSES = 0;
  ecu_bus_write(&E, 0xBF800820u, 4, 0xFFFFFFFFu);                /* PR2 */
  ecu_bus_write(&E, 0xBF800800u, 4, 0x8038);                     /* T2CON ON, 1:8, T32: 5 MHz */
  ecu_bus_write(&E, 0xBF803010u, 4, 1000);                       /* OC1R  = 200 us */
  ecu_bus_write(&E, 0xBF803020u, 4, 3000);                       /* OC1RS = 600 us */
  ecu_bus_write(&E, 0xBF803000u, 4, 0x8024);                     /* ON, OC32, dual compare single pulse */
  ecu_run(&E, 24000);                                             /* 300 us: the rising edge (200 us) is past */
  ecu_bus_write(&E, 0xBF803020u, 4, 4000);                       /* late refinement: fire at 800 us */
  ecu_bus_write(&E, 0xBF803000u, 4, 0x8024);                     /* rewriting ON with the same mode: no re-arm */
  ecu_run(&E, 80000);
  CHECK_EQ(PULSES, 1);
  CHECK(ON_C >= 15990 && ON_C <= 16010);                          /* 1000 ticks x 16 cycles */
  CHECK(OFF_C >= 63990 && OFF_C <= 64010);                        /* 4000 ticks x 16 cycles */
  ecu_free(&E);
}

TEST(oc_pulse_does_not_repeat_after_the_timer_wrap) {
  /* PIC32MX FRM output-compare chapter (DS61111): in dual-compare single-pulse mode (OCM=100) the channel produces
   * one pulse per arming; a compare target already passed does not re-fire when the 32-bit timer wraps. */
  setup();
  E.oc_sink = sink; PULSES = 0;
  ecu_bus_write(&E, 0xBF800820u, 4, 0xFFFFFFFFu);                /* PR2: free-running 32-bit */
  ecu_bus_write(&E, 0xBF800800u, 4, 0x8038);                     /* T2CON ON, 1:8, T32: 5 MHz */
  ecu_bus_write(&E, 0xBF803010u, 4, 1000);                       /* OC1R */
  ecu_bus_write(&E, 0xBF803020u, 4, 3000);                       /* OC1RS */
  ecu_bus_write(&E, 0xBF803000u, 4, 0x8024);                     /* ON, OC32, dual compare single pulse */
  ecu_run(&E, 80000);                                            /* the pulse fires (1000..3000 ticks x 16 cycles) */
  CHECK_EQ(PULSES, 1);
  ecu_bus_write(&E, 0xBF800810u, 4, 0xFFFFF000u);                /* TMR2 near the 32-bit wrap */
  ecu_run(&E, 120000);                                           /* past the wrap and both compare values again */
  CHECK_EQ(PULSES, 1);                                           /* no repeat */
  ecu_free(&E);
}

/* An edge the block hook checks late (one long block straddles it) must still fire. 47-instruction loop body: the hook lands 27 cycles after the 200-us
 * rise and 34 cycles after the 600-us fall (> 1 TMR2 tick = 16 cycles). The 2-instruction loop above never overshoots. */
TEST(oc_edge_survives_block_overshoot) {
  memset(BFL, 0, sizeof BFL);
  uint32_t w[47]; for (int i = 0; i < 47; i++) w[i] = 0;          /* nops */
  w[45] = 0x1000FFD2u;                                            /* b -45 instr (back to 0) ; w[46] = delay-slot nop */
  memcpy(BFL, w, sizeof w);
  uint32_t devcfg1 = 0xFFFFDFFFu;                                 /* DEVCFG1.FPBDIV = /2 */
  memcpy(BFL + 0x2FF8, &devcfg1, 4);
  memset(&E, 0, sizeof E);
  if (ecu_init(&E, BFL, sizeof BFL, PFL, sizeof PFL)) abort();
  E.oc_sink = sink; PULSES = 0;
  ecu_bus_write(&E, 0xBF800820u, 4, 0xFFFFFFFFu);
  ecu_bus_write(&E, 0xBF800800u, 4, 0x8038);
  ecu_bus_write(&E, 0xBF803010u, 4, 1000);
  ecu_bus_write(&E, 0xBF803020u, 4, 3000);
  ecu_bus_write(&E, 0xBF803000u, 4, 0x8024);
  ecu_run(&E, 80000);
  CHECK_EQ(PULSES, 1);
  CHECK(ON_C >= 15990 && ON_C <= 16010);
  CHECK(OFF_C >= 47990 && OFF_C <= 48010);
  ecu_free(&E);
}

typedef struct { uint8_t last; unsigned n; } fake_dev;
static uint8_t fx(void *ctx, uint8_t mosi) { fake_dev *d = ctx; d->last = mosi; d->n++; return (uint8_t)~mosi; }

TEST(spi_exchange_only_with_selected_device) {
  setup();
  static fake_dev D;
  memset(&D, 0, sizeof D);
  ecu_spi_attach(&E, 2, 6, 9, (s32_spi_dev){"dev", &D, fx, NULL});
  ecu_bus_write(&E, 0xBF805A00u, 4, 0x8120);                     /* SPI2CON ON, master */
  ecu_bus_write(&E, 0xBF8861A0u, 4, 1u << 9);                    /* LATG9 high: deselected */
  ecu_bus_write(&E, 0xBF805A20u, 4, 0x5A);
  CHECK_EQ(D.n, 0);
  CHECK_EQ(ecu_bus_read(&E, 0xBF805A20u, 4), 0xFF);              /* nothing drives MISO */
  ecu_bus_write(&E, 0xBF8861A4u, 4, 1u << 9);                    /* LATGCLR: select */
  ecu_bus_write(&E, 0xBF805A20u, 4, 0x5A);
  CHECK_EQ(D.last, 0x5A);
  CHECK_EQ(ecu_bus_read(&E, 0xBF805A10u, 4) & 1, 1);             /* SPIRBF */
  CHECK_EQ(ecu_bus_read(&E, 0xBF805A20u, 4), 0xA5);
  ecu_free(&E);
}

TEST(adc_scan_fills_buffers_in_pin_order) {
  setup();
  for (unsigned ch = 5; ch <= 14; ch++) ecu_set_an(&E, ch, (uint16_t)(ch * 10));
  ecu_bus_write(&E, 0xBF809050u, 4, 0x7FE0);                     /* AD1CSSL: AN5..AN14 (as v86) */
  ecu_bus_write(&E, 0xBF809020u, 4, 0x120A);                     /* AD1CON3 (as v86) */
  ecu_bus_write(&E, 0xBF809010u, 4, 0x6424);                     /* AD1CON2: CSCNA, SMPI 9 (as v86) */
  ecu_bus_write(&E, 0xBF809000u, 4, 0x80E4);                     /* ON, auto-convert, ASAM */
  ecu_run(&E, 200000);
  CHECK_EQ(ecu_bus_read(&E, 0xBF881040u, 4) & 0x2, 0x2);         /* IFS1 bit 1 = IRQ 33 ADC */
  CHECK_EQ(ecu_bus_read(&E, 0xBF809070u, 4), 50);                /* ADC1BUF0 = AN5 */
  CHECK_EQ(ecu_bus_read(&E, 0xBF809070u + 0x90u, 4), 140);       /* ADC1BUF9 = AN14 */
  ecu_free(&E);
}

TEST(adc_conversion_time_follows_tad) {
  /* DS61104 section 17 (ADC): a conversion sequence takes SMPI+1 conversions of (SAMC + 12) TAD each, with
   * TAD = 2*(ADCS+1) PBCLK cycles (PBCLK = SYSCLK / 2 in this setup). */
  setup();
  ecu_set_an(&E, 5, 700);
  ecu_bus_write(&E, 0xBF809050u, 4, 0x0020);                     /* AD1CSSL: AN5 only */
  ecu_bus_write(&E, 0xBF809020u, 4, 0x010A);                     /* AD1CON3: SAMC 1 (SAMC 0 is not legal), ADCS 10 -> TAD = 22 PBCLK = 44 SYSCLK */
  ecu_bus_write(&E, 0xBF809010u, 4, 0x0400);                     /* AD1CON2: CSCNA, SMPI 0 -> 1 conversion */
  ecu_bus_write(&E, 0xBF809000u, 4, 0x80E4);                     /* ON, auto-convert, ASAM; one sequence = 13 TAD = 572 cycles */
  ecu_run(&E, 565);
  CHECK_EQ(ecu_bus_read(&E, 0xBF881040u, 4) & 0x2, 0);           /* ADC1IF (IRQ 33) not yet */
  ecu_run(&E, 20);                                               /* 585 >= 572 */
  CHECK_EQ(ecu_bus_read(&E, 0xBF881040u, 4) & 0x2, 0x2);
  CHECK_EQ(ecu_bus_read(&E, 0xBF809070u, 4), 700);               /* ADC1BUF0 = AN5 */
  ecu_free(&E);
}

TEST(nvm_flash_erase_and_program) {
  setup();
  /* 1. Attempt page erase at 0x1D007000 without NVMKEY unlock -> must be ignored */
  ecu_bus_write(&E, 0xBF80F420u, 4, 0x1D007000u);                 /* NVMADDR */
  ecu_bus_write(&E, 0xBF80F400u, 4, 0x4004u);                     /* NVMCON = WREN | Page Erase */
  ecu_bus_write(&E, 0xBF80F408u, 4, 0x8000u);                     /* NVMCONSET = WR (no key!) */
  CHECK_EQ(E.nvm_erases, 0u);
  /* 2. Valid NVMKEY sequence (0xAA996655 -> 0x556699AA) + NVMCONSET = WR erases page to 0xFFFFFFFF */
  ecu_bus_write(&E, 0xBF80F410u, 4, 0xAA996655u);
  ecu_bus_write(&E, 0xBF80F410u, 4, 0x556699AAu);
  ecu_bus_write(&E, 0xBF80F408u, 4, 0x8000u);
  CHECK_EQ(E.nvm_erases, 1u);
  CHECK_EQ(ecu_bus_read(&E, 0xBF80F400u, 4) & 0x8000u, 0u);       /* WR self-cleared */
  /* 3. Word Program (0x4001) at 0x1D007000 writes NVMDATA = 0x12345678 */
  ecu_bus_write(&E, 0xBF80F420u, 4, 0x1D007000u);                 /* NVMADDR */
  ecu_bus_write(&E, 0xBF80F430u, 4, 0x12345678u);                 /* NVMDATA */
  ecu_bus_write(&E, 0xBF80F400u, 4, 0x4001u);                     /* NVMCON = WREN | Word Program */
  ecu_bus_write(&E, 0xBF80F410u, 4, 0xAA996655u);
  ecu_bus_write(&E, 0xBF80F410u, 4, 0x556699AAu);
  ecu_bus_write(&E, 0xBF80F408u, 4, 0x8000u);
  CHECK_EQ(E.nvm_writes, 1u);
  /* 4. The chip's contents read back (as a programmer would) hold the erase and the program; a re-initialised core
   *    (power cycle) started from them sees the same flash. */
  static uint8_t chip[ECU_PFL_BYTES];
  CHECK_EQ(ecu_pflash_read(&E, chip, sizeof chip), (size_t)ECU_PFL_BYTES);
  uint32_t w; memcpy(&w, chip + 0x7000, 4);
  CHECK_EQ(w, 0x12345678u);
  memcpy(&w, chip + 0x7004, 4);
  CHECK_EQ(w, 0xFFFFFFFFu);                                       /* rest of the erased page */
  memcpy(&w, chip + 0x8000, 4);
  CHECK_EQ(w, 0xFFFFFFFFu);                                       /* beyond the 4 KB image: erased flash */
  ecu_free(&E);
  memset(&E, 0, sizeof E);
  if (ecu_init(&E, BFL, sizeof BFL, chip, sizeof chip)) abort();
  CHECK_EQ(ecu_bus_read(&E, 0xBF80F400u, 4) & 0x8000u, 0u);
  static uint8_t again[ECU_PFL_BYTES];
  ecu_pflash_read(&E, again, sizeof again);
  CHECK(memcmp(chip, again, sizeof chip) == 0);
  ecu_free(&E);
}

TEST(nvm_word_program_only_clears_bits) {
  /* PIC32MX FRM section 5 (Flash program memory): a program operation changes 1s to 0s only. A word programmed over
   * a written word reads back as the AND of the two; writing a bit back to 1 takes a page erase. (Microchip forbids
   * re-programming a word without an erase in between: this pins the model's semantics, not a supported use.) */
  setup();
  ecu_bus_write(&E, 0xBF80F420u, 4, 0x1D007000u);                 /* NVMADDR */
  ecu_bus_write(&E, 0xBF80F400u, 4, 0x4004u);                     /* NVMCON = WREN | Page Erase */
  ecu_bus_write(&E, 0xBF80F410u, 4, 0xAA996655u);
  ecu_bus_write(&E, 0xBF80F410u, 4, 0x556699AAu);
  ecu_bus_write(&E, 0xBF80F408u, 4, 0x8000u);                     /* NVMCONSET = WR */
  CHECK_EQ(E.nvm_erases, 1u);
  ecu_bus_write(&E, 0xBF80F420u, 4, 0x1D007000u);
  ecu_bus_write(&E, 0xBF80F430u, 4, 0xF0F0F0F0u);                 /* NVMDATA */
  ecu_bus_write(&E, 0xBF80F400u, 4, 0x4001u);                     /* WREN | Word Program */
  ecu_bus_write(&E, 0xBF80F410u, 4, 0xAA996655u);
  ecu_bus_write(&E, 0xBF80F410u, 4, 0x556699AAu);
  ecu_bus_write(&E, 0xBF80F408u, 4, 0x8000u);
  CHECK_EQ(E.nvm_writes, 1u);
  ecu_bus_write(&E, 0xBF80F430u, 4, 0x0FF00FF0u);                 /* second program, no erase */
  ecu_bus_write(&E, 0xBF80F400u, 4, 0x4001u);
  ecu_bus_write(&E, 0xBF80F410u, 4, 0xAA996655u);
  ecu_bus_write(&E, 0xBF80F410u, 4, 0x556699AAu);
  ecu_bus_write(&E, 0xBF80F408u, 4, 0x8000u);
  CHECK_EQ(E.nvm_writes, 2u);
  static uint8_t chip[ECU_PFL_BYTES];
  CHECK_EQ(ecu_pflash_read(&E, chip, sizeof chip), (size_t)ECU_PFL_BYTES);
  uint32_t w; memcpy(&w, chip + 0x7000, 4);
  CHECK_EQ(w, 0x00F000F0u);                                       /* AND, not the new value */
  ecu_free(&E);
}

static void put_words(uint8_t *img, size_t at, const uint32_t *w, size_t n) {
  for (size_t i = 0; i < n; i++)
    for (unsigned k = 0; k < 4; k++) img[at + 4 * i + k] = (uint8_t)(w[i] >> (8 * k));   /* little-endian */
}
/* A bootloader programs a new application and jumps to it without a reset: the coprocessor-0 sites the SoC model
 * handles must follow the words now in the chip. Power-on image: `mtc0 t0, Cause` at 0x100. Programmed image: at
 * 0x100 `mtc0 t0, Count`, at 0x110 `mfc0 t1, Count`. Boot flash sets t0 = 0x100 (the CS0 request bit, were it written
 * to Cause) and jumps to offset 0x100 of program flash. The old Cause site must not raise CS0IF, the new Count write
 * must set the core timer and the new Count read must return it; the chip keeps the real instruction words. Both
 * programming paths (word and row). */
#define NVMCON 0xBF80F400u
static void nvm_op(uint32_t op, uint32_t pa) {
  ecu_bus_write(&E, 0xBF80F420u, 4, pa);                         /* NVMADDR */
  ecu_bus_write(&E, NVMCON, 4, 0x4000u | op);                     /* WREN | NVMOP */
  ecu_bus_write(&E, 0xBF80F410u, 4, 0xAA996655u);                 /* NVMKEY unlock */
  ecu_bus_write(&E, 0xBF80F410u, 4, 0x556699AAu);
  ecu_bus_write(&E, NVMCON + 8u, 4, 0x8000u);                     /* NVMCONSET = WR */
}
static void reprogram_moves_cp0_sites(bool row) {
  enum { T0 = 8, T1 = 9, T9 = 25, CP0_COUNT = 9 };
  memset(BFL, 0, sizeof BFL);
  const uint32_t boot[5] = {MIPS_I(MIPS_OP_ORI, MIPS_ZERO, T0, 0x100), MIPS_I(0x0F, 0, T9, 0x9D00),   /* lui t9 */
                            MIPS_I(MIPS_OP_ORI, T9, T9, 0x100), MIPS_R(T9, 0, 0, 0, MIPS_FN_JR), 0};
  put_words(BFL, 0, boot, 5);
  uint32_t devcfg1 = 0xFFFFDFFFu;
  memcpy(BFL + 0x2FF8, &devcfg1, 4);
  static uint8_t img[0x1000];
  memset(img, 0, sizeof img);
  const uint32_t old_code[3] = {MIPS_COP0(MIPS_COP0_MT, T0, MIPS_CP0_CAUSE, 0), 0x1000FFFFu, 0};   /* ; b . ; nop */
  put_words(img, 0x100, old_code, 3);
  memset(&E, 0, sizeof E);
  if (ecu_init(&E, BFL, sizeof BFL, img, sizeof img)) abort();
  static uint32_t page[128];
  memset(page, 0, sizeof page);
  const uint32_t new_code[7] = {MIPS_COP0(MIPS_COP0_MT, T0, CP0_COUNT, 0), 0, 0, 0,
                                MIPS_COP0(MIPS_COP0_MF, T1, CP0_COUNT, 0), 0x1000FFFFu, 0};
  memcpy(page + 0x100 / 4, new_code, sizeof new_code);
  nvm_op(0x4u, 0x1D000000u);                                      /* page erase */
  if (row) {
    uc_mem_write((uc_engine *)E.uc, 0x1000u, page, sizeof page);  /* RAM source of a 512-byte row */
    ecu_bus_write(&E, 0xBF80F440u, 4, 0x1000u);                   /* NVMSRCADDR */
    nvm_op(0x3u, 0x1D000000u);                                    /* row program */
  } else {
    for (unsigned i = 0; i < 128; i++) {
      ecu_bus_write(&E, 0xBF80F430u, 4, page[i]);                 /* NVMDATA */
      nvm_op(0x1u, 0x1D000000u + 4u * i);                         /* word program */
    }
  }
  ecu_run(&E, 1000);
  CHECK_EQ(E.last_err, 0);
  CHECK_EQ(ecu_bus_read(&E, IFS0, 4) & 0x6u, 0);                  /* no CS0IF/CS1IF from the reprogrammed word */
  uint32_t t1 = ecu_reg(&E, UC_MIPS_REG_0 + T1);
  CHECK(t1 >= 0x100u && t1 <= 0x100u + 8u);                       /* Count written at 0x100, read 4 instructions on */
  CHECK_EQ(E.cp0_count_base_val, 0x100u);                         /* the model's core timer took the new Count write */
  uint32_t w = 0;
  uc_mem_read((uc_engine *)E.uc, 0x1D000110u, &w, 4);
  CHECK_EQ(w, MIPS_R(T1, MIPS_ZERO, T1, 0, MIPS_FN_ADDU));        /* the core runs the model's stand-in for the read */
  static uint8_t chip[ECU_PFL_BYTES];
  ecu_pflash_read(&E, chip, sizeof chip);
  memcpy(&w, chip + 0x110, 4);
  CHECK_EQ(w, new_code[4]);                                       /* the chip holds mfc0, not the core's stand-in */
  ecu_free(&E);
}
TEST(reprogram_moves_cp0_sites_word_program) { reprogram_moves_cp0_sites(false); }
TEST(reprogram_moves_cp0_sites_row_program) { reprogram_moves_cp0_sites(true); }

/* Watchdog: off by default (fuse clear), started by WDTCON.ON, cleared by WDTCLR, expires after 1 ms x 2^WDTPS; the
 * DEVCFG1.FWDTEN fuse enables it from reset. */
TEST(watchdog_period_clear_and_fuse) {
  memset(BFL, 0, sizeof BFL);
  uint32_t loop[2] = {0x1000FFFFu, 0x00000000u};
  memcpy(BFL, loop, sizeof loop);
  uint32_t devcfg1 = 0xFF60DDDBu;                                /* FWDTEN = 0, WDTPS = 0 (1 ms) */
  memcpy(BFL + 0x2FF8, &devcfg1, 4);
  memset(&E, 0, sizeof E);
  if (ecu_init(&E, BFL, sizeof BFL, PFL, sizeof PFL)) abort();
  ecu_run(&E, 200000);                                           /* 2.5 ms: disabled, never expires */
  CHECK(!ecu_wdt_poll(&E));
  ecu_bus_write(&E, 0xBF800000u, 4, 0x8000u);                    /* WDTCON.ON */
  ecu_run(&E, 60000);
  CHECK(!ecu_wdt_poll(&E));
  ecu_bus_write(&E, 0xBF800008u, 4, 1u);                         /* WDTCONSET.WDTCLR */
  CHECK_EQ(ecu_bus_read(&E, 0xBF800000u, 4) & 1u, 0);
  ecu_run(&E, 60000);
  CHECK(!ecu_wdt_poll(&E));                                      /* 0.75 ms since the clear */
  ecu_run(&E, 40000);
  CHECK(ecu_wdt_poll(&E));                                       /* 1.25 ms: expired */
  ecu_mark_reset_cause(&E, ECU_RCON_WDTO);
  CHECK(ecu_bus_read(&E, 0xBF80F600u, 4) & ECU_RCON_WDTO);
  ecu_free(&E);
  devcfg1 |= 1u << 23;                                           /* FWDTEN: running from reset */
  memcpy(BFL + 0x2FF8, &devcfg1, 4);
  memset(&E, 0, sizeof E);
  if (ecu_init(&E, BFL, sizeof BFL, PFL, sizeof PFL)) abort();
  ecu_run(&E, 100000);
  CHECK(ecu_wdt_poll(&E));
  ecu_free(&E);
}

/* Idle loops: a loop that goes round with nothing changing is skipped to the next event; a loop that changes state or
 * reads a peripheral is executed. */
static void run_boot_loop(const uint32_t *code, size_t n, bool skip) {
  memset(BFL, 0, sizeof BFL);
  put_words(BFL, 0, code, n);
  uint32_t devcfg1 = 0xFFFFDFFFu;
  memcpy(BFL + 0x2FF8, &devcfg1, 4);
  memset(&E, 0, sizeof E);
  if (ecu_init(&E, BFL, sizeof BFL, PFL, sizeof PFL)) abort();
  ecu_idle_skip(&E, skip);
  ecu_run(&E, 400000);
  CHECK_EQ(E.last_err, 0);
  CHECK(E.cyc >= 400000u && E.cyc <= 400004u);                        /* the last block may end a little over */
}
TEST(idle_loop_skipped_only_when_nothing_changes) {
  const uint32_t pure[2] = {0x1000FFFFu, 0x00000000u};                       /* b . ; nop */
  const uint32_t count[3] = {0x25080001u, 0x1000FFFEu, 0x00000000u};        /* addiu t0,t0,1 ; b -1 ; nop */
  const uint32_t poll[4] = {0x3C09BF88u, 0x8D281030u, 0x1000FFFEu, 0x00000000u};   /* lui t1 ; lw t0,IFS0 ; b -1 ; nop */
  const uint32_t store[4] = {0x3C08A000u, 0xAD000100u, 0x1000FFFEu, 0x00000000u};  /* lui t0,0xA000 ; sw zero,0x100(t0) ; b -1 */
  run_boot_loop(pure, 2, true);
  CHECK(E.idle_skipped > 390000u);
  ecu_free(&E);
  run_boot_loop(pure, 2, false);                                      /* skipping off */
  CHECK_EQ(E.idle_skipped, 0u);
  ecu_free(&E);
  run_boot_loop(count, 3, true);                                      /* a register changes every time round */
  CHECK_EQ(E.idle_skipped, 0u);
  ecu_free(&E);
  run_boot_loop(poll, 4, true);                                       /* a peripheral read */
  CHECK_EQ(E.idle_skipped, 0u);
  ecu_free(&E);
  run_boot_loop(store, 4, true);                                      /* a store of the value already there */
  CHECK(E.idle_skipped > 390000u);
  ecu_free(&E);
}

TEST(reprogram_hook_table_full_sets_emu_err) {
  static uint32_t img[256];
  uint32_t mtc0_count = MIPS_COP0(MIPS_COP0_MT, 8, 9, 0);
  for (unsigned i = 0; i < 256; i++) img[i] = mtc0_count;
  memset(BFL, 0, sizeof BFL);
  uint32_t devcfg1 = 0xFFFFDFFFu;
  memcpy(BFL + 0x2FF8, &devcfg1, 4);
  memset(&E, 0, sizeof E);
  if (ecu_init(&E, BFL, sizeof BFL, (const uint8_t *)img, sizeof img)) abort();
  CHECK_EQ(E.last_err, 0);
  CHECK(!ecu_is_emu_err(UC_ERR_OK));
  CHECK(!ecu_is_emu_err(UC_ERR_INSN_INVALID));
  CHECK(!ecu_is_emu_err(UC_ERR_EXCEPTION));
  ecu_bus_write(&E, 0xBF80F430u, 4, mtc0_count);
  nvm_op(0x1u, 0x1D000400u);                                          /* 257th CP0 site overflows MAX_CODE_HOOKS */
  CHECK_EQ(E.last_err, UC_ERR_RESOURCE);
  CHECK(ecu_is_emu_err(E.last_err));
  ecu_free(&E);
}

/* Exception-raising and undefined opcodes stop the core (ecu.last_err) instead of executing as no-ops. On silicon they
 * vector (Reserved Instruction / Breakpoint / Debug); seen from outside, an ECU that takes one goes silent, which a
 * stopped core matches (and the watchdog model resets a stopped core). */
static int run_one(uint32_t insn) {
  memset(BFL, 0, sizeof BFL);
  uint32_t code[4] = {0x00000000u, insn, 0x1000FFFFu, 0x00000000u};   /* nop ; insn ; b . ; nop */
  memcpy(BFL, code, sizeof code);
  uint32_t devcfg1 = 0xFFFFDFFFu;
  memcpy(BFL + 0x2FF8, &devcfg1, 4);
  memset(&E, 0, sizeof E);
  if (ecu_init(&E, BFL, sizeof BFL, PFL, sizeof PFL)) abort();
  ecu_run(&E, 1000);
  int err = E.last_err;
  ecu_free(&E);
  return err;
}
TEST(exception_and_undefined_opcodes_stop_the_core) {
  CHECK_EQ(run_one(0x00000000u), UC_ERR_OK);             /* control: nop runs into the loop */
  CHECK_EQ(run_one(0x7000003Fu), UC_ERR_EXCEPTION);      /* SDBBP */
  CHECK_EQ(run_one(0x0000000Du), UC_ERR_EXCEPTION);      /* BREAK */
  CHECK_EQ(run_one(0x7000003Eu), UC_ERR_INSN_INVALID);   /* undefined SPECIAL2 function */
  CHECK_EQ(run_one(0x7C00003Eu), UC_ERR_INSN_INVALID);   /* undefined SPECIAL3 function */
  CHECK_EQ(run_one(0x00000001u), UC_ERR_INSN_INVALID);   /* SPECIAL fn 1 (MOVCI, FPU): not on the M4K */
}

/* No boot region (an application-only image): the core starts at the application's start address in the
 * declared hand-off state, the boot region reads erased, and the pins the boot code drives are driven from the first
 * run on (src/ecu.c, "application start"). */
static unsigned LAT_EVENTS; static uint64_t LAT_FIRST_CYC = UINT64_MAX;
static void count_lat(void *ctx, ecu_t *e, uint64_t cyc, unsigned port, uint16_t o, uint16_t n) {
  (void)ctx; (void)e; (void)port; (void)o; (void)n;
  if (!LAT_EVENTS++) LAT_FIRST_CYC = cyc;
}
TEST(application_start_without_boot_region) {
  static uint8_t app[ECU_APP_FLAT_BYTES], chip[ECU_PFL_BYTES], img[ECU_PFL_BYTES], bfl[ECU_BFL_BYTES];
  memset(app, 0xFF, sizeof app);
  uint32_t loop[2] = {0x1000FFFFu, 0x00000000u};               /* b . ; nop at the application's start address */
  memcpy(app + 0x1000, loop, sizeof loop);
  memset(img, 0x00, sizeof img);                                 /* a full image: something below the window too */
  memcpy(img + ECU_APP_OFF, app, sizeof app);
  CHECK_EQ(ecu_place_pflash(chip, img, sizeof img), ECU_PFL_BYTES);
  CHECK_EQ(chip[0], 0x00);
  size_t n = ecu_place_pflash(chip, app, sizeof app);           /* application only: lands at the window */
  CHECK_EQ(n, ECU_APP_OFF + ECU_APP_FLAT_BYTES);
  CHECK(chip[0] == 0xFF && memcmp(chip + ECU_APP_OFF, app, sizeof app) == 0);
  ecu_app_boot_flash(bfl, sizeof bfl);
  uint32_t c1, c2, rv; memcpy(&c1, bfl + 0x2FF8, 4); memcpy(&c2, bfl + 0x2FF4, 4); memcpy(&rv, bfl, 4);
  CHECK_EQ(rv, 0xFFFFFFFFu);                                     /* no code at the reset vector */
  CHECK_EQ(c1 & 7u, 3);                                          /* FNOSC primary oscillator with PLL */
  CHECK_EQ((c1 >> 12) & 3u, 1);                                  /* FPBDIV /2 */
  CHECK_EQ((c1 >> 23) & 1u, 0);                                  /* FWDTEN off */
  CHECK_EQ((c2 >> 4) & 7u, 5);                                   /* FPLLMUL x20 */
  for (int pass = 0; pass < 2; pass++) {                         /* NULL boot region, then the declared boot flash */
    memset(&E, 0, sizeof E);
    if (ecu_init(&E, pass ? bfl : NULL, pass ? sizeof bfl : 0, pass ? chip : img, pass ? n : sizeof img)) abort();
    CHECK(E.app_start && E.handoff_pending);
    CHECK_EQ(ecu_reg(&E, UC_MIPS_REG_CP0_STATUS), 0x00000000u);  /* BEV and ERL clear */
    CHECK_EQ(ecu_reg(&E, UC_MIPS_REG_CP0_CAUSE), 0x00800000u);   /* IV */
    CHECK_EQ(ecu_reg(&E, UC_MIPS_REG_CP0_INTCTL) & 0x3E0u, 0x20u);
    CHECK(ecu_pbclk_hz(&E) == ECU_SYSCLK_HZ / 2);
    CHECK(!E.wdt_fuse);
    uint8_t back[ECU_PFL_BYTES];
    ecu_pflash_read(&E, back, sizeof back);
    CHECK(back[0] == 0xFF && back[ECU_APP_OFF - 1] == 0xFF);   /* nothing below the window */
    CHECK_EQ(ecu_bus_read(&E, 0xBF886140u, 4), 0x313Fu);         /* TRISF still at reset before the first run */
    LAT_EVENTS = 0; LAT_FIRST_CYC = UINT64_MAX;
    E.lat_sink = count_lat;
    uint64_t sfr_w = E.sfr_writes;
    CHECK_EQ(E.boot_cycles, ECU_APP_BOOT_CYCLES);
    ecu_run(&E, ECU_APP_BOOT_CYCLES - 1);
    CHECK(E.handoff_pending && LAT_EVENTS == 0);                 /* held while the boot code would run */
    ecu_run(&E, 1001);
    CHECK(!E.handoff_pending);
    CHECK_EQ(E.sfr_writes, sfr_w);                                /* the hand-off is not the application's I/O */
    CHECK(LAT_EVENTS > 0 && LAT_FIRST_CYC == ECU_APP_BOOT_CYCLES); /* driven pins appear at the hand-off */
    CHECK_EQ(ecu_bus_read(&E, 0xBF886140u, 4), 0x0024u);         /* TRISF as the boot code leaves it */
    CHECK_EQ(ecu_bus_read(&E, 0xBF886160u, 4), 0xFFF4u);         /* LATF */
    CHECK_EQ(ecu_bus_read(&E, 0xBF80F200u, 4), 0x0000u);         /* DDPCON: JTAG off */
    uint32_t pc = ecu_reg(&E, UC_MIPS_REG_PC);
    uint32_t entry = 0x80000000u | (0x1D000000u + ECU_APP_OFF + 0x1000u);   /* KSEG0: window + 0x1000 */
    CHECK(pc == entry || pc == entry + 4u);                       /* running the application's start loop */
    CHECK_EQ(E.last_err, 0);
    ecu_free(&E);
  }
  memset(&E, 0, sizeof E);                                        /* a card in the slot: card detect RB3 low */
  if (ecu_init(&E, NULL, 0, chip, n)) abort();
  ecu_set_pin(&E, 1, 3, false);
  ecu_run(&E, ECU_APP_CARD_BOOT_CYCLES - 1);
  CHECK(E.handoff_pending && E.handoff_card);
  ecu_run(&E, 1001);
  CHECK(!E.handoff_pending);
  CHECK_EQ(ecu_bus_read(&E, 0xBF886140u, 4), 0x0014u);           /* TRISF: SDI4 in, SDO4 out */
  CHECK_EQ(ecu_bus_read(&E, 0xBF805C00u, 4) & 0x8000u, 0x8000u);  /* SPI4 on */
  ecu_free(&E);
}

int main(void) {
  RUN(application_start_without_boot_region);
  RUN(portals_clr_set_inv);
  RUN(port_reads_latch_for_outputs_and_pins_for_inputs);
  RUN(timer1_period_sets_t1if_on_time);
  RUN(eic_presents_highest_priority_vector);
  RUN(uart_rx_line_rate_and_overrun);
  RUN(uart_rx_ninth_byte_saved_by_a_read);
  RUN(uart_rx_oerr_clear_via_portal_resets_fifo);
  RUN(uart_rx_oerr_clear_via_sta_write_resets_fifo);
  RUN(uart_rx_interrupt_reasserts_while_data_remains);
  RUN(uart_rx_interrupt_threshold_urxisel01);
  RUN(int1_falling_edge_sets_flag);
  RUN(oc_single_pulse_with_late_refinement);
  RUN(oc_pulse_does_not_repeat_after_the_timer_wrap);
  RUN(oc_edge_survives_block_overshoot);
  RUN(spi_exchange_only_with_selected_device);
  RUN(adc_scan_fills_buffers_in_pin_order);
  RUN(nvm_flash_erase_and_program);
  RUN(reprogram_moves_cp0_sites_word_program);
  RUN(reprogram_moves_cp0_sites_row_program);
  RUN(idle_loop_skipped_only_when_nothing_changes);
  RUN(reprogram_hook_table_full_sets_emu_err);
  RUN(watchdog_period_clear_and_fuse);
  RUN(pbclk_follows_devcfg1_and_osccon);
  RUN(timer23_pair_interrupts_on_timer3_flag);
  RUN(timer_above_pr_wraps_at_ffff_then_matches);
  RUN(timer_above_pr_multi_cycle_advance_keeps_every_match);
  RUN(adc_conversion_time_follows_tad);
  RUN(nvm_word_program_only_clears_bits);
  RUN(exception_and_undefined_opcodes_stop_the_core);
  MT_REPORT();
}
