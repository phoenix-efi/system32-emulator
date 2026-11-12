/* Unit tests of the harness model (emu/common/s32board.c), no firmware needed: every knob family drives the MCU pins,
 * ADC inputs, UARTs and SPI devices the way the I/O ledger (model/io_ledger.json) says. The ECUs run a two-instruction
 * idle loop and the Controller is never started, so only the harness and the SoC pin / peripheral models are tested. */
#include "minitest.h"
#include "s32board.h"
#include "s32snap.h"
#include "s32panel.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <time.h>

static double now_s(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }

static s32board B;
static s32ct CT;
static ecu_t E[2];
static s32_fram FR[3];
static s32_rtc RTC;
static s32_sd SD[3];
static uint8_t SDIMG[3][64 * 1024];
static uint8_t BFL[0x3000], PFL[0x1000], CTP[3 * 64];
static int BOOTS[3], RESETS[3];

static void boot(void *ctx, int who, bool cold) {
  (void)ctx;
  if (cold) BOOTS[who]++; else RESETS[who]++;
  if (who != SB_CT) {
    int k = who - 1;
    uint64_t cyc = (uint64_t)(B.now * s32board_hz(&B, who));
    ecu_free(&E[k]);
    if (ecu_init(&E[k], BFL, sizeof BFL, PFL, sizeof PFL)) abort();
    E[k].cyc = cyc;
  }
  s32board_attach(&B);
}

static void setup(void) {
  memset(BFL, 0, sizeof BFL);
  uint32_t loop[2] = {0x1000FFFFu, 0x00000000u};                /* b . ; nop */
  memcpy(BFL, loop, sizeof loop);
  uint32_t devcfg1 = 0xFFFFDFFFu;                                 /* FPBDIV /2 */
  memcpy(BFL + 0x2FF8, &devcfg1, 4);
  for (int k = 0; k < 2; k++) { ecu_free(&E[k]); memset(&E[k], 0, sizeof E[k]); if (ecu_init(&E[k], BFL, sizeof BFL, PFL, sizeof PFL)) abort(); }
  memset(CTP, 0, sizeof CTP);
  s32ct_init(&CT, CTP, sizeof CTP);
  s32_rtc_init(&RTC, &CT.cpu.cycles, SB_FCY, 1791806400);
  for (int w = 0; w < 3; w++) { s32_fram_init(&FR[w], 0); memset(SDIMG[w], 0, sizeof SDIMG[w]); s32_sd_init(&SD[w], SDIMG[w], sizeof SDIMG[w]); }
  memset(BOOTS, 0, sizeof BOOTS); memset(RESETS, 0, sizeof RESETS);
  s32board_init(&B, (s32board_host){.boot = boot});
  B.ct = &CT; B.ecu[0] = &E[0]; B.ecu[1] = &E[1];
  for (int w = 0; w < 3; w++) B.fram[w] = &FR[w];
  B.rtc = &RTC;
  s32ct_spi_attach(&CT, 2, (s32_pin){6, 7}, s32_fram_dev(&FR[0]));
  s32ct_spi_attach(&CT, 2, (s32_pin){5, 3}, s32_rtc_dev(&RTC));
  s32board_attach(&B);
}
static int knob(const char *n, const char *v) { int r = s32board_knob(&B, n, v); if (r) fprintf(stderr, "  (%s)\n", B.err); return r; }
/* run the harness (and the idle ECUs) from the current quantum up to emulated time t */
static void run_to(double t) {
  long last = (long)llround(t / SB_QUANTUM_S);
  for (long q = B.q + 1; q <= last; q++) {
    s32board_step(&B, q);
    for (int k = 0; k < 2; k++) {
      uint64_t tg = s32board_target(&B, SB_E1 + k, q);
      if (B.on[SB_E1 + k] && !E[k].last_err) ecu_run_to(&E[k], tg); else E[k].cyc = tg;
    }
    CT.cpu.cycles = s32board_target(&B, SB_CT, q);
    s32board_watchdogs(&B);
  }
}
static bool ecu_pin(int k, unsigned port, unsigned bit) { return (E[k].pin_in[port] >> bit) & 1u; }
static bool ct_pin(unsigned port, unsigned bit) { return (CT.pin_in[port] >> bit) & 1u; }

TEST(knob_table_parses_every_knob) {
  setup();
  static const char *const OK[][2] = {
    {"S32_RPM", "2400"}, {"S32_START", "1"}, {"S32_RPM_AT", "2=150,4=0"}, {"S32_CYL", "6"}, {"S32_SYNC_BEFORE", "30"},
    {"S32_MAG1", "80"}, {"S32_MAG2", "77"}, {"S32_HALL_WIDTH", "5"}, {"S32_CRANK_JITTER", "10"}, {"S32_CRANK_GLITCH", "1:2.5,2:3:20:sync"},
    {"S32_CRANK_DROP", "1:2-3:all"}, {"S32_CRANK_EMI", "2:100@1-2"}, {"S32_HALL_STUCK", "1:sync=low@1-2"}, {"S32_AN", "5=900,1:9=600"},
    {"S32_CT_AN", "15=300"}, {"S32_TPS_NOISE", "4"}, {"S32_TPS_FAULT", "2:512~400@6"}, {"S32_AN_FAULT", "ct:12=intermittent:20@1-2"},
    {"S32_TPS_SUPPLY", "1"}, {"S32_TPS_UNPOWERED", "float"}, {"S32_ET_WIRING", "both"}, {"S32_BATT", "1=12.5,2=13@4"},
    {"S32_AFR", "14.7"}, {"S32_SWITCHES", "on plead"}, {"S32_ECU_SELECT", "2"}, {"S32_SELECT_GAP_MS", "30"}, {"S32_RETARD", "1:on"},
    {"S32_DIP", "2:3=on"}, {"S32_PUMP", "2=on"}, {"S32_DIGIN", "4=lo"}, {"S32_SWITCH", "3:select=1,4:1:plead=stop"},
    {"S32_BOUNCE_MS", "5"}, {"S32_BOUNCE_EDGES", "3"}, {"S32_PIN_FAULT", "1:RC13=gnd@2-3"}, {"S32_ECU_POWER", "1:off@4,1:on@5"},
    {"S32_SUPPLY", "2:9.5@3"}, {"S32_BOR_V", "6"}, {"S32_RESET", "ct@2"}, {"S32_XTAL_PPM", "1=20"}, {"S32_ECU_LINK", "1:ber=0.01@1-2"},
    {"S32_LINK_TABLE_SEAL", "full"}, {"S32_CONSOLE_IN", "1:1:1:get CONFIG2\\r"}, {"S32_DEV_FAULT", "ct:sd=readerr@1"}, {"S32_RTC", "osf"}, {"S32_RTC_EPOCH", "1500000000"},
    {"S32_CT_SD_WP", "1"}, {"S32_ECU_SD_WP", "1"}, {"S32_INJ_DEAD", "auto"}, {"S32_INPUT", "1:menu,2:enc:1"}};
  for (size_t i = 0; i < sizeof OK / sizeof OK[0]; i++) CHECK_EQ(knob(OK[i][0], OK[i][1]), 0);
  int n = 0; for (const char *const *p = s32board_knob_names(); *p; p++) n++;
  CHECK_EQ(n, (int)(sizeof OK / sizeof OK[0]));                     /* every knob has a sample above */
  CHECK_EQ(s32board_knob(&B, "S32_NOPE", "1"), -1);
  CHECK_EQ(s32board_knob(&B, "S32_CYL", "5"), -2);
  /* an empty write-protect value must be a defined "off", not an uninitialized read (fuzz-found) */
  setup();
  CHECK_EQ(s32board_knob(&B, "S32_CT_SD_WP", ""), 0);
  CHECK_EQ(B.lvl[SB_CT][4] & (1u << 4), 0u);
  CHECK_EQ(s32board_knob(&B, "S32_CT_SD_WP", "on"), 0);
  CHECK(B.lvl[SB_CT][4] & (1u << 4));
  CHECK_EQ(s32board_knob(&B, "S32_ECU_SD_WP", ""), 0);
  CHECK_EQ(s32board_knob(&B, "S32_AN_FAULT", "1:5=bogus@1"), -2);
  CHECK_EQ(s32board_knob(&B, "S32_PIN_FAULT", "1:RE8=gnd@1"), -2);   /* crank pins: Hall knobs only */
  CHECK_EQ(s32board_knob(&B, "S32_INJ_DEAD", "fast"), -2);
}

/* An ECU whose watchdog runs and is never cleared is reset by the harness when it expires (here: started by software,
 * 1 ms period), also while its core is stopped; the other ECU is untouched. */
TEST(ecu_watchdog_resets_the_core) {
  setup();
  run_to(0.001);
  ecu_bus_write(&E[0], 0xBF800000u, 4, 0x8000u);                   /* ECU1 WDTCON.ON; WDTPS 0 = 1 ms in this image */
  E[0].wdt_ps = 0;
  run_to(0.0015);
  CHECK_EQ(RESETS[SB_E1], 0);
  run_to(0.003);
  CHECK_EQ(RESETS[SB_E1], 1);
  CHECK_EQ(B.wdt_resets[0], 1);
  CHECK(ecu_bus_read(&E[0], 0xBF80F600u, 4) & ECU_RCON_WDTO);
  CHECK_EQ(RESETS[SB_E2], 0);
  run_to(0.01);
  CHECK_EQ(RESETS[SB_E1], 1);                                      /* after the reset the watchdog is off again */
}

TEST(crank_edges_per_revolution) {
  setup();
  knob("S32_CYL", "6"); knob("S32_RPM", "1200"); knob("S32_START", "0");
  run_to(1.0);                                                     /* 20 revolutions */
  CHECK(E[0].int_edges[1] >= 59 && E[0].int_edges[1] <= 61);       /* INT1 falling edges: 3 per revolution */
  CHECK(E[0].int_edges[2] >= 19 && E[0].int_edges[2] <= 21);       /* INT2: once per revolution */
  CHECK_EQ(B.crank_edges[0][0], B.crank_edges[1][0]);
}

TEST(crank_glitch_drop_emi_stuck_jitter) {
  setup();
  knob("S32_CYL", "6"); knob("S32_RPM", "1200"); knob("S32_START", "0");
  knob("S32_CRANK_GLITCH", "1:0.5003");                            /* one extra pulse on ECU1 */
  knob("S32_CRANK_DROP", "2:0.2-0.4:sync");                        /* ECU2 loses its sync pulses for 0.2 s */
  run_to(1.0);
  CHECK(E[0].int_edges[1] == E[1].int_edges[1] + 1);
  CHECK(E[1].int_edges[2] + 4 <= E[0].int_edges[2]);
  setup();
  knob("S32_CYL", "6"); knob("S32_RPM", "1200"); knob("S32_START", "0");
  knob("S32_HALL_STUCK", "1:crank=low@0.2-0.6"); knob("S32_CRANK_EMI", "2:2000@0-1");
  run_to(0.4);
  CHECK(!ecu_pin(0, 4, 8));                                        /* held low: short to ground */
  run_to(1.0);
  CHECK(ecu_pin(0, 4, 8));                                         /* released */
  CHECK(E[0].int_edges[1] < 50);                                   /* edges missing while held */
  CHECK(E[1].int_edges[1] > 70);                                   /* EMI adds edges */
  setup();
  knob("S32_CYL", "6"); knob("S32_RPM", "1200"); knob("S32_START", "0"); knob("S32_CRANK_JITTER", "50");
  run_to(1.0);
  CHECK(E[0].int_edges[1] >= 59 && E[0].int_edges[1] <= 61);       /* jitter moves edges, never drops them */
}

TEST(analog_fault_modes) {
  setup();
  knob("S32_AN", "5=700,6=600,1:13=100");
  knob("S32_AN_FAULT", "1:5=gnd@0.1-0.2,1:6=open@0.1-0.2,2:5=vcc@0.1-0.2,1:8=raw:300@0.1,1:10=noise:20@0.1,"
                       "2:6=float@0.1-0.2,1:13=stuck@0.1,2:12=intermittent:20@0.1,ct:13=raw:400@0.1,ct:15=open@0.1");
  run_to(0.05);
  CHECK_EQ(E[0].an[5], 700); CHECK_EQ(E[0].an[6], 600); CHECK_EQ(E[0].an[13], 100);
  run_to(0.15);
  CHECK_EQ(E[0].an[5], 0); CHECK_EQ(E[0].an[6], 1023); CHECK_EQ(E[1].an[5], 1023); CHECK_EQ(E[0].an[8], 300);
  CHECK_EQ(E[0].an_noise[10], 20); CHECK_EQ(E[1].an[6], 512); CHECK_EQ(E[1].an_noise[6], 400);
  CHECK_EQ(E[0].an[13], 100);
  CHECK_EQ(CT.an[13], 400); CHECK_EQ(CT.an[15], 0);
  s32board_set_an(&B, SB_E1, 13, 900);
  run_to(0.25);
  CHECK_EQ(E[0].an[5], 700);                                       /* window over */
  CHECK_EQ(E[0].an[13], 100);                                      /* stuck holds the value it had */
  bool seen_open = false, seen_mid = false;
  for (double t = 0.26; t < 0.4; t += 0.005) { run_to(t); if (E[1].an[12] == 0) seen_open = true; if (E[1].an[12] == 512) seen_mid = true; }
  CHECK(seen_open && seen_mid);                                    /* intermittent alternates */
}

TEST(tps_shared_supply_unpowered_modes) {
  static const char *const MODE[3] = {"gnd", "float", "pullup"};
  static const int RAW[3] = {0, 512, 1023}, NOISE[3] = {0, 400, 0};
  for (int m = 0; m < 3; m++) {
    setup();
    knob("S32_AN", "9=600"); knob("S32_TPS_UNPOWERED", MODE[m]); knob("S32_ECU_POWER", "1:off@0.1");
    run_to(0.05); CHECK_EQ(E[1].an[9], 600);
    run_to(0.2); CHECK_EQ(E[1].an[9], RAW[m]); CHECK_EQ(E[1].an_noise[9], NOISE[m]);
  }
  setup();                                                         /* ECU2 powering it: ECU1 loss changes nothing */
  knob("S32_AN", "9=600"); knob("S32_TPS_SUPPLY", "2"); knob("S32_ECU_POWER", "1:off@0.1");
  run_to(0.2); CHECK_EQ(E[1].an[9], 600);
}

TEST(et_wired_to_ecu1_only) {
  setup();
  knob("S32_AN", "7=300");
  run_to(0.01);
  CHECK_EQ(E[0].an[7], 300); CHECK_EQ(E[1].an[7], 1023);
  setup();
  knob("S32_ET_WIRING", "both"); knob("S32_AN", "7=300");
  run_to(0.01);
  CHECK_EQ(E[1].an[7], 300);
}

TEST(batt_follows_supply_curve) {
  setup();
  knob("S32_SUPPLY", "2:12.0@0.1");
  run_to(0.05);
  CHECK_EQ(E[0].an[11], 305);                                      /* (13.8 - 0.22) / 0.04456 */
  run_to(0.15);
  CHECK_EQ(E[1].an[11], 264);                                      /* (12.0 - 0.22) / 0.04456 */
}

TEST(ct_battery_and_afr_senses) {
  setup();
  knob("S32_BATT", "1=13.8,2=12.6"); knob("S32_AFR", "14.7");
  run_to(0.02);
  CHECK_EQ(CT.an[10], 357); CHECK_EQ(CT.an[11], 326);             /* volts / (3.3 x 12) x 1023 */
  CHECK_EQ(CT.an[12], 182);                                        /* (14.7 - 10) / 2 V / 4 / 3.3 x 1023 */
}

TEST(select_switch_grounds_one_enable_and_gates_injectors) {
  setup();
  run_to(0.01);
  CHECK(ecu_pin(0, 2, 13)); CHECK(!ecu_pin(1, 2, 13));             /* ECU1 selected: ECU2's wire grounded */
  CHECK(B.inj.enabled[0]); CHECK(!B.inj.enabled[1]);
  knob("S32_SWITCH", "0.1:select=2");
  run_to(0.2);
  CHECK(!ecu_pin(0, 2, 13)); CHECK(ecu_pin(1, 2, 13));
  CHECK(!B.inj.enabled[0]); CHECK(B.inj.enabled[1]);
  /* an injector pulse from a gated ECU never reaches the engine */
  E[0].lat_sink(E[0].lat_ctx, &E[0], (uint64_t)(0.21 * 80e6), 0, 0, 1);
  E[0].lat_sink(E[0].lat_ctx, &E[0], (uint64_t)(0.22 * 80e6), 0, 1, 0);
  E[1].lat_sink(E[1].lat_ctx, &E[1], (uint64_t)(0.21 * 80e6), 0, 0, 2);
  E[1].lat_sink(E[1].lat_ctx, &E[1], (uint64_t)(0.22 * 80e6), 0, 2, 0);
  run_to(0.3);
  inj_meter_settle(&B.inj, (uint64_t)(0.3 * 80e6));
  CHECK_EQ(B.inj.union_pulses[0], 0); CHECK_EQ(B.inj.union_pulses[1], 1);
  setup(); knob("S32_ECU_SELECT", "both"); run_to(0.01); CHECK(B.inj.enabled[0] && B.inj.enabled[1]);
  setup(); knob("S32_ECU_SELECT", "none"); run_to(0.01); CHECK(!B.inj.enabled[0] && !B.inj.enabled[1]);
}

TEST(select_switch_break_before_make) {
  setup();
  knob("S32_SELECT_GAP_MS", "30"); knob("S32_SWITCH", "0.1:select=2");
  run_to(0.115);
  CHECK(ecu_pin(0, 2, 13) && ecu_pin(1, 2, 13));                   /* in transit: both enable wires open */
  run_to(0.14);
  CHECK(!ecu_pin(0, 2, 13) && ecu_pin(1, 2, 13));
}

TEST(plead_retard_dip_pump_digin_contacts) {
  setup();
  run_to(0.01);
  CHECK(ecu_pin(0, 1, 2)); CHECK(!ecu_pin(0, 3, 6)); CHECK(ecu_pin(0, 3, 9));   /* RUN, retard off, DIP open */
  CHECK(ct_pin(6, 1)); CHECK(!ct_pin(6, 8)); CHECK(ct_pin(6, 9)); CHECK(ct_pin(6, 14));
  knob("S32_SWITCH", "0.1:1:plead=stop,0.1:2:retard=on,0.1:1:dip2=on,0.1:pump2=on,0.1:pump1=off,0.1:digin4=lo");
  run_to(0.12);
  CHECK(!ecu_pin(0, 1, 2)); CHECK(ecu_pin(1, 3, 6)); CHECK(!ecu_pin(0, 3, 10));
  CHECK(!ct_pin(6, 1)); CHECK(ct_pin(6, 8)); CHECK(!ct_pin(6, 14));
  setup(); knob("S32_SWITCHES", "plead inj"); run_to(0.01);
  CHECK(!ecu_pin(0, 1, 2)); CHECK(ecu_pin(0, 2, 13)); CHECK(ecu_pin(1, 1, 2)); CHECK(!ecu_pin(1, 2, 13));
  setup(); knob("S32_RETARD", "on"); knob("S32_DIP", "1:4=on"); run_to(0.01);
  CHECK(ecu_pin(0, 3, 6) && ecu_pin(1, 3, 6)); CHECK(!ecu_pin(0, 3, 12));
}

TEST(contact_bounce_chatter) {
  setup();
  knob("S32_BOUNCE_MS", "8"); knob("S32_BOUNCE_EDGES", "4"); knob("S32_SWITCH", "0.1:1:plead=stop");
  int changes = 0; bool last = true;
  for (double t = 0.1; t < 0.12; t += SB_QUANTUM_S) { run_to(t); bool l = ecu_pin(0, 1, 2); if (l != last) changes++; last = l; }
  CHECK(changes >= 3);                                             /* chatters before settling */
  CHECK(!ecu_pin(0, 1, 2));                                        /* settled grounded */
}

TEST(pin_wire_faults) {
  setup();
  knob("S32_PIN_FAULT", "1:RB2=gnd@0.1-0.2,ct:RG1=open@0.1-0.2,2:RC13=stuck@0.1-0.3,1:RB0=gnd@0.1,ct:RA9=gnd@0.1,1:RB2=intermittent:20@0.3-0.5");
  knob("S32_SWITCH", "0.28:1:plead=stop");                         /* intermittent wire on a grounded switch: low / open */
  knob("S32_PUMP", "1=off");
  run_to(0.15);
  CHECK(!ecu_pin(0, 1, 2)); CHECK(ct_pin(6, 1)); CHECK(!ecu_pin(1, 2, 13)); CHECK(!ecu_pin(0, 1, 0)); CHECK(!ct_pin(0, 9));
  knob("S32_SWITCH", "0.2:select=2");
  run_to(0.25);
  CHECK(!ecu_pin(1, 2, 13));                                       /* stuck: the switch move does not reach the pin */
  CHECK(ecu_pin(0, 1, 2)); CHECK(!ct_pin(6, 1));
  bool lo = false, hi = false;
  for (double t = 0.3; t < 0.45; t += 0.002) { run_to(t); if (ecu_pin(0, 1, 2)) hi = true; else lo = true; }
  CHECK(lo && hi);
}

TEST(power_brownout_and_cat5_controller_supply) {
  setup();
  knob("S32_SUPPLY", "1:5.0@0.1,1:13.8@0.2");
  run_to(0.15);
  CHECK(!B.on[SB_E1]); CHECK(B.on[SB_CT]); CHECK(!ct_pin(0, 2)); CHECK(ct_pin(0, 1));
  run_to(0.25);
  CHECK(B.on[SB_E1]); CHECK_EQ(BOOTS[SB_E1], 1); CHECK(ct_pin(0, 2));
  setup();
  knob("S32_ECU_POWER", "1:off@0.1,2:off@0.1,2:on@0.3");
  run_to(0.2);
  CHECK(!B.on[SB_CT]);                                             /* both cables dead: the Controller is off */
  run_to(0.35);
  CHECK(B.on[SB_CT]); CHECK(B.on[SB_E2]); CHECK(!B.on[SB_E1]); CHECK_EQ(BOOTS[SB_CT], 1);
  setup();
  knob("S32_ECU_POWER", "2:off@0");                                /* never powered */
  run_to(0.01);
  CHECK(!B.on[SB_E2]); CHECK(!ct_pin(0, 1));
}

TEST(controller_slot_empty) {                                      /* s32pair none ...: no Controller on the bench */
  setup();
  B.ct_absent = true;
  knob("S32_ECU_POWER", "1:off@0.1,1:on@0.2");
  run_to(0.3);
  CHECK(B.on[SB_E1]); CHECK(B.on[SB_E2]); CHECK(!B.on[SB_CT]); CHECK_EQ(BOOTS[SB_CT], 0);
  CHECK_EQ(BOOTS[SB_E1], 1);                                       /* the ECUs still power-cycle as usual */
}

TEST(mclr_reset_events) {
  setup();
  knob("S32_RESET", "ct@0.1,2@0.1");
  run_to(0.2);
  CHECK_EQ(RESETS[SB_CT], 1); CHECK_EQ(RESETS[SB_E2], 1); CHECK_EQ(RESETS[SB_E1], 0);
}

TEST(link_dead_cut_ber_garbage) {
  setup();
  knob("S32_ECU_LINK", "1:dead@0.1-0.2,2:cut-c2e@0.1-0.2,1:ber=0.5@0.3-0.4,2:garbage=10@0.3");
  uint8_t buf[64]; memset(buf, 0x55, sizeof buf);
  run_to(0.15);
  CHECK_EQ(s32board_link(&B, 0, true, buf, 8, sizeof buf), 0);
  CHECK_EQ(s32board_link(&B, 0, false, buf, 8, sizeof buf), 0);
  CHECK_EQ(s32board_link(&B, 1, true, buf, 8, sizeof buf), 0);
  CHECK_EQ(s32board_link(&B, 1, false, buf, 8, sizeof buf), 8);
  run_to(0.35);
  CHECK_EQ(s32board_link(&B, 0, true, buf, 16, sizeof buf), 16);
  CHECK(B.link_bits_flipped[0] > 20);
  CHECK_EQ(s32board_link(&B, 1, false, buf, 4, sizeof buf), 14); /* 10 garbage bytes to the Controller, once */
  CHECK_EQ(s32board_link(&B, 1, false, buf, 4, sizeof buf), 4);
  B.link_dead[1] = true;
  CHECK_EQ(s32board_link(&B, 1, false, buf, 4, sizeof buf), 0);
}

TEST(link_bench_frames) {
  setup();
  knob("S32_ECU_LINK",
       "1:c2e:cfg:8=260@0.01,1:c2e:cfg-short:8@0.02,1:c2e:trim:5@0.03,1:c2e:trim-short:5@0.04,"
       "1:c2e:fuel-row:8=140:full*2@0.05,1:c2e:fuel-short:16@0.06,"
       "2:cut-e2c@0.01,2:e2c:status:1950:full@0.01,2:e2c:status-short:1950:full@0.02");
  link_deframer d_c2e, d_e2c;
  link_deframer_init(&d_c2e, NULL, NULL, NULL);
  link_deframer_init(&d_e2c, NULL, NULL, NULL);
  uint8_t buf[2048];
  for (long q = 1; q <= 1000; q++) {
    s32board_step(&B, q);
    unsigned n = s32board_link(&B, 0, true, buf, 0, sizeof buf);
    for (unsigned i = 0; i < n; i++) link_deframe_byte(&d_c2e, buf[i]);
    buf[0] = 0xAA;
    n = s32board_link(&B, 1, false, buf, 1, sizeof buf);
    for (unsigned i = 0; i < n; i++) link_deframe_byte(&d_e2c, buf[i]);
  }
  CHECK_EQ(d_c2e.frames, 7);
  CHECK_EQ(d_c2e.bad, 0);
  CHECK_EQ(d_c2e.by_type[LINK_C2E_SET_CONFIG], 2);
  CHECK_EQ(d_c2e.by_type[LINK_C2E_TRIM_DELTA], 2);
  CHECK_EQ(d_c2e.by_type[LINK_C2E_WRITE_FUEL], 3);
  CHECK_EQ(d_e2c.frames, 2);
  CHECK_EQ(d_e2c.bad, 0);
  CHECK_EQ(d_e2c.by_type[LINK_E2C_STATUS], 2);
}

/* S32_ECU_LINK e2c:cfg-block: a 112-byte config_block frame (type 5) with the named slots set and the rest zero --
 * the frame an ECU puts on the wire when the Controller asks for its configuration (for example a blank-FRAM
 * ECU's reply (cylinder count 0), delivered on the bench so a check does not depend on the boot race). */
static uint8_t CB[112];
static unsigned CB_N;
static void cb_frame(void *ctx, unsigned type, const uint8_t *pl, unsigned len, unsigned wire) {
  (void)ctx; (void)wire;
  if (type == LINK_E2C_CONFIG_BLOCK) { CHECK_EQ(len, 112u); memcpy(CB, pl, sizeof CB); CB_N++; }
}
static uint32_t cb_word(unsigned slot) {
  const uint8_t *w = CB + (slot - 1) * 4;
  return (uint32_t)w[0] | ((uint32_t)w[1] << 8) | ((uint32_t)w[2] << 16) | ((uint32_t)w[3] << 24);
}

TEST(link_bench_cfg_block) {
  setup();
  CHECK(knob("S32_ECU_LINK", "1:e2c:cfg-block:0=5@0.01") != 0);      /* slots are 1..28 */
  CHECK(knob("S32_ECU_LINK", "1:e2c:cfg-block:29=5@0.01") != 0);
  CHECK(knob("S32_ECU_LINK", "1:e2c:cfg-block:2=@0.01") != 0);
  CHECK(knob("S32_ECU_LINK", "1:e2c:cfg-block:2=5+@0.01") != 0);
  CHECK(knob("S32_ECU_LINK", "1:e2c:cfg-block:@0.01") != 0);
  CHECK_EQ(knob("S32_ECU_LINK", "1:e2c:cfg-block:19=1000+22=50+23=200+26=35+27=100@0.01"), 0);
  link_deframer d;
  link_deframer_init(&d, cb_frame, NULL, NULL);
  uint8_t buf[2048];
  CB_N = 0;
  for (long q = 1; q <= 2000; q++) {                                  /* paced: the 118-byte frame takes ~0.05 s */
    s32board_step(&B, q);
    unsigned n = s32board_link(&B, 0, false, buf, 0, sizeof buf);
    for (unsigned i = 0; i < n; i++) link_deframe_byte(&d, buf[i]);
  }
  CHECK_EQ(CB_N, 1u);
  CHECK_EQ(d.bad, 0);
  CHECK_EQ(cb_word(2), 0u);                                           /* a blank-FRAM ECU's reply: cylinder count 0 */
  CHECK_EQ(cb_word(19), 1000u);
  CHECK_EQ(cb_word(22), 50u);
  CHECK_EQ(cb_word(23), 200u);
  CHECK_EQ(cb_word(26), 35u);
  CHECK_EQ(cb_word(27), 100u);
  for (unsigned s = 1; s <= 28; s++)
    if (s != 19 && s != 22 && s != 23 && s != 26 && s != 27) CHECK_EQ(cb_word(s), 0u);
}

/* S32_LINK_TABLE_SEAL: a fuel-row bench frame carries the stored format's 161-byte table sum by default and the
 * 322-byte sum with "full"; the table it sends is ECU1's FRAM fuel table with that row set. */
static uint8_t SEAL_PL[FRAM_TABLE_BYTES]; static unsigned SEAL_N;
static void seal_frame(void *ctx, unsigned type, const uint8_t *pl, unsigned len, unsigned wire) {
  (void)ctx; (void)wire;
  if (type == LINK_C2E_WRITE_FUEL && len == FRAM_TABLE_BYTES) { memcpy(SEAL_PL, pl, len); SEAL_N++; }
}
static uint16_t seal_word(void) { return (uint16_t)(SEAL_PL[0x142] | (SEAL_PL[0x143] << 8)); }
static void seal_send(const char *seal) {
  setup();
  FR[1].mem[FRAM_ECU_FUEL_TABLE] = 14; FR[1].mem[FRAM_ECU_FUEL_TABLE + 1] = 9;
  for (unsigned i = 0; i < 256u; i++) FR[1].mem[FRAM_ECU_FUEL_TABLE + 0x42 + i] = (uint8_t)(40 + i % 50);
  if (seal) CHECK_EQ(knob("S32_LINK_TABLE_SEAL", seal), 0);
  CHECK_EQ(knob("S32_ECU_LINK", "1:c2e:fuel-row:8=140@0.01"), 0);
  link_deframer d; link_deframer_init(&d, seal_frame, NULL, NULL);
  uint8_t buf[2048]; SEAL_N = 0;
  for (long q = 1; q <= 4000; q++) {                               /* paced: the 330-byte frame takes ~0.13 s */
    s32board_step(&B, q);
    unsigned n = s32board_link(&B, 0, true, buf, 0, sizeof buf);
    for (unsigned i = 0; i < n; i++) link_deframe_byte(&d, buf[i]);
  }
  CHECK_EQ(SEAL_N, 1u);
  CHECK_EQ(SEAL_PL[0x42 + 8 * 16 + 3], 140);
}

TEST(link_noise_is_flag_free_and_paced) {
  setup();
  CHECK_EQ(knob("S32_ECU_LINK", "1:e2c:noise:50:g10@0.1"), 0);
  uint8_t buf[2048], got[64]; unsigned all = 0; long at[64];
  for (long q = 1; q <= 3000 && all < sizeof got; q++) {
    s32board_step(&B, q);
    unsigned n = s32board_link(&B, 0, false, buf, 0, sizeof buf);
    for (unsigned i = 0; i < n; i++) { got[all] = buf[i]; at[all] = q; all++; }
  }
  CHECK_EQ(all, 51u);                                   /* one 0x7E, then 50 noise bytes */
  CHECK_EQ(got[0], 0x7E);
  for (unsigned i = 1; i < all; i++) {
    CHECK(got[i] != 0x7E && got[i] != 0x7D);            /* never a flag or an escape inside */
    CHECK_EQ(got[i], (uint8_t)('A' + (i - 1) % 26));
  }
  CHECK(at[0] >= 1000 && at[0] <= 1002);                /* fires at 0.1 s (quantum 1000) */
  CHECK(at[50] - at[0] >= 499 && at[50] - at[0] <= 502); /* one byte every 10 quanta */
  CHECK_EQ(knob("S32_ECU_LINK", "1:e2c:noise:0@0.1"), -2);
  CHECK_EQ(knob("S32_ECU_LINK", "1:e2c:noise:50:g0@0.1"), -2);
  CHECK_EQ(knob("S32_ECU_LINK", "1:e2c:noise:50:g300@0.1"), -2);
}

TEST(link_bench_table_seal) {
  seal_send(NULL);
  CHECK_EQ(seal_word(), fram_table_checksum(SEAL_PL));
  CHECK(seal_word() != fram_table_checksum_full(SEAL_PL));
  seal_send("half");
  CHECK_EQ(seal_word(), fram_table_checksum(SEAL_PL));
  seal_send("full");
  CHECK_EQ(seal_word(), fram_table_checksum_full(SEAL_PL));
  CHECK(seal_word() != fram_table_checksum(SEAL_PL));
  setup();
  CHECK(knob("S32_LINK_TABLE_SEAL", "322") != 0);
}

TEST(console_input_bytes) {
  setup();
  knob("S32_CONSOLE_IN", "1:4:0.1:get CONFIG2\\r;ct:2:0.1:hello;ct:1:0.1:F;ct:1:0.2:R;ct:1:0.3:1");
  ecu_bus_write(&E[0], 0xBF806200u, 4, 0x8000);                    /* U4MODE.ON */
  ecu_bus_write(&E[0], 0xBF806210u, 4, 0x1000);                    /* U4STA.URXEN */
  run_to(0.15);
  CHECK_EQ(E[0].uart[3].rx_bytes + E[0].uart[3].rx_overruns, 12);   /* every byte reached the pin (8-deep FIFO) */
  CHECK_EQ((CT.uart[1].rt - CT.uart[1].rh + S32_UART_RING) % S32_UART_RING, 5);
  CHECK_EQ((CT.uart[0].rt - CT.uart[0].rh + S32_UART_RING) % S32_UART_RING, 1);
  CHECK_EQ(CT.uart[0].ring[CT.uart[0].rh], 'F');
  run_to(0.25);
  CHECK_EQ((CT.uart[0].rt - CT.uart[0].rh + S32_UART_RING) % S32_UART_RING, 2);
  run_to(0.35);
  CHECK_EQ((CT.uart[0].rt - CT.uart[0].rh + S32_UART_RING) % S32_UART_RING, 3);
  CHECK_EQ(CT.uart[0].ring[(CT.uart[0].rh + 1) % S32_UART_RING], 'R');
  CHECK_EQ(CT.uart[0].ring[(CT.uart[0].rh + 2) % S32_UART_RING], '1');
}

TEST(fram_faults) {
  setup();
  FR[1].mem[0x10] = 0x5A;
  knob("S32_DEV_FAULT", "1:fram=flip:10.0@0.1,1:fram=absent@0.2-0.3,ct:fram=miso-low@0.2,2:fram=wp@0.2");
  run_to(0.15);
  CHECK_EQ(FR[1].mem[0x10], 0x5B);
  run_to(0.25);
  s32_spi_dev d = s32_fram_dev(&FR[1]);
  d.select(d.ctx, true); d.exchange(d.ctx, 0x9F); CHECK_EQ(d.exchange(d.ctx, 0), 0xFF); d.select(d.ctx, false);
  d = s32_fram_dev(&FR[0]); d.select(d.ctx, true); CHECK_EQ(d.exchange(d.ctx, 0x9F), 0x00); d.select(d.ctx, false);
  d = s32_fram_dev(&FR[2]);                                        /* write-protected: WREN + WRITE leave the cell alone */
  d.select(d.ctx, true); d.exchange(d.ctx, 0x06); d.select(d.ctx, false);
  d.select(d.ctx, true); d.exchange(d.ctx, 0x02); d.exchange(d.ctx, 0); d.exchange(d.ctx, 0); d.exchange(d.ctx, 0); d.exchange(d.ctx, 0xAA); d.select(d.ctx, false);
  CHECK_EQ(FR[2].mem[0], 0);
  run_to(0.35);
  d = s32_fram_dev(&FR[1]); d.select(d.ctx, true); d.exchange(d.ctx, 0x9F); CHECK_EQ(d.exchange(d.ctx, 0), 0x7F); d.select(d.ctx, false);
}

TEST(sd_faults) {
  setup();
  B.sd[SB_CT] = &SD[0]; B.sd[SB_E1] = &SD[1];
  B.lvl[SB_CT][4] &= (uint16_t)~(1u << 3); B.lvl[SB_E1][1] &= (uint16_t)~((1u << 3) | (1u << 4));
  knob("S32_DEV_FAULT", "ct:sd=remove@0.1,ct:sd=insert@0.2,1:sd=readerr@0.1-0.2,1:sd=absent@0.3");
  run_to(0.05);
  CHECK(!ct_pin(4, 3)); CHECK(!ecu_pin(0, 1, 3)); CHECK(!ecu_pin(0, 1, 4));
  run_to(0.15);
  CHECK(ct_pin(4, 3)); CHECK_EQ(SD[0].fault, S32_DEV_ABSENT); CHECK_EQ(SD[1].fault, S32_DEV_RDERR);
  s32_spi_dev d = s32_sd_dev(&SD[1]);                              /* CMD17 answers the data-error token */
  d.select(d.ctx, true);
  const uint8_t c17[6] = {0x51, 0, 0, 0, 0, 0xFF};
  for (int i = 0; i < 6; i++) d.exchange(d.ctx, c17[i]);
  uint8_t r[4]; for (int i = 0; i < 4; i++) r[i] = d.exchange(d.ctx, 0xFF);
  CHECK_EQ(r[1], 0x00); CHECK_EQ(r[3], 0x08);
  d.select(d.ctx, false);
  run_to(0.25);
  CHECK(!ct_pin(4, 3)); CHECK_EQ(SD[0].fault, S32_DEV_OK);
  run_to(0.35);
  d = s32_sd_dev(&SD[1]); d.select(d.ctx, true); d.exchange(d.ctx, 0x40); CHECK_EQ(d.exchange(d.ctx, 0), 0xFF); d.select(d.ctx, false);
  setup(); B.sd[SB_E1] = &SD[1]; knob("S32_ECU_SD_WP", "1"); run_to(0.01); CHECK(ecu_pin(0, 1, 4));
}

TEST(rtc_osf_invalid_absent) {
  setup();
  knob("S32_RTC", "osf");
  s32_spi_dev d = s32_rtc_dev(&RTC);
  d.select(d.ctx, true); d.exchange(d.ctx, 0xA3); uint8_t sec = d.exchange(d.ctx, 0); d.select(d.ctx, false);
  CHECK(sec & 0x80);                                               /* oscillator-stop flag set */
  d.select(d.ctx, true); d.exchange(d.ctx, 0x23); d.exchange(d.ctx, 0x00); d.select(d.ctx, false);   /* write seconds */
  d.select(d.ctx, true); d.exchange(d.ctx, 0xA3); sec = d.exchange(d.ctx, 0); d.select(d.ctx, false);
  CHECK(!(sec & 0x80));
  setup(); knob("S32_RTC", "invalid");
  d.select(d.ctx, true); d.exchange(d.ctx, 0xA3); d.exchange(d.ctx, 0); uint8_t min = d.exchange(d.ctx, 0); d.select(d.ctx, false);
  CHECK_EQ(min, 0x7F);                                             /* not a BCD value */
  setup(); knob("S32_RTC", "absent");
  d.select(d.ctx, true); d.exchange(d.ctx, 0xA3); CHECK_EQ(d.exchange(d.ctx, 0), 0xFF); d.select(d.ctx, false);
  setup(); knob("S32_RTC_EPOCH", "1500000000"); CHECK(s32_rtc_now(&RTC) == 1500000000);
}

TEST(rtc_epoch_rejects_out_of_range) {
  setup();
  CHECK(knob("S32_RTC_EPOCH", "1e300") < 0);         /* past time_t: the (time_t) cast is undefined there */
  CHECK(knob("S32_RTC_EPOCH", "inf") < 0);
  CHECK(knob("S32_RTC_EPOCH", "nan") < 0);
  CHECK(knob("S32_RTC_EPOCH", "-1") < 0);
  CHECK(knob("S32_RTC_EPOCH", "12junk") < 0);
  CHECK(knob("S32_RTC_EPOCH", "1e17") < 0);          /* gmtime_r fails there; the chip keeps a two-digit year */
  CHECK(knob("S32_RTC_EPOCH", "4102444801") < 0);    /* past 2099 */
  CHECK_EQ(knob("S32_RTC_EPOCH", "4102444800"), 0);  /* the cap itself is still accepted */
  CHECK(s32_rtc_now(&RTC) == 4102444800);
  CHECK_EQ(knob("S32_RTC_EPOCH", "1500000001"), 0);  /* a good value still lands after the rejects */
  CHECK(s32_rtc_now(&RTC) == 1500000001);
}

TEST(crystal_ppm_targets) {
  setup();
  knob("S32_XTAL_PPM", "1=100,ct=-50");
  CHECK_EQ(s32board_target(&B, SB_E2, 10000), 80000000ull);
  CHECK_EQ(s32board_target(&B, SB_E1, 10000), 80008000ull);
  CHECK_EQ(s32board_target(&B, SB_CT, 10000), 15999200ull);
  s32board_attach(&B);
  CHECK(fabs(E[0].hz - 80008000.0) < 1.0);
}

TEST(clock_check_config_words) {
  static uint8_t bfl[0x3000], ct[(0x2AC00u / 2u) * 3u];
  uint32_t c1 = 0xFF7FDDDBu, c2 = 0xFFF8FFD9u;                    /* XT + PLL /2 x20 /1, FPBDIV /2, watchdog off */
  memcpy(bfl + 0x2FF8, &c1, 4); memcpy(bfl + 0x2FF4, &c2, 4);
  size_t i2 = (0x2ABFCu / 2u) * 3u, i1 = (0x2ABFEu / 2u) * 3u;
  ct[i2] = 0xAD; ct[i2 + 1] = 0x1B; ct[i1] = 0x7F; ct[i1 + 1] = 0x1E;   /* XT, PLLDIV /2, 96 MHz PLL; watchdog off */
  FILE *null = fopen("/dev/null", "w");
  CHECK_EQ(s32board_clock_check(bfl, sizeof bfl, ct, sizeof ct, null), 0);
  c2 = 0xFFF8FFC9u;                                                /* x19: 76 MHz */
  memcpy(bfl + 0x2FF4, &c2, 4);
  CHECK_EQ(s32board_clock_check(bfl, sizeof bfl, ct, sizeof ct, null), 1);
  fclose(null);
}

TEST(bezel_script_buttons_encoder) {
  setup();
  knob("S32_INPUT", "0.1:menu:100,0.3:tune+monitor:100,0.5:edge:2@100");
  run_to(0.15); CHECK(!ct_pin(0, 3)); CHECK(ct_pin(0, 4));
  run_to(0.25); CHECK(ct_pin(0, 3));
  run_to(0.35); CHECK(!ct_pin(0, 4)); CHECK(!ct_pin(0, 7));
  run_to(0.55); CHECK(ct_pin(0, 14) != ct_pin(0, 15) || !ct_pin(0, 14));   /* encoder moved off its rest state */
  run_to(0.65); CHECK(!ct_pin(0, 15));
}

/* A quadrature edge moves one contact: with bounce, only that contact chatters and the other holds its level. A detent
 * = A closes, B closes, A opens, B opens (positive turn); "@a/b/c" sets the states of an uneven detent. */
TEST(encoder_bounce_switching_contact_only) {
  setup();
  knob("S32_BOUNCE_MS", "5"); knob("S32_BOUNCE_EDGES", "4");
  knob("S32_INPUT", "0.1:enc:1@20,0.3:enc:1@30/15/15");
  int ch[2] = {0, 0}, stray = 0; bool last[2] = {true, true};
  for (long m = 0; m < 900; m++) {                                 /* quanta from 0.1 s; edges at 0, 20, 40, 60 ms */
    run_to(0.1 + m * SB_QUANTUM_S);
    int moving = (int)((m / 200) % 2);
    for (int c = 0; c < 2; c++) {
      bool l = ct_pin(0, 14 + (unsigned)c);
      if (l != last[c]) { ch[c]++; if (c != moving) stray++; }
      last[c] = l;
    }
  }
  CHECK_EQ(stray, 0);                                              /* the contact at rest never chatters */
  CHECK(ch[0] >= 6); CHECK(ch[1] >= 6);                            /* each contact chattered on both of its edges */
  CHECK(ct_pin(0, 14)); CHECK(ct_pin(0, 15));                      /* back at rest: both open */
  run_to(0.31); CHECK(!ct_pin(0, 14)); CHECK(ct_pin(0, 15));       /* 30/15/15: A closed at 0.300 */
  run_to(0.336); CHECK(!ct_pin(0, 14)); CHECK(!ct_pin(0, 15));     /* B closed at 0.330 */
  run_to(0.351); CHECK(ct_pin(0, 14)); CHECK(!ct_pin(0, 15));      /* A open at 0.345 */
  run_to(0.366); CHECK(ct_pin(0, 14)); CHECK(ct_pin(0, 15));       /* B open at 0.360 */
}

/* The crank jitter knob's own description (emu/common/s32board.c, the S32_CRANK_JITTER row of the knob table, also
 * listed in the I/O ledger's ecu.crank row) is "uniform +-us timing noise on every Hall edge": no edge may move by
 * more than the set amplitude. Two runs with the same board RNG seed schedule the same edges; only the jitter
 * differs. The ECUs are never run, so the pin queue keeps every scheduled edge. */
TEST(crank_jitter_within_documented_amplitude) {
  static uint64_t nom[128], jit[128];
  unsigned n;
  setup();
  knob("S32_CYL", "6"); knob("S32_RPM", "1200"); knob("S32_START", "0");
  for (long q = B.q + 1; q <= 5000; q++) s32board_step(&B, q);    /* 0.5 s; the ECUs are not run: the pin queue keeps every edge */
  n = E[0].pinq_n;
  CHECK(n > 30 && n <= 128);
  for (unsigned i = 0; i < n; i++) nom[i] = E[0].pinq[i].at;
  setup();
  knob("S32_CYL", "6"); knob("S32_RPM", "1200"); knob("S32_START", "0"); knob("S32_CRANK_JITTER", "100");
  for (long q = B.q + 1; q <= 5000; q++) s32board_step(&B, q);
  CHECK_EQ(E[0].pinq_n, n);
  for (unsigned i = 0; i < n; i++) jit[i] = E[0].pinq[i].at;
  unsigned moved = 0;
  for (unsigned i = 0; i < n; i++) {
    uint64_t d = jit[i] > nom[i] ? jit[i] - nom[i] : nom[i] - jit[i];
    if (d) moved++;
    CHECK(d <= 100u * ECU_SYSCLK_HZ / 1000000u);                  /* within +-100 us on the 80 MHz ECU clock */
  }
  CHECK(moved > 0);                                               /* the jitter really moved some edges */
}

TEST(panel_backlight_permille_uses_pr2_plus_1) {
  /* DS39969 section 14 (output compare, PWM): the PWM period is PR2+1 Tcy, so the backlight duty the panel model
   * reports is OC1R / (PR2 + 1). */
  setup();
  pic24_poke16(&CT.cpu, 0x010C, 3);                               /* PR2 */
  pic24_poke16(&CT.cpu, 0x0196, 1);                               /* OC1R */
  pic24_poke16(&CT.cpu, 0x0190, 0x0006);                          /* OC1CON: OCM 6 (PWM) */
  static uint8_t rgb[S32_PANEL_W * S32_PANEL_H * 3];
  s32_panel_state st;
  s32_panel_render(&CT, true, rgb, &st);
  CHECK_EQ(st.backlight_permille, 250);                           /* 1/4, not 1/3 */
}

/* Every knob parser accepts values beyond 4096 bytes (and console items beyond 512 bytes) when they fit the board's
 * event/text tables, and refuses with a non-zero exit and message when a table is full (never silently truncating). */
TEST(input_script_refuses_what_it_cannot_run) {
  setup();
  CHECK_EQ(s32board_knob(&B, "S32_INPUT", "0.1:enc:1@30/15"), -1);
  CHECK_EQ(s32board_knob(&B, "S32_INPUT", "0.1:enc:1@30/15/15/15/15"), -1);
  CHECK_EQ(s32board_knob(&B, "S32_INPUT", "0.1:enc:1@30x"), -1);
  /* INT_MIN, whose magnitude abs() cannot hold: the count must be refused (queue full), not wrapped to zero work */
  setup();
  CHECK(s32board_knob(&B, "S32_INPUT", "0.1:enc:-2147483648") != 0);
  CHECK_EQ(B.nev, SB_MAX_EV);
  setup();
  CHECK(s32board_knob(&B, "S32_INPUT", "0.1:edge:-2147483648") != 0);
  CHECK_EQ(B.nev, SB_MAX_EV);
  /* a giant count is refused fast: parsing stops as soon as the event queue is full */
  setup();
  double t0 = now_s();
  CHECK(s32board_knob(&B, "S32_INPUT", "0.1:edge:20403304030") != 0);
  CHECK(now_s() - t0 < 0.5);
  size_t cap = 120000, n = 0;
  char *s = malloc(cap);
  CHECK(s != NULL);
  if (!s) return;
  setup();
  for (int i = 0; i < 3000; i++) n += (size_t)snprintf(s + n, cap - n, "%s%.2f:enc:1@20", i ? "," : "", 100.0 + i * 0.01);
  CHECK(n > 4096);
  CHECK_EQ(s32board_knob(&B, "S32_INPUT", s), -1);
  setup();
  n = 0;
  for (int i = 0; i < 600; i++) n += (size_t)snprintf(s + n, cap - n, "%s%.2f:enc:1@20", i ? "," : "", 100.0 + i * 0.01);
  CHECK(n > 4096);                                                 /* longer than the old 4096-byte buffer, and fits */
  CHECK_EQ(s32board_knob(&B, "S32_INPUT", s), 0);
  /* each_item: > 4096 bytes accepted in full; > SB_MAX_EV refused with non-zero and error message */
  setup();
  n = 0;
  for (int i = 0; i < 600; i++) n += (size_t)snprintf(s + n, cap - n, "%s%.2f=%d", i ? "," : "", 1.0 + i * 0.01, 1000 + i);
  CHECK(n > 4096);
  CHECK_EQ(s32board_knob(&B, "S32_RPM_AT", s), 0);
  CHECK_EQ(B.nev, 600u);
  CHECK_EQ((int)B.ev[599].v, 1599);
  setup();
  n = 0;
  for (int i = 0; i < SB_MAX_EV + 10; i++) n += (size_t)snprintf(s + n, cap - n, "%s%.3f=1200", i ? "," : "", 1.0 + i * 0.001);
  B.err[0] = 0;
  CHECK(s32board_knob(&B, "S32_RPM_AT", s) < 0);
  CHECK(B.err[0] != 0);
  setup();
  n = 0;
  for (int i = 0; i < 600; i++) n += (size_t)snprintf(s + n, cap - n, "%s1:5=%d", i ? "," : "", i % 500);
  n += (size_t)snprintf(s + n, cap - n, ",1:13=777");
  CHECK(n > 4096);
  CHECK_EQ(s32board_knob(&B, "S32_AN", s), 0);
  CHECK_EQ(B.an_nom[SB_E1][13], 777);
  /* S32_CONSOLE_IN: item > 512 B and script > 4096 B accepted in full; > SB_TEXT refused */
  setup();
  char item900[901];
  memset(item900, 'A', 900); item900[900] = 0;
  n = 0;
  for (int i = 0; i < 6; i++) {
    item900[899] = (char)('0' + i);
    n += (size_t)snprintf(s + n, cap - n, "%s1:1:%.2f:%s", i ? ";" : "", 1.0 + i * 0.1, item900);
  }
  CHECK(n > 4096);
  CHECK_EQ(s32board_knob(&B, "S32_CONSOLE_IN", s), 0);
  CHECK_EQ(B.nev, 6u);
  CHECK_EQ(strlen(B.text + B.ev[5].s), 900u);
  CHECK_EQ((B.text + B.ev[5].s)[899], '5');
  setup();
  n = 0;
  for (int i = 0; i < 20; i++) n += (size_t)snprintf(s + n, cap - n, "%s1:1:%.2f:%s", i ? ";" : "", 1.0 + i * 0.1, item900);
  B.err[0] = 0;
  CHECK(s32board_knob(&B, "S32_CONSOLE_IN", s) < 0);
  CHECK(B.err[0] != 0);
  /* S32_INPUT snap: > 128 B name accepted; > SB_TEXT refused */
  setup();
  char snap200[201];
  memset(snap200, 's', 200); snap200[200] = 0;
  snprintf(s, cap, "1.0:snap:%s", snap200);
  CHECK_EQ(s32board_knob(&B, "S32_INPUT", s), 0);
  CHECK_EQ(B.nev, 1u);
  CHECK_EQ(strlen(B.text + B.ev[0].s), 200u);
  setup();
  n = 0;
  for (int i = 0; i < 90; i++) n += (size_t)snprintf(s + n, cap - n, "%s%.2f:snap:%s", i ? "," : "", 1.0 + i * 0.01, snap200);
  B.err[0] = 0;
  CHECK(s32board_knob(&B, "S32_INPUT", s) < 0);
  CHECK(B.err[0] != 0);
  /* A fault mode longer than its field is refused, never cut to a different value (raw:5120... read as raw:51) */
  setup();
  CHECK_EQ(s32board_knob(&B, "S32_AN_FAULT", "1:5=raw:512@1"), 0);
  CHECK(s32board_knob(&B, "S32_AN_FAULT", "1:5=raw:0000000000000000000000000512@1") < 0);
  CHECK(s32board_knob(&B, "S32_PIN_FAULT", "1:RB2=intermittent:5000000000000000000.5@1") < 0);
  CHECK(s32board_knob(&B, "S32_DEV_FAULT", "1:fram=flip:500.0000000000000000000000003@1") < 0);
  CHECK_EQ(s32board_knob(&B, "S32_DEV_FAULT", "1:fram=flip:500.3@1"), 0);
  free(s);
}

TEST(output_observers) {
  setup();
  E[0].lat_sink(E[0].lat_ctx, &E[0], 80000, 3, 0, 1u << 13);      /* tach rising edges */
  E[0].lat_sink(E[0].lat_ctx, &E[0], 90000, 3, 1u << 13, 0);
  E[0].lat_sink(E[0].lat_ctx, &E[0], 100000, 3, 0, 1u << 13);
  E[0].lat_sink(E[0].lat_ctx, &E[0], 110000, 3, 1u << 13, (1u << 13) | (1u << 8));   /* fuel-flow edge */
  E[0].lat_sink(E[0].lat_ctx, &E[0], 120000, 3, 0, 1u << 7);      /* STOP LED */
  E[0].lat_sink(E[0].lat_ctx, &E[0], 130000, 6, 0, 1u << 14);     /* RUN LED */
  E[0].lat_sink(E[0].lat_ctx, &E[0], 140000, 5, 0, 2);            /* test point RF1 */
  CHECK_EQ(B.eo[0].tach_edges, 2); CHECK_EQ(B.eo[0].ff_edges, 1); CHECK_EQ(B.eo[0].leds & 1, 1); CHECK_EQ((B.eo[0].leds >> 3) & 1, 1);
  CHECK_EQ(B.eo[0].tp_edges[1], 1);
  CT.lat_sink(CT.lat_ctx, &CT, 1000, 4, 0, (1u << 8) | 1u);        /* DIGOUT 1 and 3 */
  CT.lat_sink(CT.lat_ctx, &CT, 1000, 0, 1, 0);                     /* setup-bus CS falls */
  CHECK_EQ(B.co.digout, 0x5); CHECK_EQ(B.co.tcon_frames, 1);
  FILE *null = fopen("/dev/null", "w"); s32board_report(&B, null); s32board_fault_summary(&B, 1.0, null); fclose(null);
}

TEST(ff_edges_statistics_window) {
  setup();
  double hz = s32board_hz(&B, SB_E1);
  B.ff_win_set = true; B.ff_win_from = 0.001;                              /* the run's statistics window starts at 1 ms */
  E[0].lat_sink(E[0].lat_ctx, &E[0], (uint64_t)(0.0005 * hz), 3, 0, 1u << 8);        /* before the window: total only */
  E[0].lat_sink(E[0].lat_ctx, &E[0], (uint64_t)(0.0010 * hz), 3, 1u << 8, 0);        /* at its start: counts */
  E[0].lat_sink(E[0].lat_ctx, &E[0], (uint64_t)(0.0015 * hz), 3, 0, 1u << 8);
  E[0].lat_sink(E[0].lat_ctx, &E[0], (uint64_t)(0.0020 * hz), 3, 1u << 8, 0);
  CHECK_EQ(B.eo[0].ff_edges, 4);
  CHECK_EQ(B.eo[0].ff_edges_win, 3);
  setup();                                                                 /* no window set: the count runs from t = 0 (the total) */
  E[0].lat_sink(E[0].lat_ctx, &E[0], (uint64_t)(0.0005 * hz), 3, 0, 1u << 8);
  CHECK_EQ(B.eo[0].ff_edges, 1);
  CHECK_EQ(B.eo[0].ff_edges_win, 1);
}

TEST(panel_frame_period_report) {
  setup();
  CHECK_EQ(s32_gfx_frame_tcy(&CT.gfx), 0);                                 /* registers unprogrammed: no frames */
  CT.gfx.reg[0x10] = 370; CT.gfx.reg[0x11] = 246;                          /* the stock firmware's panel (370 x 246 px, OBSERVED) */
  uint64_t ft = s32_gfx_frame_tcy(&CT.gfx);
  CHECK(ft > 246400u && ft < 246700u);                                     /* 15.4 ms at Fcy 16 MHz: 370 x 246 px at the fixed
                                                                              modelled 5.907 MHz pixel clock (measured: the
                                                                              Controller programs GCLKDIV = 61) */
  CT.gfx.frames = 12;
  char buf[65536] = {0};
  FILE *f = fmemopen(buf, sizeof buf, "w");
  s32board_report(&B, f);
  fclose(f);
  CHECK(strstr(buf, "Controller panel display: 12 frames, 15.409 ms per frame (370 x 246 px)") != NULL);
}

TEST(ff_window_survives_snapshot_from_a_windowless_run) {
  setup();
  double hz = s32board_hz(&B, SB_E1);
  E[0].lat_sink(E[0].lat_ctx, &E[0], (uint64_t)(0.0005 * hz), 3, 0, 1u << 8);   /* run A: no statistics window set */
  E[0].lat_sink(E[0].lat_ctx, &E[0], (uint64_t)(0.0010 * hz), 3, 1u << 8, 0);
  CHECK_EQ(B.eo[0].ff_edges_win, 2);            /* counted anyway: a windowless run counts from t = 0 */
  s32_tbuf blob = {0};
  s32board_snap_blob(&B, &blob);
  setup();                                      /* run B: the same boot with S32_STATS_FROM=0 */
  B.ff_win_set = true; B.ff_win_from = 0.0;
  char why[160] = {0};
  CHECK_EQ(s32board_snap_restore(&B, (const uint8_t *)blob.p, blob.n, 0.0011, why, sizeof why), 0);
  free(blob.p);
  CHECK(B.ff_win_set);                          /* the window is this run's, not the snapshot's */
  CHECK_EQ(B.eo[0].ff_edges_win, 2);            /* and the count the saved run gathered carries over */
  E[0].lat_sink(E[0].lat_ctx, &E[0], (uint64_t)(0.0015 * hz), 3, 0, 1u << 8);
  CHECK_EQ(B.eo[0].ff_edges_win, 3);
  char buf[65536] = {0};
  FILE *f = fmemopen(buf, sizeof buf, "w");
  s32board_report(&B, f);
  fclose(f);
  CHECK(strstr(buf, "ECU1 fuel-flow output (RD8) at or after 0.000000 s: 3 edges") != NULL);
}

static void gfx_cmd(uint8_t opc, uint32_t op) {
  s32_gfx_write(&CT.gfx, &CT.cpu, 0x0700, (uint16_t)(op & 0xFFFFu), 0xFFFFu);
  s32_gfx_write(&CT.gfx, &CT.cpu, 0x0702, (uint16_t)(((uint32_t)opc << 8) | ((op >> 16) & 0xFFu)), 0xFFFFu);
}
static void gfx_fill(unsigned x, unsigned y, unsigned w, unsigned h, unsigned color) {
  gfx_cmd(0x63, y * 320u + x);
  gfx_cmd(0x64, (w << 12) | h);
  gfx_cmd(0x66, color);
  gfx_cmd(0x67, 0x0060u);
}

/* The panel renderer reads the backlight timer and compare registers like a camera would see the backlight: the
 * firmware's idle-poll state must be as before, so an observer read cannot move a later event. */
TEST(observer_sfr_reads_leave_the_controller_alone) {
  setup();
  CT.last_sfr_addr = 0x0222; CT.last_sfr_val = 0x0010; CT.same_sfr_reads = 3;
  (void)s32ct_sfr_value(&CT, 0x010C); (void)s32ct_sfr_value(&CT, 0x0196); (void)s32ct_sfr_value(&CT, 0x0190);
  CHECK_EQ(CT.last_sfr_addr, 0x0222); CHECK_EQ(CT.last_sfr_val, 0x0010); CHECK_EQ(CT.same_sfr_reads, 3);
}

TEST(home_anchor_resolves_on_unblank) {
  setup();
  double t = 0; const char *end = NULL;
  CHECK(s32board_parse_time("{H}", &end, &t) && *end == 0 && fabs(t - SB_HOME_T0) < 1e-6);
  CHECK(s32board_parse_time("{H+1.25}:x", &end, &t) && !strcmp(end, ":x") && fabs(t - (SB_HOME_T0 + 1.25)) < 1e-6);
  CHECK(!s32board_parse_time("{H-1.5}", &end, &t));                  /* pre-home {H-x} is refused */
  CHECK_EQ(s32board_knob(&B, "S32_START", "{H-1}"), -2);
  CHECK_EQ(s32board_knob(&B, "S32_INPUT", "{H-1}:menu:100"), -2);   /* refused, not skipped */
  CHECK_EQ(s32board_knob(&B, "S32_INPUT", "bogus"), -2);
  knob("S32_START", "{H}");
  knob("S32_INPUT", "{H+0.1}:menu:100");
  knob("S32_ECU_POWER", "1:off@{H+0.2}");
  s32_gfx_write(&CT.gfx, &CT.cpu, 0x0704, 0x8040, 0xFFFF);           /* G1CON1: G1EN, 4 bpp */
  s32_gfx_write(&CT.gfx, &CT.cpu, 0x0724, 0x0041, 0xFFFF);           /* G1CON2: 4 bpp, TFT */
  s32_gfx_write(&CT.gfx, &CT.cpu, 0x0726, 0x0200, 0xFFFF);           /* G1CON3: DPPINOE */
  s32_gfx_write(&CT.gfx, &CT.cpu, 0x0714, 320, 0xFFFF);              /* G1PUW */
  s32_gfx_write(&CT.gfx, &CT.cpu, 0x0716, 240, 0xFFFF);              /* G1PUH */
  s32_gfx_write(&CT.gfx, &CT.cpu, 0x071C, 320, 0xFFFF);              /* G1DPW */
  s32_gfx_write(&CT.gfx, &CT.cpu, 0x071E, 240, 0xFFFF);              /* G1DPH */
  s32_gfx_write(&CT.gfx, &CT.cpu, 0x0710, 0x4800, 0xFFFF);           /* G1W2ADRL */
  s32_gfx_write(&CT.gfx, &CT.cpu, 0x0718, 0x4800, 0xFFFF);           /* G1DPADRL */
  s32_gfx_write(&CT.gfx, &CT.cpu, 0x0730, 0x8101, 0xFFFF);           /* G1CLUT: enable + write entry 1 */
  s32_gfx_write(&CT.gfx, &CT.cpu, 0x0732, 0xFFFF, 0xFFFF);           /* entry 1 = white */
  /* Multi-fill splash screen at 0.10 s: records splash_seen, does not fire {H}. */
  gfx_fill(0, 0, 320, 120, 1);
  gfx_fill(0, 120, 320, 120, 1);
  s32_gfx_vblank(&CT.gfx, &CT.cpu);
  run_to(0.10);
  CHECK(B.splash_seen); CHECK(B.home_at < 0);
  /* Partially drawn home page (all widgets except the last Fuel Trim label at 185,179): does not fire {H}. */
  gfx_fill(0, 0, 320, 240, 0);
  gfx_fill(25, 35, 206, 1, 1); gfx_fill(25, 160, 206, 1, 1);
  gfx_fill(25, 35, 1, 126, 1); gfx_fill(230, 35, 1, 126, 1);
  static const uint16_t BOX[][2] = {
    {29, 14},  {84, 14},  {149, 14}, {204, 14},
    {60, 50},  {160, 50}, {50, 77},  {155, 77},
    {40, 110}, {165, 110},{37, 134}, {92, 134},
    {160, 134},{252, 75}, {50, 180},
    {200, 179},{103, 213},{121, 213},{169, 213}
  };
  for (size_t i = 0; i < sizeof BOX / sizeof BOX[0]; i++) gfx_fill(BOX[i][0], BOX[i][1], 4, 4, 1);
  s32_gfx_vblank(&CT.gfx, &CT.cpu);
  run_to(0.15);
  CHECK(B.home_at < 0); CHECK(ct_pin(0, 3)); CHECK(B.on[SB_E1]);
  /* Last home-page widget completes the screen: {H} fires on the next vblank. */
  gfx_fill(185, 179, 4, 4, 1);
  s32_gfx_vblank(&CT.gfx, &CT.cpu);
  run_to(0.20);
  CHECK(fabs(B.home_at - 0.15) < 1e-6);
  CHECK(fabs(B.start_s - 0.15) < 1e-6);
  run_to(0.26); CHECK(!ct_pin(0, 3));                                /* menu pressed at H+0.1 = 0.25 s */
  run_to(0.36); CHECK(ct_pin(0, 3)); CHECK(!B.on[SB_E1]);            /* menu released at 0.35 s, ECU1 off at H+0.2 = 0.35 s */
}

/* Phase anchors: "<t>~<event>[#type][+x]" waits for the event at or after t, then acts x after it, on the quantum
 * grid; the times a knob derives from the anchored one move with it; an anchor whose event never comes is reported. */
static unsigned send_frame(int k, bool to_ecu, unsigned type) {
  uint8_t raw[32];
  size_t n = link_encode(type, NULL, 0, raw, sizeof raw);
  return s32board_link(&B, k, to_ecu, raw, (unsigned)n, sizeof raw);
}
TEST(phase_anchor_link_frame_and_never_occurred) {
  setup();
  double t = 0; const char *end = NULL;
  CHECK(!s32board_parse_time("1~ct1", &end, &t));                     /* outside a knob: refused (no board to anchor in) */
  CHECK_EQ(s32board_knob(&B, "S32_START", "1~bogus"), -2);
  CHECK_EQ(s32board_knob(&B, "S32_START", "1~panel#8"), -2);          /* a panel refresh has no message type */
  CHECK_EQ(s32board_knob(&B, "S32_START", "1~ct1+-0.5"), -2);         /* negative offsets refused */
  CHECK_EQ(s32board_knob(&B, "S32_START", "{H-1}~ct1"), -2);
  CHECK_EQ(B.nanchor, 0u);
  knob("S32_INPUT", "0.05~ct1#8+0.0023:menu:100");                   /* press 2.3 ms after ECU1's keep-alive, held 100 ms */
  knob("S32_ECU_POWER", "2:off@0.05~ecu2#2");                         /* ECU2 sends no status here: never occurs */
  CHECK_EQ(B.nanchor, 2u);
  run_to(0.0301);
  send_frame(0, true, 8);                                             /* before the base: ignored */
  send_frame(0, true, 9);                                             /* wrong type */
  run_to(0.0701);
  CHECK(!B.anchor[0].fired);
  send_frame(0, true, 9);                                             /* after the base, wrong type: ignored */
  CHECK(!B.anchor[0].fired);
  send_frame(0, true, 8);                                             /* ends in the quantum [0.0700, 0.0701): fires at 0.0701 */
  CHECK(B.anchor[0].fired);
  CHECK(fabs(B.anchor[0].at - 0.0701) < 1e-9);
  run_to(0.0723); CHECK(ct_pin(0, 3));                                /* 0.0701 + 0.0023 = 0.0724: not yet */
  run_to(0.0725); CHECK(!ct_pin(0, 3));                               /* pressed */
  run_to(0.1723); CHECK(!ct_pin(0, 3));
  run_to(0.1725); CHECK(ct_pin(0, 3));                                /* released 100 ms later: the derived time moved too */
  CHECK(B.on[SB_E2]);
  char buf[512] = {0};
  FILE *f = fmemopen(buf, sizeof buf, "w");
  CHECK_EQ(s32board_anchor_check(&B, f), 1);
  fclose(f);
  CHECK(strstr(buf, "emulator error: anchor '0.05~ecu2#2' never occurred") != NULL);
  /* times with the same base and event share one anchor (one event, several offsets after it) */
  setup();
  knob("S32_INPUT", "0.01~ct2+0.01:menu:20,0.01~ct2+0.05:tune:20");
  knob("S32_START", "0.01~ct2#8");
  CHECK_EQ(B.nanchor, 2u);
  run_to(0.0201); send_frame(1, true, 8);                             /* fires both at 0.0201 */
  CHECK(B.anchor[0].fired && B.anchor[1].fired);
  CHECK(fabs(B.start_s - 0.0201) < 1e-6);
  run_to(0.0302); CHECK(!ct_pin(0, 3));                               /* menu at 0.0301 */
  run_to(0.0702); CHECK(!ct_pin(0, 4));                               /* tune at 0.0701 */
  /* chained: wait for ECU1's keep-alive, then for the next one at least 5 ms later */
  setup();
  knob("S32_INPUT", "0.01~ct1#8+0.005~ct1#8+0.001:menu:20");
  CHECK_EQ(B.nanchor, 2u);
  run_to(0.0201); send_frame(0, true, 8);                             /* first: 0.0201, chained base 0.0251 */
  CHECK(B.anchor[0].fired && !B.anchor[1].fired);
  run_to(0.0231); send_frame(0, true, 8);                             /* before the chained base: ignored */
  CHECK(!B.anchor[1].fired);
  run_to(0.0301); send_frame(0, true, 8);                             /* fires at 0.0301: menu 1 ms later, 0.0311 */
  CHECK(B.anchor[1].fired && fabs(B.anchor[1].at - 0.0301) < 1e-9);
  run_to(0.0311); CHECK(ct_pin(0, 3));
  run_to(0.0312); CHECK(!ct_pin(0, 3));
  /* a chained anchor whose first event never comes is reported as such */
  setup();
  knob("S32_START", "0.01~ecu2#2+1~ct1");
  char buf2[512] = {0};
  FILE *f2 = fmemopen(buf2, sizeof buf2, "w");
  CHECK_EQ(s32board_anchor_check(&B, f2), 2);
  fclose(f2);
  CHECK(strstr(buf2, "the anchor it is chained to never occurred") != NULL);
}

/* Phase anchors fire only on a frame the sender itself put on its own wire: complete, CRC good, this link, this
 * direction, sender powered. A bench-injected frame (S32_ECU_LINK c2e:/e2c:), a bad-CRC frame, the same type on the
 * other link or in the other direction, and a frame from an unpowered sender never satisfy one. */
static unsigned send_bytes(int k, bool to_ecu, const uint8_t *raw, size_t n) {
  uint8_t b[64];
  memcpy(b, raw, n);
  return s32board_link(&B, k, to_ecu, b, (unsigned)n, sizeof b);
}
TEST(phase_anchor_only_frames_the_sender_sent) {
  setup();
  CHECK_EQ(knob("S32_START", "0.01~ct1#9"), 0);
  CHECK_EQ(knob("S32_INPUT", "0.01~ecu1#2+0.001:menu:20"), 0);
  CHECK_EQ(knob("S32_ECU_LINK", "1:c2e:trim:5:full@0.02,1:e2c:status:1950:full@0.02"), 0);
  CHECK_EQ(B.nanchor, 2u);
  link_deframer dc, de;
  link_deframer_init(&dc, NULL, NULL, NULL);
  link_deframer_init(&de, NULL, NULL, NULL);
  uint8_t buf[2048];
  for (long q = B.q + 1; q <= 600; q++) {                            /* to 0.06 s: both bench frames delivered */
    s32board_step(&B, q);
    unsigned n = s32board_link(&B, 0, true, buf, 0, sizeof buf);
    for (unsigned i = 0; i < n; i++) link_deframe_byte(&dc, buf[i]);
    n = s32board_link(&B, 0, false, buf, 0, sizeof buf);
    for (unsigned i = 0; i < n; i++) link_deframe_byte(&de, buf[i]);
  }
  CHECK_EQ(dc.by_type[LINK_C2E_TRIM_DELTA], 1);                      /* the injected frames did cross the wire */
  CHECK_EQ(de.by_type[LINK_E2C_STATUS], 1);
  CHECK(!B.anchor[0].fired && !B.anchor[1].fired);                   /* ... and fired nothing */
  uint8_t raw[32];
  const uint8_t pl[4] = {1, 0, 0, 0};
  size_t n = link_encode(LINK_C2E_TRIM_DELTA, pl, sizeof pl, raw, sizeof raw);
  CHECK(n > 0);
  raw[3] ^= 0x01u;                                                   /* a payload bit: CRC bad */
  run_to(0.0611); send_bytes(0, true, raw, n);
  CHECK(!B.anchor[0].fired);
  raw[3] ^= 0x01u;                                                   /* good again */
  run_to(0.0621); send_bytes(1, true, raw, n);                       /* the other link (ct2) */
  CHECK(!B.anchor[0].fired);
  run_to(0.0631); send_bytes(0, false, raw, n);                      /* the other direction (ecu1, type 9) */
  CHECK(!B.anchor[0].fired && !B.anchor[1].fired);
  run_to(0.0641);
  B.on[SB_CT] = false;                                               /* bytes from an unpowered Controller: ignored */
  send_bytes(0, true, raw, n);
  CHECK(!B.anchor[0].fired);
  B.on[SB_CT] = true;
  run_to(0.0651); send_bytes(0, true, raw, n / 2);                   /* a frame split over two quanta ... */
  CHECK(!B.anchor[0].fired);
  run_to(0.0652); send_bytes(0, true, raw + n / 2, n - n / 2);       /* ... ends in [0.0651, 0.0652): fires at 0.0652 */
  CHECK(B.anchor[0].fired);
  CHECK(fabs(B.anchor[0].at - 0.0652) < 1e-9);
  CHECK(!B.anchor[1].fired);
  uint8_t st[64];
  uint8_t spl[48] = {0};
  size_t m = link_encode(LINK_E2C_STATUS, spl, sizeof spl, st, sizeof st);
  CHECK(m > 0);
  run_to(0.0701); send_bytes(0, false, st, m);                       /* ECU1's own status frame */
  CHECK(B.anchor[1].fired);
  CHECK(fabs(B.anchor[1].at - 0.0701) < 1e-9);
}

/* ---- boot-snapshot stimulus signature (s32board_stimulus; the items the boot-once planner compares before it lets
 * one case restore another's snapshot). A stimulus field left out of the timed items lets a restored run replay a
 * sibling case's stimulus: two runs that differ only in that field would share a snapshot. */
static bool tbuf_eq(const s32_tbuf *a, const s32_tbuf *b) { return a->n == b->n && (a->n == 0 || memcmp(a->p, b->p, a->n) == 0); }
static bool items_agree(const s32_tbuf *a, const s32_tbuf *b, double t_future, char *why, size_t nwhy) {
  return s32snap_items_equal_before((const uint8_t *)a->p, a->n, (const uint8_t *)b->p, b->n, t_future, SB_HOME_T0, -1, 0, why, nwhy);
}

/* Two knob strings that differ in one stimulus field must produce one shared static part and different timed items. */
static void stim_pair(const char *knob_a, const char *knob_b, s32_tbuf *ita, s32_tbuf *itb) {
  char why[256];
  s32_tbuf sta = {0}, stb = {0};
  *ita = (s32_tbuf){0}; *itb = (s32_tbuf){0};
  setup(); CHECK_EQ(knob("S32_ECU_LINK", knob_a), 0); s32board_stimulus(&B, &sta, ita);
  setup(); CHECK_EQ(knob("S32_ECU_LINK", knob_b), 0); s32board_stimulus(&B, &stb, itb);
  CHECK(tbuf_eq(&sta, &stb));                                        /* the difference is stimulus-only */
  CHECK(!tbuf_eq(ita, itb));
  CHECK(!items_agree(ita, itb, 4.0, why, sizeof why));               /* no shared snapshot past the fault */
  s32_tbuf_free(&sta); s32_tbuf_free(&stb);
}

TEST(snapshot_sig_link_fault_arg3_and_cfg28) {
  char why[256];
  s32_tbuf ita = {0}, itb = {0};
  /* Two runs whose link faults differ only in the padded frame's on-the-wire length (sb_lfault.arg3): the timed
   * items must differ (a run taking the other's snapshot would dispatch the other's frame). */
  stim_pair("1:c2e:cfg-pad:5=100:100@2", "1:c2e:cfg-pad:5=100:200@2", &ita, &itb);
  /* control: identical knobs agree (the compare is not simply always-different) */
  s32_tbuf stc = {0}, itc = {0};
  setup(); CHECK_EQ(knob("S32_ECU_LINK", "1:c2e:cfg-pad:5=100:100@2"), 0); s32board_stimulus(&B, &stc, &itc);
  CHECK(tbuf_eq(&ita, &itc));
  CHECK(items_agree(&ita, &itc, 4.0, why, sizeof why));
  s32_tbuf_free(&ita); s32_tbuf_free(&itb); s32_tbuf_free(&stc); s32_tbuf_free(&itc);
  /* The same, for one config word of a cfg-block frame (sb_lfault.cfg28). */
  stim_pair("2:e2c:cfg-block:1=10+2=20@2", "2:e2c:cfg-block:1=10+2=21@2", &ita, &itb);
  s32_tbuf_free(&ita); s32_tbuf_free(&itb);
}

/* Completeness: every field of every stimulus item kind must reach the timed-item bytes -- the boot-once planner
 * compares only these bytes, so a field left out would let a restored run replay a sibling case's
 * stimulus. Each table below lists the struct's own fields (offsetof: a renamed or removed field breaks the build).
 * The sizeof/used-bytes pins are those of the LP64 hosts that run this test; a struct that gains a field moves
 * one of them, and tests/scripts/test_stimulus_signature.py derives the same field lists from s32board.h, so a
 * field cannot slip in through padding either. Both fail loudly until the new field is hashed or the exception is
 * justified here. hashed=0 marks the one field that is runtime state on purpose: sb_ev.dyn is set only when the
 * running system adds an event (ev_add: b->q > 0), so it is 0 in every parsed stimulus and the restore merge treats
 * such events as state, not stimulus. */
typedef struct { const char *name; size_t off, size; int hashed; } sfield;
#define SF(T, f, h) {#f, offsetof(T, f), sizeof(((T *)0)->f), h}
static const sfield AF_FIELDS[] = {
  SF(sb_afault, t0, 1), SF(sb_afault, t1, 1), SF(sb_afault, period, 1), SF(sb_afault, val, 1),
  SF(sb_afault, noise, 1), SF(sb_afault, frozen, 1), SF(sb_afault, mode, 1), SF(sb_afault, latched, 1),
};
static const sfield PF_FIELDS[] = {
  SF(sb_pfault, t0, 1), SF(sb_pfault, t1, 1), SF(sb_pfault, period, 1), SF(sb_pfault, us, 1),
  SF(sb_pfault, who, 1), SF(sb_pfault, port, 1), SF(sb_pfault, bit, 1), SF(sb_pfault, mode, 1),
  SF(sb_pfault, latched, 1), SF(sb_pfault, frozen, 1),
};
static const sfield CF_FIELDS[] = {
  SF(sb_cfault, t0, 1), SF(sb_cfault, t1, 1), SF(sb_cfault, us, 1), SF(sb_cfault, rate, 1),
  SF(sb_cfault, k, 1), SF(sb_cfault, kind, 1), SF(sb_cfault, elem, 1),
};
static const sfield LF_FIELDS[] = {
  SF(sb_lfault, t0, 1), SF(sb_lfault, t1, 1), SF(sb_lfault, v, 1), SF(sb_lfault, dt, 1), SF(sb_lfault, next_t, 1),
  SF(sb_lfault, n, 1), SF(sb_lfault, arg1, 1), SF(sb_lfault, arg2, 1), SF(sb_lfault, arg3, 1), SF(sb_lfault, cfg28, 1),
  SF(sb_lfault, k, 1), SF(sb_lfault, kind, 1), SF(sb_lfault, sub, 1), SF(sb_lfault, pace, 1), SF(sb_lfault, burst, 1),
  SF(sb_lfault, done, 1),
};
static const sfield EV_FIELDS[] = {
  SF(sb_ev, t, 1), SF(sb_ev, a, 1), SF(sb_ev, v, 1), SF(sb_ev, s, 1), SF(sb_ev, kind, 1),
  SF(sb_ev, who, 1), SF(sb_ev, port, 1), SF(sb_ev, bit, 1), SF(sb_ev, level, 1), SF(sb_ev, dyn, 0),
};

static void stim_items(s32_tbuf *it) {
  s32_tbuf st = {0};
  *it = (s32_tbuf){0};
  s32board_stimulus(&B, &st, it);
  s32_tbuf_free(&st);
}

/* Zero the fault, then flip one field at a time to distinctive nonzero bytes (byte 0 = 1: a bool stays a bool) and
 * require the timed items to change exactly when the field is hashed. */
static void check_fields(const char *what, void *f, size_t fsz, const sfield *fs, unsigned nf) {
  s32_tbuf base = {0};
  memset(f, 0, fsz);
  stim_items(&base);
  for (unsigned i = 0; i < nf; i++) {
    memset(f, 0, fsz);
    uint8_t *p = (uint8_t *)f + fs[i].off;
    for (size_t j = 0; j < fs[i].size; j++) p[j] = (uint8_t)(j + 1);
    s32_tbuf cur = {0};
    stim_items(&cur);
    bool same = tbuf_eq(&base, &cur);
    if (same == (bool)fs[i].hashed)
      fprintf(stderr, "  [snapshot_sig_covers_every_stimulus_field] %s.%s %s the timed items\n",
              what, fs[i].name, fs[i].hashed ? "does not change" : "changes");
    CHECK(same != (bool)fs[i].hashed);
    s32_tbuf_free(&cur);
  }
  s32_tbuf_free(&base);
}
#define CHECK_FIELDS(what, f, fs) check_fields(what, f, sizeof *(f), fs, sizeof fs / sizeof fs[0])

TEST(snapshot_sig_covers_every_stimulus_field) {
  /* layout pins (on LP64 hosts, arm64 and x86-64): a struct that gains a field moves one of these */
  CHECK_EQ(sizeof(sb_afault), 32); CHECK_EQ(offsetof(sb_afault, latched) + sizeof(bool), 32);
  CHECK_EQ(sizeof(sb_pfault), 40); CHECK_EQ(offsetof(sb_pfault, frozen) + sizeof(bool), 38);
  CHECK_EQ(sizeof(sb_cfault), 40); CHECK_EQ(offsetof(sb_cfault, elem) + sizeof(uint8_t), 35);
  CHECK_EQ(sizeof(sb_lfault), 176); CHECK_EQ(offsetof(sb_lfault, done) + sizeof(bool), 174);
  CHECK_EQ(sizeof(sb_ev), 32); CHECK_EQ(offsetof(sb_ev, dyn) + sizeof(uint8_t), 32);

  setup(); B.naf[SB_E1][5] = 1; CHECK_FIELDS("sb_afault", &B.af[SB_E1][5][0], AF_FIELDS);
  setup(); B.npf = 1; CHECK_FIELDS("sb_pfault", &B.pf[0], PF_FIELDS);
  setup(); B.ncf = 1; CHECK_FIELDS("sb_cfault", &B.cf[0], CF_FIELDS);
  setup(); B.nlf = 1; CHECK_FIELDS("sb_lfault", &B.lf[0], LF_FIELDS);
  setup(); B.nev = 1; CHECK_FIELDS("sb_ev", &B.ev[0], EV_FIELDS);

  /* the engine item: the run's speed and start time (item kind 5) */
  s32_tbuf base = {0}, cur = {0};
  setup(); stim_items(&base);
  B.rpm = 2400; stim_items(&cur);
  CHECK(!tbuf_eq(&base, &cur));                                      /* rpm */
  B.rpm = 0; B.start_s = 4; stim_items(&cur);
  CHECK(!tbuf_eq(&base, &cur));                                      /* start_s */
  s32_tbuf_free(&base); s32_tbuf_free(&cur);

  /* a text event: the text the event carries is hashed, and s selects it */
  setup(); CHECK_EQ(knob("S32_CONSOLE_IN", "ct:1:0.5:hello"), 0);
  CHECK_EQ(B.nev, 1);
  stim_items(&base);
  B.text[0] = 'j'; stim_items(&cur);
  CHECK(!tbuf_eq(&base, &cur));                                      /* "jello" != "hello" */
  B.text[0] = 'h'; B.ev[0].s = 1; stim_items(&cur);
  CHECK(!tbuf_eq(&base, &cur));                                      /* "ello" != "hello" */
  s32_tbuf_free(&base); s32_tbuf_free(&cur);
}

int main(void) {
  RUN(knob_table_parses_every_knob);
  RUN(ecu_watchdog_resets_the_core);
  RUN(crank_edges_per_revolution);
  RUN(crank_glitch_drop_emi_stuck_jitter);
  RUN(crank_jitter_within_documented_amplitude);
  RUN(analog_fault_modes);
  RUN(tps_shared_supply_unpowered_modes);
  RUN(et_wired_to_ecu1_only);
  RUN(batt_follows_supply_curve);
  RUN(ct_battery_and_afr_senses);
  RUN(select_switch_grounds_one_enable_and_gates_injectors);
  RUN(select_switch_break_before_make);
  RUN(plead_retard_dip_pump_digin_contacts);
  RUN(contact_bounce_chatter);
  RUN(pin_wire_faults);
  RUN(power_brownout_and_cat5_controller_supply);
  RUN(controller_slot_empty);
  RUN(mclr_reset_events);
  RUN(link_dead_cut_ber_garbage);
  RUN(link_bench_frames);
  RUN(link_noise_is_flag_free_and_paced);
  RUN(link_bench_cfg_block);
  RUN(link_bench_table_seal);
  RUN(console_input_bytes);
  RUN(fram_faults);
  RUN(sd_faults);
  RUN(rtc_osf_invalid_absent);
  RUN(rtc_epoch_rejects_out_of_range);
  RUN(crystal_ppm_targets);
  RUN(clock_check_config_words);
  RUN(bezel_script_buttons_encoder);
  RUN(panel_backlight_permille_uses_pr2_plus_1);
  RUN(encoder_bounce_switching_contact_only);
  RUN(input_script_refuses_what_it_cannot_run);
  RUN(output_observers);
  RUN(ff_edges_statistics_window);
  RUN(panel_frame_period_report);
  RUN(ff_window_survives_snapshot_from_a_windowless_run);
  RUN(observer_sfr_reads_leave_the_controller_alone);
  RUN(home_anchor_resolves_on_unblank);
  RUN(phase_anchor_link_frame_and_never_occurred);
  RUN(phase_anchor_only_frames_the_sender_sent);
  RUN(snapshot_sig_link_fault_arg3_and_cfg28);
  RUN(snapshot_sig_covers_every_stimulus_field);
  for (int k = 0; k < 2; k++) ecu_free(&E[k]);
  MT_REPORT();
}
