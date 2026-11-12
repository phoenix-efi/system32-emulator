/* Unit tests for the emulator front ends' shared observers: link_decode (wire protocol), fram_format (FRAM layout),
 * bench_engine (injector meter, air model, wideband sensor).
 *
 * Real data: tests/fixtures/link_capture.txt holds link bytes captured on the emulated wires (see its header). FRAM
 * images are bring-your-own: every *.fram file in $S32_FIRMWARE_DIR and $FRAM_DIR is checked against the documented
 * layout; with neither set those checks are reported as SKIP. */
#include "minitest.h"
#include "bench_engine.h"
#include "fram_format.h"
#include "link_decode.h"
#include <dirent.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK_NEAR(a, b, tol) CHECK(fabs((double)(a) - (double)(b)) <= (tol))

/* ---------------------------------------------------------------- link_decode */

typedef struct { unsigned type, len, wire; uint8_t p[400]; } got_frame;
static got_frame GOT[64];
static unsigned NGOT, NTEXT;
static void on_frame(void *ctx, unsigned type, const uint8_t *p, unsigned len, unsigned wire) {
  (void)ctx;
  if (NGOT < 64 && len <= sizeof GOT[0].p) { GOT[NGOT].type = type; GOT[NGOT].len = len; GOT[NGOT].wire = wire; memcpy(GOT[NGOT].p, p, len); NGOT++; }
}
static void on_text(void *ctx, uint8_t b) { (void)ctx; (void)b; NTEXT++; }

TEST(crc_check_value) {
  CHECK_EQ(link_crc16((const uint8_t *)"123456789", 9), 0x6F91);   /* CRC-16/MCRF4XX catalogue check value */
}

TEST(encode_deframe_roundtrip_with_escapes) {
  uint8_t pay[6] = {0x7E, 0x01, 0x7D, 0x02, 0x7E, 0x7D}, wire[64];
  size_t n = link_encode(12, pay, sizeof pay, wire, sizeof wire);
  CHECK(n > 0);
  CHECK_EQ(wire[0], 0x7E);
  CHECK_EQ(wire[n - 1], 0x7E);
  unsigned esc = 0;
  for (size_t i = 1; i + 1 < n; i++) { CHECK(wire[i] != 0x7E); if (wire[i] == 0x7D) esc++; }
  CHECK(esc >= 4);
  link_deframer d;
  link_deframer_init(&d, on_frame, on_text, NULL);
  NGOT = NTEXT = 0;
  const char *console = "> ctl\r\n";
  for (const char *c = console; *c; c++) link_deframe_byte(&d, (uint8_t)*c);   /* plain text before the first flag */
  CHECK_EQ(NTEXT, strlen(console));
  CHECK(!link_deframer_idle(&d));
  for (size_t i = 0; i < n; i++) link_deframe_byte(&d, wire[i]);
  CHECK_EQ(NGOT, 1);
  CHECK_EQ(GOT[0].type, 12);
  CHECK_EQ(GOT[0].len, sizeof pay);
  CHECK(memcmp(GOT[0].p, pay, sizeof pay) == 0);
  CHECK_EQ(GOT[0].wire, n);
  CHECK(link_deframer_idle(&d));
  wire[3] ^= 0x01;                                        /* corrupt one byte: CRC fails, counted bad */
  for (size_t i = 0; i < n; i++) link_deframe_byte(&d, wire[i]);
  CHECK_EQ(NGOT, 1);
  CHECK_EQ(d.bad, 1);
  CHECK(link_encode(12, pay, sizeof pay, wire, 8) == 0);  /* too small */
}

TEST(idle_only_between_frames) {
  link_deframer d;
  link_deframer_init(&d, NULL, NULL, NULL);
  link_deframe_byte(&d, 0x7E);
  CHECK(link_deframer_idle(&d));
  link_deframe_byte(&d, 0x08);
  CHECK(!link_deframer_idle(&d));
  link_deframe_byte(&d, 0x7D);
  CHECK(!link_deframer_idle(&d));
}

TEST(trim_tracker) {
  link_trim_tracker k;
  link_trim_init(&k, 0.05);
  CHECK(!k.have);
  link_trim_status(&k, 1.00, 0);
  link_trim_delta(&k, 1.20, +1);
  link_trim_delta(&k, 1.35, +1);
  CHECK_EQ(link_trim_estimate(&k), 2);
  link_trim_delta(&k, 1.48, +1);                          /* within lag_s of the next report: not inside it */
  link_trim_status(&k, 1.50, 2);                          /* report built before the third delta landed */
  CHECK_EQ(link_trim_estimate(&k), 3);
  CHECK_EQ(k.mismatches, 0);
  link_trim_status(&k, 2.00, 3);
  CHECK_EQ(link_trim_estimate(&k), 3);
  CHECK_EQ(k.mismatches, 0);
  link_trim_status(&k, 2.50, 7);                          /* someone else moved it: counted, then followed */
  CHECK_EQ(k.mismatches, 1);
  CHECK_EQ(link_trim_estimate(&k), 7);
}

/* The captured wire bytes, one direction at a time. */
typedef struct { double t; unsigned type, len, wire; uint8_t p[120]; } cap_frame;
static cap_frame CAP[64];
static unsigned NCAP;
static double CAP_T;
static void cap_frame_cb(void *ctx, unsigned type, const uint8_t *p, unsigned len, unsigned wire) {
  (void)ctx;
  if (NCAP < 64 && len <= sizeof CAP[0].p) {
    CAP[NCAP].t = CAP_T; CAP[NCAP].type = type; CAP[NCAP].len = len; CAP[NCAP].wire = wire;
    memcpy(CAP[NCAP].p, p, len); NCAP++;
  }
}
static int load_capture(const char *dir, link_deframer *d) {
  FILE *f = fopen(FIXTURES "/link_capture.txt", "r");
  if (!f) return -1;
  char line[512];
  double prev = -1.0;
  NCAP = 0;
  link_deframer_init(d, cap_frame_cb, NULL, NULL);
  while (fgets(line, sizeof line, f)) {
    if (line[0] == '#') continue;
    char dd[16];
    int off = 0;
    if (sscanf(line, "%lf %15s%n", &CAP_T, dd, &off) != 2 || strcmp(dd, dir)) continue;
    if (prev >= 0.0 && CAP_T - prev > 1.0) { d->in = false; d->n = 0; d->esc = false; }   /* next window: resync */
    prev = CAP_T;
    unsigned b;
    int used;
    for (char *p = line + off; sscanf(p, "%x%n", &b, &used) == 1; p += used) link_deframe_byte(d, (uint8_t)b);
  }
  fclose(f);
  return 0;
}

TEST(capture_ecu1_to_controller) {
  link_deframer d;
  CHECK_EQ(load_capture("ECU1->CT", &d), 0);
  CHECK_EQ(d.bad, 0);
  unsigned nstat = 0, ncfg = 0;
  link_status last = {0};
  for (unsigned i = 0; i < NCAP; i++) {
    if (CAP[i].type == LINK_E2C_STATUS) { CHECK(link_decode_status(CAP[i].p, CAP[i].len, &last)); nstat++; }
    if (CAP[i].type == 5) {                                /* configuration block: 28 x s32, settings 1..28 */
      CHECK_EQ(CAP[i].len, 112);
      int32_t c1, c2;
      CHECK(link_decode_s32(CAP[i].p, CAP[i].len, &c1));
      CHECK(link_decode_s32(CAP[i].p + 4, CAP[i].len - 4, &c2));
      CHECK_EQ(c1, 80);                                    /* vendor default magnet position */
      CHECK_EQ(c2, 4);                                     /* vendor default cylinders */
      ncfg++;
    }
  }
  CHECK_EQ(ncfg, 1);
  CHECK_EQ(nstat, 3);
  CHECK_EQ(last.fw_version, 86001);
  CHECK_EQ(last.rpm, 2400);
  CHECK_NEAR(last.inj_duty_pct, 28.05, 0.01);
  CHECK_NEAR(last.tps_pct, 55.0, 0.01);
  CHECK_EQ(last.fuel_trim_pct, -3);
  CHECK_EQ(last.map_x100, 2399);                           /* MAP input 24.0 inHg */
  CHECK_EQ(last.et_f, 185);
  CHECK_EQ(last.iat_f, 72);
  CHECK(!link_decode_status(CAP[0].p, 47, &last));
}

TEST(status_signed_et_and_iat) {
  uint8_t p[48] = {0};
  /* Cold AN7=1007 -> ET -2 F (0xFFFE), cold AN6=958 -> IAT -9 F (0xFFF7) */
  p[40] = 0xFE; p[41] = 0xFF;
  p[42] = 0xF7; p[43] = 0xFF;
  link_status st = {0};
  CHECK(link_decode_status(p, sizeof p, &st));
  CHECK_EQ(st.et_f, -2);
  CHECK_EQ(st.iat_f, -9);
  /* Open ET sensor -> 9000 F (0x2328), cold AN6=1023 -> IAT -20 F (0xFFEC) */
  p[40] = 0x28; p[41] = 0x23;
  p[42] = 0xEC; p[43] = 0xFF;
  CHECK(link_decode_status(p, sizeof p, &st));
  CHECK_EQ(st.et_f, 9000);
  CHECK_EQ(st.iat_f, -20);
}

TEST(capture_trim_deltas_and_tracker) {
  link_deframer d;
  CHECK_EQ(load_capture("CT->ECU1", &d), 0);
  CHECK_EQ(d.bad, 0);
  static cap_frame c2e[64];
  unsigned n = NCAP, n9 = 0;
  memcpy(c2e, CAP, sizeof CAP);
  for (unsigned i = 0; i < n; i++) {
    int32_t v;
    if (c2e[i].type == LINK_C2E_TRIM_DELTA) { CHECK(link_decode_s32(c2e[i].p, c2e[i].len, &v)); CHECK_EQ(v, -1); n9++; }
  }
  CHECK_EQ(n9, 3);
  CHECK_EQ(load_capture("ECU1->CT", &d), 0);
  /* replay both directions in time order through the tracker, as emu/pair does */
  link_trim_tracker k;
  link_trim_init(&k, 0.05);
  unsigned i = 0, j = 0;
  while (i < n || j < NCAP) {
    bool take_c = j >= NCAP || (i < n && c2e[i].t <= CAP[j].t);
    if (take_c) {
      int32_t v;
      if (c2e[i].type == LINK_C2E_TRIM_DELTA && link_decode_s32(c2e[i].p, c2e[i].len, &v)) link_trim_delta(&k, c2e[i].t, v);
      i++;
    } else {
      link_status st;
      if (CAP[j].type == LINK_E2C_STATUS && link_decode_status(CAP[j].p, CAP[j].len, &st))
        link_trim_status(&k, CAP[j].t - CAP[j].wire * (10.0 / 38462.0), st.fuel_trim_pct);
      j++;
    }
  }
  CHECK_EQ(k.reports, 3);
  CHECK_EQ(k.mismatches, 0);
  CHECK_EQ(link_trim_estimate(&k), -3);
}

/* ---------------------------------------------------------------- fram_format */

static void synth_table(uint8_t *raw) {
  fram_table t;
  memset(&t, 0, sizeof t);
  t.nrows = 14; t.ncols = 9;
  for (unsigned i = 0; i < 16; i++) { t.rpm[i] = (uint16_t)(i < 14 ? 50 + 200 * i : 0); t.map_x100[i] = (uint16_t)(i < 9 ? 100 + 300 * i : 0); }
  for (unsigned r = 0; r < 16; r++) for (unsigned c = 0; c < 16; c++) t.cell[r][c] = (uint8_t)(r * 16 + c);
  fram_table_encode(&t, raw);
  raw[0x142] = (uint8_t)(fram_table_checksum(raw) & 0xFF);
  raw[0x143] = (uint8_t)(fram_table_checksum(raw) >> 8);
}

TEST(fram_table_roundtrip_and_reseal) {
  uint8_t raw[FRAM_TABLE_BYTES];
  synth_table(raw);
  CHECK(fram_table_valid(raw));
  fram_table t;
  fram_table_decode(raw, &t);
  CHECK_EQ(t.nrows, 14);
  CHECK_EQ(t.rpm[13], 2650);
  CHECK_EQ(t.map_x100[8], 2500);
  CHECK_EQ(t.cell[8][5], 133);
  CHECK(fram_table_set_cell(raw, 2, 3, 200));               /* row 2 lies inside bytes 0..160 */
  CHECK(fram_table_valid(raw));
  CHECK_EQ(raw[0x42 + 2 * 16 + 3], 200);
  uint16_t before = fram_table_checksum(raw);
  CHECK(fram_table_set_cell(raw, 12, 3, 1));                /* the sum covers bytes 0..160 only: unchanged */
  CHECK_EQ(fram_table_checksum(raw), before);
  CHECK(!fram_table_set_cell(raw, 16, 0, 1));
  uint8_t zero[FRAM_TABLE_BYTES] = {0};
  CHECK_EQ(fram_table_checksum(zero), 0x00FF);
  raw[0x10] ^= 0xFF;
  CHECK(!fram_table_valid(raw));
  /* what the ECU accepts in a table write (its ACK result, link types 8/9; OBSERVED): nrows != 0, ncols != 0,
   * checksum != 0, 161-byte sum */
  synth_table(raw);
  uint16_t s322 = 0;
  for (unsigned i = 0; i < 0x142; i++) s322 = (uint16_t)(s322 + raw[i]);
  raw[0x142] = (uint8_t)(s322 & 0xFF); raw[0x143] = (uint8_t)(s322 >> 8);
  CHECK(!fram_table_valid(raw));                            /* a 322-byte sum: no match (bytes past 160 set) */
  synth_table(raw);
  raw[0] = 0;
  uint16_t s = fram_table_checksum(raw);
  raw[0x142] = (uint8_t)(s & 0xFF); raw[0x143] = (uint8_t)(s >> 8);
  CHECK(!fram_table_valid(raw));                            /* nrows == 0 rejected even with matching checksum */
  synth_table(raw);
  raw[1] = 0;
  s = fram_table_checksum(raw);
  raw[0x142] = (uint8_t)(s & 0xFF); raw[0x143] = (uint8_t)(s >> 8);
  CHECK(!fram_table_valid(raw));                            /* ncols == 0 rejected even with matching checksum */
  synth_table(raw);
  raw[0x142] = 0; raw[0x143] = 0;
  CHECK(!fram_table_valid(raw));                            /* checksum == 0 rejected */
}

static uint8_t IMG[FRAM_BYTES];
static int check_fram_dir(const char *dir, unsigned *n_ecu, unsigned *n_ct) {
  DIR *dp = dir && *dir ? opendir(dir) : NULL;
  if (!dp) return 0;
  struct dirent *de;
  while ((de = readdir(dp))) {
    size_t L = strlen(de->d_name);
    if (L < 6 || strcmp(de->d_name + L - 5, ".fram")) continue;
    char path[2048];
    snprintf(path, sizeof path, "%s/%s", dir, de->d_name);
    FILE *f = fopen(path, "rb");
    if (!f) continue;
    size_t got = fread(IMG, 1, sizeof IMG, f);
    fclose(f);
    if (got != FRAM_BYTES) { fprintf(stderr, "  %s: %zu bytes, not an FM25V10 image\n", path, got); CHECK(0); continue; }
    bool ecu = IMG[0] >= 1 && IMG[0] <= 16 && IMG[1] >= 1 && IMG[1] <= 16 && IMG[0x500] >= 1 && IMG[0x500] <= 16;
    if (ecu) {
      CHECK(fram_table_valid(IMG + FRAM_ECU_IGN_TABLE));
      CHECK(fram_table_valid(IMG + FRAM_ECU_FUEL_TABLE));
      CHECK_EQ(fram_ecu_config_sum(IMG), fram_ecu_config_sum_stored(IMG));
      int32_t mag, cyl;
      CHECK(fram_ecu_config(IMG, 1, &mag) && mag > 0 && mag < 180);
      CHECK(fram_ecu_config(IMG, 2, &cyl) && (cyl == 4 || cyl == 6));
      fram_table t;
      fram_table_decode(IMG + FRAM_ECU_FUEL_TABLE, &t);
      for (unsigned i = 1; i < t.nrows; i++) CHECK(t.rpm[i] > t.rpm[i - 1]);
      for (unsigned i = 1; i < t.ncols; i++) CHECK(t.map_x100[i] > t.map_x100[i - 1]);
      CHECK(t.map_x100[t.ncols - 1] <= 4000);                  /* hundredths of inHg */
      (*n_ecu)++;
    } else {
      CHECK_EQ(fram_ct_config_sum(IMG), fram_ct_config_sum_stored(IMG));
      (*n_ct)++;
    }
  }
  closedir(dp);
  return 1;
}

TEST(fram_real_images) {
  unsigned n_ecu = 0, n_ct = 0;
  int dirs = check_fram_dir(getenv("S32_FIRMWARE_DIR"), &n_ecu, &n_ct) + check_fram_dir(getenv("FRAM_DIR"), &n_ecu, &n_ct);
  if (!dirs || !(n_ecu + n_ct)) printf("  SKIP fram_real_images: no *.fram in $S32_FIRMWARE_DIR / $FRAM_DIR\n");
  else printf("  fram_real_images: %u ECU and %u Controller images checked\n", n_ecu, n_ct);
}

/* ---------------------------------------------------------------- bench_engine */

#define HZ 80e6
static void pulse(inj_meter *m, int src, unsigned ch, double t_open, double width_s) {
  inj_meter_edge(m, src, (uint64_t)(t_open * HZ), 0, (uint16_t)(1u << ch));
  inj_meter_edge(m, src, (uint64_t)((t_open + width_s) * HZ), (uint16_t)(1u << ch), 0);
}

TEST(air_model) {
  bench_engine e = bench_engine_six();
  /* 0.868 x 540 cu in x 2400/120 x rho(24.0 inHg, 59 F) = 150.9 g/s */
  CHECK_NEAR(bench_air_gps(&e, 2400, 24.0, 59.0), 150.9, 0.3);
  CHECK_NEAR(bench_air_gps(&e, 1200, 24.0, 59.0), 150.9 / 2, 0.2);
  CHECK(bench_air_gps(&e, 2400, 24.0, 100.0) < bench_air_gps(&e, 2400, 24.0, 59.0));
  CHECK_EQ(bench_air_gps(&e, 0, 24.0, 59.0), 0.0);
}

/* The one-time VE calibration: a six-cylinder configuration at 2400 RPM / 24.0 inHg / IAT 59 F / trim 0,
 * one ECU fuelling, gives 7.064 ms pulses on each of six channels once per revolution (OBSERVED); the gauge reads
 * AFR 13.3 there. */
TEST(six_cylinder_calibration_point) {
  bench_engine e = bench_engine_six();
  inj_meter m;
  inj_meter_init(&m, HZ, e.inj_dead_ms, e.inj_flow_lb_hr);
  double rev = 60.0 / 2400.0;
  for (int r = 0; r < 20; r++)
    for (unsigned ch = 0; ch < 6; ch++) pulse(&m, 0, ch, 1.0 + r * rev + ch * rev / 6, 7.064e-3);
  uint64_t now = (uint64_t)((1.0 + 20.5 * rev) * HZ);
  inj_meter_settle(&m, now);
  double fuel = inj_meter_fuel_gps(&m, now, 0.5);
  CHECK_NEAR(fuel, 6 * (7.064e-3 - 0.8e-3) * (60 * 453.59237 / 3600) / rev, 0.01);
  CHECK_NEAR(bench_afr(bench_air_gps(&e, 2400, 24.0, 59.0), fuel, 20.0), 13.29, 0.03);
  CHECK_NEAR(m.grams_total, 20 * 6 * (7.064e-3 - 0.8e-3) * (60 * 453.59237 / 3600), 1e-5);
}

TEST(union_of_parallel_drivers) {
  inj_meter m;
  inj_meter_init(&m, HZ, 0.8, 60.0);
  pulse(&m, 0, 0, 1.000, 5e-3);
  pulse(&m, 1, 0, 1.000, 5e-3);                           /* coincident: one injector opening */
  pulse(&m, 0, 1, 1.010, 5e-3);
  pulse(&m, 1, 1, 1.0102, 5e-3);                          /* 0.2 ms late: union 5.2 ms */
  inj_meter_settle(&m, (uint64_t)(1.02 * HZ));
  CHECK_EQ(m.union_pulses[0], 1);
  CHECK_EQ(m.union_pulses[1], 1);
  CHECK_NEAR(m.last_net_s[0], 4.2e-3, 1e-7);
  CHECK_NEAR(m.last_net_s[1], 4.4e-3, 1e-7);
}

TEST(merge_waits_for_open_pulses) {
  inj_meter m;
  inj_meter_init(&m, HZ, 0.0, 60.0);
  pulse(&m, 0, 0, 1.000, 5e-3);
  inj_meter_edge(&m, 1, (uint64_t)(1.004 * HZ), 0, 1);    /* ECU2's pulse still open: it may still extend the union */
  inj_meter_settle(&m, (uint64_t)(1.006 * HZ));
  CHECK_EQ(m.union_pulses[0], 0);
  inj_meter_edge(&m, 1, (uint64_t)(1.007 * HZ), 1, 0);
  inj_meter_settle(&m, (uint64_t)(1.008 * HZ));
  CHECK_EQ(m.union_pulses[0], 1);
  CHECK_NEAR(m.last_net_s[0], 7e-3, 1e-7);
}

TEST(dead_time_power_and_enable) {
  inj_meter m;
  inj_meter_init(&m, HZ, 0.8, 60.0);
  pulse(&m, 0, 0, 1.0, 0.5e-3);                           /* shorter than the dead time: no fuel */
  inj_meter_settle(&m, (uint64_t)(1.01 * HZ));
  CHECK_EQ(m.union_pulses[0], 1);
  CHECK_EQ(m.grams_total, 0.0);
  inj_meter_edge(&m, 1, (uint64_t)(1.02 * HZ), 0, 1);     /* ECU2 opens, then loses power: the pulse never counts */
  inj_meter_drop_source(&m, 1);
  inj_meter_edge(&m, 1, (uint64_t)(1.03 * HZ), 1, 0);
  inj_meter_settle(&m, (uint64_t)(1.04 * HZ));
  CHECK_EQ(m.union_pulses[0], 1);
  inj_meter_enable_source(&m, 0, false);                  /* injector-enable switch off: ECU1's drive is cut */
  pulse(&m, 0, 0, 1.05, 5e-3);
  inj_meter_settle(&m, (uint64_t)(1.06 * HZ));
  CHECK_EQ(m.union_pulses[0], 1);
  inj_meter_enable_source(&m, 0, true);
  pulse(&m, 0, 0, 1.07, 5e-3);
  inj_meter_settle(&m, (uint64_t)(1.08 * HZ));
  CHECK_EQ(m.union_pulses[0], 2);
  CHECK_EQ(inj_meter_fuel_gps(&m, (uint64_t)(1.08 * HZ), 0.5), inj_meter_fuel_gps(&m, (uint64_t)(1.08 * HZ), 0.5));
  CHECK_EQ(inj_meter_fuel_gps(&m, (uint64_t)(3.0 * HZ), 0.5), 0.0);   /* stale */
}

/* The injector's dead time follows its supply (the aircraft bus): longer at low voltage, so the same commanded open
 * time delivers less fuel; a fixed dead time (S32_INJ_DEAD=<ms>) ignores the supply. */
TEST(dead_time_follows_supply) {
  CHECK_NEAR(inj_dead_factor(13.8), 1.0, 1e-12);
  CHECK(inj_dead_factor(11.5) > 1.2 && inj_dead_factor(11.5) < 1.4);
  CHECK(inj_dead_factor(12.0) > inj_dead_factor(14.0));
  CHECK(inj_dead_factor(14.0) > inj_dead_factor(16.0));
  CHECK(isinf(inj_dead_factor(5.0)));
  CHECK_NEAR(inj_dead_factor(30.0), 0.69, 1e-12);
  inj_meter m;
  inj_meter_init(&m, HZ, 0.8, 60.0);
  pulse(&m, 0, 0, 1.0, 3e-3);                             /* 13.8 V: 3.0 - 0.8 ms */
  m.volts = 11.5;
  pulse(&m, 0, 1, 1.0, 3e-3);                             /* 11.5 V: longer dead time */
  m.volts = 5.0;
  pulse(&m, 0, 2, 1.0, 3e-3);                             /* below the opening voltage: no fuel */
  inj_meter_settle(&m, (uint64_t)(1.01 * HZ));
  CHECK_NEAR(m.last_net_s[0], 2.2e-3, 1e-7);
  CHECK_NEAR(m.last_net_s[1], 3e-3 - 0.8e-3 * inj_dead_factor(11.5), 1e-7);
  CHECK_EQ(m.last_net_s[2], 0.0);
  m.dead_fixed = true;
  m.volts = 11.5;
  pulse(&m, 0, 3, 1.02, 3e-3);
  inj_meter_settle(&m, (uint64_t)(1.03 * HZ));
  CHECK_NEAR(m.last_net_s[3], 2.2e-3, 1e-7);
}

TEST(plx_transfer_and_lag) {
  CHECK_EQ(plx_smafr_volts(10.0), 0.0);
  CHECK_EQ(plx_smafr_volts(20.0), 5.0);
  CHECK_EQ(plx_smafr_volts(25.0), 5.0);
  CHECK_EQ(plx_smafr_volts(8.0), 0.0);
  CHECK_NEAR(plx_smafr_afr(plx_smafr_volts(14.7)), 14.7, 1e-12);
  CHECK_EQ(bench_afr(100.0, 0.0, 20.0), 20.0);
  CHECK_NEAR(bench_lag(20.0, 10.0, 0.1, 0.1), 10.0 + 10.0 * exp(-1.0), 1e-12);
  CHECK_EQ(bench_lag(20.0, 10.0, 0.1, 0.0), 10.0);
}

int main(void) {
  RUN(crc_check_value);
  RUN(encode_deframe_roundtrip_with_escapes);
  RUN(idle_only_between_frames);
  RUN(trim_tracker);
  RUN(capture_ecu1_to_controller);
  RUN(status_signed_et_and_iat);
  RUN(capture_trim_deltas_and_tracker);
  RUN(fram_table_roundtrip_and_reseal);
  RUN(fram_real_images);
  RUN(air_model);
  RUN(six_cylinder_calibration_point);
  RUN(union_of_parallel_drivers);
  RUN(merge_waits_for_open_pulses);
  RUN(dead_time_power_and_enable);
  RUN(dead_time_follows_supply);
  RUN(plx_transfer_and_lag);
  MT_REPORT();
}
