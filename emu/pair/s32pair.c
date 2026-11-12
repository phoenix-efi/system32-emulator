/* s32pair -- the whole target system in one process: the Cockpit Controller (emu/controller, PIC24 core) and two ECUs
 * (emu/ecu, PIC32), wired as on the aircraft by the shared harness model (emu/common/s32board.c): Controller UART3 <->
 * ECU1 UART1, Controller UART4 <-> ECU2 UART1 over the cat5 cables, and every sensor, switch, supply and device that
 * model/io_ledger.json lists. Co-simulated in 100-us quanta of emulated time (well under one 260-us byte at 38,462 Bd).
 * Each MCU runs on its own thread; bytes a UART sends during a quantum go to that producer's outbox and are delivered at
 * the barrier (deterministic; <= 1 quantum of extra latency).
 *   s32pair <CT_prog.bin> <EC_bflash.bin> <EC_pflash.bin> [seconds=10]
 *   s32pair --knobs          list every harness knob (environment variables, emu/common/s32board.c)
 *
 * Harness and fault knobs (engine, sensors, switches, power, links, consoles, devices, bezel): `s32pair --knobs`.
 * Front-end options (environment):
 *      S32_SNAPDIR=<dir> one Controller screen PPM per second (S32_SNAP_MS=<n>: every n ms); S32_PPM=<file> the final screen; S32_LINKLOG=<file> every
 *      link frame both ways; S32_LINKRAW=<file> every link byte as delivered; S32_STATUSLOG=<file> every ECU status frame
 *      decoded (t, ECU, RPM, MAP x100, TPS %, trim %, advance, ET, IAT, injector / P-lead inhibit, flags); S32_CT_CONSOLE=<file>
 *      Controller UART1 console output; ECU_CONSOLE=<prefix> ECU consoles to <prefix>1.txt / <prefix>2.txt (UART1 before `ctl`,
 *      UART4 after); S32_SPARKLOG=<file> every spark (ECU, OC, on, off, crank angle); S32_INJLOG=<file> every injector pulse at
 *      the driver pin (ECU, RAn, open s, width ms);
 *      S32_STATS_FROM=<s> end-of-run spark/injector statistics count only pulses that END at or after s;
 *      S32_READHIST=1 ECU SFR read/write hot spots; S32_IOCOV=<file> I/O coverage (SFRs, pins, ADC channels; JSON).
 *      S32_SNAPSHOT_OUT=<file>@<t>[,...] / S32_SNAPSHOT_IN=<file> / S32_STIMULUS=<file>: snapshots (boot once; the section
 *      "snapshots" below, emu/common/s32snap.h, docs/conformance.md).
 *      S32_TRACE=<file> the canonical conformance trace (emu/common/s32trace.h: every output pin, every UART byte, every
 *      byte delivered over the cat5 pairs, a framebuffer hash per displayed frame, FRAM / SD / flash hashes at the end).
 *      S32_SERIAL=1 run the three MCUs one after another on one thread instead of one thread each (same results; the
 *      WebAssembly build always runs this way).
 *      The text logs above are written between quanta: lines produced on the MCU threads during a quantum are buffered
 *      per thread and merged in emulated-time order (ties: Controller, ECU1, ECU2), so they never tear or reorder.
 * No boot region (application start, emu/ecu/src/ecu.c): `none` as the boot flash argument and an application-only
 *      image (the application window of program flash from physical 0x1D006000, 499,700 bytes: ECU_APP_FLAT_BYTES) as the
 *      program flash argument; S32_ECU_APP_START=1 drops a given boot flash; S32_ECU_BOOT_CYCLES=<n> holds each ECU n SYSCLK cycles before the hand-off (default 16379, the hand-off time
 *      of a full image with no card; 0 starts the application at once). `none` as the Controller argument leaves its slot empty (no Controller on the bench).
 * Program flash: S32_CT_PFLASH / S32_ECU1_PFLASH / S32_ECU2_PFLASH=<image>|blank per MCU (default: the image argument;
 *      blank = erased 0xFF, boot code only on ECUs); each MCU's flash keeps what was erased and programmed across power
 *      cycles and resets; S32_CT_PFLASH_OUT / S32_ECU1_PFLASH_OUT / S32_ECU2_PFLASH_OUT=<file> read it back at exit (as a
 *      device programmer would).
 * Exit status: 0 = ran; 2 = bad arguments or setup; 3 = an ECU could not be restarted; 4 = an internal emulator error
 *      (ecu_is_emu_err, e.g. the engine's hook table full) stopped an ECU: the run ends after that quantum and prints
 *      "emulator error: ...". A guest CPU fault (invalid instruction; unmapped, unaligned or protected access; SYSCALL,
 *      BREAK, trap) only stops that ECU's core (on the bench that ECU goes silent), and shows in its err=.
 * Storage: S32_CT_SD=<FAT32 image> in the Controller's slot (SPI1, CS RE2, detect RE3, write protect RE4),
 *      S32_ECU1_SD / S32_ECU2_SD=<image> in each ECU's slot (SPI4, CS RF12, detect RB3, write enable RB4); images are
 *      written back on exit if changed. S32_CT_FRAM / S32_ECU1_FRAM / S32_ECU2_FRAM=<file> FRAM images (layout:
 *      model/s32_fram.ksy; blank otherwise), S32_CT_FRAM_OUT / S32_ECU1_FRAM_OUT / S32_ECU2_FRAM_OUT=<file> saved at exit.
 * Inputs from outside: S32_XPLANE=<host>[:port] live X-Plane feed (RREF, paced to wall time): engine RPM, MAP, OAT
 *      (IAT), oil temperature (ET channel, INFERRED), throttle (TPS), bus volts (ECU supplies and the Controller's battery
 *      senses), ignition key (P-lead switches: L = ECU1, R = ECU2, INFERRED). S32_REPLAY=<file> replays a real
 *      Datalog.CSV block (prepared into the row format below): one row per emulated second, UNPACED. Row 0 starts the
 *      engine at S32_REPLAY_START (default 8 s); row k >= 1 is applied at S32_REPLAY_START + k + S32_REPLAY_PHASE
 *      (default 0.64 s: the first Datalog.CSV row lands 1.54 s after row 0 starts the engine -- OBSERVED). A row =
 *      crank RPM, per-ECU raw ADC counts AN5 MAP, AN6 IAT, AN7 ET, AN9 TPS, AN11 BATT, per-ECU switch wires (bit1 RC13
 *      injector enable open, bit0 RB2 P-lead open), the Controller's own BATT1/2 ADC (AN10/AN11) and the fuel-trim
 *      target %. Fuel trim goes through the Controller's encoder input, as a pilot turning the knob would: from
 *      t = 2 s, every S32_REPLAY_TRIM_MS (default 150) one single edge (+-1 %) toward the target while ECU1's trim
 *      differs. ECU1's trim is read off the link like a protocol analyser would (emu/common/link_decode.c, link_trim_tracker;
 *      S32_TRIM_LAG_MS overrides the build-to-wire allowance). */
#include "s32ct.h"
#include "s32dev.h"
#include "s32board.h"
#include "ecu.h"
#include "xp_rref.h"
#include "link_decode.h"
#include "s32trace.h"
#include "s32panel.h"
#include "s32snap.h"
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>

#define QUANTUM_US 100

static uint8_t *slurp(const char *p, size_t *n) {
  FILE *f = fopen(p, "rb"); if (!f) { perror(p); exit(2); }
  fseek(f, 0, SEEK_END); long l = ftell(f); fseek(f, 0, SEEK_SET);
  uint8_t *b = malloc((size_t)l); if (!b) { perror("malloc"); exit(2); }
  if (fread(b, 1, (size_t)l, f) != (size_t)l) { perror("read"); exit(2); }
  fclose(f); *n = (size_t)l; return b;
}

static s32board B;
static s32trace TR;                                        /* S32_TRACE */

/* ---- text logs written from the MCU threads: per-thread line buffers, merged between quanta ---- */
typedef struct { double t; size_t off, len; } tl_line;
typedef struct { s32_tbuf txt; tl_line *ln; size_t n, cap; } tl_src;
typedef struct { FILE *f; tl_src s[3]; } tlog;            /* s[SB_CT / SB_E1 / SB_E2]: one writer thread each */
static tlog LINKLOG, STATUSLOG, SPARKLOG, INJLOG;
static void tlog_printf(tlog *L, int who, double t, const char *fmt, ...) __attribute__((format(printf, 4, 5)));
static void tlog_printf(tlog *L, int who, double t, const char *fmt, ...) {
  if (!L->f) return;
  tl_src *s = &L->s[who];
  char tmp[512];
  va_list ap; va_start(ap, fmt); int n = vsnprintf(tmp, sizeof tmp, fmt, ap); va_end(ap);
  if (n < 0) return;
  if ((size_t)n >= sizeof tmp) n = (int)sizeof tmp - 1;
  if (s->n == s->cap) {
    s->cap = s->cap ? 2 * s->cap : 256;
    tl_line *nl = realloc(s->ln, s->cap * sizeof *nl); if (!nl) { perror("realloc"); exit(2); }
    s->ln = nl;
  }
  s->ln[s->n++] = (tl_line){t, s->txt.n, (size_t)n};
  s32_tbuf_put(&s->txt, tmp, (size_t)n);
}
static void tlog_flush(tlog *L) {                          /* between quanta (MCU threads parked) */
  if (!L->f) return;
  size_t i[3] = {0, 0, 0};
  for (;;) {
    int pick = -1;
    for (int w = 0; w < 3; w++)
      if (i[w] < L->s[w].n && (pick < 0 || L->s[w].ln[i[w]].t < L->s[pick].ln[i[pick]].t)) pick = w;
    if (pick < 0) break;
    const tl_line *l = &L->s[pick].ln[i[pick]++];
    fwrite(L->s[pick].txt.p + l->off, 1, l->len, L->f);
  }
  for (int w = 0; w < 3; w++) { L->s[w].n = 0; L->s[w].txt.n = 0; }
}
static void tlog_close(tlog *L) {
  if (!L->f) return;
  tlog_flush(L); fclose(L->f); L->f = NULL;
  for (int w = 0; w < 3; w++) { s32_tbuf_free(&L->s[w].txt); free(L->s[w].ln); }
}

/* ---- link analysers (emu/common/link_decode.c), one per direction per ECU ---- */
static FILE *LINKRAW;
static bool LINKLOG_FULL;   /* S32_LINKLOG_FULL=1: log whole frame bodies plus the 2 CRC bytes (default: first 20 body bytes) */
static double NOW_S;
#define LINK_BYTE_S (10.0 / 38462.0)                       /* one 8N1 byte on the wire at the link's 38,462 Bd */
typedef struct { link_deframer d; const char *dir; int ecu; bool to_ecu; } link_tap;
static link_trim_tracker TRK;                              /* ECU1's live trim, from the wire (replay trim follower) */
#define TRIM_LAG_S 0.050   /* link_trim_tracker allowance (OBSERVED: no disagreements at 50 ms, a few at 0-30 and 100 ms) */
typedef struct { double t; int32_t v; bool status; } trk_ev;
typedef struct { trk_ev ev[64]; unsigned n; } trk_q;
static trk_q TRKQ[2];                                      /* [0] Controller -> ECU1 deltas, [1] ECU1 status */
static link_status LAST_ST[2]; static bool HAVE_ST[2];      /* each ECU's newest status frame (what the bench observes) */
static void on_link_frame(void *ctx, unsigned t, const uint8_t *p, unsigned len, unsigned wire) {
  const link_tap *tap = ctx;
  int who = tap->to_ecu ? SB_CT : SB_E1 + tap->ecu;          /* the thread this deframer runs on */
  if (LINKLOG.f) {
    char hx[3 * 700 + 1]; size_t o = 0;
    unsigned nlog = LINKLOG_FULL ? len + 2u : (len < 20u ? len : 20u);   /* p[len], p[len+1] = the frame's CRC */
    for (unsigned i = 0; i < nlog && o + 4 < sizeof hx; i++) o += (size_t)snprintf(hx + o, sizeof hx - o, " %02X", p[i]);
    hx[o] = '\0';
    tlog_printf(&LINKLOG, who, NOW_S, "%9.4f %s type %u len %u:%s\n", NOW_S, tap->dir, t, len, hx);
  }
  link_status st;
  if (!tap->to_ecu && t == LINK_E2C_STATUS && link_decode_status(p, len, &st)) {
    LAST_ST[tap->ecu] = st; HAVE_ST[tap->ecu] = true;
    tlog_printf(&STATUSLOG, who, NOW_S, "%.4f %d %u %u %.2f %d %d %d %d %u %u %u\n", NOW_S, tap->ecu + 1, (unsigned)st.rpm,
                           (unsigned)st.map_x100, (double)st.tps_pct, (int)st.fuel_trim_pct, (int)st.ign_adv_deg, (int)st.et_f,
                           (int)st.iat_f, (unsigned)st.inj_inhibit, (unsigned)st.plead_inhibit, (unsigned)st.status_flags);
  }
  if (tap->ecu != 0) return;
  /* Runs on the thread that drives this direction (Controller: main; ECU1: its own), so events are queued per direction
   * and applied in time order at the quantum barrier (trk_drain). */
  int32_t v;
  trk_q *qq = &TRKQ[tap->to_ecu ? 0 : 1];
  if (qq->n >= sizeof qq->ev / sizeof qq->ev[0]) return;
  if (tap->to_ecu && t == LINK_C2E_TRIM_DELTA && link_decode_s32(p, len, &v)) {
    qq->ev[qq->n].t = NOW_S; qq->ev[qq->n].v = v; qq->ev[qq->n].status = false; qq->n++;
  } else if (!tap->to_ecu && t == LINK_E2C_STATUS && link_decode_status(p, len, &st)) {
    qq->ev[qq->n].t = NOW_S - wire * LINK_BYTE_S; qq->ev[qq->n].v = st.fuel_trim_pct; qq->ev[qq->n].status = true; qq->n++;
  }
}
static void trk_drain(void) {                              /* between quanta: ECU threads parked */
  trk_q *a = &TRKQ[0], *b = &TRKQ[1];
  unsigned i = 0, j = 0;
  while (i < a->n || j < b->n) {
    bool take_a = j >= b->n || (i < a->n && a->ev[i].t <= b->ev[j].t);
    const trk_ev *e = take_a ? &a->ev[i++] : &b->ev[j++];
    if (e->status) link_trim_status(&TRK, e->t, e->v);
    else link_trim_delta(&TRK, e->t, e->v);
  }
  a->n = b->n = 0;
}

/* ---- output capture at the ECU pins: coil pulses (OC1-4) and injector drivers (PORTA) ---- */
static struct {
  struct { uint64_t n; double dwell_ms_sum, adv_sum, adv_min, adv_max, own_sum, own_min, own_max; } coil[2][4];
  struct { uint64_t n; double width_ms_sum; uint64_t open_at[16]; } inj[2][16];
  float widths[2][200000]; unsigned nw[2];
  uint64_t p0[2][4096][2]; unsigned n0[2];                  /* RA0 pulses (open, close) per ECU: two-live-ECU union */
  int own_ev[2][4], votes[2][4][8], nvotes[2][4];          /* each coil's learned cylinder event (+1; 0 = not yet) */
} ENG;
static double STATS_FROM;
static int cmpf(const void *a, const void *b) { float x = *(const float *)a, y = *(const float *)b; return (x > y) - (x < y); }
static void on_coil(void *ctx, ecu_t *e, unsigned oc, uint64_t on, uint64_t off) {
  int k = (int)(intptr_t)ctx;
  double toff = off / e->hz, a = s32board_angle(&B, toff);
  tlog_printf(&SPARKLOG, SB_E1 + k, toff, "%d %u %.6f %.6f %.3f\n", k + 1, oc, on / e->hz, toff, fmod(a, 360.0));
  if (B.trace) s32trace_coil(B.trace, SB_E1 + k, oc, on, off);
  s32board_on_coil(&B, k, toff);
  double sp = 720.0 / B.ncyl, adv = sp - fmod(a, sp);          /* degrees before the next TDC */
  if (adv >= sp) adv -= sp;
  /* Advance before the coil's OWN cylinder, as a technician who knows which plug each coil feeds would measure it: each
   * coil's firing event (and its wasted-spark partner 360 deg on) is learned from its first 8 sparks; afterwards a spark
   * landing near some other cylinder's TDC shows here, while the plain advance above would still look normal. */
  int nev = (int)llround(360.0 / sp), ev_next = ((int)floor(a / sp) + 1) % nev;   /* the TDC this spark precedes */
  int *own_ev = &ENG.own_ev[k][oc - 1], *votes = ENG.votes[k][oc - 1];
  if (!*own_ev) {                                              /* learn the coil's cylinder from its first 8 sparks */
    votes[ev_next & 7]++;
    if (++ENG.nvotes[k][oc - 1] == 8) { int best = 0; for (int i = 1; i < 8; i++) if (votes[i] > votes[best]) best = i; *own_ev = best + 1; }
  }
  double own = adv;
  if (*own_ev) {
    own = fmod(fmod((*own_ev - 1) * sp - a, 360.0) + 360.0, 360.0);
    if (own > 300.0) own -= 360.0;                             /* a late spark reads as negative (after TDC) */
  }
  if (toff < STATS_FROM) return;                               /* S32_STATS_FROM window (SPARKLOG above logs all) */

  typeof(ENG.coil[0][0]) *c = &ENG.coil[k][oc - 1];
  if (!c->n) { c->adv_min = c->adv_max = adv; c->own_min = c->own_max = own; }
  c->n++; c->dwell_ms_sum += (off > on ? (off - on) : 0) / e->hz * 1e3; c->adv_sum += adv; c->own_sum += own;
  if (adv < c->adv_min) c->adv_min = adv;
  if (adv > c->adv_max) c->adv_max = adv;
  if (own < c->own_min) c->own_min = own;
  if (own > c->own_max) c->own_max = own;
}
static void on_porta(void *ctx, ecu_t *e, uint64_t cyc, uint16_t old, uint16_t neu) {
  int k = (int)(intptr_t)ctx;
  for (int b = 0; b < 16; b++) {
    bool was = (old >> b) & 1u, now = (neu >> b) & 1u;
    if (!was && now) ENG.inj[k][b].open_at[0] = cyc;
    if (was && !now && ENG.inj[k][b].open_at[0]) {
      double w = (cyc - ENG.inj[k][b].open_at[0]) / e->hz * 1e3;
      tlog_printf(&INJLOG, SB_E1 + k, cyc / e->hz, "%d %d %.6f %.4f\n", k + 1, b, ENG.inj[k][b].open_at[0] / e->hz, w);
      if (cyc / e->hz < STATS_FROM) { ENG.inj[k][b].open_at[0] = 0; continue; }
      if (b == 0 && ENG.n0[k] < 4096) { ENG.p0[k][ENG.n0[k]][0] = ENG.inj[k][b].open_at[0]; ENG.p0[k][ENG.n0[k]][1] = cyc; ENG.n0[k]++; }
      if (ENG.nw[k] < 200000) ENG.widths[k][ENG.nw[k]++] = (float)w;
      ENG.inj[k][b].n++; ENG.inj[k][b].width_ms_sum += w;
      ENG.inj[k][b].open_at[0] = 0;
    }
  }
}

/* ---- real-datalog replay (S32_REPLAY): device-level inputs, one row per emulated second ---- */
typedef struct { float rpm; uint16_t an[2][5], ct_batt[2]; uint8_t sw[2]; int8_t trim; } rp_row;
static const unsigned RP_CH[5] = {5, 6, 7, 9, 11};         /* MAP, IAT, ET, TPS, BATT */
static rp_row *RP; static long RP_N, RP_APPLIED = -1;
static double RP_START = 8.0, RP_PHASE = 0.64, RP_TRIM_S = 0.15, RP_NEXT_EDGE;
static int RP_TRIM_CUR;
static uint64_t RP_EDGES;
static long rp_row_at(double t) {
  if (t < RP_START) return -1;
  double k = floor(t - RP_START - RP_PHASE);
  return k < 0 ? 0 : k >= RP_N ? -1 : (long)k;
}
static void rp_load(const char *path) {
  FILE *f = fopen(path, "r"); if (!f) { perror(path); exit(2); }
  char line[512]; long cap = 4096; RP = malloc(cap * sizeof *RP); if (!RP) { perror("malloc"); exit(2); }
  while (fgets(line, sizeof line, f)) {
    if (line[0] == '#' || line[0] == '\n') continue;
    rp_row r; unsigned a[10], s1, s2, b1, b2; int tr; float rpm;
    if (sscanf(line, "%f %u %u %u %u %u %u %u %u %u %u %u %u %d %u %u", &rpm, &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6],
               &a[7], &a[8], &a[9], &s1, &s2, &tr, &b1, &b2) != 16) { fprintf(stderr, "S32_REPLAY: bad line: %s", line); exit(2); }
    r.rpm = rpm; for (int i = 0; i < 5; i++) { r.an[0][i] = (uint16_t)a[i]; r.an[1][i] = (uint16_t)a[5 + i]; }
    r.ct_batt[0] = (uint16_t)b1; r.ct_batt[1] = (uint16_t)b2;
    r.sw[0] = (uint8_t)s1; r.sw[1] = (uint8_t)s2; r.trim = (int8_t)tr;
    if (RP_N == cap) { cap *= 2; rp_row *nr = realloc(RP, cap * sizeof *RP); if (!nr) { perror("realloc"); exit(2); } RP = nr; }
    RP[RP_N++] = r;
  }
  fclose(f);
  if (!RP_N) { fprintf(stderr, "S32_REPLAY: no rows\n"); exit(2); }
}
static void rp_apply_row(long k) {                        /* row k's sensors + switch wires -> the harness */
  for (int e = 0; e < 2; e++) {
    for (int i = 0; i < 5; i++) s32board_set_an(&B, SB_E1 + e, RP_CH[i], RP[k].an[e][i]);
    s32board_set_switches(&B, e, RP[k].sw[e] & 1, (RP[k].sw[e] >> 1) & 1);
  }
  s32board_set_an(&B, SB_CT, 10, RP[k].ct_batt[0]); s32board_set_an(&B, SB_CT, 11, RP[k].ct_batt[1]);
}
static void rp_step(double now) {                          /* between quanta (ECU threads parked) */
  long k = rp_row_at(now);
  s32board_set_rpm(&B, k < 0 ? 0.0 : RP[k].rpm);
  if (k >= 0 && k != RP_APPLIED) { rp_apply_row(k); RP_APPLIED = k; }
  if (now >= 2.0 && now >= RP_NEXT_EDGE) {                 /* trim follower: one single encoder edge per step */
    long tk = k < 0 ? (now < RP_START ? 0 : -1) : k;
    int32_t seen = link_trim_estimate(&TRK);               /* ECU1's trim as the link reports it (status + deltas since) */
    RP_TRIM_CUR = seen;
    if (TRK.have && tk >= 0 && RP[tk].trim != seen) {
      s32board_encoder_edge(&B, now, RP[tk].trim > seen ? -1 : 1);   /* OBSERVED: a NEGATIVE edge (Gray order) raises the trim */
      RP_EDGES++;
    }
    RP_NEXT_EDGE = now + RP_TRIM_S;
  }
}

/* ---- X-Plane feed ---- */
enum { XP_RPM_I, XP_MAP_I, XP_THR_I, XP_OAT_I, XP_OIL_I, XP_VOLT_I, XP_KEY_I };
static xp_rref XP; static bool XP_ON;
static double wall_now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec / 1e9; }
static uint16_t clamp10(double v) { return (uint16_t)(v < 0 ? 0 : v > 1023 ? 1023 : v + 0.5); }
static void xp_apply(void) {                               /* X-Plane values -> sensors through the harness (between quanta) */
  if (XP.have[XP_RPM_I]) s32board_set_rpm(&B, XP.val[XP_RPM_I] > 1 ? XP.val[XP_RPM_I] : 0.0);
  for (int k = 0; k < 2; k++) {
    int w = SB_E1 + k;
    if (XP.have[XP_MAP_I]) s32board_set_an(&B, w, 5, clamp10((XP.val[XP_MAP_I] - 3.06) / 0.02674));               /* MAP inHg */
    if (XP.have[XP_OAT_I]) s32board_set_an(&B, w, 6, clamp10((245.0 - (XP.val[XP_OAT_I] * 1.8 + 32)) * 3.76));     /* IAT degF */
    if (XP.have[XP_OIL_I] && (k == 0 || B.et_both)) s32board_set_an(&B, w, 7, clamp10((265.0 - (XP.val[XP_OIL_I] * 1.8 + 32)) * 3.76));
    if (XP.have[XP_THR_I]) s32board_set_an(&B, w, 9, clamp10(166.0 + XP.val[XP_THR_I] * 800.0));                  /* TPS */
    if (XP.have[XP_VOLT_I]) s32board_set_supply(&B, k, XP.val[XP_VOLT_I], !B.cut[k]);                               /* ECU feed */
    if (XP.have[XP_KEY_I]) {                                  /* 0 off, 1 L, 2 R, 3 both, 4 start */
      int key = (int)(XP.val[XP_KEY_I] + 0.5);
      bool live = key >= 3 || (key == 1 && k == 0) || (key == 2 && k == 1);
      s32board_set_plead(&B, k, live);
    }
  }
  if (XP.have[XP_VOLT_I]) B.batt_v[0] = B.batt_v[1] = XP.val[XP_VOLT_I];
}

static s32ct CT;
static ecu_t ECU[2];
static link_tap DF_CT2E[2] = {{.dir = "CT->ECU1", .ecu = 0, .to_ecu = true}, {.dir = "CT->ECU2", .ecu = 1, .to_ecu = true}},
                DF_E2CT[2] = {{.dir = "ECU1->CT", .ecu = 0}, {.dir = "ECU2->CT", .ecu = 1}};
static FILE *CTCON, *ECON[2];

static link_deframer RX_C2E[2], RX_E2C[2];                         /* analysers at the RECEIVING ends (after the cable) */
/* Table writes (types 5/7) that crossed the cable to an ECU, and its table-write ACKs (types 8/9) that crossed back.
 * A write the ECU does not acknowledge shows only here: the frame arrived intact and no ACK came back. */
static uint64_t TBL_WR[2][2], TBL_ACK[2][2], TBL_REF[2][2];       /* [ecu][0 = ignition, 1 = fuel]; REF = ACKs whose
                                                                    * s32 result is not 1: the ECU answered, refusing */
static double TBL_WR_LAST[2][2];                                   /* end time of the newest write, per ECU and table */
static uint8_t TBL_OPEN[2][2];                                     /* the newest write is not answered yet */
static void rx_c2e_frame(void *ctx, unsigned t, const uint8_t *p, unsigned len, unsigned wire) {
  (void)p; (void)len; (void)wire;
  int k = (int)(intptr_t)ctx;
  if (t == LINK_C2E_WRITE_FUEL) { TBL_WR[k][1]++; TBL_WR_LAST[k][1] = NOW_S; TBL_OPEN[k][1] = 1; }
  else if (t == LINK_C2E_WRITE_IGN) { TBL_WR[k][0]++; TBL_WR_LAST[k][0] = NOW_S; TBL_OPEN[k][0] = 1; }
}
static void rx_e2c_frame(void *ctx, unsigned t, const uint8_t *p, unsigned len, unsigned wire) {
  (void)wire;
  int k = (int)(intptr_t)ctx;
  if (t != LINK_E2C_ACK_FUEL && t != LINK_E2C_ACK_IGN) return;
  int f = t == LINK_E2C_ACK_FUEL;
  TBL_ACK[k][f]++; TBL_OPEN[k][f] = 0;
  if (len < 4 || (p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24) != 1u) TBL_REF[k][f]++;   /* 1 = accepted */
}
typedef struct { uint8_t b[8192]; unsigned n; } outbox;
static outbox OB_CT2E[2], OB_E2CT[2];                               /* one writer each (its producer's thread) */
static void ob_put(outbox *o, uint8_t b) { if (o->n < sizeof o->b / 2) o->b[o->n++] = b; }   /* half left for garbage */

static void link_raw(const char *dir, const uint8_t *b, unsigned n) {   /* S32_LINKRAW: wire bytes as delivered */
  if (!LINKRAW || !n) return;
  fprintf(LINKRAW, "%.4f %s", NOW_S, dir);
  for (unsigned i = 0; i < n; i++) fprintf(LINKRAW, " %02X", b[i]);
  fputc('\n', LINKRAW);
}
static void ct_tx(void *ctx, unsigned uart, uint8_t b) {           /* Controller UART1 -> console; UART3/4 -> ECU1/2 UART1 */
  (void)ctx;
  if (B.trace) s32trace_tx(B.trace, SB_CT, CT.cpu.cycles, uart, b);
  if (uart == 1 && CTCON) fputc(b, CTCON);
  if (uart != 3 && uart != 4) return;
  int k = uart == 3 ? 0 : 1;
  link_deframe_byte(&DF_CT2E[k].d, b);
  ob_put(&OB_CT2E[k], b);
}
static void ecu_tx(void *ctx, ecu_t *e, unsigned uart, uint8_t b) { /* ECU UART1 -> Controller; UART4 -> console */
  int k = (int)(intptr_t)ctx;
  if (B.trace) s32trace_tx(B.trace, SB_E1 + k, e->cyc, uart, b);
  if (uart == 1) { link_deframe_byte(&DF_E2CT[k].d, b); ob_put(&OB_E2CT[k], b); if (ECON[k] && !DF_E2CT[k].d.in) fputc(b, ECON[k]); }
  else if (uart == 4 && ECON[k]) fputc(b, ECON[k]);
}

/* ---- the MCUs and their boards' devices ---- */
static uint8_t *CT_IMG, *BFL, *PFL; static size_t NCT, NB, NP;
static uint8_t *CT_PFL;                                    /* Controller's program flash chip (persists over power cycles) */
static uint8_t *PFLK[2]; static size_t NPK[2];            /* each ECU's program flash chip (persists over power cycles) */
static int EMU_ERR[2]; static uint32_t EMU_ERR_PC[2];     /* internal emulator error on ECUk (ecu_is_emu_err) */
static s32_fram CT_FRAM, ECU_FRAM[2];
static s32_rtc RTC;
static s32_sd CT_SD, ECU_SD[2];
static uint8_t *SD_IMG[3]; static size_t SD_N[3]; static const char *SD_PATH[3];
static bool NOIDLESKIP;

static void ct_setup(void) {
  s32ct_spi_attach(&CT, 2, (s32_pin){6, 7}, s32_fram_dev(&CT_FRAM));   /* FM25V10 on SPI2, CS RG7 */
  s32ct_spi_attach(&CT, 2, (s32_pin){5, 3}, s32_rtc_dev(&RTC));        /* PCF2127 on SPI2, CS RF3 */
  if (SD_IMG[SB_CT]) s32ct_spi_attach(&CT, 1, (s32_pin){4, 2}, s32_sd_dev(&CT_SD));   /* SD on SPI1, CS RE2 */
  for (unsigned u = 1; u <= 4; u++) s32ct_uart_sink(&CT, u, ct_tx, NULL);
  s32ct_idle_skip(&CT, !NOIDLESKIP);                                   /* idle loops, found by watching execution */
}
static void ecu_setup(int k) {
  ecu_spi_attach(&ECU[k], 2, 6, 9, s32_fram_dev(&ECU_FRAM[k]));        /* FM25V10 on SPI2, CS RG9 */
  if (SD_IMG[SB_E1 + k]) ecu_spi_attach(&ECU[k], 4, 5, 12, s32_sd_dev(&ECU_SD[k]));   /* SD on SPI4, CS RF12 */
  ecu_uart_sink(&ECU[k], 1, ecu_tx, (void *)(intptr_t)k);
  ecu_uart_sink(&ECU[k], 4, ecu_tx, (void *)(intptr_t)k);
  ecu_idle_skip(&ECU[k], !NOIDLESKIP);                                 /* idle loops, found by watching execution */
  if (getenv("S32_READHIST") && !ECU[k].read_hist) { ECU[k].read_hist = calloc(ECU_SFR_SIZE / 4, sizeof(uint32_t)); ECU[k].write_hist = calloc(ECU_SFR_SIZE / 4, sizeof(uint32_t)); }
  ECU[k].rng ^= (uint32_t)(k + 1) * 0x85EBCA6Bu;
  ECU[k].oc_sink = on_coil; ECU[k].porta_sink = on_porta; ECU[k].out_ctx = (void *)(intptr_t)k;
  { const char *bc = getenv("S32_ECU_BOOT_CYCLES"); if (bc && *bc) ECU[k].boot_cycles = strtoull(bc, NULL, 0); }
}
/* A reset the Controller's core performed itself (illegal opcode / BREAK without a debugger, RESET instruction, trap
 * conflict), reported as it happens like the harness's MCLR and power events. */
static void ct_reset_note(void) {
  pic24_status w = CT.cpu.last_reset;
  if (w == PIC24_STOP_ILLEGAL)
    fprintf(stderr, "Controller illegal-opcode reset (RCON.IOPUWR) at t=%.3f s: opcode %06X at %06X\n", NOW_S,
            CT.cpu.stop_insn, CT.cpu.last_reset_pc);
  else if (w == PIC24_STOP_RESET || w == PIC24_STOP_TRAP_CONFLICT)
    fprintf(stderr, "Controller %s reset at t=%.3f s (PC %06X)\n", w == PIC24_STOP_TRAP_CONFLICT ? "trap-conflict" : "RESET-instruction",
            NOW_S, CT.cpu.last_reset_pc);
  else fprintf(stderr, "Controller reset counter changed with no reset cause (%d) at t=%.3f s: emulator defect\n", (int)w, NOW_S);
}
/* The harness asks for a (re)boot: cold = power restored, else an MCLR pulse. FRAM, SD and RTC keep their contents. */
static void host_boot(void *ctx, int who, bool cold) {
  (void)ctx;
  if (who == SB_CT) {
    uint64_t cyc = CT.cpu.cycles;
    if (cold) { s32ct_pflash_read(&CT, CT_PFL, S32CT_PFL_BYTES); s32ct_init(&CT, CT_PFL, S32CT_PFL_BYTES); ct_setup(); }
    else s32ct_reset(&CT);
    CT.cpu.cycles = cyc;                                     /* emulated time goes on (the RTC runs from it) */
  } else {
    int k = who - 1;
    uint64_t cyc = (uint64_t)(B.now * s32board_hz(&B, who));
    NPK[k] = ecu_pflash_read(&ECU[k], PFLK[k], ECU_PFL_BYTES);         /* the flash chip keeps what was programmed */
    ecu_free(&ECU[k]); s32_fram_dev(&ECU_FRAM[k]).select(&ECU_FRAM[k], false);
    if (ecu_init(&ECU[k], BFL, NB, PFLK[k], NPK[k])) { fprintf(stderr, "ecu_init reboot %d failed\n", k + 1); exit(3); }
    ECU[k].cyc = cyc;
    ecu_setup(k);
  }
  s32board_attach(&B);
}
#define MAX_SNAP_FILES 64
static char SNAP_FILES[MAX_SNAP_FILES][256]; static unsigned N_SNAP_FILES;   /* bezel "snap:" files written so far */
static void host_snap(void *ctx, const char *name) {
  (void)ctx; char fn[256]; snprintf(fn, sizeof fn, "%s.ppm", name);
  if (N_SNAP_FILES < MAX_SNAP_FILES) snprintf(SNAP_FILES[N_SNAP_FILES++], sizeof SNAP_FILES[0], "%s", fn);
  s32_panel_ppm(&CT, B.on[SB_CT], fn); fprintf(stderr, "snap %s at t=%.2f\n", fn, NOW_S);
}
static uint32_t ct_sfr_val(void *ctx, uint32_t a) { return s32ct_sfr_value(ctx, (uint16_t)a); }
static uint32_t ecu_sfr_val(void *ctx, uint32_t a) { return ecu_sfr_value(ctx, a); }
static void load_into(uint8_t *dst, size_t cap, const char *env) {
  const char *p = getenv(env);
  if (!p || !*p) return;
  size_t n; uint8_t *img = slurp(p, &n); memcpy(dst, img, n < cap ? n : cap); free(img);
}

/* ---- per-quantum barrier: main bumps GO; each ECU thread runs one quantum and bumps DONE ---- */
static atomic_long GO, DONE;
static atomic_bool QUIT;
static long Q0;                /* quanta already run when the loop starts (> 0 after S32_SNAPSHOT_IN) */
static double THREAD_CPU[3];   /* CT, ECU1, ECU2: CPU seconds spent emulating (not spinning at the barrier) */
static double tcpu(void) { struct timespec t = {0, 0}; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t); return t.tv_sec + t.tv_nsec / 1e9; }
static void spin_until(atomic_long *v, long target) {
  for (unsigned i = 0; atomic_load_explicit(v, memory_order_acquire) < target; i++) if (i > 2000) sched_yield();
}
static void ecu_quantum(int k, long q) {
  double c0 = tcpu();
  uint64_t target = s32board_target(&B, SB_E1 + k, q);
  if (!B.on[SB_E1 + k] || ECU[k].last_err) ECU[k].cyc = target;    /* unpowered or core stopped: time passes */
  else ecu_run_to(&ECU[k], target);
  THREAD_CPU[1 + k] += tcpu() - c0;
}
static void *ecu_thread(void *arg) {
  int k = (int)(intptr_t)arg;
  for (long gen = Q0 + 1;; gen++) {
    spin_until(&GO, gen);
    if (atomic_load(&QUIT)) return NULL;
    ecu_quantum(k, gen);
    atomic_fetch_add_explicit(&DONE, 1, memory_order_release);
  }
}

/* ---- snapshots (emu/common/s32snap.h): S32_SNAPSHOT_OUT, S32_SNAPSHOT_IN, S32_STIMULUS ----
 * S32_SNAPSHOT_OUT=<file>@<t>[,<file>@<t>...] writes the whole system's state at the end of the quantum that ends at t
 * (rounded to a quantum; t may be "{H+x}", x after the home page the run detects); S32_SNAPSHOT_IN=<file> starts this run from such a state instead of from power-on, with this
 * run's own knobs from t + S32SNAP_GUARD_S on; S32_STIMULUS=<file> writes what this run's knobs set up (JSON: the static
 * part's hash and every timed item) and exits without running, so a runner can tell which runs share a start. The
 * logs and the trace a restored run writes include what the snapshot's run had written up to t, so they are the same
 * files a run from power-on writes. Not with S32_REPLAY, S32_XPLANE, S32_IOCOV, S32_READHIST or ECU_CONSOLE; a restored
 * run also not with S32_SNAPDIR (the screens before t are not re-created). */
/* The build stamp: the content identity of the sources, compiler and flags (emu/common/build_id.sh, passed by the
 * Makefiles as S32_BUILD_ID), so two builds of the same tree carry the same stamp. A build made some other way falls
 * back to its compile time: never shared with another build, so a snapshot is never restored into different code. */
#ifdef S32_BUILD_ID
#define BUILD_STAMP "s32pair src " S32_BUILD_ID
#else
#define BUILD_STAMP "s32pair " __DATE__ " " __TIME__
#endif
static const char *const LOG_ENV[6] = {"S32_LINKLOG", "S32_STATUSLOG", "S32_SPARKLOG", "S32_INJLOG", "S32_LINKRAW", "S32_CT_CONSOLE"};
static FILE **log_file(int i) {
  static FILE **const F[6] = {&LINKLOG.f, &STATUSLOG.f, &SPARKLOG.f, &INJLOG.f, &LINKRAW, &CTCON};
  return F[i];
}
static uint8_t *SD_ORIG[3];                                /* each card as loaded (snapshots store the blocks written) */
static FILE *TRACE_F; static const char *TRACE_PATH;
static bool IOCOV_ON;

static uint32_t snap_layout(void) {
  uint64_t v[] = {S32SNAP_VERSION, sizeof(void *), sizeof(s32ct), sizeof(ecu_t), ecu_snap_size(), sizeof(s32board),
                  sizeof(s32_fram), sizeof(s32_rtc), sizeof(s32_sd), sizeof(ENG), sizeof(link_deframer),
                  sizeof(link_trim_tracker), sizeof(TRKQ), sizeof(OB_CT2E), sizeof(s32_sha256)};
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < sizeof v / sizeof v[0]; i++) for (int b = 0; b < 8; b++) { h ^= (uint8_t)(v[i] >> (8 * b)); h *= 16777619u; }
  return h;
}
/* Why this run cannot write or take a snapshot ("" when it can). */
static const char *snap_unsupported(void) {
  if (!ecu_snap_size()) return "this build's ECU core cannot snapshot (pure-C shim only)";
  if (RP) return "S32_REPLAY";
  if (XP_ON) return "S32_XPLANE";
  if (IOCOV_ON) return "S32_IOCOV";
  if (getenv("S32_READHIST")) return "S32_READHIST";
  if (ECON[0] || ECON[1]) return "ECU_CONSOLE";
  if (B.nanchor) return "phase-anchored times (~)";
  return "";
}
static uint8_t *read_all(const char *p, size_t *n) {
  FILE *f = fopen(p, "rb"); if (!f) return NULL;
  fseek(f, 0, SEEK_END); long l = ftell(f); fseek(f, 0, SEEK_SET);
  uint8_t *b = malloc(l > 0 ? (size_t)l : 1);
  if (b && l > 0 && fread(b, 1, (size_t)l, f) != (size_t)l) { free(b); b = NULL; }
  fclose(f); *n = l > 0 ? (size_t)l : 0; return b;
}

/* What this run's start-up and knobs set up, taken before the first quantum: the static part as one hash per
 * component (every MCU, device and card as initialised, the board's settings, the front end's own options) and the
 * timed items. Two runs whose static hashes match and whose items before t + guard match are the same run up to t. */
typedef struct { const char *name; char hx[65]; } sig_part;
#define N_SIG_PARTS 9
static sig_part SIG[N_SIG_PARTS]; static char SIG_ALL[65];
static s32_tbuf SIG_ITEMS;
static void sig_hash(int i, const char *name, s32_tbuf *b) {
  SIG[i].name = name; s32_sha256_hex(b->p ? b->p : "", b->n, SIG[i].hx); b->n = 0;
}
/* A card's contents for the stimulus hash: its size and every 512-byte block that is not all zero, with its index (a
 * 64 MB card is mostly empty; hashing only what is written keeps S32_STIMULUS fast). */
static void card_hash(const uint8_t *img, size_t n, s32_tbuf *b) {
  static const uint8_t ZERO[512];
  s32_sha256 s; uint8_t d[32];
  s32_sha256_init(&s);
  uint64_t nn = n; s32_sha256_update(&s, &nn, sizeof nn);
  for (size_t off = 0; off < n; off += 512) {
    size_t len = n - off < 512 ? n - off : 512;
    if (!memcmp(img + off, ZERO, len)) continue;
    uint64_t idx = off / 512; s32_sha256_update(&s, &idx, sizeof idx); s32_sha256_update(&s, img + off, len);
  }
  s32_sha256_final(&s, d);
  s32snap_b(b, d, sizeof d);
}
static void stimulus_take(void) {
  s32_tbuf b = {0}, st = {0};
  s32board_stimulus(&B, &st, &SIG_ITEMS);
  s32snap_item(&SIG_ITEMS, 6, 0, STATS_FROM, &STATS_FROM, sizeof STATS_FROM);   /* S32_STATS_FROM window start */
  sig_hash(0, "board", &st);
  s32snap_ct_blob(&CT, &b); sig_hash(1, "controller", &b);
  for (int k = 0; k < 2; k++) {
    size_t n = ecu_snap_size(); uint8_t *e = calloc(1, n ? n : 1);
    if (n && ecu_snap_save(&ECU[k], e) == 0) s32snap_b(&b, e, n);
    free(e); sig_hash(2 + k, k ? "ECU2" : "ECU1", &b);
  }
  s32snap_b(&b, &CT_FRAM, sizeof CT_FRAM); s32snap_b(&b, &ECU_FRAM[0], sizeof ECU_FRAM[0]); s32snap_b(&b, &ECU_FRAM[1], sizeof ECU_FRAM[1]);
  sig_hash(4, "FRAM", &b);
  s32snap_rtc_blob(&RTC, &b); sig_hash(5, "RTC", &b);
  { s32_sd *sds[3] = {&CT_SD, &ECU_SD[0], &ECU_SD[1]};
    for (int w = 0; w < 3; w++) {
      s32snap_u8(&b, SD_IMG[w] != NULL);
      if (SD_IMG[w]) { s32snap_sd_blob(sds[w], &b); card_hash(SD_IMG[w], SD_N[w], &b); }
    }
    sig_hash(6, "SD cards", &b); }
  for (int k = 0; k < 2; k++) { s32snap_u64(&b, NPK[k]); s32snap_b(&b, PFLK[k], ECU_PFL_BYTES); }
  sig_hash(7, "program flash", &b);
  { char fe[256]; snprintf(fe, sizeof fe, "%s noidleskip=%d linklog_full=%d trim_lag=%.17g", BUILD_STAMP, (int)NOIDLESKIP,
                           (int)LINKLOG_FULL, TRK.lag_s);
    s32snap_b(&b, fe, strlen(fe)); sig_hash(8, "front end", &b); }
  s32_tbuf all = {0};
  for (int i = 0; i < N_SIG_PARTS; i++) s32snap_b(&all, SIG[i].hx, 64);
  s32_sha256_hex(all.p, all.n, SIG_ALL);
  s32_tbuf_free(&all); s32_tbuf_free(&b); s32_tbuf_free(&st);
}
static void stimulus_json(const char *path) {
  FILE *f = fopen(path, "w"); if (!f) { perror(path); exit(2); }
  const char *why = snap_unsupported();
  fprintf(f, "{\"static\": \"%s\", \"guard\": %.17g, \"quantum\": %.17g, \"snapshot\": %s, \"why\": \"%s\", \"parts\": {",
          SIG_ALL, S32SNAP_GUARD_S, QUANTUM_US / 1e6, *why ? "false" : "true", why);
  for (int i = 0; i < N_SIG_PARTS; i++) fprintf(f, "%s\"%s\": \"%.16s\"", i ? ", " : "", SIG[i].name, SIG[i].hx);
  fputs("}, \"items\": ", f);
  s32snap_items_json((const uint8_t *)SIG_ITEMS.p, SIG_ITEMS.n, SB_HOME_T0, f);
  fputs("}\n", f);
  fclose(f);
}

static void snap_write(const char *path, long q) {
  s32snap_w w; s32snap_w_init(&w);
  s32_tbuf b = {0};
  double t = q * QUANTUM_US / 1e6;
  uint64_t qq = (uint64_t)q;
  s32snap_put(&w, "build", BUILD_STAMP, strlen(BUILD_STAMP));
  s32snap_put(&w, "quantum", &qq, sizeof qq);
  s32snap_put(&w, "sig:static", SIG_ALL, 64);
  { s32_tbuf p = {0}; for (int i = 0; i < N_SIG_PARTS; i++) s32snap_b(&p, SIG[i].hx, 64);
    s32snap_put(&w, "sig:parts", p.p, p.n); s32_tbuf_free(&p); }
  s32snap_put(&w, "sig:items", SIG_ITEMS.p, SIG_ITEMS.n);
  s32snap_put(&w, "home", &B.home_at, sizeof B.home_at);          /* home page time (< 0: not seen yet) */
  s32snap_ct_blob(&CT, &b); s32snap_put(&w, "ct", b.p, b.n); b.n = 0;
  for (int k = 0; k < 2; k++) {
    size_t n = ecu_snap_size(); uint8_t *e = malloc(n);
    if (!e || ecu_snap_save(&ECU[k], e)) { fprintf(stderr, "snapshot: ECU%d state not saved\n", k + 1); exit(2); }
    s32snap_put(&w, k ? "ecu2" : "ecu1", e, n); free(e);
  }
  s32board_snap_blob(&B, &b); s32snap_put(&w, "board", b.p, b.n); b.n = 0;
  s32snap_put(&w, "fram0", &CT_FRAM, sizeof CT_FRAM);
  s32snap_put(&w, "fram1", &ECU_FRAM[0], sizeof ECU_FRAM[0]); s32snap_put(&w, "fram2", &ECU_FRAM[1], sizeof ECU_FRAM[1]);
  s32snap_rtc_blob(&RTC, &b); s32snap_put(&w, "rtc", b.p, b.n); b.n = 0;
  { s32_sd *sds[3] = {&CT_SD, &ECU_SD[0], &ECU_SD[1]};
    for (int w2 = 0; w2 < 3; w2++) if (SD_IMG[w2]) {
      char tag[16];
      snprintf(tag, sizeof tag, "sd%d", w2); s32snap_sd_blob(sds[w2], &b); s32snap_put(&w, tag, b.p, b.n); b.n = 0;
      snprintf(tag, sizeof tag, "sddiff%d", w2); s32snap_sd_diff(sds[w2], SD_ORIG[w2], &b); s32snap_put(&w, tag, b.p, b.n); b.n = 0;
    } }
  for (int k = 0; k < 2; k++) {
    s32snap_u64(&b, NPK[k]); s32snap_b(&b, PFLK[k], ECU_PFL_BYTES);
    s32snap_put(&w, k ? "pflash2" : "pflash1", b.p, b.n); b.n = 0;
  }
  s32snap_put(&w, "eng", &ENG, sizeof ENG);
  { link_deframer d[8] = {DF_CT2E[0].d, DF_CT2E[1].d, DF_E2CT[0].d, DF_E2CT[1].d, RX_C2E[0], RX_C2E[1], RX_E2C[0], RX_E2C[1]};
    for (int i = 0; i < 8; i++) { d[i].on_frame = NULL; d[i].on_text = NULL; d[i].ctx = NULL; }
    s32snap_put(&w, "deframers", d, sizeof d); }
  s32snap_put(&w, "trim", &TRK, sizeof TRK); s32snap_put(&w, "trimq", TRKQ, sizeof TRKQ);
  s32snap_put(&w, "tblwr", TBL_WR, sizeof TBL_WR); s32snap_put(&w, "tblack", TBL_ACK, sizeof TBL_ACK);
  s32snap_put(&w, "tblref", TBL_REF, sizeof TBL_REF); s32snap_put(&w, "tblopen", TBL_OPEN, sizeof TBL_OPEN);
  s32snap_put(&w, "tblwrlast", TBL_WR_LAST, sizeof TBL_WR_LAST);
  s32snap_put(&w, "status", LAST_ST, sizeof LAST_ST); s32snap_put(&w, "status_have", HAVE_ST, sizeof HAVE_ST);
  s32snap_put(&w, "outbox_c2e", OB_CT2E, sizeof OB_CT2E); s32snap_put(&w, "outbox_e2c", OB_E2CT, sizeof OB_E2CT);
  for (int i = 0; i < 6; i++) {                                 /* the logs so far (flushed between quanta) */
    FILE *f = *log_file(i);
    if (!f) continue;
    fflush(f);
    size_t n; uint8_t *d = read_all(getenv(LOG_ENV[i]), &n);
    if (!d) { fprintf(stderr, "snapshot: cannot read back %s\n", LOG_ENV[i]); exit(2); }
    char tag[48]; snprintf(tag, sizeof tag, "log:%s", LOG_ENV[i]); s32snap_put(&w, tag, d, n); free(d);
  }
  if (TRACE_F) {
    s32trace_flush(&TR); fflush(TRACE_F);
    size_t n; uint8_t *d = read_all(TRACE_PATH, &n);
    if (!d) { fprintf(stderr, "snapshot: cannot read back S32_TRACE\n"); exit(2); }
    s32snap_put(&w, "trace:bytes", d, n); free(d);
    s32snap_trace_blob(&TR, &b); s32snap_put(&w, "trace:state", b.p, b.n); b.n = 0;
  }
  for (unsigned i = 0; i < N_SNAP_FILES; i++) {                /* bezel "snap:" screens already written */
    size_t n; uint8_t *d = read_all(SNAP_FILES[i], &n);
    if (!d) continue;
    char tag[300]; snprintf(tag, sizeof tag, "ppm:%.250s", SNAP_FILES[i]);
    if (strlen(tag) < sizeof ((s32snap_chunk *)0)->tag) s32snap_put(&w, tag, d, n);
    free(d);
  }
  s32_tbuf_free(&b);
  if (s32snap_w_save(&w, path, snap_layout())) { perror(path); exit(2); }
  fprintf(stderr, "snapshot: wrote %s at t=%.4f s (quantum %ld)\n", path, t, q);
}

static void snap_refuse(const char *path, const char *why) {
  fprintf(stderr, "snapshot: cannot restore %s: %s\n", path, why);
  exit(5);
}
static const s32snap_chunk *need(const s32snap_r *r, const char *path, const char *tag) {
  const s32snap_chunk *c = s32snap_get(r, tag);
  if (!c) { char why[96]; snprintf(why, sizeof why, "no %s in the snapshot", tag); snap_refuse(path, why); }
  return c;
}
static void snap_restore(const char *path) {
  s32snap_r r;
  char why[512];
  if (s32snap_r_load(&r, path, snap_layout())) snap_refuse(path, r.err);
  const s32snap_chunk *c = need(&r, path, "build");
  if (c->n != strlen(BUILD_STAMP) || memcmp(c->p, BUILD_STAMP, c->n)) snap_refuse(path, "written by another build of the emulator");
  uint64_t qq;
  c = need(&r, path, "quantum");
  if (c->n != 8) snap_refuse(path, "damaged");
  memcpy(&qq, c->p, 8);
  double t = (double)qq * QUANTUM_US / 1e6, t_future = t + S32SNAP_GUARD_S;
  if (getenv("S32_SNAPDIR")) snap_refuse(path, "S32_SNAPDIR (the screens before the snapshot would be missing)");
  c = need(&r, path, "sig:parts");
  if (c->n != 64 * N_SIG_PARTS) snap_refuse(path, "damaged");
  for (int i = 0; i < N_SIG_PARTS; i++) if (memcmp(c->p + 64 * i, SIG[i].hx, 64)) {
    snprintf(why, sizeof why, "this run's start differs from the snapshot's (%s)", SIG[i].name); snap_refuse(path, why);
  }
  double home_at;
  c = need(&r, path, "home");
  if (c->n != sizeof home_at) snap_refuse(path, "damaged");
  memcpy(&home_at, c->p, sizeof home_at);
  c = need(&r, path, "sig:items");
  if (!s32snap_items_equal_before(c->p, c->n, (const uint8_t *)SIG_ITEMS.p, SIG_ITEMS.n, t_future, SB_HOME_T0, home_at, t,
                                  why, sizeof why)) snap_refuse(path, why);
  /* the same start and the same stimulus up to t_future: take the state */
  c = need(&r, path, "ct"); if (s32snap_ct_restore(&CT, c->p, c->n)) snap_refuse(path, "Controller state");
  s32ct_pflash_read(&CT, CT_PFL, S32CT_PFL_BYTES);
  for (int k = 0; k < 2; k++) { c = need(&r, path, k ? "ecu2" : "ecu1"); if (ecu_snap_restore(&ECU[k], c->p, c->n)) snap_refuse(path, "ECU state"); }
  c = need(&r, path, "board"); if (s32board_snap_restore(&B, c->p, c->n, t_future, why, sizeof why)) snap_refuse(path, why);
  s32_fram *fr[3] = {&CT_FRAM, &ECU_FRAM[0], &ECU_FRAM[1]};
  for (int i = 0; i < 3; i++) {
    char tag[8]; snprintf(tag, sizeof tag, "fram%d", i);
    c = need(&r, path, tag);
    if (c->n != sizeof *fr[i]) snap_refuse(path, "FRAM state");
    memcpy(fr[i], c->p, c->n);
  }
  c = need(&r, path, "rtc"); if (s32snap_rtc_restore(&RTC, c->p, c->n)) snap_refuse(path, "RTC state");
  { s32_sd *sds[3] = {&CT_SD, &ECU_SD[0], &ECU_SD[1]};
    for (int w = 0; w < 3; w++) if (SD_IMG[w]) {
      char tag[16];
      snprintf(tag, sizeof tag, "sd%d", w); c = need(&r, path, tag); if (s32snap_sd_restore(sds[w], c->p, c->n)) snap_refuse(path, "SD card state");
      snprintf(tag, sizeof tag, "sddiff%d", w); c = need(&r, path, tag); if (s32snap_sd_apply(sds[w], c->p, c->n)) snap_refuse(path, "SD card contents");
    } }
  for (int k = 0; k < 2; k++) {
    c = need(&r, path, k ? "pflash2" : "pflash1");
    if (c->n != 8 + ECU_PFL_BYTES) snap_refuse(path, "program flash");
    uint64_t n;
    memcpy(&n, c->p, 8);
    NPK[k] = (size_t)n;
    memcpy(PFLK[k], c->p + 8, ECU_PFL_BYTES);
  }
  c = need(&r, path, "eng");
  if (c->n != sizeof ENG) snap_refuse(path, "output statistics");
  memcpy(&ENG, c->p, sizeof ENG);
  c = need(&r, path, "deframers");
  { link_deframer *d[8] = {&DF_CT2E[0].d, &DF_CT2E[1].d, &DF_E2CT[0].d, &DF_E2CT[1].d, &RX_C2E[0], &RX_C2E[1], &RX_E2C[0], &RX_E2C[1]};
    if (c->n != 8 * sizeof(link_deframer)) snap_refuse(path, "link analysers");
    for (int i = 0; i < 8; i++) {
      link_frame_fn f = d[i]->on_frame; link_text_fn tx = d[i]->on_text; void *ctx = d[i]->ctx;
      memcpy(d[i], c->p + i * sizeof(link_deframer), sizeof(link_deframer));
      d[i]->on_frame = f; d[i]->on_text = tx; d[i]->ctx = ctx;
    } }
#define TAKE(tag, obj) do { c = need(&r, path, tag); if (c->n != sizeof(obj)) snap_refuse(path, tag); memcpy(&(obj), c->p, sizeof(obj)); } while (0)
  { double lag = TRK.lag_s; TAKE("trim", TRK); TRK.lag_s = lag; }
  TAKE("trimq", TRKQ); TAKE("status", LAST_ST); TAKE("status_have", HAVE_ST); TAKE("outbox_c2e", OB_CT2E); TAKE("outbox_e2c", OB_E2CT);
  TAKE("tblwr", TBL_WR); TAKE("tblack", TBL_ACK); TAKE("tblref", TBL_REF); TAKE("tblopen", TBL_OPEN); TAKE("tblwrlast", TBL_WR_LAST);
#undef TAKE
  for (int i = 0; i < 6; i++) {                                 /* the logs: what the snapshot's run had written */
    FILE *f = *log_file(i);
    if (!f) continue;
    char tag[48]; snprintf(tag, sizeof tag, "log:%s", LOG_ENV[i]);
    c = s32snap_get(&r, tag);
    if (!c) { snprintf(why, sizeof why, "%s is set but the snapshot's run did not write it", LOG_ENV[i]); snap_refuse(path, why); }
    if (c->n && fwrite(c->p, 1, c->n, f) != c->n) snap_refuse(path, "log write failed");
  }
  if (TRACE_F) {
    const s32snap_chunk *tb = s32snap_get(&r, "trace:bytes"), *ts = s32snap_get(&r, "trace:state");
    if (!tb || !ts) snap_refuse(path, "S32_TRACE is set but the snapshot's run wrote no trace");
    fflush(TRACE_F);
    if (ftell(TRACE_F) != 0) snap_refuse(path, "trace file already written");
    if (s32snap_trace_restore(&TR, ts->p, ts->n)) snap_refuse(path, "trace state");
    if (tb->n && fwrite(tb->p, 1, tb->n, TRACE_F) != tb->n) snap_refuse(path, "trace write failed");
  }
  for (size_t i = 0; i < r.nc; i++) if (!strncmp(r.c[i].tag, "ppm:", 4)) {
    FILE *f = fopen(r.c[i].tag + 4, "wb");
    if (f) { fwrite(r.c[i].p, 1, r.c[i].n, f); fclose(f); }
    if (N_SNAP_FILES < MAX_SNAP_FILES) snprintf(SNAP_FILES[N_SNAP_FILES++], sizeof SNAP_FILES[0], "%s", r.c[i].tag + 4);
  }
  Q0 = (long)qq;
  NOW_S = t;
  s32snap_r_free(&r);
  fprintf(stderr, "snapshot: restored %s at t=%.4f s (quantum %ld)\n", path, t, Q0);
}

int main(int argc, char **argv) {
  if (argc >= 2 && !strcmp(argv[1], "--knobs")) {
    s32board_knob_help(stdout);
    static const char *const FRONT[][2] = {   /* this front end's own options (see the header) */
      {"S32_CT_SD", "FAT32 image in the Controller's SD slot"}, {"S32_ECU1_SD", "FAT32 image in ECU1's SD slot"},
      {"S32_ECU2_SD", "FAT32 image in ECU2's SD slot"}, {"S32_CT_FRAM", "Controller FRAM image"}, {"S32_CT_FRAM_OUT", "save it at exit"},
      {"S32_ECU1_FRAM", "ECU1 FRAM image"}, {"S32_ECU2_FRAM", "ECU2 FRAM image"}, {"S32_ECU1_FRAM_OUT", "save it at exit"},
      {"S32_ECU2_FRAM_OUT", "save it at exit"}, {"S32_CT_PFLASH", "Controller program flash image, or blank (default: the image argument)"},
      {"S32_CT_PFLASH_OUT", "Controller program flash read back at exit"},
      {"S32_ECU1_PFLASH", "ECU1 program flash image, or blank (default: the image argument)"},
      {"S32_ECU2_PFLASH", "ECU2 program flash image, or blank"}, {"S32_ECU1_PFLASH_OUT", "ECU1 program flash read back at exit"},
      {"S32_ECU2_PFLASH_OUT", "ECU2 program flash read back at exit"}, {"S32_REPLAY", "replay a datalog block"}, {"S32_XPLANE", "live X-Plane feed"},
      {"S32_SNAPDIR", "one screen per second"}, {"S32_SNAP_MS", "S32_SNAPDIR period in ms instead (files m<ms>.ppm)"}, {"S32_PPM", "final screen"}, {"S32_LINKLOG", "every link frame"}, {"S32_LINKLOG_FULL", "log whole frame bodies + CRC"},
      {"S32_LINKRAW", "every link byte"}, {"S32_STATUSLOG", "every decoded ECU status frame"}, {"S32_SPARKLOG", "every spark"}, {"S32_INJLOG", "every injector pulse"},
      {"S32_STATS_FROM", "statistics window start"},
      {"S32_IOCOV", "I/O coverage JSON"}, {"S32_READHIST", "ECU SFR hot spots"},
      {"S32_CT_CONSOLE", "Controller UART1 console output"}, {"ECU_CONSOLE", "ECU console capture prefix"},
      {"S32_SNAPSHOT_OUT", "<file>@<t>[,...]: write the whole system's state at t or {H+x} (emu/common/s32snap.h)"},
      {"S32_SNAPSHOT_IN", "start from a snapshot (this run's knobs from t + 10 ms on)"},
      {"S32_STIMULUS", "write what the knobs set up (JSON) and exit"},
      {"S32_ECU_APP_START", "1: start the ECU application directly even when a boot flash image is given (no boot region)"},
      {"S32_ECU_BOOT_CYCLES", "application start: SYSCLK cycles each ECU is held before the hand-off (default 16379, the boot code's run with no card; 0 = at once)"}};
    for (size_t i = 0; i < sizeof FRONT / sizeof FRONT[0]; i++) printf("%-18s %s (s32pair option)\n", FRONT[i][0], FRONT[i][1]);
    return 0;
  }
  bool ct_absent = false;
  if (argc < 4) {
    fprintf(stderr, "usage: s32pair <CT_prog.bin|none> <EC_bflash.bin|none> <EC_pflash.bin|EC_app.flat> [seconds]\n"
                    "       s32pair --knobs\n");
    return 2;
  }
  int secs_arg = 4;
  if (!strcmp(argv[1], "none")) {                     /* no Controller on the bench: its slot stays empty and unpowered */
    NCT = (0x2AC00u / 2u) * 3u; CT_IMG = malloc(NCT); if (!CT_IMG) { perror("malloc"); return 2; }
    memset(CT_IMG, 0xFF, NCT); ct_absent = true;
  } else CT_IMG = slurp(argv[1], &NCT);
  PFL = slurp(argv[3], &NP);
  { const char *as = getenv("S32_ECU_APP_START"); bool app = as && *as && *as != '0';
    if (app || !strcmp(argv[2], "none")) {          /* no boot region: application start (emu/ecu/src/ecu.c) */
      NB = ECU_BFL_BYTES; BFL = malloc(NB); if (!BFL) { perror("malloc"); return 2; }
      ecu_app_boot_flash(BFL, NB);
      fprintf(stderr, "ECU: no boot region, application start (the state the boot code hands over in, no card)\n");
    } else BFL = slurp(argv[2], &NB); }
  double secs = 10.0;
  { const char *end = NULL;
    if (argc > secs_arg && (!s32board_parse_time(argv[secs_arg], &end, &secs) || *end)) {
      fprintf(stderr, "s32pair: bad seconds '%s'\n", argv[secs_arg]); return 2;
    } }
  const char *ll = getenv("S32_LINKLOG"), *snap = getenv("S32_SNAPDIR"), *econ = getenv("ECU_CONSOLE");
  { const char *ni = getenv("S32_NOIDLESKIP"); NOIDLESKIP = ni && *ni && *ni != '0'; }
  LINKLOG.f = ll ? fopen(ll, "w") : NULL;
  LINKLOG_FULL = getenv("S32_LINKLOG_FULL") != NULL;
  LINKRAW = getenv("S32_LINKRAW") ? fopen(getenv("S32_LINKRAW"), "w") : NULL;
  CTCON = getenv("S32_CT_CONSOLE") ? fopen(getenv("S32_CT_CONSOLE"), "w") : NULL;
  STATUSLOG.f = getenv("S32_STATUSLOG") ? fopen(getenv("S32_STATUSLOG"), "w") : NULL;
  bool serial = false;
#ifdef __EMSCRIPTEN__
  serial = true;                                                 /* one thread: the WebAssembly build */
#else
  { const char *se = getenv("S32_SERIAL"); serial = se && *se && *se != '0'; }
#endif
  FILE *trace_f = NULL;
  { const char *tp = getenv("S32_TRACE");
    if (tp && *tp) { trace_f = fopen(tp, "wb"); if (!trace_f) { perror(tp); return 2; } s32trace_open(&TR, trace_f); }
    TRACE_F = trace_f; TRACE_PATH = tp; }
  for (int k = 0; k < 2; k++) {
    link_deframer_init(&DF_CT2E[k].d, on_link_frame, NULL, &DF_CT2E[k]);
    link_deframer_init(&DF_E2CT[k].d, on_link_frame, NULL, &DF_E2CT[k]);
    link_deframer_init(&RX_C2E[k], rx_c2e_frame, NULL, (void *)(intptr_t)k);
    link_deframer_init(&RX_E2C[k], rx_e2c_frame, NULL, (void *)(intptr_t)k);
  }
  link_trim_init(&TRK, getenv("S32_TRIM_LAG_MS") ? atof(getenv("S32_TRIM_LAG_MS")) / 1000.0 : TRIM_LAG_S);
  if (getenv("S32_STATS_FROM")) {
    const char *end = NULL;
    if (!s32board_parse_time(getenv("S32_STATS_FROM"), &end, &STATS_FROM) || *end) {
      fprintf(stderr, "s32pair: bad S32_STATS_FROM '%s'\n", getenv("S32_STATS_FROM")); return 2;
    }
  }
  if (s32board_clock_check(BFL, NB, ct_absent ? NULL : CT_IMG, NCT, stderr)) fprintf(stderr, "WARNING: the images' clock configuration differs from the emulator's\n");

  /* devices on each board */
  s32_fram_init(&CT_FRAM, 0x00); load_into(CT_FRAM.mem, sizeof CT_FRAM.mem, "S32_CT_FRAM");
  for (int k = 0; k < 2; k++) { s32_fram_init(&ECU_FRAM[k], 0x00); load_into(ECU_FRAM[k].mem, sizeof ECU_FRAM[k].mem, k ? "S32_ECU2_FRAM" : "S32_ECU1_FRAM"); }
  static const char *SD_ENV[3] = {"S32_CT_SD", "S32_ECU1_SD", "S32_ECU2_SD"};
  s32_sd *sds[3] = {&CT_SD, &ECU_SD[0], &ECU_SD[1]};
  for (int w = 0; w < 3; w++) {
    SD_PATH[w] = getenv(SD_ENV[w]);
    if (SD_PATH[w] && *SD_PATH[w]) { SD_IMG[w] = slurp(SD_PATH[w], &SD_N[w]); s32_sd_init(sds[w], SD_IMG[w], (uint32_t)SD_N[w]); }
    if (SD_IMG[w] && getenv("S32_SNAPSHOT_OUT")) {          /* snapshots store the blocks that differ from the card as loaded */
      SD_ORIG[w] = malloc(SD_N[w] ? SD_N[w] : 1); if (!SD_ORIG[w]) { perror("malloc"); return 2; }
      memcpy(SD_ORIG[w], SD_IMG[w], SD_N[w]);
    }
  }

  /* the harness */
  s32board_init(&B, (s32board_host){.boot = host_boot, .snap = host_snap});
  if (getenv("S32_STATS_FROM")) { B.ff_win_set = true; B.ff_win_from = STATS_FROM; }   /* windowed fuel-flow count (init clears B) */
  B.ct_absent = ct_absent;
  if (ct_absent) fprintf(stderr, "Controller: none (its slot is empty)\n");
  if (trace_f) B.trace = &TR;
  B.ct = &CT; B.ecu[0] = &ECU[0]; B.ecu[1] = &ECU[1];
  B.fram[SB_CT] = &CT_FRAM; B.fram[SB_E1] = &ECU_FRAM[0]; B.fram[SB_E2] = &ECU_FRAM[1]; B.rtc = &RTC;
  for (int w = 0; w < 3; w++) if (SD_IMG[w]) {
    B.sd[w] = sds[w];
    if (w == SB_CT) B.lvl[SB_CT][4] &= (uint16_t)~(1u << 3);          /* RE3 low = card present */
    else B.lvl[w][1] &= (uint16_t)~((1u << 3) | (1u << 4));           /* RB3 low = present, RB4 low = writable */
  }

  /* the Controller and two ECUs */
  const char *iocov = getenv("S32_IOCOV");
  CT_PFL = malloc(S32CT_PFL_BYTES); if (!CT_PFL) { perror("malloc"); return 2; }
  memset(CT_PFL, 0xFF, S32CT_PFL_BYTES);
  { const char *cp = getenv("S32_CT_PFLASH");
    if (cp && !strcmp(cp, "blank")) { /* erased 0xFF */ }
    else if (cp && *cp) { size_t n; uint8_t *img = slurp(cp, &n); memcpy(CT_PFL, img, n < S32CT_PFL_BYTES ? n : S32CT_PFL_BYTES); free(img); }
    else memcpy(CT_PFL, CT_IMG, NCT < S32CT_PFL_BYTES ? NCT : S32CT_PFL_BYTES); }
  s32ct_init(&CT, CT_PFL, S32CT_PFL_BYTES);
  if (iocov && *iocov) { s32ct_iocov_enable(&CT); IOCOV_ON = true; }
  s32_rtc_init(&RTC, &CT.cpu.cycles, SB_FCY, 1791806400);              /* the clock's fixed start time (Unix s) */
  ct_setup();
  for (int k = 0; k < 2; k++) {                                         /* program flash: the image, or S32_ECUk_PFLASH */
    static const char *const PENV[2] = {"S32_ECU1_PFLASH", "S32_ECU2_PFLASH"};
    const char *pe = getenv(PENV[k]);
    PFLK[k] = malloc(ECU_PFL_BYTES); if (!PFLK[k]) { perror("malloc"); return 2; }
    memset(PFLK[k], 0xFF, ECU_PFL_BYTES);
    if (pe && !strcmp(pe, "blank")) NPK[k] = ECU_PFL_BYTES;             /* boot code only, no application programmed */
    else if (pe && *pe) { size_t n; uint8_t *img = slurp(pe, &n); NPK[k] = ecu_place_pflash(PFLK[k], img, n); free(img); }
    else NPK[k] = ecu_place_pflash(PFLK[k], PFL, NP);     /* a full image, or an application-only one at its window */
    if (ecu_init(&ECU[k], BFL, NB, PFLK[k], NPK[k])) { fprintf(stderr, "ecu_init %d failed\n", k + 1); return 3; }
    if (iocov && *iocov) ecu_iocov_enable(&ECU[k]);
    ecu_setup(k);
    if (econ) { char fn[512]; snprintf(fn, sizeof fn, "%s%d.txt", econ, k + 1); ECON[k] = fopen(fn, "w"); }
  }
  s32board_knobs_from_env(&B);                                           /* every harness / fault knob */
  RTC.fcy = s32board_hz(&B, SB_CT);
  s32board_attach(&B);

  { const char *rp = getenv("S32_REPLAY");
    if (rp && *rp) {
      rp_load(rp);
      if (getenv("S32_REPLAY_START")) RP_START = atof(getenv("S32_REPLAY_START"));
      if (getenv("S32_REPLAY_TRIM_MS")) RP_TRIM_S = atof(getenv("S32_REPLAY_TRIM_MS")) / 1000.0;
      if (getenv("S32_REPLAY_PHASE")) RP_PHASE = atof(getenv("S32_REPLAY_PHASE"));
      rp_apply_row(0);                                     /* engine stopped, row 0's sensors + switches from boot */
      fprintf(stderr, "replay: %ld rows from t=%.1f s (ends t=%.2f s)\n", RP_N, RP_START, RP_START + RP_PHASE + RP_N);
    } }
  if (getenv("S32_SPARKLOG")) SPARKLOG.f = fopen(getenv("S32_SPARKLOG"), "w");
  if (getenv("S32_INJLOG")) INJLOG.f = fopen(getenv("S32_INJLOG"), "w");
  const char *xps = getenv("S32_XPLANE");
  if (xps && *xps) {
    char host[64] = "127.0.0.1"; int port = 49000;
    if (sscanf(xps, "%63[^:]:%d", host, &port) < 1) snprintf(host, sizeof host, "%s", xps);
    if (xp_open(&XP, host, port)) { fprintf(stderr, "X-Plane socket failed\n"); return 4; }
    xp_add(&XP, "sim/cockpit2/engine/indicators/engine_speed_rpm[0]");
    xp_add(&XP, "sim/cockpit2/engine/indicators/MPR_in_hg[0]");
    xp_add(&XP, "sim/cockpit2/engine/actuators/throttle_ratio_all");
    xp_add(&XP, "sim/cockpit2/temperature/outside_air_temp_degc");
    xp_add(&XP, "sim/cockpit2/engine/indicators/oil_temperature_deg_C[0]");
    xp_add(&XP, "sim/cockpit2/electrical/bus_volts[0]");
    xp_add(&XP, "sim/cockpit2/engine/actuators/ignition_key[0]");
    xp_subscribe(&XP, 20);
    XP_ON = true;
    fprintf(stderr, "X-Plane feed: %s:%d, paced to wall time\n", host, port);
  }

  /* snapshots: this run's stimulus, then S32_STIMULUS (report and exit), S32_SNAPSHOT_IN (start from a saved state) */
  const char *snap_in = getenv("S32_SNAPSHOT_IN"), *snap_out = getenv("S32_SNAPSHOT_OUT"), *stim = getenv("S32_STIMULUS");
  if (snap_in && !*snap_in) snap_in = NULL;
  if (snap_out && !*snap_out) snap_out = NULL;
  if (stim && *stim) { stimulus_take(); stimulus_json(stim); return 0; }
  enum { MAX_SNAP_OUT = 64 };
  char snap_path[MAX_SNAP_OUT][512]; double snap_t[MAX_SNAP_OUT]; long snap_at[MAX_SNAP_OUT]; int nsnap = 0, nwritten = 0;
  if (snap_in || snap_out) {
    const char *why = snap_unsupported();
    if (*why) { fprintf(stderr, "snapshot: not with %s\n", why); return 2; }
    stimulus_take();
  }
  if (snap_in) snap_restore(snap_in);
  if (snap_out) {        /* "<file>@<t>,...": t in seconds, or "{H+x}" (taken x after the home page appears) */
    char buf[8192]; snprintf(buf, sizeof buf, "%s", snap_out);
    char *save = NULL;
    for (char *tok = strtok_r(buf, ",", &save); tok; tok = strtok_r(NULL, ",", &save)) {
      char *at = strrchr(tok, '@');
      const char *end = NULL;
      double t;
      if (!at || nsnap >= MAX_SNAP_OUT || !s32board_parse_time(at + 1, &end, &t) || *end) {
        fprintf(stderr, "S32_SNAPSHOT_OUT: bad item %s\n", tok); return 2;
      }
      *at = '\0';
      int i = nsnap++;
      snap_t[i] = t; snap_at[i] = -1; snprintf(snap_path[i], sizeof snap_path[i], "%s", tok);
      if (t < SB_HOME_T0 / 2) {
        snap_at[i] = llround(t * 1e6 / QUANTUM_US);
        if (snap_at[i] <= Q0) { fprintf(stderr, "S32_SNAPSHOT_OUT: %s at t=%s is not after the start of this run\n", tok, at + 1); return 2; }
      }
    }
  }
  /* A run started from a snapshot taken after the home page knows it already: its run length and statistics window
   * may be home-page times. */
  if (B.home_at >= 0 && secs >= SB_HOME_T0 / 2) secs = fmax(0.0, B.home_at + (secs - SB_HOME_T0));
  if (B.home_at >= 0 && STATS_FROM >= SB_HOME_T0 / 2) STATS_FROM = fmax(0.0, B.home_at + (STATS_FROM - SB_HOME_T0));
  B.ff_win_from = STATS_FROM;

  struct timespec ts0; clock_gettime(CLOCK_MONOTONIC, &ts0);
  long quanta = (long)(((secs >= SB_HOME_T0 / 2 ? 15.0 : secs) * 1e6) / QUANTUM_US + 1e-6);
  if (snap_in && quanta < Q0) snap_refuse(snap_in, "snapshot is after this run's end");
  long snap_q = 0;                                               /* S32_SNAP_MS > 0: snapshot period in quanta */
  { const char *sm = getenv("S32_SNAP_MS"); int ms = sm ? atoi(sm) : 0; if (ms > 0) snap_q = (long)ms * 1000 / QUANTUM_US; }
  double wall0 = wall_now();
  pthread_t th[2];
  if (!serial) for (int k = 0; k < 2; k++) pthread_create(&th[k], NULL, ecu_thread, (void *)(intptr_t)k);
  for (long q = Q0 + 1; q <= quanta; q++) {
    NOW_S = q * QUANTUM_US / 1e6;
    double tq = (q - 1) * QUANTUM_US / 1e6;
    trk_drain();
    if (RP) rp_step(tq);
    if (XP_ON && q % 10 == 0) {
      double w = wall_now();
      if (xp_poll(&XP, w)) xp_apply();
      if (q % 10000 == 0 && (!XP.last_rx || w - XP.last_rx > 1.0)) xp_subscribe(&XP, 20);
      double ahead = NOW_S - (w - wall0);                       /* pace: never run ahead of the wall clock */
      if (ahead > 0.002) { struct timespec ts = {0, (long)(ahead * 1e9)}; nanosleep(&ts, NULL); }
    }
    s32board_step(&B, q);                                       /* events, power, pins, sensors, crank edges */
    if (B.home_at >= 0) {
      if (secs >= SB_HOME_T0 / 2) {
        secs = B.home_at + (secs - SB_HOME_T0);
        if (secs < 0.0) secs = 0.0;
        quanta = (long)((secs * 1e6) / QUANTUM_US + 1e-6);
      }
      if (STATS_FROM >= SB_HOME_T0 / 2) {
        STATS_FROM = B.home_at + (STATS_FROM - SB_HOME_T0);
        if (STATS_FROM < 0.0) STATS_FROM = 0.0;
        B.ff_win_from = STATS_FROM;
      }
    }
    if (!serial) atomic_store_explicit(&GO, q, memory_order_release);   /* ECU1 + ECU2 run this quantum on their threads */
    { double c0 = tcpu(); uint64_t target = s32board_target(&B, SB_CT, q);
      if (B.on[SB_CT]) {
        uint32_t r0 = CT.cpu.resets;
        s32ct_run_to(&CT, target);
        if (CT.cpu.resets != r0) ct_reset_note();
      } else CT.cpu.cycles = target;
      THREAD_CPU[0] += tcpu() - c0; }
    if (serial) { ecu_quantum(0, q); ecu_quantum(1, q); }
    else spin_until(&DONE, 2 * (q - Q0));
    tlog_flush(&LINKLOG); tlog_flush(&STATUSLOG); tlog_flush(&SPARKLOG); tlog_flush(&INJLOG);
    for (int k = 0; k < 2; k++)
      if (!EMU_ERR[k] && ecu_is_emu_err(ECU[k].last_err)) { EMU_ERR[k] = ECU[k].last_err; EMU_ERR_PC[k] = ECU[k].pc; }
    if (EMU_ERR[0] || EMU_ERR[1]) break;
    s32board_watchdogs(&B);                                     /* an expired ECU watchdog resets that core */
    for (int k = 0; k < 2; k++) {                               /* deliver the quantum's bytes over each cat5 cable */
      static const char *const C2E[2] = {"CT->ECU1", "CT->ECU2"}, *const E2C[2] = {"ECU1->CT", "ECU2->CT"};
      unsigned n = s32board_link(&B, k, true, OB_CT2E[k].b, OB_CT2E[k].n, sizeof OB_CT2E[k].b);
      if (n) { ecu_uart_feed(&ECU[k], 1, OB_CT2E[k].b, n); link_raw(C2E[k], OB_CT2E[k].b, n); for (unsigned i = 0; i < n; i++) link_deframe_byte(&RX_C2E[k], OB_CT2E[k].b[i]); }
      if (B.trace) s32trace_link(B.trace, q, k, true, OB_CT2E[k].b, n);
      OB_CT2E[k].n = 0;
      n = s32board_link(&B, k, false, OB_E2CT[k].b, OB_E2CT[k].n, sizeof OB_E2CT[k].b);
      if (n) { s32ct_uart_feed(&CT, k == 0 ? 3u : 4u, OB_E2CT[k].b, n); link_raw(E2C[k], OB_E2CT[k].b, n); for (unsigned i = 0; i < n; i++) link_deframe_byte(&RX_E2C[k], OB_E2CT[k].b[i]); }
      if (B.trace) s32trace_link(B.trace, q, k, false, OB_E2CT[k].b, n);
      OB_E2CT[k].n = 0;
    }
    if (B.trace) s32trace_quantum(B.trace, q, &CT, B.on[SB_CT]);
    if (snap && snap_q && q % snap_q == 0) {                    /* S32_SNAP_MS: the panel every N ms, named by ms */
      char fn[512]; snprintf(fn, sizeof fn, "%s/m%07ld.ppm", snap, q * QUANTUM_US / 1000); s32_panel_ppm(&CT, B.on[SB_CT], fn);
    }
    if (q % (long)(1e6 / QUANTUM_US) == 0) {
      int sec = (int)(q / (long)(1e6 / QUANTUM_US));
      if (snap && !snap_q) { char fn[512]; snprintf(fn, sizeof fn, "%s/t%03d.ppm", snap, sec); s32_panel_ppm(&CT, B.on[SB_CT], fn); }
      fprintf(stderr, "t=%ds  CT->E1 %llu  E1->CT %llu (text %llu)  CT->E2 %llu  E2->CT %llu (text %llu)\n", sec,
              (unsigned long long)DF_CT2E[0].d.frames, (unsigned long long)DF_E2CT[0].d.frames, (unsigned long long)DF_E2CT[0].d.text,
              (unsigned long long)DF_CT2E[1].d.frames, (unsigned long long)DF_E2CT[1].d.frames, (unsigned long long)DF_E2CT[1].d.text);
    }
    if (nwritten < nsnap) {                                      /* S32_SNAPSHOT_OUT (MCU threads parked) */
      for (int i = 0; i < nsnap; i++) if (snap_at[i] < 0 && B.home_at >= 0) {
        /* "{H+x}": the home page is known now; a point already passed (x <= one quantum) is taken at this quantum's end */
        long qs = llround((B.home_at + (snap_t[i] - SB_HOME_T0)) * 1e6 / QUANTUM_US);
        snap_at[i] = qs < q ? q : qs;
      }
      for (int i = 0; i < nsnap; i++) if (snap_at[i] == q) { snap_write(snap_path[i], q); snap_at[i] = 0; nwritten++; }
    }
  }
  if (!serial) {
    atomic_store(&QUIT, true); atomic_store_explicit(&GO, (quanta > Q0 ? quanta : Q0) + 1, memory_order_release);
    for (int k = 0; k < 2; k++) pthread_join(th[k], NULL);
  }
  struct timespec ts1; clock_gettime(CLOCK_MONOTONIC, &ts1);
  double wall = (double)(ts1.tv_sec - ts0.tv_sec) + (ts1.tv_nsec - ts0.tv_nsec) / 1e9;
  printf("emulated %.1f s in %.1f s wall (%.2fx real time); emulation CPU s: CT %.2f ECU1 %.2f ECU2 %.2f\n", secs, wall,
         secs / wall, THREAD_CPU[0], THREAD_CPU[1], THREAD_CPU[2]);
  s32board_fault_summary(&B, secs, stdout);
  if (STATS_FROM > 0) printf("statistics: pulses ending at/after t=%.3f s only\n", STATS_FROM);
  for (int k = 0; k < 2; k++) {
    printf("ECU%d: CT->ECU %llu frames (bad %llu); ECU->CT %llu frames (bad %llu, text bytes %llu); err=%d\n", k + 1,
           (unsigned long long)DF_CT2E[k].d.frames, (unsigned long long)DF_CT2E[k].d.bad, (unsigned long long)DF_E2CT[k].d.frames,
           (unsigned long long)DF_E2CT[k].d.bad, (unsigned long long)DF_E2CT[k].d.text, ECU[k].last_err);
    printf("   delivered over the cable: CT->ECU %llu frames (bad %llu); ECU->CT %llu frames (bad %llu)\n",
           (unsigned long long)RX_C2E[k].frames, (unsigned long long)RX_C2E[k].bad, (unsigned long long)RX_E2C[k].frames, (unsigned long long)RX_E2C[k].bad);
    if (TBL_WR[k][0] || TBL_WR[k][1]) {
      long uf = TBL_WR[k][1] > TBL_ACK[k][1] ? (long)(TBL_WR[k][1] - TBL_ACK[k][1]) : 0,   /* an e2c bench ACK can */
           ui = TBL_WR[k][0] > TBL_ACK[k][0] ? (long)(TBL_WR[k][0] - TBL_ACK[k][0]) : 0;   /* outnumber the writes */
      printf("   table writes delivered to this ECU: fuel %llu, ignition %llu; acknowledged by it: fuel %llu, ignition %llu\n",
             (unsigned long long)TBL_WR[k][1], (unsigned long long)TBL_WR[k][0], (unsigned long long)TBL_ACK[k][1], (unsigned long long)TBL_ACK[k][0]);
      if (TBL_REF[k][1] || TBL_REF[k][0])
        printf("   %llu of those acknowledgements refused the write (result not 1): that table was not stored\n",
               (unsigned long long)(TBL_REF[k][1] + TBL_REF[k][0]));
      if (uf > 0 || ui > 0) {
        double last = TBL_WR_LAST[k][1] > TBL_WR_LAST[k][0] ? TBL_WR_LAST[k][1] : TBL_WR_LAST[k][0];
        bool open = (uf > 0 && TBL_OPEN[k][1]) || (ui > 0 && TBL_OPEN[k][0]);   /* the newest write still unanswered */
        printf("   %ld table write%s crossed the cable but %s not acknowledged%s\n",
               uf + ui, uf + ui == 1 ? "" : "s", uf + ui == 1 ? "was" : "were",
               !open ? " (a later write of the same table was answered: a resend got through)"
               : secs - last < 1.0 ? " (it ended less than 1 s before the run's end: it may simply be in flight)" : "");
      }
    }
    printf("   spin fast-forward: %llu warps, %.3f s skipped; idle skipped %.3f s\n", (unsigned long long)ECU[k].warps,
           ECU[k].warped_cycles / ECU[k].hz, ECU[k].idle_skipped / ECU[k].hz);
    printf("   timer IRQs T1..T5: %llu %llu %llu %llu %llu; events in hook %llu\n", (unsigned long long)ecu_timer_irqs(&ECU[k], 1),
           (unsigned long long)ecu_timer_irqs(&ECU[k], 2), (unsigned long long)ecu_timer_irqs(&ECU[k], 3),
           (unsigned long long)ecu_timer_irqs(&ECU[k], 4), (unsigned long long)ecu_timer_irqs(&ECU[k], 5),
           (unsigned long long)ECU[k].events_inline);
    printf("   idle skips %llu\n", (unsigned long long)ECU[k].idle_blocks);
    printf("   SFR reads %llu writes %llu (per emulated s: %.0f reads); UART1 tx %llu UART4 tx %llu\n",
           (unsigned long long)ECU[k].sfr_reads, (unsigned long long)ECU[k].sfr_writes, ECU[k].sfr_reads / secs,
           (unsigned long long)ECU[k].uart[0].tx_bytes, (unsigned long long)ECU[k].uart[3].tx_bytes);
    printf("   UART1 rx %llu bytes, FIFO peak %u, overruns %llu; UART4 rx %llu, peak %u, overruns %llu\n",
           (unsigned long long)ECU[k].uart[0].rx_bytes, ECU[k].uart[0].fn_peak, (unsigned long long)ECU[k].uart[0].rx_overruns,
           (unsigned long long)ECU[k].uart[3].rx_bytes, ECU[k].uart[3].fn_peak, (unsigned long long)ECU[k].uart[3].rx_overruns);
    if (ECU[k].read_hist) {
      for (int r = 0; r < 6; r++) {
        unsigned bi = 0; for (unsigned i = 0; i < ECU_SFR_SIZE / 4; i++) if (ECU[k].read_hist[i] > ECU[k].read_hist[bi]) bi = i;
        const char *nm = ecu_sfr_name(0xBF800000u + bi * 4);
        printf("   hot SFR read %08X %-10s x%u\n", 0xBF800000u + bi * 4, nm ? nm : "?", ECU[k].read_hist[bi]);
        ECU[k].read_hist[bi] = 0;
      }
      for (int r = 0; r < 5; r++) {
        unsigned bi = 0; for (unsigned i = 0; i < ECU_SFR_SIZE / 4; i++) if (ECU[k].write_hist[i] > ECU[k].write_hist[bi]) bi = i;
        uint32_t a = 0xBF800000u + bi * 4; const char *nm = ecu_sfr_name(a); if (!nm) nm = ecu_sfr_name(a & ~0xFu);
        printf("   hot SFR write %08X %-10s +%X x%u\n", a, nm ? nm : "?", a & 0xFu, ECU[k].write_hist[bi]);
        ECU[k].write_hist[bi] = 0;
      }
    }
    printf("   ECU->CT by type:"); for (int t = 0; t < 16; t++) if (DF_E2CT[k].d.by_type[t]) printf(" %d:%llu", t, (unsigned long long)DF_E2CT[k].d.by_type[t]);
    printf("\n   CT->ECU by type:"); for (int t = 0; t < 16; t++) if (DF_CT2E[k].d.by_type[t]) printf(" %d:%llu", t, (unsigned long long)DF_CT2E[k].d.by_type[t]);
    printf("\n");
    if (HAVE_ST[k]) printf("   last status frame: RPM %u MAP %.2f inHg TPS %.1f%% IAT %d F ET %d F advance %d deg trim %d%% duty %.1f%%\n",
                           (unsigned)LAST_ST[k].rpm, LAST_ST[k].map_x100 / 100.0, (double)LAST_ST[k].tps_pct, LAST_ST[k].iat_f, LAST_ST[k].et_f,
                           LAST_ST[k].ign_adv_deg, (int)LAST_ST[k].fuel_trim_pct, (double)LAST_ST[k].inj_duty_pct);
  }
  printf("Controller idle skipped %.3f s; TRISA=%04X PORTA=%04X LATA=%04X ANSA=%04X\n", CT.idle_skipped / SB_FCY,
         s32ct_sfr_value(&CT, 0x02C0), s32ct_sfr_value(&CT, 0x02C2), s32ct_sfr_value(&CT, 0x02C4), s32ct_sfr_value(&CT, 0x04E0));
  printf("Controller resets since power-on: %u illegal-opcode (RCON.IOPUWR), %u RESET instruction, %u trap conflict; RCON=%04X\n",
         CT.illegal_resets, CT.sw_resets, CT.trap_resets, s32ct_sfr_value(&CT, 0x0740));
  printf("Controller UART1 rx %llu bytes, FIFO peak %u, overruns %llu; UART3 rx %llu, peak %u, overruns %llu; UART4 rx %llu, peak %u, overruns %llu\n",
         (unsigned long long)CT.uart[0].rx_bytes, CT.uart[0].fn_peak, (unsigned long long)CT.uart[0].rx_overruns,
         (unsigned long long)CT.uart[2].rx_bytes, CT.uart[2].fn_peak, (unsigned long long)CT.uart[2].rx_overruns,
         (unsigned long long)CT.uart[3].rx_bytes, CT.uart[3].fn_peak, (unsigned long long)CT.uart[3].rx_overruns);
  if (XP_ON) printf("X-Plane: %lu packets; last RPM %.0f MAP %.1f thr %.2f OAT %.1fC oil %.1fC bus %.1fV key %.0f\n", XP.packets,
                    XP.val[0], XP.val[1], XP.val[2], XP.val[3], XP.val[4], XP.val[5], XP.val[6]);
  if (RP) printf("replay: %ld rows, last applied %ld, trim edges %llu (ECU1 trim last seen %d %%; link trim tracker: %llu "
                 "status frames, %llu disagreed with the running estimate)\n", RP_N, RP_APPLIED, (unsigned long long)RP_EDGES,
                 RP_TRIM_CUR, (unsigned long long)TRK.reports, (unsigned long long)TRK.mismatches);
  if (B.rpm > 0 || XP_ON || RP || ENG.nw[0] || ENG.nw[1] || B.nev) {
    printf("engine: %.0f RPM from t=%.1f s, %d cyl, sync %.0f deg before event 0\n", B.rpm, B.start_s, B.ncyl, B.sync_before);
    for (int k = 0; k < 2; k++) {
      printf("ECU%d INT1 edges %llu, INT2 edges %llu; INTCON=%08X IEC0=%08X IFS0=%08X IPC1=%08X IPC2=%08X\n", k + 1,
             (unsigned long long)ECU[k].int_edges[1], (unsigned long long)ECU[k].int_edges[2], ecu_sfr_value(&ECU[k], 0xBF881000u),
             ecu_sfr_value(&ECU[k], 0xBF881060u), ecu_sfr_value(&ECU[k], 0xBF881030u), ecu_sfr_value(&ECU[k], 0xBF8810A0u), ecu_sfr_value(&ECU[k], 0xBF8810B0u));
      printf("   OCxCON: %04X %04X %04X %04X  oc_pulses %llu %llu %llu %llu\n", ecu_sfr_value(&ECU[k], 0xBF803000u) & 0xFFFF,
             ecu_sfr_value(&ECU[k], 0xBF803200u) & 0xFFFF, ecu_sfr_value(&ECU[k], 0xBF803400u) & 0xFFFF, ecu_sfr_value(&ECU[k], 0xBF803600u) & 0xFFFF,
             (unsigned long long)ECU[k].oc_pulses[0], (unsigned long long)ECU[k].oc_pulses[1], (unsigned long long)ECU[k].oc_pulses[2],
             (unsigned long long)ECU[k].oc_pulses[3]);
      for (int c = 0; c < 4; c++) if (ENG.coil[k][c].n)
        printf("   coil OC%d: %llu sparks, dwell %.2f ms, advance %.1f deg BTDC (min %.1f max %.1f)\n", c + 1,
               (unsigned long long)ENG.coil[k][c].n, ENG.coil[k][c].dwell_ms_sum / ENG.coil[k][c].n,
               ENG.coil[k][c].adv_sum / ENG.coil[k][c].n, ENG.coil[k][c].adv_min, ENG.coil[k][c].adv_max);
      for (int c = 0; c < 4; c++) if (ENG.coil[k][c].n)
        printf("   coil OC%d own cylinder: mean %.1f deg BTDC (min %.1f max %.1f)\n", c + 1, ENG.coil[k][c].own_sum / ENG.coil[k][c].n,
               ENG.coil[k][c].own_min, ENG.coil[k][c].own_max);
      if (ENG.nw[k]) {
        static float tmp[200000]; memcpy(tmp, ENG.widths[k], ENG.nw[k] * sizeof(float)); qsort(tmp, ENG.nw[k], sizeof(float), cmpf);
        float med = tmp[ENG.nw[k] / 2]; unsigned enr = 0; double extra = 0;
        for (unsigned i = 0; i < ENG.nw[k]; i++) if (ENG.widths[k][i] > 1.1f * med) { enr++; extra += ENG.widths[k][i] - med; }
        printf("   injector pulses %u, median %.3f ms, ENRICHED (>1.1x median) %u (%.1f%%), extra fuel %.1f%% of total\n", ENG.nw[k], med,
               enr, 100.0 * enr / ENG.nw[k], 100.0 * extra / (med * ENG.nw[k]));
      }
      for (int b = 0; b < 16; b++) if (ENG.inj[k][b].n)
        printf("   injector RA%d: %llu pulses, mean %.3f ms\n", b, (unsigned long long)ENG.inj[k][b].n, ENG.inj[k][b].width_ms_sum / ENG.inj[k][b].n);
    }
  }
  if (ENG.n0[0] && ENG.n0[1]) {                                  /* RA0 of both ECUs at the driver pins, side by side */
    unsigned i = 0, j = 0, m = 0; double sum1 = 0, sumu = 0, lag_sum = 0, ovl = 0;
    while (i < ENG.n0[0] && j < ENG.n0[1]) {
      uint64_t a0 = ENG.p0[0][i][0], a1 = ENG.p0[0][i][1], b0 = ENG.p0[1][j][0], b1 = ENG.p0[1][j][1];
      if (b1 < a0) { j++; continue; }
      if (a1 < b0) { i++; continue; }
      uint64_t u0 = a0 < b0 ? a0 : b0, u1 = a1 > b1 ? a1 : b1, o0 = a0 > b0 ? a0 : b0, o1 = a1 < b1 ? a1 : b1;
      sum1 += (a1 - a0) / ECU_SYSCLK_HZ * 1e3; sumu += (u1 - u0) / ECU_SYSCLK_HZ * 1e3; ovl += (o1 - o0) / ECU_SYSCLK_HZ * 1e3;
      lag_sum += ((double)b0 - (double)a0) / ECU_SYSCLK_HZ * 1e6; m++; i++; j++;
    }
    if (m) printf("two live ECUs, injector RA0: %u paired pulses; ECU1 alone %.3f ms; UNION (parallel wiring) %.3f ms (+%.1f%%); "
                  "overlap %.3f ms; ECU2 opens %.0f us after ECU1 on average\n", m, sum1 / m, sumu / m, 100.0 * (sumu - sum1) / sum1,
                  ovl / m, lag_sum / m);
  }
  s32board_report(&B, stdout);
  { time_t rt = s32_rtc_now(&RTC); struct tm tm; gmtime_r(&rt, &tm); char buf[32]; strftime(buf, sizeof buf, "%Y-%m-%d %H:%M:%S", &tm);
    printf("RTC chip: %s%s; Controller flash: %u page erases, %u row/word writes; ECU flash: %u/%u erases, %u/%u writes\n", buf,
           (RTC.reg[3] & 0x80) ? " (oscillator-stop flag set)" : "", CT.nvm_erases, CT.nvm_writes, ECU[0].nvm_erases, ECU[1].nvm_erases,
           ECU[0].nvm_writes, ECU[1].nvm_writes); }
  const char *ppm = getenv("S32_PPM") ? getenv("S32_PPM") : "s32pair_screen.ppm";
  { s32_panel_state ps; static uint8_t rgb[S32_PANEL_W * S32_PANEL_H * 3];   /* the panel as it shows (emu/common/s32panel.h) */
    s32_panel_render(&CT, B.on[SB_CT], rgb, &ps);
    printf("screen -> %s (%d)%s: panel %s%s, backlight %u per mille, G1CON1 %04X G1CON2 %04X G1CON3 %04X\n", ppm,
           s32_panel_ppm(&CT, B.on[SB_CT], ppm), B.on[SB_CT] ? "" : " (Controller unpowered: dark)",
           ps.lit ? "lit" : "dark: ", ps.lit ? "" : ps.why, (unsigned)ps.backlight_permille, ps.con1, ps.con2, ps.con3); }
  for (int w = 0; w < 3; w++) if (SD_IMG[w]) {
    if (sds[w]->dirty) { FILE *f = fopen(SD_PATH[w], "wb"); if (f) { fwrite(SD_IMG[w], 1, SD_N[w], f); fclose(f); } }
    printf("%s SD: %llu block reads, %llu block writes%s\n", w == SB_CT ? "Controller" : w == SB_E1 ? "ECU1" : "ECU2",
           (unsigned long long)sds[w]->reads, (unsigned long long)sds[w]->writes, sds[w]->dirty ? " (image written back)" : "");
  }
  for (int k = 0; k < 2; k++) {
    const char *fo = getenv(k == 0 ? "S32_ECU1_FRAM_OUT" : "S32_ECU2_FRAM_OUT");
    if (fo && *fo) { FILE *f = fopen(fo, "wb"); if (f) { fwrite(ECU_FRAM[k].mem, 1, sizeof ECU_FRAM[k].mem, f); fclose(f); } }
  }
  for (int k = 0; k < 2; k++) {                                         /* device-programmer readback of each ECU's flash */
    const char *fo = getenv(k == 0 ? "S32_ECU1_PFLASH_OUT" : "S32_ECU2_PFLASH_OUT");
    if (fo && *fo) {
      size_t n = ecu_pflash_read(&ECU[k], PFLK[k], ECU_PFL_BYTES);
      FILE *f = fopen(fo, "wb"); if (f) { fwrite(PFLK[k], 1, n, f); fclose(f); }
    }
  }
  { const char *fo = getenv("S32_CT_PFLASH_OUT");                       /* device-programmer readback of Controller flash */
    if (fo && *fo) {
      size_t n = s32ct_pflash_read(&CT, CT_PFL, S32CT_PFL_BYTES);
      FILE *f = fopen(fo, "wb"); if (f) { fwrite(CT_PFL, 1, n, f); fclose(f); }
    } }
  if (getenv("S32_CT_FRAM_OUT")) { FILE *f = fopen(getenv("S32_CT_FRAM_OUT"), "wb"); if (f) { fwrite(CT_FRAM.mem, 1, sizeof CT_FRAM.mem, f); fclose(f); } }
  if (iocov && *iocov) {
    FILE *f = fopen(iocov, "w");
    if (f) {
      fputs("{\"controller\":", f); s32_iocov_dump(CT.cov, f, ct_sfr_val, &CT);
      for (int k = 0; k < 2; k++) { fprintf(f, ",\"ecu%d\":", k + 1); s32_iocov_dump(ECU[k].cov, f, ecu_sfr_val, &ECU[k]); }
      fputs("}\n", f); fclose(f);
    }
  }
  if (trace_f) {                                                  /* final state, as a device programmer would read it */
    const uint8_t *pf[2]; size_t npf[2];
    for (int k = 0; k < 2; k++) { npf[k] = ecu_pflash_read(&ECU[k], PFLK[k], ECU_PFL_BYTES); pf[k] = PFLK[k]; }
    char sha[65];
    s32trace_finish(&TR, &B, pf, npf, sha);
    printf("trace: %llu lines, %llu bytes, sha256 %s\n", (unsigned long long)TR.lines, (unsigned long long)TR.bytes, sha);
    fclose(trace_f); s32trace_free(&TR);
  }
  int emu_rc = 0;
  for (int k = 0; k < 2; k++) if (EMU_ERR[k]) {
    printf("emulator error: ECU%d stopped with %s (err=%d) at PC=%08X\n",
           k + 1, ecu_err_str(EMU_ERR[k]), EMU_ERR[k], EMU_ERR_PC[k]);
    fprintf(stderr, "emulator error: ECU%d stopped with %s (err=%d) at PC=%08X\n",
            k + 1, ecu_err_str(EMU_ERR[k]), EMU_ERR[k], EMU_ERR_PC[k]);
    emu_rc = 4;
  }
  if (s32board_anchor_check(&B, stdout)) {   /* a phase-anchored stimulus that never acted: the run is not the case */
    s32board_anchor_check(&B, stderr);
    if (!emu_rc) emu_rc = 6;
  }
  tlog_close(&LINKLOG);
  if (LINKRAW) fclose(LINKRAW);
  if (CTCON) fclose(CTCON);
  tlog_close(&STATUSLOG); tlog_close(&SPARKLOG); tlog_close(&INJLOG);
  for (int k = 0; k < 2; k++) { if (ECON[k]) fclose(ECON[k]); ecu_free(&ECU[k]); free(PFLK[k]); }
  free(CT_PFL);
  return emu_rc;
}
