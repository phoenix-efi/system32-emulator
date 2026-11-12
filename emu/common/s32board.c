/* s32board.c -- the target system's harness and physical world (see s32board.h). Evidence labels for every signal are
 * in model/io_ledger.json; the comments here name the source of each physical rule. */
#include "s32board.h"
#include "s32panel.h"
#include "s32snap.h"
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ---------------------------------------------------------------- pins (MCU port index 0 = A .. 6 = G) */
enum { PA, PB, PC, PD, PE, PF, PG };
/* ECU: P-lead RB2, injector enable RC13, retard RD6, DIP 1-4 RD9-RD12, SD card detect RB3 / write enable RB4 (low =
 * card present / writable), crank (timing element) RE8 = INT1, sync element RE9 = INT2. */
#define E_PLEAD PB, 2
#define E_INJ PC, 13
#define E_RETARD PD, 6
#define E_SDCD PB, 3
#define E_SDWE PB, 4
/* Controller: ECU1 / ECU2 power sense RA2 / RA1 (cat5), buttons RA5 (knob push) RA3 (menu) RA4 (tune) RA7 (monitor),
 * encoder RA14/RA15, DB-15 digital inputs RG1 (pump 1 sense) RG8 (pump 2 sense), spare RG9 / RG14, SD detect RE3 (low =
 * present), SD write protect RE4 (high = protected). */
#define C_SDCD PE, 3
#define C_SDWP PE, 4

enum { EV_CONTACT = 1, EV_PLEAD, EV_INJ, EV_SELECT, EV_SELECT_SETTLE, EV_RETARD, EV_DIP, EV_PUMP, EV_DIGIN, EV_POWER,
       EV_SUPPLY, EV_BATT, EV_RESET, EV_CONSOLE, EV_SNAP, EV_FRAMFLIP, EV_DEV, EV_SDCARD, EV_RPM };
enum { AF_RAW, AF_OPEN, AF_GND, AF_VCC, AF_NOISE, AF_FLOAT, AF_STUCK, AF_INTERMIT };
enum { PF_OPEN, PF_GND, PF_VCC, PF_STUCK, PF_INTERMIT, PF_CHATTER };
enum { CF_GLITCH, CF_DROP, CF_STUCK_LO, CF_STUCK_HI, CF_EMI };
enum { LF_DEAD, LF_CUT_C2E, LF_CUT_E2C, LF_BER, LF_GARBAGE_CT, LF_GARBAGE_ECU, LF_FRAME_C2E, LF_FRAME_E2C };
enum { LF_SUB_CFG, LF_SUB_CFG_SHORT, LF_SUB_TRIM, LF_SUB_TRIM_SHORT, LF_SUB_FUEL_ROW, LF_SUB_FUEL_SHORT, LF_SUB_STATUS, LF_SUB_STATUS_SHORT, LF_SUB_CFG_BLOCK, LF_SUB_CFG_ANY, LF_SUB_CFG_PAD, LF_SUB_STATUS_PAD, LF_SUB_NOISE, LF_SUB_NOISE_CONT };
enum { DEV_FRAM, DEV_SD, DEV_RTC };

static uint32_t rnd(s32board *b) { b->rng ^= b->rng << 13; b->rng ^= b->rng >> 17; b->rng ^= b->rng << 5; return b->rng; }
static double rnd01(s32board *b) { return (rnd(b) >> 8) / 16777216.0; }
static uint16_t clamp10(double v) { return (uint16_t)(v < 0 ? 0 : v > 1023 ? 1023 : (int)(v + 0.5)); }
static int fail(s32board *b, const char *name, const char *what, const char *item) {
  snprintf(b->err, sizeof b->err, "%s: %s '%s'", name, what, item);
  return -2;
}

/* ---------------------------------------------------------------- events */
static sb_ev *ev_add(s32board *b, double t, uint8_t kind) {
  if (b->nev >= SB_MAX_EV) return NULL;
  unsigned i = b->nev++;
  while (i > b->iev && b->ev[i - 1].t > t) { b->ev[i] = b->ev[i - 1]; i--; }   /* stable: equal times keep order */
  memset(&b->ev[i], 0, sizeof b->ev[i]);
  b->ev[i].t = t; b->ev[i].kind = kind; b->ev[i].dyn = b->q > 0;   /* dyn: added by the running system (s32board_snap_restore) */
  return &b->ev[i];
}
static uint16_t text_add(s32board *b, const char *s) {
  size_t n = strlen(s) + 1;
  if (b->ntext + n > SB_TEXT) return 0xFFFF;
  memcpy(b->text + b->ntext, s, n);
  uint16_t at = (uint16_t)b->ntext; b->ntext += (unsigned)n;
  return at;
}
/* One mechanical contact moving to `level` at t: with bounce, it chatters before settling (S32_BOUNCE_MS). */
static void contact(s32board *b, double t, int who, unsigned port, unsigned bit, bool level, bool mechanical) {
  int n = mechanical && b->bounce_s > 0 ? b->bounce_edges : 0;
  for (int i = 0; i < n; i++) {
    sb_ev *e = ev_add(b, t + i * b->bounce_s / n, EV_CONTACT);
    if (e) { e->who = (uint8_t)who; e->port = (uint8_t)port; e->bit = (uint8_t)bit; e->level = (uint8_t)((i % 2) ? !level : level); }
  }
  sb_ev *e = ev_add(b, t + (n ? b->bounce_s : 0.0), EV_CONTACT);
  if (e) { e->who = (uint8_t)who; e->port = (uint8_t)port; e->bit = (uint8_t)bit; e->level = level; }
}
static bool enable_grounded(const s32board *b, int k) {   /* Drawing 9: the ECU Select switch grounds one enable wire */
  if (b->sw_legacy) return b->inj_gnd[k];
  return b->select == 3 || (b->select == 1 && k == 1) || (b->select == 2 && k == 0);
}

/* ---------------------------------------------------------------- init */
void s32board_init(s32board *b, s32board_host host) {
  memset(b, 0, sizeof *b);
  b->host = host;
  b->rng = 0x2545F491u;
  b->supply_v[0] = b->supply_v[1] = 13.8;
  b->batt_v[0] = b->batt_v[1] = 13.8;
  b->bor_v = 6.0;
  for (int w = 0; w < SB_WHO; w++) b->on[w] = true;
  b->first_spark_after_on[0] = b->first_spark_after_on[1] = -1;
  b->first_inj_after_on[0] = b->first_inj_after_on[1] = -1;
  b->on_at[0] = b->on_at[1] = -1;
  b->home_at = -1.0;
  b->start_s = 3.0; b->ext_rpm = -1; b->sync_before = 30.0; b->mag[0] = b->mag[1] = 80.0; b->hall_deg = 5.0; b->ncyl = 4;
  b->eh_last = -1;
  b->tps_supply = 0; b->tps_unpowered = 0;
  b->afr_fixed = -1; b->afr_live = 20.0;
  b->bench = bench_engine_six();
  inj_meter_init(&b->inj, ECU_SYSCLK_HZ, b->bench.inj_dead_ms, b->bench.inj_flow_lb_hr);
  b->select = 1;
  b->select_gap_s = 0.020;
  b->bounce_edges = 4;
  b->pump[0] = true;
  for (int w = 0; w < SB_WHO; w++) for (int c = 0; c < 32; c++) b->an_nom[w][c] = w == SB_CT ? 0 : 512;
  b->an_nom[SB_CT][15] = 512;                              /* ambient light sensor mid-scale */
  /* Board-owned input pins and their harness levels at power-on (idle = released / open / pulled up). */
  for (int k = 0; k < 2; k++) {
    int w = SB_E1 + k;
    b->own[w][PB] = (1u << 2) | (1u << 3) | (1u << 4);
    b->own[w][PC] = 1u << 13;
    b->own[w][PD] = (1u << 6) | (0xFu << 9);
    b->lvl[w][PB] = (1u << 2) | (1u << 3) | (1u << 4);    /* RUN, no SD card, not writable */
    b->lvl[w][PC] = enable_grounded(b, k) ? 0 : 1u << 13;
    b->lvl[w][PD] = 0xFu << 9;                             /* retard off (low), DIP switches open */
  }
  b->own[SB_CT][PA] = (1u << 1) | (1u << 2) | (1u << 3) | (1u << 4) | (1u << 5) | (1u << 7) | (1u << 14) | (1u << 15);
  b->own[SB_CT][PE] = (1u << 3) | (1u << 4);
  b->own[SB_CT][PG] = (1u << 1) | (1u << 8) | (1u << 9) | (1u << 14);
  b->lvl[SB_CT][PA] = b->own[SB_CT][PA];                  /* both ECUs powered, buttons released, encoder at rest */
  b->lvl[SB_CT][PE] = (1u << 3);                           /* no card */
  b->lvl[SB_CT][PG] = (1u << 1) | (1u << 9) | (1u << 14);  /* pump 1 on, pump 2 off, spare inputs open */
}

/* ---------------------------------------------------------------- knob parsing helpers */
/* The board whose knobs or bezel script are being parsed (s32board_knob, s32board_knobs_from_env,
 * s32board_input_script): a phase anchor in a time takes a slot there. NULL elsewhere (run length, snapshot points),
 * where an anchor is refused. Knobs are parsed on one thread, before or between quanta. */
static s32board *PARSE_B;
static bool parse_time_base(const char *s, const char **endp, double *out) {
  while (isspace((unsigned char)*s)) s++;
  if (s[0] == '{' && (s[1] == 'H' || s[1] == 'h')) {
    const char *p = s + 2;
    double off = 0.0;
    if (*p == '+') {
      char *end; off = strtod(p, &end);
      if (end == p || off < 0.0) return false;
      p = end;
    }
    if (*p != '}') return false;
    *out = SB_HOME_T0 + off;
    if (endp) *endp = p + 1;
    return true;
  }
  if (!strncmp(s, "home", 4) && !isalnum((unsigned char)s[4]) && s[4] != '_') {
    const char *p = s + 4;
    double off = 0.0;
    if (*p == '+') {
      char *end; off = strtod(p, &end);
      if (end == p || off < 0.0) return false;
      p = end;
    }
    *out = SB_HOME_T0 + off;
    if (endp) *endp = p;
    return true;
  }
  char *end; double v = strtod(s, &end);
  if (end == s) return false;
  *out = v;
  if (endp) *endp = end;
  return true;
}
static const struct { const char *name; uint8_t src; } ANCHOR_SRC[] = {
  {"ct1", SB_ANC_CT1}, {"ct2", SB_ANC_CT2}, {"ecu1", SB_ANC_ECU1}, {"ecu2", SB_ANC_ECU2}, {"panel", SB_ANC_PANEL}};
/* "~<event>[#type][+x]" after a time `base` (s .. p is the text so far): the anchored time, or false. */
static bool parse_anchor(const char *s, const char *p, double base, const char **endp, double *out);
bool s32board_parse_time(const char *s, const char **endp, double *out) {
  const char *p;
  double t;
  if (!parse_time_base(s, &p, &t)) return false;
  while (*p == '~')                         /* chained: "<t>~ct1#8+3~ct1#8" waits again, from 3 s after the first event */
    if (!parse_anchor(s, p, t, &p, &t)) return false;
  *out = t;
  if (endp) *endp = p;
  return true;
}
static bool parse_anchor(const char *s, const char *p, double base, const char **endp, double *out) {
  s32board *b = PARSE_B;
  if (!b || b->nanchor >= SB_MAX_ANCHOR || !(base >= 0.0)) return false;
  const char *q = p + 1;
  int src = -1;
  for (size_t i = 0; i < sizeof ANCHOR_SRC / sizeof ANCHOR_SRC[0]; i++) {
    size_t n = strlen(ANCHOR_SRC[i].name);
    if (!strncmp(q, ANCHOR_SRC[i].name, n) && !isalnum((unsigned char)q[n])) { src = ANCHOR_SRC[i].src; q += n; break; }
  }
  if (src < 0) return false;
  long type = -1;
  if (*q == '#') {
    if (src == SB_ANC_PANEL) return false;
    char *e; type = strtol(q + 1, &e, 10);
    if (e == q + 1 || type < 0 || type > 0xFFFF) return false;
    q = e;
  }
  const char *spec_end = q;                 /* the anchor is "<base>~<event>[#type]"; "+x" is this time's own offset */
  double off = 0.0;
  if (*q == '+') {
    char *e; off = strtod(q + 1, &e);
    if (e == q + 1 || !(off >= 0.0) || off >= SB_ANCHOR_SPAN / 2) return false;
    q = e;
  }
  /* One slot per anchor: times with the same base, event and type share it (one event, several offsets after it). */
  unsigned slot = 0;
  while (slot < b->nanchor && !(b->anchor[slot].base == base && b->anchor[slot].src == src &&
                                b->anchor[slot].type == type && !b->anchor[slot].fired)) slot++;
  if (slot == b->nanchor) {
    while (s < p && isspace((unsigned char)*s)) s++;
    sb_anchor *a = &b->anchor[b->nanchor++];
    memset(a, 0, sizeof *a);
    a->base = base; a->src = (uint8_t)src; a->type = (int16_t)type; a->at = -1.0;
    size_t n = (size_t)(spec_end - s) < sizeof a->spec - 1 ? (size_t)(spec_end - s) : sizeof a->spec - 1;
    memcpy(a->spec, s, n); a->spec[n] = '\0';
  }
  *out = SB_ANCHOR_T0 + slot * SB_ANCHOR_SPAN + off;
  *endp = q;
  return true;
}
static int who_of(const char *s, const char **rest) {
  if (!strncmp(s, "ct", 2) || !strncmp(s, "CT", 2)) { *rest = s + 2; return SB_CT; }
  if (s[0] == '1' || s[0] == '2') { *rest = s + 1; return s[0] - '0'; }
  return -1;
}
/* "@t0[-t1]" -> window (t1 = +inf if absent). Returns false if no '@'. */
static bool window(const char *s, double *t0, double *t1) {
  const char *at = strrchr(s, '@'), *end;
  if (!at || !s32board_parse_time(at + 1, &end, t0)) return false;
  if (*end == '-') {
    if (!s32board_parse_time(end + 1, &end, t1)) return false;
  } else *t1 = INFINITY;
  return true;
}
typedef int (*item_fn)(s32board *b, const char *name, char *item);
static int each_item(s32board *b, const char *name, const char *value, item_fn fn) {
  char *buf = strdup(value);
  if (!buf) return fail(b, name, "out of memory", "");
  char *save = NULL;
  for (char *t = strtok_r(buf, ",", &save); t; t = strtok_r(NULL, ",", &save)) {
    while (isspace((unsigned char)*t)) t++;
    if (!*t) continue;
    int r = fn(b, name, t);
    if (r) { free(buf); return r; }
  }
  free(buf);
  return 0;
}
static bool parse_pin(const char *s, unsigned *port, unsigned *bit) {    /* "RB2" */
  if (toupper((unsigned char)s[0]) != 'R' || toupper((unsigned char)s[1]) < 'A' || toupper((unsigned char)s[1]) > 'G') return false;
  char *end; long n = strtol(s + 2, &end, 10);
  if (end == s + 2 || n < 0 || n > 15) return false;
  *port = (unsigned)(toupper((unsigned char)s[1]) - 'A'); *bit = (unsigned)n;
  return true;
}
static bool on_off(const char *v, bool *out) {
  if (!strcmp(v, "on") || !strcmp(v, "1") || !strcmp(v, "high") || !strcmp(v, "hi")) { *out = true; return true; }
  if (!strcmp(v, "off") || !strcmp(v, "0") || !strcmp(v, "low") || !strcmp(v, "lo")) { *out = false; return true; }
  return false;
}

/* ---------------------------------------------------------------- knobs */
static int k_rpm(s32board *b, const char *n, const char *v) { (void)n; b->rpm = atof(v); return 0; }
static int k_start(s32board *b, const char *n, const char *v) {
  const char *end;
  if (!s32board_parse_time(v, &end, &b->start_s) || *end) return fail(b, n, "want start time", v);
  return 0;
}
static int k_cyl(s32board *b, const char *n, const char *v) {
  int c = atoi(v); if (c != 4 && c != 6 && c != 8) return fail(b, n, "cylinders must be 4, 6 or 8", v);
  b->ncyl = c; return 0;
}
static int k_sync(s32board *b, const char *n, const char *v) { (void)n; b->sync_before = atof(v); return 0; }
static int k_mag1(s32board *b, const char *n, const char *v) { (void)n; b->mag[0] = atof(v); return 0; }
static int k_mag2(s32board *b, const char *n, const char *v) { (void)n; b->mag[1] = atof(v); return 0; }
static int k_hall_w(s32board *b, const char *n, const char *v) {
  double d = atof(v); if (d <= 0 || d >= 60) return fail(b, n, "pulse width must be 0..60 deg", v);
  b->hall_deg = d; return 0;
}
static int k_jitter(s32board *b, const char *n, const char *v) { (void)n; b->jitter_us = fabs(atof(v)); return 0; }
static int it_rpm_at(s32board *b, const char *n, char *t) {
  const char *end; double at, r;
  if (!s32board_parse_time(t, &end, &at) || *end != '=' || sscanf(end + 1, "%lf", &r) != 1) return fail(b, n, "want t=rpm", t);
  sb_ev *e = ev_add(b, at, EV_RPM);
  if (!e) return fail(b, n, "event queue full", t);
  e->v = r; return 0;
}
static int k_rpm_at(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_rpm_at); }
/* crank faults: "k:t[:us]" glitch; "k:t0-t1[:crank|sync|all]" drop; "k:crank|sync=low|high@t0[-t1]" stuck;
 * "k:<hz>@t0[-t1]" EMI */
static int crank_elem(const char *s) { return !strcmp(s, "sync") ? 2 : !strcmp(s, "all") ? 3 : !strcmp(s, "crank") || !*s ? 1 : -1; }
static int it_glitch(s32board *b, const char *n, char *t) {
  int k; double at, us = 20.0; char el[16] = "crank"; const char *end; int used = 0;
  if (sscanf(t, "%d:%n", &k, &used) < 1 || !used || !s32board_parse_time(t + used, &end, &at))
    return fail(b, n, "want k:t[:us[:crank|sync]]", t);
  if (*end == ':') sscanf(end + 1, "%lf:%15s", &us, el);
  else if (*end) return fail(b, n, "want k:t[:us[:crank|sync]]", t);
  if (k < 1 || k > 2 || b->ncf >= SB_MAX_CF || crank_elem(el) < 1) return fail(b, n, "want k:t[:us[:crank|sync]]", t);
  b->cf[b->ncf++] = (sb_cfault){.k = (uint8_t)(k - 1), .kind = CF_GLITCH, .t0 = at, .t1 = at, .us = us, .elem = (uint8_t)crank_elem(el)};
  return 0;
}
static int k_glitch(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_glitch); }
static int it_drop(s32board *b, const char *n, char *t) {
  int k; double a, c; char el[16] = ""; const char *end; int used = 0;
  if (sscanf(t, "%d:%n", &k, &used) < 1 || !used || !s32board_parse_time(t + used, &end, &a) || *end != '-' ||
      !s32board_parse_time(end + 1, &end, &c))
    return fail(b, n, "want k:t0-t1[:crank|sync|all]", t);
  if (*end == ':') snprintf(el, sizeof el, "%s", end + 1);
  else if (*end) return fail(b, n, "want k:t0-t1[:crank|sync|all]", t);
  if (k < 1 || k > 2 || b->ncf >= SB_MAX_CF || crank_elem(el) < 1) return fail(b, n, "want k:t0-t1[:crank|sync|all]", t);
  b->cf[b->ncf++] = (sb_cfault){.k = (uint8_t)(k - 1), .kind = CF_DROP, .t0 = a, .t1 = c, .elem = (uint8_t)crank_elem(el)};
  return 0;
}
static int k_drop(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_drop); }
static int it_stuck(s32board *b, const char *n, char *t) {
  int k; char el[16], lv[16]; double a, c;
  if (sscanf(t, "%d:%15[^=]=%15[^@]", &k, el, lv) != 3 || k < 1 || k > 2 || crank_elem(el) < 1 || crank_elem(el) == 3 ||
      !window(t, &a, &c) || b->ncf >= SB_MAX_CF || (strcmp(lv, "low") && strcmp(lv, "high")))
    return fail(b, n, "want k:crank|sync=low|high@t0[-t1]", t);
  b->cf[b->ncf++] = (sb_cfault){.k = (uint8_t)(k - 1), .kind = !strcmp(lv, "low") ? CF_STUCK_LO : CF_STUCK_HI, .t0 = a, .t1 = c, .elem = (uint8_t)crank_elem(el)};
  return 0;
}
static int k_stuck(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_stuck); }
static int it_emi(s32board *b, const char *n, char *t) {
  int k; double hz, a, c; char el[16] = "crank";
  if (sscanf(t, "%d:%lf", &k, &hz) != 2 || k < 1 || k > 2 || hz <= 0 || hz > 20000 || !window(t, &a, &c) || b->ncf >= SB_MAX_CF)
    return fail(b, n, "want k:<pulses per s>[:sync]@t0[-t1]", t);
  if (strstr(t, ":sync")) snprintf(el, sizeof el, "sync");
  b->cf[b->ncf++] = (sb_cfault){.k = (uint8_t)(k - 1), .kind = CF_EMI, .t0 = a, .t1 = c, .rate = hz, .us = 10.0, .elem = (uint8_t)crank_elem(el)};
  return 0;
}
static int k_emi(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_emi); }

/* analogue */
static int an_set_item(s32board *b, const char *n, char *t, int dflt_who) {
  const char *rest = t; int w = dflt_who;
  if (strchr(t, ':')) { w = who_of(t, &rest); if (w < 0 || *rest != ':') return fail(b, n, "want [ct|1|2:]ch=raw", t); rest++; }
  unsigned ch, raw;
  if (sscanf(rest, "%u=%u", &ch, &raw) != 2 || ch > 31 || raw > 1023) return fail(b, n, "want ch=raw (0..1023)", t);
  if (w < 0) {                                           /* both ECUs: the sensor value (ET reaches ECU2 only if wired) */
    for (int k = 0; k < 2; k++) if (!(ch == 7 && k == 1 && !b->et_both)) { b->an_nom[SB_E1 + k][ch] = (uint16_t)raw; b->an_host[SB_E1 + k][ch] = true; }
  } else { b->an_nom[w][ch] = (uint16_t)raw; b->an_host[w][ch] = true; }
  return 0;
}
static int it_an(s32board *b, const char *n, char *t) { return an_set_item(b, n, t, -1); }
static int k_an(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_an); }
static int it_ct_an(s32board *b, const char *n, char *t) { return an_set_item(b, n, t, SB_CT); }
static int k_ct_an(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_ct_an); }
static int k_tps_noise(s32board *b, const char *n, const char *v) {
  (void)n; uint16_t z = (uint16_t)atoi(v); b->an_noise[SB_E1][9] = b->an_noise[SB_E2][9] = z; return 0;
}
static int af_add(s32board *b, const char *n, int w, unsigned ch, sb_afault f, const char *t) {
  if (w < 0 || ch > 31 || b->naf[w][ch] >= SB_MAX_AF) return fail(b, n, "too many faults / bad channel", t);
  b->af[w][ch][b->naf[w][ch]++] = f; return 0;
}
static int it_tps_fault(s32board *b, const char *n, char *t) {   /* "k:raw[~noise]@t" (one ECU's AN9) */
  int k; unsigned raw, noise = 0; double s, t1;
  if (sscanf(t, "%d:%u~%u@", &k, &raw, &noise) != 3) { noise = 0; if (sscanf(t, "%d:%u@", &k, &raw) != 2) k = 0; }
  if (k < 1 || k > 2 || raw > 1023 || noise > 1023 || !window(t, &s, &t1)) return fail(b, n, "want k:raw[~noise]@t", t);
  return af_add(b, n, k, 9, (sb_afault){.t0 = s, .t1 = INFINITY, .mode = AF_RAW, .val = (uint16_t)raw, .noise = (uint16_t)noise}, t);
}
static int k_tps_fault(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_tps_fault); }
/* "who:ch=mode@t0[-t1]": open | gnd | vcc | raw:N | noise:N | float | stuck | intermittent:ms */
static int it_an_fault(s32board *b, const char *n, char *t) {
  const char *rest; int w = who_of(t, &rest);
  unsigned ch; char mode[32];
  double a, c;
  if (w < 0 || *rest != ':' || sscanf(rest + 1, "%u=%31[^@]", &ch, mode) != 2 || !window(t, &a, &c)) return fail(b, n, "want who:ch=mode@t0[-t1]", t);
  if (strlen(mode) >= sizeof mode - 1) return fail(b, n, "mode too long", t);   /* never act on a cut-off value */
  sb_afault f = {.t0 = a, .t1 = c};
  if (!strcmp(mode, "open")) f.mode = AF_OPEN;
  else if (!strcmp(mode, "gnd")) f.mode = AF_GND;
  else if (!strcmp(mode, "vcc")) f.mode = AF_VCC;
  else if (!strcmp(mode, "float")) f.mode = AF_FLOAT;
  else if (!strcmp(mode, "stuck")) f.mode = AF_STUCK;
  else if (!strncmp(mode, "raw:", 4)) { f.mode = AF_RAW; f.val = (uint16_t)atoi(mode + 4); }
  else if (!strncmp(mode, "noise:", 6)) { f.mode = AF_NOISE; f.noise = (uint16_t)atoi(mode + 6); }
  else if (!strncmp(mode, "intermittent:", 13)) { f.mode = AF_INTERMIT; f.period = atof(mode + 13) / 1000.0; if (f.period <= 0) return fail(b, n, "bad period", t); }
  else if (isdigit((unsigned char)mode[0])) { f.mode = AF_RAW; f.val = (uint16_t)atoi(mode); }
  else return fail(b, n, "mode: open|gnd|vcc|raw:N|noise:N|float|stuck|intermittent:ms", t);
  if (f.val > 1023 || f.noise > 1023) return fail(b, n, "counts are 0..1023", t);
  return af_add(b, n, w, ch, f, t);
}
static int k_an_fault(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_an_fault); }
static int k_tps_supply(s32board *b, const char *n, const char *v) {
  int k = atoi(v); if (k != 1 && k != 2) return fail(b, n, "1 or 2", v); b->tps_supply = k - 1; return 0;
}
static int k_tps_unpowered(s32board *b, const char *n, const char *v) {
  if (!strcmp(v, "gnd")) b->tps_unpowered = 0; else if (!strcmp(v, "float")) b->tps_unpowered = 1;
  else if (!strcmp(v, "pullup")) b->tps_unpowered = 2; else return fail(b, n, "gnd|float|pullup", v);
  return 0;
}
static int k_et_wiring(s32board *b, const char *n, const char *v) {
  if (!strcmp(v, "ecu1")) b->et_both = false; else if (!strcmp(v, "both")) b->et_both = true; else return fail(b, n, "ecu1|both", v);
  return 0;
}
static int it_batt(s32board *b, const char *n, char *t) {   /* "n=volts[@t]" */
  int i; double vv, at = 0;
  if (sscanf(t, "%d=%lf", &i, &vv) != 2 || i < 1 || i > 2) return fail(b, n, "want 1|2=volts[@t]", t);
  double t1;
  if (window(t, &at, &t1)) {
    sb_ev *e = ev_add(b, at, EV_BATT);
    if (!e) return fail(b, n, "event queue full", t);
    e->a = (uint32_t)(i - 1); e->v = vv;
  } else b->batt_v[i - 1] = vv;
  return 0;
}
static int k_batt(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_batt); }
static int k_afr(s32board *b, const char *n, const char *v) {
  if (!strcmp(v, "auto")) { b->afr_fixed = -1; return 0; }
  double a = atof(v); if (a < 8 || a > 22) return fail(b, n, "auto or an AFR 8..22", v);
  b->afr_fixed = a; return 0;
}

/* switches */
static int k_switches(s32board *b, const char *n, const char *v) {   /* legacy "e1 e2": on|plead|inj|off */
  char a[16] = "on", c[16] = "on";
  if (sscanf(v, "%15s %15s", a, c) < 1) return fail(b, n, "want <ecu1> <ecu2>", v);
  const char *m[2] = {a, c};
  for (int k = 0; k < 2; k++) {
    if (strcmp(m[k], "on") && strcmp(m[k], "plead") && strcmp(m[k], "inj") && strcmp(m[k], "off")) return fail(b, n, "on|plead|inj|off", m[k]);
    b->plead_stop[k] = !strcmp(m[k], "plead") || !strcmp(m[k], "off");
    b->inj_gnd[k] = !strcmp(m[k], "inj") || !strcmp(m[k], "off");
  }
  b->sw_legacy = true;
  for (int k = 0; k < 2; k++) {
    int w = SB_E1 + k;
    b->lvl[w][PB] = (uint16_t)((b->lvl[w][PB] & ~(1u << 2)) | (b->plead_stop[k] ? 0 : 1u << 2));
    b->lvl[w][PC] = (uint16_t)((b->lvl[w][PC] & ~(1u << 13)) | (b->inj_gnd[k] ? 0 : 1u << 13));
  }
  return 0;
}
static int select_pos(const char *v) {
  return !strcmp(v, "1") ? 1 : !strcmp(v, "2") ? 2 : !strcmp(v, "both") || !strcmp(v, "open") ? 0 : !strcmp(v, "none") || !strcmp(v, "gnd") ? 3 : -1;
}
static int k_select(s32board *b, const char *n, const char *v) {
  int p = select_pos(v); if (p < 0) return fail(b, n, "1|2|both (no switch: both enable wires open)|none (both grounded)", v);
  b->select = p; b->sw_legacy = false;
  for (int k = 0; k < 2; k++) b->lvl[SB_E1 + k][PC] = (uint16_t)((b->lvl[SB_E1 + k][PC] & ~(1u << 13)) | (enable_grounded(b, k) ? 0 : 1u << 13));
  return 0;
}
static int k_select_gap(s32board *b, const char *n, const char *v) { (void)n; b->select_gap_s = atof(v) / 1000.0; return 0; }
static int k_retard(s32board *b, const char *n, const char *v) {   /* "on|off" (both) or "1:on,2:off" */
  bool on;
  if (on_off(v, &on)) { for (int k = 0; k < 2; k++) { b->retard[k] = on; b->lvl[SB_E1 + k][PD] = (uint16_t)((b->lvl[SB_E1 + k][PD] & ~(1u << 6)) | (on ? 1u << 6 : 0)); } return 0; }
  int k; char s[8];
  if (sscanf(v, "%d:%7s", &k, s) == 2 && k >= 1 && k <= 2 && on_off(s, &on)) {
    b->retard[k - 1] = on; b->lvl[SB_E1 + k - 1][PD] = (uint16_t)((b->lvl[SB_E1 + k - 1][PD] & ~(1u << 6)) | (on ? 1u << 6 : 0)); return 0;
  }
  return fail(b, n, "on|off or k:on|off", v);
}
static int it_dip(s32board *b, const char *n, char *t) {   /* "k:n=on|off" (on = closed to ground) */
  int k, d; char s[8]; bool on;
  if (sscanf(t, "%d:%d=%7s", &k, &d, s) != 3 || k < 1 || k > 2 || d < 1 || d > 4 || !on_off(s, &on)) return fail(b, n, "want k:n=on|off", t);
  b->dip[k - 1][d - 1] = on;
  b->lvl[SB_E1 + k - 1][PD] = (uint16_t)((b->lvl[SB_E1 + k - 1][PD] & ~(1u << (8 + d))) | (on ? 0 : 1u << (8 + d)));
  return 0;
}
static int k_dip(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_dip); }
static int it_pump(s32board *b, const char *n, char *t) {   /* "n=on|off" */
  int i; char s[8]; bool on;
  if (sscanf(t, "%d=%7s", &i, s) != 2 || i < 1 || i > 2 || !on_off(s, &on)) return fail(b, n, "want 1|2=on|off", t);
  b->pump[i - 1] = on;
  unsigned bit = i == 1 ? 1 : 8;
  b->lvl[SB_CT][PG] = (uint16_t)((b->lvl[SB_CT][PG] & ~(1u << bit)) | (on ? 1u << bit : 0));
  return 0;
}
static int k_pump(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_pump); }
static int it_digin(s32board *b, const char *n, char *t) {   /* "3|4=lo|hi" */
  int i; char s[8]; bool hi;
  if (sscanf(t, "%d=%7s", &i, s) != 2 || i < 3 || i > 4 || !on_off(s, &hi)) return fail(b, n, "want 3|4=lo|hi", t);
  b->digin_lo[i - 3] = !hi;
  unsigned bit = i == 3 ? 9 : 14;
  b->lvl[SB_CT][PG] = (uint16_t)((b->lvl[SB_CT][PG] & ~(1u << bit)) | (hi ? 1u << bit : 0));
  return 0;
}
static int k_digin(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_digin); }
/* timed switch moves: "t:k:plead=stop|run", "t:k:inj=gnd|open", "t:select=1|2|both|none", "t:k:retard=on|off",
 * "t:k:dip<n>=on|off", "t:pump<n>=on|off", "t:digin<n>=lo|hi" */
static int it_switch(s32board *b, const char *n, char *t) {
  double at; char rest[64]; const char *end;
  if (!s32board_parse_time(t, &end, &at) || *end != ':' || strlen(end + 1) >= sizeof rest || sscanf(end + 1, "%63s", rest) != 1) return fail(b, n, "want t:...", t);
  int k = 0; char *what = rest;
  if ((rest[0] == '1' || rest[0] == '2') && rest[1] == ':') { k = rest[0] - '0'; what = rest + 2; }
  char *eq = strchr(what, '='); if (!eq) return fail(b, n, "want name=value", t);
  *eq = 0; const char *val = eq + 1; bool on;
  sb_ev *e = NULL;
  if (!strcmp(what, "select")) { int p = select_pos(val); if (p < 0) return fail(b, n, "select=1|2|both|none", t); e = ev_add(b, at, EV_SELECT); if (!e) return fail(b, n, "event queue full", t); e->a = (uint32_t)p; return 0; }
  if (!strncmp(what, "pump", 4)) { int i = atoi(what + 4); if (i < 1 || i > 2 || !on_off(val, &on)) return fail(b, n, "pump1|pump2=on|off", t); e = ev_add(b, at, EV_PUMP); if (!e) return fail(b, n, "event queue full", t); e->a = (uint32_t)(i - 1); e->level = on; return 0; }
  if (!strncmp(what, "digin", 5)) { int i = atoi(what + 5); if (i < 3 || i > 4 || !on_off(val, &on)) return fail(b, n, "digin3|digin4=lo|hi", t); e = ev_add(b, at, EV_DIGIN); if (!e) return fail(b, n, "event queue full", t); e->a = (uint32_t)(i - 3); e->level = on; return 0; }
  if (k < 1) return fail(b, n, "this switch needs an ECU (t:k:...)", t);
  if (!strcmp(what, "plead")) { if (strcmp(val, "stop") && strcmp(val, "run")) return fail(b, n, "plead=stop|run", t); e = ev_add(b, at, EV_PLEAD); if (!e) return fail(b, n, "event queue full", t); e->a = (uint32_t)(k - 1); e->level = !strcmp(val, "stop"); return 0; }
  if (!strcmp(what, "inj")) { if (strcmp(val, "gnd") && strcmp(val, "open")) return fail(b, n, "inj=gnd|open", t); e = ev_add(b, at, EV_INJ); if (!e) return fail(b, n, "event queue full", t); e->a = (uint32_t)(k - 1); e->level = !strcmp(val, "gnd"); return 0; }
  if (!strcmp(what, "retard")) { if (!on_off(val, &on)) return fail(b, n, "retard=on|off", t); e = ev_add(b, at, EV_RETARD); if (!e) return fail(b, n, "event queue full", t); e->a = (uint32_t)(k - 1); e->level = on; return 0; }
  if (!strncmp(what, "dip", 3)) { int d = atoi(what + 3); if (d < 1 || d > 4 || !on_off(val, &on)) return fail(b, n, "dip1..4=on|off", t); e = ev_add(b, at, EV_DIP); if (!e) return fail(b, n, "event queue full", t); e->a = (uint32_t)(k - 1); e->bit = (uint8_t)(d - 1); e->level = on; return 0; }
  return fail(b, n, "unknown switch", t);
}
static int k_switch(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_switch); }
static int k_bounce_ms(s32board *b, const char *n, const char *v) { (void)n; b->bounce_s = atof(v) / 1000.0; return 0; }
static int k_bounce_edges(s32board *b, const char *n, const char *v) {
  int e = atoi(v); if (e < 0 || e > 50) return fail(b, n, "0..50", v); b->bounce_edges = e; return 0;
}
/* digital wire faults: "who:RXn=open|gnd|vcc|stuck|intermittent:ms|chatter:hz@t0[-t1]" */
static int it_pin_fault(s32board *b, const char *n, char *t) {
  const char *rest; int w = who_of(t, &rest);
  char pin[8], mode[32]; unsigned port, bit; double a, c;
  if (w < 0 || *rest != ':' || sscanf(rest + 1, "%7[^=]=%31[^@]", pin, mode) != 2 || !parse_pin(pin, &port, &bit) || !window(t, &a, &c) || b->npf >= SB_MAX_PF)
    return fail(b, n, "want who:RXn=mode@t0[-t1]", t);
  if (strlen(pin) >= sizeof pin - 1 || strlen(mode) >= sizeof mode - 1) return fail(b, n, "pin or mode too long", t);
  sb_pfault f = {.t0 = a, .t1 = c, .who = (uint8_t)w, .port = (uint8_t)port, .bit = (uint8_t)bit};
  if (!strcmp(mode, "open")) f.mode = PF_OPEN; else if (!strcmp(mode, "gnd")) f.mode = PF_GND; else if (!strcmp(mode, "vcc")) f.mode = PF_VCC;
  else if (!strcmp(mode, "stuck")) f.mode = PF_STUCK;
  else if (!strncmp(mode, "intermittent:", 13)) { f.mode = PF_INTERMIT; f.period = atof(mode + 13) / 1000.0; }
  else if (!strncmp(mode, "chatter:", 8)) { f.mode = PF_CHATTER; f.period = atof(mode + 8); }
  else return fail(b, n, "mode: open|gnd|vcc|stuck|intermittent:ms|chatter:hz", t);
  if ((f.mode == PF_INTERMIT || f.mode == PF_CHATTER) && f.period <= 0) return fail(b, n, "bad period/rate", t);
  if (w != SB_CT && port == PE && (bit == 8 || bit == 9)) return fail(b, n, "crank/sync pins: use S32_HALL_STUCK / S32_CRANK_DROP", t);
  if (!(b->own[w][port] & (1u << bit))) {                  /* a wire fault makes the pin the board's to drive: idle high */
    b->own[w][port] |= (uint16_t)(1u << bit);
    b->lvl[w][port] |= (uint16_t)(1u << bit);
  }
  b->pf[b->npf++] = f;
  return 0;
}
static int k_pin_fault(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_pin_fault); }

/* power */
static int it_power(s32board *b, const char *n, char *t) {   /* "k:off@t" "k:on@t" "k:boot@t" */
  int k; double s, t1; char what[8];
  if (sscanf(t, "%d:%7[a-z]@", &k, what) != 2 || !window(t, &s, &t1) || k < 1 || k > 2 || (strcmp(what, "off") && strcmp(what, "on") && strcmp(what, "boot")))
    return fail(b, n, "want k:off|on|boot@t", t);
  sb_ev *e = ev_add(b, s, EV_POWER);
  if (!e) return fail(b, n, "event queue full", t);
  e->a = (uint32_t)(k - 1); e->level = strcmp(what, "off") != 0;
  return 0;
}
static int k_power(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_power); }
static int it_supply(s32board *b, const char *n, char *t) {   /* "k:volts@t" */
  int k; double vv, s, t1;
  if (sscanf(t, "%d:%lf@", &k, &vv) != 2 || !window(t, &s, &t1) || k < 1 || k > 2 || vv < 0 || vv > 40) return fail(b, n, "want k:volts@t", t);
  sb_ev *e = ev_add(b, s, EV_SUPPLY);
  if (!e) return fail(b, n, "event queue full", t);
  e->a = (uint32_t)(k - 1); e->v = vv;
  return 0;
}
static int k_supply(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_supply); }
static int k_bor(s32board *b, const char *n, const char *v) { (void)n; b->bor_v = atof(v); return 0; }
static int it_reset(s32board *b, const char *n, char *t) {   /* "who@t" */
  const char *rest, *end; int w = who_of(t, &rest); double s;
  if (w < 0 || *rest != '@' || !s32board_parse_time(rest + 1, &end, &s) || *end) return fail(b, n, "want ct|1|2@t", t);
  sb_ev *e = ev_add(b, s, EV_RESET);
  if (!e) return fail(b, n, "event queue full", t);
  e->who = (uint8_t)w;
  return 0;
}
static int k_reset(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_reset); }
static int it_ppm(s32board *b, const char *n, char *t) {   /* "who=ppm" */
  const char *rest; int w = who_of(t, &rest); double p;
  if (w < 0 || sscanf(rest, "=%lf", &p) != 1 || fabs(p) > 10000) return fail(b, n, "want ct|1|2=ppm", t);
  b->ppm[w] = p; return 0;
}
static int k_ppm(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_ppm); }

/* links */
static int parse_link_frame(s32board *b, const char *n, const char *t, const char *body_in, sb_lfault *f) {
  char body[128];
  if (strlen(body_in) >= sizeof body) return fail(b, n, "frame spec too long", t);
  snprintf(body, sizeof body, "%s", body_in);
  f->pace = 4; f->burst = 1;
  char *star = strrchr(body, '*');
  if (star) {
    int bv = atoi(star + 1);
    if (bv < 1 || bv > 8) return fail(b, n, "burst count must be 1..8", t);
    f->burst = (uint8_t)bv; *star = '\0';
  }
  {                                                     /* :gN suffix: pace one byte every N quanta (0.1 ms each) */
    char *gc = strrchr(body, ':');
    if (gc && gc[1] == 'g' && gc[2]) {
      char *end = NULL; long gv = strtol(gc + 2, &end, 10);
      if (!end || *end || gv < 1 || gv > 255) return fail(b, n, ":gN pace must be 1..255 quanta", t);
      f->pace = (uint8_t)gv; *gc = '\0';
    }
  }
  size_t blen = strlen(body);
  if (blen > 5 && !strcmp(body + blen - 5, ":full")) { f->pace = 0; body[blen - 5] = '\0'; }
  int a1 = 0, a2 = 0, a3 = 0;
  if (sscanf(body, "cfg:%d=%d", &a1, &a2) == 2 && a1 >= 1 && a1 <= 28) { f->sub = LF_SUB_CFG; f->arg1 = a1; f->arg2 = a2; }
  else if (sscanf(body, "cfg-short:%d", &a1) == 1 && a1 >= 1 && a1 <= 28) { f->sub = LF_SUB_CFG_SHORT; f->arg1 = a1; }
  else if (sscanf(body, "cfg-any:%d=%d", &a1, &a2) == 2 && a1 >= 0 && a1 <= 1000) { f->sub = LF_SUB_CFG_ANY; f->arg1 = a1; f->arg2 = a2; }
  else if (sscanf(body, "cfg-pad:%d=%d:%d", &a1, &a2, &a3) == 3 && a1 >= 0 && a1 <= 1000 && a3 >= 12 && a3 <= 440) { f->sub = LF_SUB_CFG_PAD; f->arg1 = a1; f->arg2 = a2; f->arg3 = a3; }
  else if (sscanf(body, "status-pad:%d:%d", &a1, &a3) == 2 && a1 >= 0 && a1 <= 20000 && a3 >= 52 && a3 <= 440) { f->sub = LF_SUB_STATUS_PAD; f->arg1 = a1; f->arg3 = a3; }
  else if (sscanf(body, "trim:%d", &a1) == 1) { f->sub = LF_SUB_TRIM; f->arg1 = a1; }
  else if (sscanf(body, "trim-short:%d", &a1) == 1) { f->sub = LF_SUB_TRIM_SHORT; f->arg1 = a1; }
  else if (sscanf(body, "fuel-row:%d=%d", &a1, &a2) == 2 && a1 >= 0 && a1 < 16 && a2 >= 0 && a2 <= 255) { f->sub = LF_SUB_FUEL_ROW; f->arg1 = a1; f->arg2 = a2; }
  else if (sscanf(body, "fuel-short:%d", &a1) == 1 && a1 >= 1 && a1 < (int)FRAM_TABLE_BYTES) { f->sub = LF_SUB_FUEL_SHORT; f->arg1 = a1; }
  else if (sscanf(body, "status:%d", &a1) == 1 && a1 >= 0 && a1 <= 20000) { f->sub = LF_SUB_STATUS; f->arg1 = a1; }
  else if (sscanf(body, "status-short:%d", &a1) == 1 && a1 >= 0 && a1 <= 20000) { f->sub = LF_SUB_STATUS_SHORT; f->arg1 = a1; }
  else if (!strncmp(body, "cfg-block:", 10)) {
    /* e2c config_block (28 config words): the block an ECU would send, named slots set, the rest zero. A test can
     * carry words measured from a real ECU's reply (a blank-FRAM ECU's early reply, for one). */
    const char *w = body + 10;
    f->sub = LF_SUB_CFG_BLOCK;
    memset(f->cfg28, 0, sizeof f->cfg28);
    if (!*w) return fail(b, n, "cfg-block: want N=V[+N=V...], N a config slot 1..28", t);
    while (*w) {
      char *end = NULL;
      long idx = strtol(w, &end, 10);
      if (end == w || *end != '=' || idx < 1 || idx > 28)
        return fail(b, n, "cfg-block: want N=V[+N=V...], N a config slot 1..28", t);
      const char *v0 = end + 1;
      long val = strtol(v0, &end, 10);
      if (end == v0) return fail(b, n, "cfg-block: want N=V[+N=V...], N a config slot 1..28", t);
      f->cfg28[idx - 1] = (int32_t)val;
      w = end;
      if (*w == '+') {
        w++;
        if (!*w) return fail(b, n, "cfg-block: want N=V[+N=V...], N a config slot 1..28", t);
      } else if (*w) return fail(b, n, "cfg-block: want N=V[+N=V...], N a config slot 1..28", t);
    }
  }
  else if (sscanf(body, "noise-cont:%d", &a1) == 1 && a1 >= 1 && a1 <= 1000) { f->sub = LF_SUB_NOISE_CONT; f->arg1 = a1; }
  else if (sscanf(body, "noise:%d", &a1) == 1 && a1 >= 1 && a1 <= 1000) { f->sub = LF_SUB_NOISE; f->arg1 = a1; }
  else return fail(b, n, "frame spec: cfg:N=V|cfg-short:N|cfg-any:N=V|cfg-pad:N=V:L|cfg-block:N=V[+N=V...]|trim:D|trim-short:D|fuel-row:R=V|fuel-short:L|status:RPM|status-short:RPM|status-pad:RPM:L|noise:N|noise-cont:N[:full|:gN][*B]", t);
  return 0;
}
static int it_link(s32board *b, const char *n, char *t) {
  int k; char what[24]; double a, c;
  if (sscanf(t, "%d:%23[^@=]", &k, what) != 2 || k < 1 || k > 2 || !window(t, &a, &c) || b->nlf >= SB_MAX_LF) return fail(b, n, "want k:what[=v]@t0[-t1]", t);
  sb_lfault f = {.k = (uint8_t)(k - 1), .t0 = a, .t1 = c, .next_t = a};
  const char *at = strrchr(t, '@'), *sl = at ? strchr(at, '/') : NULL;
  if (sl) { f.dt = atof(sl + 1); if (f.dt <= 0) return fail(b, n, "bad repeat period", t); }
  const char *colon = strchr(t, ':');
  if (colon && (!strncmp(colon + 1, "c2e:", 4) || !strncmp(colon + 1, "e2c:", 4))) {
    f.kind = colon[1] == 'c' ? LF_FRAME_C2E : LF_FRAME_E2C;
    char spec[128]; size_t slen = (size_t)(at - (colon + 5));
    if (slen >= sizeof spec) return fail(b, n, "frame spec too long", t);
    memcpy(spec, colon + 5, slen); spec[slen] = '\0';
    int r = parse_link_frame(b, n, t, spec, &f);
    if (r) return r;
    b->lf[b->nlf++] = f;
    return 0;
  }
  const char *eq = strchr(t, '=');
  if (!strcmp(what, "dead")) f.kind = LF_DEAD;
  else if (!strcmp(what, "cut-c2e")) f.kind = LF_CUT_C2E;
  else if (!strcmp(what, "cut-e2c")) f.kind = LF_CUT_E2C;
  else if (!strcmp(what, "ber") && eq) { f.kind = LF_BER; f.v = atof(eq + 1); if (f.v <= 0 || f.v > 0.5) return fail(b, n, "ber=<probability per bit, <= 0.5>", t); }
  else if (!strcmp(what, "garbage") && eq) { f.kind = LF_GARBAGE_CT; f.n = (uint32_t)atoi(eq + 1); }
  else if (!strcmp(what, "garbage-ecu") && eq) { f.kind = LF_GARBAGE_ECU; f.n = (uint32_t)atoi(eq + 1); }
  else return fail(b, n, "dead|cut-c2e|cut-e2c|ber=p|garbage=N|garbage-ecu=N|c2e:<frame>|e2c:<frame>", t);
  b->lf[b->nlf++] = f;
  return 0;
}
static int k_link(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_link); }

/* consoles: "who:uart:t:text" (text: \r \n escapes; no commas -- use one item per line) */
static int it_console(s32board *b, const char *n, char *t) {
  const char *rest, *end; int w = who_of(t, &rest); unsigned u; double at; int used = 0;
  if (w < 0 || sscanf(rest, ":%u:%n", &u, &used) < 1 || !used || !s32board_parse_time(rest + used, &end, &at) || *end != ':')
    return fail(b, n, "want who:uart:t:text", t);
  if ((w == SB_CT && (u < 1 || u > 2)) || (w != SB_CT && u != 1 && u != 4)) return fail(b, n, "Controller UART 1|2, ECU UART 1|4", t);
  char *out = malloc(strlen(end + 1) + 1);
  if (!out) return fail(b, n, "out of memory", "");
  size_t o = 0;
  for (const char *p = end + 1; *p; p++) {
    if (p[0] == '\\' && p[1] == 'r') { out[o++] = '\r'; p++; }
    else if (p[0] == '\\' && p[1] == 'n') { out[o++] = '\n'; p++; }
    else out[o++] = *p;
  }
  out[o] = 0;
  uint16_t s = text_add(b, out);
  free(out);
  if (s == 0xFFFF) return fail(b, n, "text table full", t);
  sb_ev *e = ev_add(b, at, EV_CONSOLE);
  if (!e) return fail(b, n, "event queue full", t);
  e->who = (uint8_t)w; e->a = u; e->s = s;
  return 0;
}
static int k_console(s32board *b, const char *n, const char *v) {   /* items separated by ';' (commas are text) */
  char *buf = strdup(v);
  if (!buf) return fail(b, n, "out of memory", "");
  char *save = NULL;
  for (char *t = strtok_r(buf, ";", &save); t; t = strtok_r(NULL, ";", &save)) {
    int r = it_console(b, n, t);
    if (r) { free(buf); return r; }
  }
  free(buf);
  return 0;
}

/* devices: "who:fram|sd|rtc=mode@t" */
static int it_dev(s32board *b, const char *n, char *t) {
  const char *rest; int w = who_of(t, &rest);
  char dev[8], mode[32]; double a, c;
  if (w < 0 || *rest != ':' || sscanf(rest + 1, "%7[a-z]=%31[^@]", dev, mode) != 2 || !window(t, &a, &c)) return fail(b, n, "want who:dev=mode@t", t);
  if (strlen(mode) >= sizeof mode - 1) return fail(b, n, "mode too long", t);
  int d = !strcmp(dev, "fram") ? DEV_FRAM : !strcmp(dev, "sd") ? DEV_SD : !strcmp(dev, "rtc") ? DEV_RTC : -1;
  if (d < 0 || (d == DEV_RTC && w != SB_CT)) return fail(b, n, "fram (any board), sd (any slot), rtc (ct)", t);
  if (d == DEV_FRAM && !strncmp(mode, "flip:", 5)) {      /* flip:ADDR.BIT -- one cell corrupted at t */
    unsigned long ad; unsigned bit;
    if (sscanf(mode + 5, "%lx.%u", &ad, &bit) != 2 || ad >= S32_FRAM_BYTES || bit > 7) return fail(b, n, "flip:<hex addr>.<bit>", t);
    sb_ev *e = ev_add(b, a, EV_FRAMFLIP);
    if (!e) return fail(b, n, "event queue full", t);
    e->who = (uint8_t)w; e->a = (uint32_t)ad; e->bit = (uint8_t)bit;
    return 0;
  }
  if (d == DEV_SD && (!strcmp(mode, "remove") || !strcmp(mode, "insert"))) {
    sb_ev *e = ev_add(b, a, EV_SDCARD);
    if (!e) return fail(b, n, "event queue full", t);
    e->who = (uint8_t)w; e->level = !strcmp(mode, "insert");
    return 0;
  }
  int m = !strcmp(mode, "ok") ? S32_DEV_OK : !strcmp(mode, "absent") ? S32_DEV_ABSENT : !strcmp(mode, "miso-low") ? S32_DEV_MISO_LOW
        : !strcmp(mode, "wp") && d == DEV_FRAM ? S32_DEV_WP : !strcmp(mode, "readerr") && d == DEV_SD ? S32_DEV_RDERR
        : !strcmp(mode, "writeerr") && d == DEV_SD ? S32_DEV_WRERR : -1;
  if (m < 0) return fail(b, n, "fram: ok|absent|miso-low|wp|flip:A.B; sd: ok|absent|miso-low|readerr|writeerr|remove|insert; rtc: ok|absent|miso-low", t);
  if (isfinite(c) && b->nev + 2 > SB_MAX_EV) return fail(b, n, "event queue full", t);
  sb_ev *e = ev_add(b, a, EV_DEV);
  if (!e) return fail(b, n, "event queue full", t);
  e->who = (uint8_t)w; e->port = (uint8_t)d; e->level = (uint8_t)m;
  if (isfinite(c)) { sb_ev *e2 = ev_add(b, c, EV_DEV); e2->who = (uint8_t)w; e2->port = (uint8_t)d; e2->level = S32_DEV_OK; }
  return 0;
}
static int k_dev(s32board *b, const char *n, const char *v) { return each_item(b, n, v, it_dev); }
/* RTC power-on state: osf (oscillator stopped while unpowered) | invalid (backup lost: garbage registers) | ok */
static int k_rtc(s32board *b, const char *n, const char *v) {
  if (!b->rtc) return 0;                                   /* no RTC attached (front end without one) */
  if (!strcmp(v, "osf")) s32_rtc_set_osf(b->rtc, true);
  else if (!strcmp(v, "invalid")) { b->rtc->invalid = true; s32_rtc_set_osf(b->rtc, true); }
  else if (!strcmp(v, "ok")) { b->rtc->invalid = false; s32_rtc_set_osf(b->rtc, false); }
  else if (!strcmp(v, "absent")) b->rtc->fault = S32_DEV_ABSENT;
  else return fail(b, n, "ok|osf|invalid|absent", v);
  return 0;
}
static int k_rtc_epoch(s32board *b, const char *n, const char *v) {
  if (!b->rtc) return 0;
  char *end; double e = strtod(v, &end);
  /* the cast to time_t must stay in range, and the PCF2127 keeps a two-digit year: accept through 2099
   * (4102444800 = 2100-01-01); past that gmtime_r fails outright long before 2^63 and base+elapsed would overflow */
  if (end == v || *end || !isfinite(e) || e < 0 || e > 4102444800.0) return fail(b, n, "unix seconds 0..4102444800", v);
  b->rtc->base = (time_t)e; return 0;
}
static int k_ct_sd_wp(s32board *b, const char *n, const char *v) {
  bool on = false; if (!on_off(v, &on) && *v) on = true; (void)n;
  b->lvl[SB_CT][PE] = (uint16_t)((b->lvl[SB_CT][PE] & ~(1u << 4)) | (on ? 1u << 4 : 0)); return 0;
}
static int k_ecu_sd_wp(s32board *b, const char *n, const char *v) {
  bool on = false; if (!on_off(v, &on) && *v) on = true; (void)n;
  for (int k = 0; k < 2; k++) if (b->sd[SB_E1 + k]) b->lvl[SB_E1 + k][PB] = (uint16_t)((b->lvl[SB_E1 + k][PB] & ~(1u << 4)) | (on ? 1u << 4 : 0));
  return 0;
}
static int k_input(s32board *b, const char *n, const char *v) { (void)n; return s32board_input_script(b, v); }
/* injector dead time: "auto" = the injector's curve against its supply (the aircraft bus), or a fixed value in ms */
static int k_inj_dead(s32board *b, const char *n, const char *v) {
  if (!strcmp(v, "auto")) { b->inj.dead_fixed = false; return 0; }
  char *end; double ms = strtod(v, &end);
  if (end == v || *end || ms < 0 || ms > 5) return fail(b, n, "auto or a fixed dead time 0..5 ms", v);
  b->inj.dead_fixed = true; b->inj.dead_s = ms / 1e3; b->bench.inj_dead_ms = ms;
  return 0;
}

/* the table checksum the bench's fuel-row frames carry: half = the stored format's 161-byte sum, full = 322 bytes */
static int k_link_table_seal(s32board *b, const char *n, const char *v) {
  if (!strcmp(v, "half")) b->link_table_full = false;
  else if (!strcmp(v, "full")) b->link_table_full = true;
  else return fail(b, n, "half | full", v);
  return 0;
}

typedef struct { const char *name; int (*fn)(s32board *, const char *, const char *); const char *help; } knob;
static const knob KNOBS[] = {
  {"S32_RPM", k_rpm, "engine speed (RPM) from S32_START on (default 0 = stopped)"},
  {"S32_START", k_start, "engine start time, s (default 3)"},
  {"S32_RPM_AT", k_rpm_at, "t=rpm,...: engine speed steps (start, run, stop profiles)"},
  {"S32_CYL", k_cyl, "cylinders: firing events per 720 deg (4, 6, 8; default 4)"},
  {"S32_SYNC_BEFORE", k_sync, "sync magnet, deg before firing event 0 (default 30)"},
  {"S32_MAG1", k_mag1, "ECU1 crank sensor's true magnet position, deg (default 80)"},
  {"S32_MAG2", k_mag2, "ECU2 crank sensor's true magnet position, deg (default 80)"},
  {"S32_HALL_WIDTH", k_hall_w, "Hall output LOW pulse width, crank deg (default 5)"},
  {"S32_CRANK_JITTER", k_jitter, "uniform +-us timing noise on every Hall edge (default 0)"},
  {"S32_CRANK_GLITCH", k_glitch, "k:t[:us[:crank|sync]]: one extra LOW pulse (EMI), default 20 us on the crank element"},
  {"S32_CRANK_DROP", k_drop, "k:t0-t1[:crank|sync|all]: edges missing (default crank element)"},
  {"S32_CRANK_EMI", k_emi, "k:<pulses/s>[:sync]@t0[-t1]: random 10-us extra pulses"},
  {"S32_HALL_STUCK", k_stuck, "k:crank|sync=low|high@t0[-t1]: sensor output held (short to ground / open wire)"},
  {"S32_AN", k_an, "[ct|1|2:]ch=raw,...: analogue pin counts (no prefix = the sensor, both ECUs)"},
  {"S32_CT_AN", k_ct_an, "ch=raw,...: Controller analogue pin counts"},
  {"S32_TPS_NOISE", k_tps_noise, "+-counts uniform noise on every TPS (AN9) sample, both ECUs"},
  {"S32_TPS_FAULT", k_tps_fault, "k:raw[~noise]@t: one ECU's TPS pin from t (alias of S32_AN_FAULT k:9=raw:N)"},
  {"S32_AN_FAULT", k_an_fault, "who:ch=open|gnd|vcc|raw:N|noise:N|float|stuck|intermittent:ms@t0[-t1]"},
  {"S32_TPS_SUPPLY", k_tps_supply, "ECU (1|2) whose 5 V excites the shared TPS (default 1, install manual)"},
  {"S32_TPS_UNPOWERED", k_tps_unpowered, "TPS signal with its supply ECU off: gnd (default) | float | pullup (INFERRED)"},
  {"S32_ET_WIRING", k_et_wiring, "ecu1 (default, install manual: ET sensor on ECU1 only) | both"},
  {"S32_BATT", k_batt, "1|2=volts[@t]: Controller DB-15 battery senses (default 13.8)"},
  {"S32_AFR", k_afr, "auto (default: engine model) | AFR: the wideband O2 module on the Controller's DB-15"},
  {"S32_SWITCHES", k_switches, "<ecu1> <ecu2>: on|plead|inj|off, each ECU's P-lead and enable wire (no select switch)"},
  {"S32_ECU_SELECT", k_select, "ECU Select switch: 1 (default) | 2 | both (both enable wires open) | none (both grounded)"},
  {"S32_SELECT_GAP_MS", k_select_gap, "ECU Select break-before-make transit, ms (default 20)"},
  {"S32_RETARD", k_retard, "timing-retard input RD6: off (default, LOW) | on, or k:on|off"},
  {"S32_DIP", k_dip, "k:n=on|off: ECU DIP switch n (on = closed to ground)"},
  {"S32_PUMP", k_pump, "1|2=on|off: Controller fuel-pump senses (default pump 1 on, pump 2 off)"},
  {"S32_DIGIN", k_digin, "3|4=lo|hi: Controller spare digital inputs RG9 / RG14 (default open = hi)"},
  {"S32_SWITCH", k_switch, "t:k:plead=stop|run, t:k:inj=gnd|open, t:select=1|2|both|none, t:k:retard=on|off, t:k:dipN=on|off, t:pumpN=on|off, t:diginN=lo|hi"},
  {"S32_BOUNCE_MS", k_bounce_ms, "contact bounce on every switch/button transition and on the encoder contact that switches, ms (default 0)"},
  {"S32_BOUNCE_EDGES", k_bounce_edges, "chatter edges per bounce (default 4)"},
  {"S32_PIN_FAULT", k_pin_fault, "who:RXn=open|gnd|vcc|stuck|intermittent:ms|chatter:hz@t0[-t1]: wire fault on a digital input"},
  {"S32_ECU_POWER", k_power, "k:off@t | k:on@t | k:boot@t: ECU breaker (off@0 = never powered; on = cold boot)"},
  {"S32_SUPPLY", k_supply, "k:volts@t: ECU supply voltage (default 13.8; below S32_BOR_V the ECU is held in reset)"},
  {"S32_BOR_V", k_bor, "supply below which a board's regulator drops out, V (default 6.0, INFERRED)"},
  {"S32_RESET", k_reset, "ct|1|2@t: MCLR reset pulse"},
  {"S32_XTAL_PPM", k_ppm, "ct|1|2=ppm: crystal frequency error per MCU"},
  {"S32_ECU_LINK", k_link, "k:dead|cut-c2e|cut-e2c|ber=p|garbage=N|garbage-ecu=N|c2e:<frame>|e2c:<frame>@t0[-t1][/dt]: cat5 data pair faults and bench frames (<frame>[*B] bursts, [:full] line rate or [:gN] one byte every N x 0.1 ms, default g4)"},
  {"S32_LINK_TABLE_SEAL", k_link_table_seal, "half (default: the stored format's 161-byte table sum, OBSERVED) | full (322 bytes): the checksum on the bench's fuel-row table frames"},
  {"S32_CONSOLE_IN", k_console, "who:uart:t:text;...: bytes typed into a console UART (ECU 1|4, Controller 1|2)"},
  {"S32_DEV_FAULT", k_dev, "who:fram|sd|rtc=ok|absent|miso-low|wp|readerr|writeerr|remove|insert|flip:A.B@t0[-t1]"},
  {"S32_RTC", k_rtc, "Controller RTC at power-on: ok | osf | invalid | absent"},
  {"S32_RTC_EPOCH", k_rtc_epoch, "RTC time at t = 0, unix seconds"},
  {"S32_CT_SD_WP", k_ct_sd_wp, "Controller SD card write-protect tab set"},
  {"S32_ECU_SD_WP", k_ecu_sd_wp, "ECU SD cards write-protected"},
  {"S32_INJ_DEAD", k_inj_dead, "injector dead time: auto (default: 0.80 ms at 13.8 V, rising as the bus voltage falls) | fixed ms"},
  {"S32_INPUT", k_input, "bezel script t:knob|menu|tune|monitor[+...]:ms, t:enc:N[@ms|@a/b/c[/d]], t:edge:N[@ms], t:snap:name"},
};
#define NKNOBS (sizeof KNOBS / sizeof KNOBS[0])

int s32board_knob(s32board *b, const char *name, const char *value) {
  for (size_t i = 0; i < NKNOBS; i++)
    if (!strcmp(KNOBS[i].name, name)) {
      s32board *was = PARSE_B; PARSE_B = b;
      int r = KNOBS[i].fn(b, name, value ? value : "");
      PARSE_B = was;
      return r;
    }
  snprintf(b->err, sizeof b->err, "unknown knob %s", name);
  return -1;
}
int s32board_knobs_from_env(s32board *b) {
  int n = 0;
  for (size_t i = 0; i < NKNOBS; i++) {
    const char *v = getenv(KNOBS[i].name);
    if (!v || !*v) continue;
    s32board *was = PARSE_B; PARSE_B = b;
    int r = KNOBS[i].fn(b, KNOBS[i].name, v);
    PARSE_B = was;
    if (r) { fprintf(stderr, "%s\n", b->err); exit(2); }
    n++;
  }
  return n;
}
void s32board_knob_help(FILE *f) { for (size_t i = 0; i < NKNOBS; i++) fprintf(f, "%-18s %s\n", KNOBS[i].name, KNOBS[i].help); }
const char *const *s32board_knob_names(void) {
  static const char *names[NKNOBS + 1];
  for (size_t i = 0; i < NKNOBS; i++) names[i] = KNOBS[i].name;
  names[NKNOBS] = NULL;
  return names;
}

/* ---------------------------------------------------------------- bezel script */
void s32board_ct_contact(s32board *b, double t, unsigned port, unsigned bit, bool level) {
  if (port > 6 || bit > 15) return;
  if (!(b->own[SB_CT][port] & (1u << bit))) { b->own[SB_CT][port] |= (uint16_t)(1u << bit); b->lvl[SB_CT][port] |= (uint16_t)(1u << bit); }
  contact(b, t, SB_CT, port, bit, level, true);
}
/* One quadrature edge moves exactly one of the two contacts (Gray code). Only that contact bounces (S32_BOUNCE_MS); the
 * other one is at rest and holds its level, as on a real mechanical encoder. */
void s32board_encoder_edge(s32board *b, double t, int dir) {
  static const int G[4][2] = {{0, 0}, {0, 1}, {1, 1}, {1, 0}};   /* Gray state 0 = both high (active low) */
  int was = b->enc_gray;
  b->enc_gray = (b->enc_gray + (dir > 0 ? 1 : 3)) & 3;
  for (unsigned c = 0; c < 2; c++)
    if (G[was][c] != G[b->enc_gray][c]) contact(b, t, SB_CT, PA, 14 + c, !G[b->enc_gray][c], true);
}
static int input_script(s32board *b, const char *spec);
int s32board_input_script(s32board *b, const char *spec) {
  s32board *was = PARSE_B; PARSE_B = b;
  int r = input_script(b, spec);
  PARSE_B = was;
  return r;
}
static int input_script(s32board *b, const char *spec) {
  static const struct { const char *n; int bit; } BTN[] = {{"knob", 5}, {"menu", 3}, {"tune", 4}, {"monitor", 7}};
  char *buf = strdup(spec);                         /* a script of any length (a detent sweep runs to kilobytes) */
  if (!buf) { snprintf(b->err, sizeof b->err, "S32_INPUT: out of memory"); return -1; }
  char *save = NULL;
  for (char *tok = strtok_r(buf, ",", &save); tok; tok = strtok_r(NULL, ",", &save)) {
    while (isspace((unsigned char)*tok)) tok++;
    double t; const char *end;
    if (!*tok) continue;                            /* an empty item (a trailing comma) */
    if (!s32board_parse_time(tok, &end, &t) || *end != ':' || !end[1] || end[1] == ':') {
      /* refuse, never skip: a dropped item would run the case without that input ("{H-x}" lands here too) */
      snprintf(b->err, sizeof b->err, "S32_INPUT: '%s': want t:what[:arg] with t in seconds, {H} or {H+x}", tok);
      free(buf);
      return -2;
    }
    char *what = (char *)(end + 1);
    char *colon = strchr(what, ':'), *arg = "";
    if (colon) { *colon = '\0'; arg = colon + 1; }
    for (char *p = arg; *p; p++) if (isspace((unsigned char)*p)) { *p = '\0'; break; }
    if (!colon) { for (char *p = what; *p; p++) if (isspace((unsigned char)*p)) { *p = '\0'; break; } }
    if (!strcmp(what, "enc")) {                     /* one detent = a full quadrature cycle, default 40 ms per state */
      int n = atoi(arg), dir = n < 0 ? -1 : 1;
      unsigned count = n < 0 ? (unsigned)(-(n + 1)) + 1u : (unsigned)n;   /* abs(), without the INT_MIN overflow */
      /* "@ms": four equal states. "@a/b/c" or "@a/b/c/d": the detent's states in ms, for an uneven turn (a detent
       * spring snapping the knob through the second half of the click); with three values the fourth state (at rest,
       * before the next detent) lasts a. */
      double st = 0.040, v[4] = {0};
      int nv = 0;
      const char *at = strchr(arg, '@');
      if (at) {
        const char *p = at + 1; char *e = NULL;
        while (nv < 4) { v[nv++] = strtod(p, &e) / 1000.0; if (*e != '/') break; p = e + 1; }
        st = v[0];
        if (nv == 2 || *e) {                        /* "@a/b", "@a/b/c/d/e" or trailing text: refuse, never guess */
          if (colon) *colon = ':';
          snprintf(b->err, sizeof b->err, "S32_INPUT: '%s': a detent is @ms or @a/b/c[/d]", tok);
          free(buf);
          return -1;
        }
      }
      if (nv < 3) {
        for (unsigned d = 0; d < count && b->nev < SB_MAX_EV; d++) for (int s = 0; s < 4; s++) s32board_encoder_edge(b, t + (d * 4 + s) * st, -dir);
      } else {
        if (nv == 3) v[3] = v[0];
        double off[4] = {0.0, v[0], v[0] + v[1], v[0] + v[1] + v[2]}, period = off[3] + v[3];
        for (unsigned d = 0; d < count && b->nev < SB_MAX_EV; d++) for (int s = 0; s < 4; s++) s32board_encoder_edge(b, t + d * period + off[s], -dir);
      }
    } else if (!strcmp(what, "edge")) {             /* single edges, default 200 ms apart (one list row each) */
      int n = atoi(arg), dir = n < 0 ? -1 : 1;
      unsigned count = n < 0 ? (unsigned)(-(n + 1)) + 1u : (unsigned)n;
      double st = strchr(arg, '@') ? atof(strchr(arg, '@') + 1) / 1000.0 : 0.200;
      for (unsigned d = 0; d < count && b->nev < SB_MAX_EV; d++) s32board_encoder_edge(b, t + d * st, dir);
    } else if (!strcmp(what, "snap")) {
      uint16_t s = text_add(b, arg);
      if (s == 0xFFFF) {
        snprintf(b->err, sizeof b->err, "S32_INPUT: the text table is full (%d bytes); the script does not fit", SB_TEXT);
        free(buf);
        return -1;
      }
      sb_ev *e = ev_add(b, t, EV_SNAP); if (e) e->s = s;
    } else {
      double hold = arg[0] ? atof(arg) / 1000.0 : 0.15;
      char *s2 = NULL;
      for (char *nm = strtok_r(what, "+", &s2); nm; nm = strtok_r(NULL, "+", &s2))
        for (unsigned i = 0; i < 4; i++)
          if (!strcmp(nm, BTN[i].n)) { contact(b, t, SB_CT, PA, (unsigned)BTN[i].bit, false, true); contact(b, t + hold, SB_CT, PA, (unsigned)BTN[i].bit, true, true); }
    }
  }
  free(buf);
  if (b->nev >= SB_MAX_EV) {                        /* ev_add drops what does not fit: never run half a script */
    snprintf(b->err, sizeof b->err, "S32_INPUT: the event queue is full (%d events); the script does not fit", SB_MAX_EV);
    return -1;
  }
  return 0;
}

/* ---------------------------------------------------------------- host inputs */
void s32board_set_rpm(s32board *b, double rpm) { b->ext_rpm = rpm; }
void s32board_set_an(s32board *b, int who, unsigned ch, uint16_t raw) {
  if (who < 0 || who >= SB_WHO || ch > 31) return;
  b->an_nom[who][ch] = raw > 1023 ? 1023 : raw; b->an_host[who][ch] = true;
}
void s32board_set_switches(s32board *b, int k, bool plead_open, bool inj_open) {
  if (k < 0 || k > 1) return;
  b->sw_legacy = true;
  if (b->plead_stop[k] != !plead_open) { b->plead_stop[k] = !plead_open; contact(b, b->now, SB_E1 + k, E_PLEAD, plead_open, true); }
  if (b->inj_gnd[k] != !inj_open) { b->inj_gnd[k] = !inj_open; contact(b, b->now, SB_E1 + k, E_INJ, inj_open, true); }
}
void s32board_set_plead(s32board *b, int k, bool open) {
  if (k < 0 || k > 1 || b->plead_stop[k] == !open) return;
  b->plead_stop[k] = !open; contact(b, b->now, SB_E1 + k, E_PLEAD, open, true);
}
void s32board_an_fault_clear(s32board *b, int who, unsigned ch) { if (who >= 0 && who < SB_WHO && ch < 32) b->naf[who][ch] = 0; }
void s32board_set_retard(s32board *b, int k, bool on) {
  if (k < 0 || k > 1 || b->retard[k] == on) return;
  b->retard[k] = on; contact(b, b->now, SB_E1 + k, E_RETARD, on, true);
}
void s32board_set_supply(s32board *b, int k, double volts, bool breaker_in) {
  if (k < 0 || k > 1) return;
  b->supply_v[k] = volts; b->cut[k] = !breaker_in;
}

/* ---------------------------------------------------------------- per-quantum */
double s32board_hz(const s32board *b, int who) { return (who == SB_CT ? SB_FCY : ECU_SYSCLK_HZ) * (1.0 + b->ppm[who] * 1e-6); }
uint64_t s32board_target(const s32board *b, int who, long q) { return (uint64_t)llround(q * SB_QUANTUM_S * s32board_hz(b, who)); }
double s32board_rpm_at(const s32board *b, double t) {
  if (b->ext_rpm >= 0) return b->ext_rpm;
  return t >= b->start_s ? b->rpm : 0.0;
}
double s32board_angle(const s32board *b, double t) {
  long last = b->eh_last;
  if (last < 0) return 0.0;
  long q = (long)(t / SB_QUANTUM_S) + 1;
  if (q > last) q = last;
  if (q < last - SB_ENG_HIST + 1) q = last - SB_ENG_HIST + 1;
  if (q < 0) q = 0;
  return b->eh[q % SB_ENG_HIST].a0 + (t - b->eh[q % SB_ENG_HIST].t0) * b->eh[q % SB_ENG_HIST].rpm * 6.0;
}

static void power_update(s32board *b) {
  for (int k = 0; k < 2; k++) {
    int w = SB_E1 + k;
    bool want = !b->cut[k] && b->supply_v[k] >= b->bor_v;
    if (want == b->on[w]) continue;
    b->on[w] = want;
    if (want) {
      if (b->host.boot) b->host.boot(b->host.ctx, w, true);
      b->power_cycles[w]++; b->on_at[k] = b->now; b->first_spark_after_on[k] = b->first_inj_after_on[k] = -1;
      fprintf(stderr, "ECU%d power ON (cold boot) at t=%.4f s\n", k + 1, b->now);
    } else {
      inj_meter_drop_source(&b->inj, k);                   /* an unpowered driver closes the injector */
      fprintf(stderr, "ECU%d power OFF at t=%.3f s\n", k + 1, b->now);
    }
  }
  bool ct_want = !b->ct_absent && (b->on[SB_E1] || b->on[SB_E2]);   /* the Controller is powered through the cat5 cables */
  if (ct_want != b->on[SB_CT]) {
    b->on[SB_CT] = ct_want;
    if (ct_want && b->host.boot) { b->host.boot(b->host.ctx, SB_CT, true); b->power_cycles[SB_CT]++; b->panel_frames = 0; b->splash_seen = false; }
    fprintf(stderr, "Controller power %s at t=%.3f s (cat5 supply)\n", ct_want ? "ON (cold boot)" : "OFF", b->now);
  }
}

static void event(s32board *b, sb_ev *e) {
  switch (e->kind) {
  case EV_CONTACT: b->lvl[e->who][e->port] = (uint16_t)((b->lvl[e->who][e->port] & ~(1u << e->bit)) | ((unsigned)e->level << e->bit)); break;
  case EV_PLEAD: b->plead_stop[e->a] = e->level; contact(b, e->t, SB_E1 + (int)e->a, E_PLEAD, !e->level, true); break;
  case EV_INJ: b->sw_legacy = true; b->inj_gnd[e->a] = e->level; contact(b, e->t, SB_E1 + (int)e->a, E_INJ, !e->level, true); break;
  case EV_SELECT: {                                         /* break before make: both wires open while in transit */
    b->sw_legacy = false;
    int to = (int)e->a;
    if (to == 1 || to == 2) {
      for (int k = 0; k < 2; k++) if (enable_grounded(b, k)) contact(b, e->t, SB_E1 + k, E_INJ, true, true);
      b->select = 0;
      sb_ev *s = ev_add(b, e->t + b->select_gap_s, EV_SELECT_SETTLE); if (s) s->a = (uint32_t)to;
    } else { b->select = to; for (int k = 0; k < 2; k++) contact(b, e->t, SB_E1 + k, E_INJ, !enable_grounded(b, k), true); }
    break;
  }
  case EV_SELECT_SETTLE: b->select = (int)e->a; for (int k = 0; k < 2; k++) if (enable_grounded(b, k)) contact(b, e->t, SB_E1 + k, E_INJ, false, true); break;
  case EV_RETARD: b->retard[e->a] = e->level; contact(b, e->t, SB_E1 + (int)e->a, E_RETARD, e->level, true); break;
  case EV_DIP: b->dip[e->a][e->bit] = e->level; contact(b, e->t, SB_E1 + (int)e->a, PD, 9u + e->bit, !e->level, true); break;
  case EV_PUMP: b->pump[e->a] = e->level; contact(b, e->t, SB_CT, PG, e->a ? 8 : 1, e->level, true); break;
  case EV_DIGIN: b->digin_lo[e->a] = !e->level; contact(b, e->t, SB_CT, PG, e->a ? 14 : 9, e->level, true); break;
  case EV_POWER: b->cut[e->a] = !e->level; if (e->level && b->on[SB_E1 + e->a]) { b->on[SB_E1 + e->a] = false; } break;   /* on@ of a running ECU = power cycle */
  case EV_SUPPLY: b->supply_v[e->a] = e->v; break;
  case EV_BATT: b->batt_v[e->a] = e->v; break;
  case EV_RESET:
    if (b->on[e->who] && b->host.boot) {
      b->host.boot(b->host.ctx, e->who, false);
      b->resets[e->who]++;
      if (e->who == SB_CT) { b->panel_frames = 0; b->splash_seen = false; }
      fprintf(stderr, "%s MCLR reset at t=%.4f s\n", e->who == SB_CT ? "Controller" : e->who == SB_E1 ? "ECU1" : "ECU2", e->t);
    }
    break;
  case EV_CONSOLE:
    if (e->s != 0xFFFF && b->on[e->who]) {
      const char *s = b->text + e->s;
      if (e->who == SB_CT) { if (b->ct) s32ct_uart_feed(b->ct, e->a, (const uint8_t *)s, strlen(s)); }
      else if (b->ecu[e->who - 1]) ecu_uart_feed(b->ecu[e->who - 1], e->a, (const uint8_t *)s, strlen(s));
    }
    break;
  case EV_SNAP: if (b->host.snap && e->s != 0xFFFF) b->host.snap(b->host.ctx, b->text + e->s); break;
  case EV_FRAMFLIP: if (b->fram[e->who]) b->fram[e->who]->mem[e->a] ^= (uint8_t)(1u << e->bit); break;
  case EV_DEV:
    if (e->port == DEV_FRAM && b->fram[e->who]) b->fram[e->who]->fault = e->level;
    else if (e->port == DEV_SD && b->sd[e->who]) b->sd[e->who]->fault = e->level;
    else if (e->port == DEV_RTC && b->rtc) b->rtc->fault = e->level;
    break;
  case EV_SDCARD:
    if (b->sd[e->who]) b->sd[e->who]->fault = e->level ? S32_DEV_OK : S32_DEV_ABSENT;
    if (e->who == SB_CT) contact(b, e->t, SB_CT, C_SDCD, !e->level, true);
    else contact(b, e->t, e->who, E_SDCD, !e->level, true);
    break;
  case EV_RPM: b->ext_rpm = e->v; break;
  default: break;
  }
}

static bool win(double t, double t0, double t1) { return t >= t0 && t < t1; }

static uint16_t open_level(int who, unsigned ch) {   /* an open analogue input: thermistor inputs have pull-ups */
  if (who != SB_CT && (ch == 6 || ch == 7)) return 1023;  /* IAT / ET: open reads as the firmware's open-sensor value */
  return 0;
}

static void apply_analog(s32board *b, int who) {
  for (unsigned ch = 0; ch < 32; ch++) {
    uint16_t v = b->an_nom[who][ch], noise = b->an_noise[who][ch];
    if (who != SB_CT) {
      int k = who - 1;
      if (ch == 11 && !b->an_host[who][11]) v = clamp10((b->supply_v[k] - 0.22) / 0.04456);    /* BATT: the ECU's own feed */
      if (ch == 7 && k == 1 && !b->et_both && !b->an_host[who][7]) v = 1023;                     /* ET not wired to ECU2 */
      if (ch == 9 && !b->on[SB_E1 + b->tps_supply]) {     /* shared TPS without its 5 V excitation */
        if (b->tps_unpowered == 0) { v = 0; noise = 0; } else if (b->tps_unpowered == 1) { v = 512; noise = 400; } else { v = 1023; noise = 0; }
      }
    } else {
      if ((ch == 10 || ch == 11) && !b->an_host[who][ch]) v = clamp10(b->batt_v[ch - 10] / (3.3 * 12.0) * 1023.0);
      if (ch == 12 && !b->an_host[who][12]) v = clamp10(plx_smafr_volts(b->afr_live) / 4.0 / 3.3 * 1023.0);
    }
    for (unsigned i = 0; i < b->naf[who][ch]; i++) {
      sb_afault *f = &b->af[who][ch][i];
      if (!win(b->now, f->t0, f->t1)) { f->latched = false; continue; }
      switch (f->mode) {
      case AF_RAW: v = f->val; noise = f->noise; break;
      case AF_OPEN: v = open_level(who, ch); noise = 0; break;
      case AF_GND: v = 0; noise = 0; break;
      case AF_VCC: v = 1023; noise = 0; break;
      case AF_NOISE: noise = f->noise; break;
      case AF_FLOAT: v = 512; noise = 400; break;
      case AF_STUCK: if (!f->latched) { f->frozen = v; f->latched = true; } v = f->frozen; noise = 0; break;
      case AF_INTERMIT: if (((long)floor((b->now - f->t0) / f->period)) & 1) { v = open_level(who, ch); noise = 0; } break;
      }
    }
    if (who == SB_CT) { if (b->ct) s32ct_set_an(b->ct, ch, v); }
    else if (b->ecu[who - 1]) { ecu_set_an(b->ecu[who - 1], ch, v); b->ecu[who - 1]->an_noise[ch] = noise; }
  }
}

static void apply_pins(s32board *b, int who) {
  uint16_t lv[7];
  memcpy(lv, b->lvl[who], sizeof lv);
  if (who == SB_CT) {                                      /* power sense rides on each ECU's cat5 cable */
    lv[PA] = (uint16_t)((lv[PA] & ~0x6u) | (b->on[SB_E1] ? 4u : 0u) | (b->on[SB_E2] ? 2u : 0u));
  }
  for (unsigned i = 0; i < b->npf; i++) {
    sb_pfault *f = &b->pf[i];
    if (f->who != who) continue;
    uint16_t m = (uint16_t)(1u << f->bit);
    if (!win(b->now, f->t0, f->t1)) { f->latched = false; continue; }
    bool lvl = (lv[f->port] & m) != 0;
    switch (f->mode) {
    case PF_OPEN: lvl = true; break;                       /* inputs idle high (pull-ups) */
    case PF_GND: lvl = false; break;
    case PF_VCC: lvl = true; break;
    case PF_STUCK: if (!f->latched) { f->frozen = lvl; f->latched = true; } lvl = f->frozen; break;
    case PF_INTERMIT: if (((long)floor((b->now - f->t0) / f->period)) & 1) lvl = true; break;
    case PF_CHATTER: if (rnd01(b) < f->period * SB_QUANTUM_S) f->frozen = !f->frozen; if (f->frozen) lvl = !lvl; break;
    }
    lv[f->port] = (uint16_t)((lv[f->port] & ~m) | (lvl ? m : 0));
  }
  for (unsigned p = 0; p < 7; p++) {
    uint16_t own = b->own[who][p];
    while (own) {
      unsigned bit = (unsigned)__builtin_ctz(own); own &= (uint16_t)(own - 1);
      bool l = (lv[p] >> bit) & 1u;
      if (who == SB_CT) { if (b->ct) s32ct_set_pin(b->ct, p, bit, l); }
      else if (b->ecu[who - 1]) ecu_set_pin(b->ecu[who - 1], p, bit, l);
    }
  }
  if (who != SB_CT) {                                      /* the enable wire also gates the injector drivers (Drawing 9) */
    int k = who - 1;
    bool open = (lv[PC] >> 13) & 1u;
    if (b->inj.enabled[k] != (open && b->on[who])) inj_meter_enable_source(&b->inj, k, open && b->on[who]);
  }
}

/* Hall sensor edges for one ECU over [t0, t0 + Q): the timing element pulses LOW for hall_deg at each firing event
 * (TDC_k - mag), the sync element once per revolution sync_before deg ahead of event 0. */
static void crank(s32board *b, int k, double t0, double a0, double rpm) {
  ecu_t *e = b->ecu[k];
  if (!e) return;
  double hz = s32board_hz(b, SB_E1 + k), t1 = (double)b->q * SB_QUANTUM_S;   /* exactly the next quantum's start */
  bool stuck[2] = {false, false};
  for (unsigned i = 0; i < b->ncf; i++) {
    sb_cfault *f = &b->cf[i];
    if (f->k != k) continue;
    if ((f->kind == CF_STUCK_LO || f->kind == CF_STUCK_HI) && win(t0, f->t0, f->t1)) {
      int el = f->elem == 2 ? 1 : 0;
      stuck[el] = true;
      ecu_set_pin(e, PE, el ? 9 : 8, f->kind == CF_STUCK_HI);
      b->hall_stuck[k][el] = true;
    }
  }
  for (int el = 0; el < 2; el++)
    if (!stuck[el] && b->hall_stuck[k][el]) { ecu_set_pin(e, PE, el ? 9 : 8, true); b->hall_stuck[k][el] = false; }
  if (rpm > 0 && b->ncyl > 0) {
    double dps = rpm * 6.0, a1 = a0 + dps * SB_QUANTUM_S;
    int nev = b->ncyl / 2;
    double sp = 720.0 / b->ncyl;
    for (double rev = floor(a0 / 360.0) * 360.0; rev < a1 + 360.0; rev += 360.0)
      for (int j = -1; j < nev; j++) {
        double a = rev + (j < 0 ? -b->mag[k] - b->sync_before : j * sp - b->mag[k]);
        int el = j < 0 ? 1 : 0;
        if (stuck[el]) continue;
        for (int edge = 0; edge < 2; edge++) {
          double ae = a + edge * b->hall_deg;
          if (ae < a0 || ae >= a1) continue;
          double t = t0 + (ae - a0) / dps;
          if (b->jitter_us > 0) t += (rnd01(b) * 2.0 - 1.0) * b->jitter_us * 1e-6;
          bool drop = false;
          for (unsigned i = 0; i < b->ncf; i++) {
            sb_cfault *f = &b->cf[i];
            if (f->k == k && f->kind == CF_DROP && win(t, f->t0, f->t1) && (f->elem == 3 || f->elem == (el ? 2 : 1))) drop = true;
          }
          if (drop) continue;
          ecu_schedule_pin(e, (uint64_t)(t * hz), PE, el ? 9u : 8u, edge == 1);
          b->crank_edges[k][el]++;
        }
      }
  }
  for (unsigned i = 0; i < b->ncf; i++) {                 /* extra pulses: single glitches, EMI */
    sb_cfault *f = &b->cf[i];
    if (f->k != k) continue;
    unsigned bit = f->elem == 2 ? 9u : 8u;
    if (f->kind == CF_GLITCH && f->t0 >= t0 && f->t0 < t1) {
      ecu_schedule_pin(e, (uint64_t)(f->t0 * hz), PE, bit, false);
      ecu_schedule_pin(e, (uint64_t)((f->t0 + f->us * 1e-6) * hz), PE, bit, true);
      b->glitches[k]++;
    } else if (f->kind == CF_EMI && win(t0, f->t0, f->t1) && rnd01(b) < f->rate * SB_QUANTUM_S) {
      double t = t0 + rnd01(b) * (SB_QUANTUM_S - f->us * 1e-6);
      ecu_schedule_pin(e, (uint64_t)(t * hz), PE, bit, false);
      ecu_schedule_pin(e, (uint64_t)((t + f->us * 1e-6) * hz), PE, bit, true);
      b->glitches[k]++;
    }
  }
}

/* The injectors are powered from the aircraft bus that also feeds both ECU breakers: the higher of the two live feeds. */
static double inj_bus_volts(const s32board *b) {
  double v = 0.0;
  for (int k = 0; k < 2; k++) if (!b->cut[k] && b->supply_v[k] > v) v = b->supply_v[k];
  return v;
}

static void afr_model(s32board *b, double rpm) {          /* wideband O2: fuel at the injectors vs air into the engine */
  uint64_t now80 = (uint64_t)(b->now * ECU_SYSCLK_HZ);
  inj_meter_settle(&b->inj, now80);
  if (b->afr_fixed >= 0) { b->afr_live = b->afr_fixed; return; }
  double map_inhg = 3.06 + 0.02674 * b->an_nom[SB_E1][5], iat_f = 245.0 - b->an_nom[SB_E1][6] * 100.0 / 376.0;
  double fuel = inj_meter_fuel_gps(&b->inj, now80, 0.5), air = bench_air_gps(&b->bench, rpm, map_inhg, iat_f);
  double afr = rpm >= 100.0 ? bench_afr(air, fuel, 20.0) : 20.0;
  b->afr_live = plx_smafr_afr(plx_smafr_volts(bench_lag(b->afr_live, afr, 0.010, 0.10)));
}

static void res_time(double *t, double h) {
  if (isfinite(*t) && *t >= SB_HOME_T0 / 2 && *t < SB_ANCHOR_T0 / 2) *t = h + (*t - SB_HOME_T0);
}
static void home_resolve(s32board *b, double h) {
  b->home_at = h;
  for (unsigned i = 0; i < b->nanchor; i++) res_time(&b->anchor[i].base, h);
  res_time(&b->start_s, h);
  for (unsigned i = b->iev; i < b->nev; i++) res_time(&b->ev[i].t, h);
  for (unsigned i = b->iev + 1; i < b->nev; i++) {
    sb_ev cur = b->ev[i];
    unsigned j = i;
    while (j > b->iev && b->ev[j - 1].t > cur.t) { b->ev[j] = b->ev[j - 1]; j--; }
    b->ev[j] = cur;
  }
  for (unsigned i = 0; i < b->ncf; i++) { res_time(&b->cf[i].t0, h); res_time(&b->cf[i].t1, h); }
  for (int w = 0; w < SB_WHO; w++) for (int ch = 0; ch < 32; ch++)
    for (unsigned i = 0; i < b->naf[w][ch]; i++) { res_time(&b->af[w][ch][i].t0, h); res_time(&b->af[w][ch][i].t1, h); }
  for (unsigned i = 0; i < b->npf; i++) { res_time(&b->pf[i].t0, h); res_time(&b->pf[i].t1, h); }
  for (unsigned i = 0; i < b->nlf; i++) { res_time(&b->lf[i].t0, h); res_time(&b->lf[i].t1, h); res_time(&b->lf[i].next_t, h); }
}

/* Phase anchors: slot i's times become `at` + their offset, on the quantum grid (the first boundary at or after it, as
 * a plain time is acted on), a hair early so that the step's `t <= tq` test is exact. */
static void anc_time(double *t, unsigned i, double at) {
  if (!isfinite(*t) || *t < SB_ANCHOR_T0 / 2) return;
  double r = *t - (SB_ANCHOR_T0 + i * SB_ANCHOR_SPAN);
  if (r < -1e-6 || r >= SB_ANCHOR_SPAN / 2) return;
  double qn = ceil(r / SB_QUANTUM_S - 1e-3);   /* the sentinel's rounding (2e-9 s near 1e7 s) is 2e-5 quanta */
  if (qn < 0) qn = 0;
  *t = at + qn * SB_QUANTUM_S - SB_QUANTUM_S * 1e-3;
}
static void anchor_fire(s32board *b, unsigned i, double at) {
  b->anchor[i].fired = true; b->anchor[i].at = at;
  for (unsigned j = 0; j < b->nanchor; j++) if (!b->anchor[j].fired) anc_time(&b->anchor[j].base, i, at);   /* chained */
  anc_time(&b->start_s, i, at);
  for (unsigned j = b->iev; j < b->nev; j++) anc_time(&b->ev[j].t, i, at);
  for (unsigned j = b->iev + 1; j < b->nev; j++) {
    sb_ev cur = b->ev[j];
    unsigned m = j;
    while (m > b->iev && b->ev[m - 1].t > cur.t) { b->ev[m] = b->ev[m - 1]; m--; }
    b->ev[m] = cur;
  }
  for (unsigned j = 0; j < b->ncf; j++) { anc_time(&b->cf[j].t0, i, at); anc_time(&b->cf[j].t1, i, at); }
  for (int w = 0; w < SB_WHO; w++) for (int ch = 0; ch < 32; ch++)
    for (unsigned j = 0; j < b->naf[w][ch]; j++) { anc_time(&b->af[w][ch][j].t0, i, at); anc_time(&b->af[w][ch][j].t1, i, at); }
  for (unsigned j = 0; j < b->npf; j++) { anc_time(&b->pf[j].t0, i, at); anc_time(&b->pf[j].t1, i, at); }
  for (unsigned j = 0; j < b->nlf; j++) {
    anc_time(&b->lf[j].t0, i, at); anc_time(&b->lf[j].t1, i, at); anc_time(&b->lf[j].next_t, i, at);
  }
}
/* An interface event at simulated time t (a quantum boundary): fire every armed anchor waiting for it. An anchor is armed
 * once its base time is known (a home-page base after the home page) and the event is at or after it. */
static void anchor_event(s32board *b, uint8_t src, int type, double t) {
  for (unsigned i = 0; i < b->nanchor; i++) {
    sb_anchor *a = &b->anchor[i];
    if (a->fired || a->src != src || (a->type >= 0 && a->type != type)) continue;
    if (a->base >= SB_HOME_T0 / 2 || t < a->base - SB_QUANTUM_S * 1e-3) continue;
    anchor_fire(b, i, t);
  }
}
/* Frame ends on one direction of a cat5 pair, from the bytes its sender put on the wire: HDLC flags, escapes, CRC-16/MCRF4XX
 * over the body (residue 0 = good), the type = the first two body bytes (emu/common/link_decode.h). */
static void frame_watch(s32board *b, sb_frame_watch *w, const uint8_t *buf, unsigned n, uint8_t src, double t_end) {
  for (unsigned j = 0; j < n; j++) {
    uint8_t c = buf[j];
    if (c == LINK_FLAG) {
      if (w->in && w->n >= 4 && w->crc == 0 && !w->esc) anchor_event(b, src, w->t0 | (w->t1 << 8), t_end);
      w->in = true; w->n = 0; w->crc = 0xFFFF; w->esc = false;
      continue;
    }
    if (!w->in) continue;
    if (c == LINK_ESC) { w->esc = true; continue; }
    if (w->esc) { c ^= LINK_ESC_XOR; w->esc = false; }
    if (w->n == 0) w->t0 = c; else if (w->n == 1) w->t1 = c;
    if (w->n < 0xFFFF) w->n++;
    w->crc ^= c;
    for (int k = 0; k < 8; k++) w->crc = (w->crc & 1u) ? (uint16_t)((w->crc >> 1) ^ 0x8408u) : (uint16_t)(w->crc >> 1);
  }
}
int s32board_anchor_check(const s32board *b, FILE *f) {
  int n = 0;
  for (unsigned i = 0; i < b->nanchor; i++) {
    const sb_anchor *a = &b->anchor[i];
    if (a->fired) continue;
    n++;
    if (a->base >= SB_ANCHOR_T0 / 2)
      fprintf(f, "emulator error: anchor '%s' never occurred: the anchor it is chained to never occurred\n", a->spec);
    else if (a->base >= SB_HOME_T0 / 2)
      fprintf(f, "emulator error: anchor '%s' never occurred: its base is a home-page time and the home page was never "
                 "shown\n", a->spec);
    else
      fprintf(f, "emulator error: anchor '%s' never occurred: no %s event at or after %.4f s before the run ended at "
                 "%.4f s\n", a->spec, a->src == SB_ANC_PANEL ? "panel refresh" : "matching link frame", a->base,
                 b->now + SB_QUANTUM_S);
  }
  return n;
}

void s32board_step(s32board *b, long q) {
  double tq = (q - 1) * SB_QUANTUM_S;
  b->now = tq; b->q = q;
  if (b->nanchor && b->ct && b->on[SB_CT] && b->ct->gfx.frames != b->anchor_frames) {   /* a panel refresh ended */
    b->anchor_frames = b->ct->gfx.frames;
    anchor_event(b, SB_ANC_PANEL, -1, tq);
  }
  if (b->ct && b->on[SB_CT] && b->home_at < 0 && b->ct->gfx.frames > b->panel_frames) {
    b->panel_frames = b->ct->gfx.frames;
    static uint8_t rgb[S32_PANEL_W * S32_PANEL_H * 3];   /* 230 KB: never on the stack (the wasm stack is small) */
    s32_panel_render(b->ct, b->on[SB_CT], rgb, NULL);
    bool home = s32_panel_is_home(rgb);
    if (!b->splash_seen) {
      if (!home) {
        for (unsigned i = 0; i < S32_PANEL_W * S32_PANEL_H * 3; i += 3)
          if (rgb[i] >= 16u || rgb[i + 1] >= 16u || rgb[i + 2] >= 16u) { b->splash_seen = true; break; }
      }
    } else if (home) {
      home_resolve(b, tq);
    }
  }
  while (b->iev < b->nev && b->ev[b->iev].t <= tq) { sb_ev e = b->ev[b->iev++]; event(b, &e); }
  power_update(b);
  b->inj.volts = inj_bus_volts(b);
  for (int w = 0; w < SB_WHO; w++) {
    if (!b->on[w]) { if (w != SB_CT) apply_pins(b, w); continue; }   /* gate update even when off */
    apply_pins(b, w);
    apply_analog(b, w);
  }
  double rpm = s32board_rpm_at(b, tq);
  if (q % 100 == 0) afr_model(b, rpm);
  b->eh[q % SB_ENG_HIST].t0 = tq; b->eh[q % SB_ENG_HIST].a0 = b->angle; b->eh[q % SB_ENG_HIST].rpm = rpm; b->eh_last = q;
  for (int k = 0; k < 2; k++) if (b->on[SB_E1 + k]) crank(b, k, tq, b->angle, rpm);
  b->angle += rpm * 6.0 * SB_QUANTUM_S;
}

int s32board_watchdogs(s32board *b) {
  int n = 0;
  for (int k = 0; k < 2; k++) {
    ecu_t *e = b->ecu[k];
    if (!e || !b->on[SB_E1 + k] || !ecu_wdt_poll(e)) continue;
    if (b->host.boot) b->host.boot(b->host.ctx, SB_E1 + k, false);
    ecu_mark_reset_cause(b->ecu[k], ECU_RCON_WDTO);
    b->resets[SB_E1 + k]++; b->wdt_resets[k]++; n++;
    fprintf(stderr, "ECU%d watchdog reset at t=%.4f s\n", k + 1, b->now + SB_QUANTUM_S);
  }
  return n;
}

static void put_u32le(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFFu); p[1] = (uint8_t)((v >> 8) & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu); p[3] = (uint8_t)((v >> 24) & 0xFFu);
}
static size_t build_link_frame(const s32board *b, int k, const sb_lfault *f, uint8_t *out, size_t cap) {
  uint8_t p[512];
  memset(p, 0, sizeof p);
  switch (f->sub) {
  case LF_SUB_CFG:
    put_u32le(p, (uint32_t)f->arg1); put_u32le(p + 4, (uint32_t)f->arg2);
    return link_encode(LINK_C2E_SET_CONFIG, p, 8, out, cap);
  case LF_SUB_CFG_ANY:                            /* any slot number, in or out of range: a protocol-analyser frame */
    put_u32le(p, (uint32_t)f->arg1); put_u32le(p + 4, (uint32_t)f->arg2);
    return link_encode(LINK_C2E_SET_CONFIG, p, 8, out, cap);
  case LF_SUB_CFG_PAD:                            /* config write zero-padded to arg3 bytes between the flags (type + payload + CRC, before escaping) */
    put_u32le(p, (uint32_t)f->arg1); put_u32le(p + 4, (uint32_t)f->arg2);
    return link_encode(LINK_C2E_SET_CONFIG, p, (size_t)f->arg3 - 4, out, cap);
  case LF_SUB_CFG_SHORT:
    put_u32le(p, (uint32_t)f->arg1);
    return link_encode(LINK_C2E_SET_CONFIG, p, 4, out, cap);
  case LF_SUB_TRIM:
    put_u32le(p, (uint32_t)f->arg1);
    return link_encode(LINK_C2E_TRIM_DELTA, p, 4, out, cap);
  case LF_SUB_TRIM_SHORT:
    p[0] = (uint8_t)(f->arg1 & 0xFF); p[1] = (uint8_t)((f->arg1 >> 8) & 0xFF);
    return link_encode(LINK_C2E_TRIM_DELTA, p, 2, out, cap);
  case LF_SUB_FUEL_ROW:
    if (b->fram[SB_E1 + k]) memcpy(p, b->fram[SB_E1 + k]->mem + FRAM_ECU_FUEL_TABLE, FRAM_TABLE_BYTES);
    else { p[0] = 14; p[1] = 9; }
    for (unsigned col = 0; col < 16u; col++) fram_table_set_cell(p, (unsigned)f->arg1, col, (uint8_t)f->arg2);
    fram_table_seal(p, b->link_table_full);
    return link_encode(LINK_C2E_WRITE_FUEL, p, FRAM_TABLE_BYTES, out, cap);
  case LF_SUB_FUEL_SHORT:
    memset(p, 0xEE, sizeof p);
    return link_encode(LINK_C2E_WRITE_FUEL, p, (size_t)f->arg1, out, cap);
  case LF_SUB_STATUS:
    put_u32le(p, 86001u); p[4] = (uint8_t)(f->arg1 & 0xFF); p[5] = (uint8_t)((f->arg1 >> 8) & 0xFF);
    return link_encode(LINK_E2C_STATUS, p, 48, out, cap);
  case LF_SUB_STATUS_SHORT:
    put_u32le(p, 86001u); p[4] = (uint8_t)(f->arg1 & 0xFF); p[5] = (uint8_t)((f->arg1 >> 8) & 0xFF);
    return link_encode(LINK_E2C_STATUS, p, 8, out, cap);
  case LF_SUB_STATUS_PAD:                         /* status frame zero-padded to arg3 bytes between the flags (type + payload + CRC, before escaping) */
    put_u32le(p, 86001u); p[4] = (uint8_t)(f->arg1 & 0xFF); p[5] = (uint8_t)((f->arg1 >> 8) & 0xFF);
    return link_encode(LINK_E2C_STATUS, p, (size_t)f->arg3 - 4, out, cap);
  case LF_SUB_CFG_BLOCK:
    for (unsigned i = 0; i < 28; i++) put_u32le(p + 4 * i, (uint32_t)f->cfg28[i]);
    return link_encode(LINK_E2C_CONFIG_BLOCK, p, 112, out, cap);
  case LF_SUB_NOISE:                            /* arg1 bytes of non-frame line noise after one 0x7E: never a frame */
    if ((size_t)f->arg1 + 1 > cap) return 0;    /* inside, never 0x7E or 0x7D (the escape), and no closing flag */
    out[0] = 0x7E;
    for (int i = 0; i < f->arg1; i++) out[i + 1] = (uint8_t)('A' + (unsigned)i % 26u);
    return (size_t)f->arg1 + 1;
  case LF_SUB_NOISE_CONT:                       /* the same noise bytes continuing an already-open frame: no flag */
    if ((size_t)f->arg1 > cap) return 0;
    for (int i = 0; i < f->arg1; i++) out[i] = (uint8_t)('A' + (unsigned)i % 26u);
    return (size_t)f->arg1;
  default: return 0;
  }
}

unsigned s32board_link(s32board *b, int k, bool to_ecu, uint8_t *buf, unsigned n, unsigned cap) {
  if (b->nanchor && n && b->on[to_ecu ? SB_CT : SB_E1 + k])   /* what the sender put on its wire, before any cable fault */
    frame_watch(b, &b->fw[k][to_ecu ? 0 : 1], buf, n,
                (uint8_t)(to_ecu ? (k ? SB_ANC_CT2 : SB_ANC_CT1) : (k ? SB_ANC_ECU2 : SB_ANC_ECU1)), b->now + SB_QUANTUM_S);
  if (!b->on[SB_E1 + k] || !b->on[SB_CT] || b->link_dead[k]) { b->link_bytes_lost[k] += n; return 0; }   /* unpowered transceiver / dead pair */
  bool cut = false;
  for (unsigned i = 0; i < b->nlf; i++) {
    sb_lfault *f = &b->lf[i];
    if (f->k != k || !win(b->now, f->t0, f->t1)) continue;
    if (f->kind == LF_DEAD) { b->link_bytes_lost[k] += n; return 0; }
    if ((f->kind == LF_CUT_C2E && to_ecu) || (f->kind == LF_CUT_E2C && !to_ecu)) cut = true;
  }
  if (cut) { b->link_bytes_lost[k] += n; n = 0; }
  typeof(b->lmux[0][0]) *m = &b->lmux[k][to_ecu ? 0 : 1];
  uint8_t want_kind = to_ecu ? LF_FRAME_C2E : LF_FRAME_E2C;
  for (unsigned i = 0; i < b->nlf; i++) {
    sb_lfault *f = &b->lf[i];
    if (f->k != k || f->kind != want_kind || f->done || b->now < f->next_t || b->now >= f->t1) continue;
    uint8_t raw[1024];
    size_t flen = build_link_frame(b, k, f, raw, sizeof raw);
    for (unsigned rep = 0; rep < f->burst && flen > 0; rep++)
      if (m->qt + flen <= sizeof m->q) { memcpy(m->q + m->qt, raw, flen); m->qt += (unsigned)flen; b->link_frames_sent[k]++; }
    m->pace = f->pace;
    if (f->dt > 0 && f->next_t + f->dt < f->t1) f->next_t += f->dt;
    else f->done = true;
  }
  if (m->qh == m->qt && m->busy_q == 0 && m->cd == 0 && m->hn == 0) {
    for (unsigned j = 0; j < n; j++) link_deframe_byte(&m->d, buf[j]);
  } else {
    uint8_t pass[512]; unsigned npass = 0;
    bool in_flight = m->busy_q > 0 || m->cd > 0 || (m->qh > 0 && m->qh < m->qt);
    for (unsigned j = 0; j < n; j++) {
      if (!in_flight && m->qh < m->qt && (cut || !m->d.in || link_deframer_idle(&m->d))) in_flight = true;
      if (in_flight) { if (m->hn < sizeof m->hold) m->hold[m->hn++] = buf[j]; }
      else { link_deframe_byte(&m->d, buf[j]); if (npass < sizeof pass) pass[npass++] = buf[j]; }
    }
    if (!in_flight && m->qh < m->qt && (cut || !m->d.in || link_deframer_idle(&m->d))) in_flight = true;
    n = npass < cap ? npass : cap;
    if (n) memcpy(buf, pass, n);
    if (in_flight && m->qh < m->qt) {
      if (m->pace == 0) {
        unsigned rem = m->qt - m->qh, take = (n + rem <= cap) ? rem : (cap - n);
        memcpy(buf + n, m->q + m->qh, take);
        n += take; m->qh += take;
        if (m->qh == m->qt) m->qh = m->qt = 0;
        m->busy_q += take * 3u;
      } else if (m->cd == 0) {
        if (n < cap) buf[n++] = m->q[m->qh++];
        if (m->qh == m->qt) m->qh = m->qt = 0;
        m->cd = m->pace - 1u;
      } else m->cd--;
    } else if (m->cd > 0) m->cd--;
    if (m->busy_q > 0) m->busy_q--;
    if (m->qh == m->qt && m->busy_q == 0 && m->cd == 0 && m->hn > 0) {
      for (unsigned j = 0; j < m->hn && n < cap; j++) { link_deframe_byte(&m->d, m->hold[j]); buf[n++] = m->hold[j]; }
      m->hn = 0;
    }
  }
  for (unsigned i = 0; i < b->nlf; i++) {
    sb_lfault *f = &b->lf[i];
    if (f->k != k || !win(b->now, f->t0, f->t1)) continue;
    if (f->kind == LF_BER)
      for (unsigned j = 0; j < n; j++) for (int bit = 0; bit < 8; bit++) if (rnd01(b) < f->v) { buf[j] ^= (uint8_t)(1u << bit); b->link_bits_flipped[k]++; }
    if (!f->done && ((f->kind == LF_GARBAGE_CT && !to_ecu) || (f->kind == LF_GARBAGE_ECU && to_ecu))) {
      f->done = true;
      for (uint32_t j = 0; j < f->n && n < cap; j++) { buf[n++] = (uint8_t)rnd(b); b->link_garbage[k]++; }
    }
  }
  return n;
}

/* ---------------------------------------------------------------- output observers */
static void ecu_lat(void *ctx, ecu_t *e, uint64_t cyc, unsigned port, uint16_t old, uint16_t neu) {
  s32board *b = ctx;
  int k = e == b->ecu[0] ? 0 : 1;
  uint16_t ch = (uint16_t)(old ^ neu);
  if (b->trace) s32trace_pin(b->trace, SB_E1 + k, cyc, port, old, neu);
  if (port == PA) {
    double hz = s32board_hz(b, SB_E1 + k);
    inj_meter_edge(&b->inj, k, (uint64_t)(cyc * (ECU_SYSCLK_HZ / hz)), old, neu);   /* one time base for both ECUs */
    if ((neu & ~old & 0xFF) && b->on_at[k] >= 0 && b->first_inj_after_on[k] < 0) b->first_inj_after_on[k] = cyc / hz;
  }
  if (port == PD && (ch & (1u << 13)) && (neu & (1u << 13))) {   /* tach out RD13: one rising edge per firing event */
    b->eo[k].tach_at[b->eo[k].tach_edges % 32] = cyc; b->eo[k].tach_edges++;
  }
  if (port == PD && (ch & (1u << 8))) {                            /* fuel-flow out RD8: every edge, and the statistics-window count */
    double t = cyc / s32board_hz(b, SB_E1 + k);
    b->eo[k].ff_edges++;
    b->eo[k].ff_last_at = t;
    if (t >= b->ff_win_from) b->eo[k].ff_edges_win++;              /* ff_win_from is 0 when the run sets no window */
  }
  uint16_t leds = b->eo[k].leds;                                /* bit 0 = RD7 (STOP), bits 1-4 = RG12-RG15 */
  if (port == PD) leds = (uint16_t)((leds & ~1u) | ((neu >> 7) & 1u));
  if (port == PG) leds = (uint16_t)((leds & 1u) | (((neu >> 12) & 0xFu) << 1));
  if (leds != b->eo[k].leds) { b->eo[k].led_changes++; b->eo[k].leds = leds; }
  if (port == PF) { if (ch & 1u) b->eo[k].tp_edges[0]++; if (ch & 2u) b->eo[k].tp_edges[1]++; if (ch & 8u) b->eo[k].tp_edges[2]++; }
  if (port == PG && (ch & 1u)) b->eo[k].tp_edges[3]++;
}
static void ct_lat(void *ctx, struct s32ct *s, uint64_t cyc, unsigned port, uint16_t old, uint16_t neu) {
  (void)s;
  s32board *b = ctx;
  uint16_t ch = (uint16_t)(old ^ neu);
  if (b->trace) s32trace_pin(b->trace, SB_CT, cyc, port, old, neu);
  if (port == PE && (ch & 0x0303u)) {                      /* DIGOUT 1-4: RE0 RE1 RE8 RE9 */
    b->co.digout = (uint16_t)((neu & 3u) | ((neu >> 6) & 0xCu)); b->co.digout_changes++;
  }
  if (port == PA && (ch & 1u) && !(neu & 1u)) b->co.tcon_frames++;          /* panel controller setup: CS (RA0) falls */
  if (port == PD && (ch & (1u << 8)) && (neu & (1u << 8))) b->co.tcon_bits++;   /* clock RD8 rising */
  if (port == PD && (ch & 1u)) { b->co.panel_en = neu & 1u; b->co.panel_en_changes++; }
}
void s32board_attach(s32board *b) {
  for (int k = 0; k < 2; k++) if (b->ecu[k]) { b->ecu[k]->lat_sink = ecu_lat; b->ecu[k]->lat_ctx = b; b->ecu[k]->hz = s32board_hz(b, SB_E1 + k); }
  if (b->ct) { b->ct->lat_sink = ct_lat; b->ct->lat_ctx = b; }
}
void s32board_on_coil(s32board *b, int k, double t_off) {
  if (b->on_at[k] >= 0 && b->first_spark_after_on[k] < 0) b->first_spark_after_on[k] = t_off;
}

/* ---------------------------------------------------------------- report */
void s32board_report(s32board *b, FILE *f) {
  uint64_t now80 = (uint64_t)((b->now + SB_QUANTUM_S) * ECU_SYSCLK_HZ);
  inj_meter_settle(&b->inj, now80);
  fprintf(f, "harness: ECU Select %s; injector enable wires ECU1 %s ECU2 %s; P-lead ECU1 %s ECU2 %s; retard %s/%s\n",
          b->sw_legacy ? "(not fitted: wires set per ECU)" : b->select == 1 ? "ECU1" : b->select == 2 ? "ECU2" : b->select == 0 ? "both open" : "both grounded",
          (b->lvl[SB_E1][PC] >> 13) & 1 ? "open" : "grounded", (b->lvl[SB_E2][PC] >> 13) & 1 ? "open" : "grounded",
          b->plead_stop[0] ? "STOP" : "RUN", b->plead_stop[1] ? "STOP" : "RUN", b->retard[0] ? "on" : "off", b->retard[1] ? "on" : "off");
  for (unsigned i = 0; i < b->nanchor; i++)
    if (b->anchor[i].fired) fprintf(f, "phase anchor '%s' at %.4f s\n", b->anchor[i].spec, b->anchor[i].at);
  for (unsigned c = 0; c < 8; c++)
    if (b->inj.union_pulses[c])
      fprintf(f, "injector %u (at the engine, after the enable gates): %llu openings, mean %.3f ms\n", c + 1,
              (unsigned long long)b->inj.union_pulses[c], 1e3 * b->inj.union_open_s[c] / (double)b->inj.union_pulses[c]);
  if (b->inj.dead_fixed)
    fprintf(f, "fuel delivered %.4f g (60 lb/hr injectors, %.2f ms dead time); wideband AFR now %.2f\n", b->inj.grams_total,
            b->bench.inj_dead_ms, b->afr_live);
  else
    fprintf(f, "fuel delivered %.4f g (60 lb/hr injectors, dead time %.3f ms at the %.1f V bus, %.2f ms at %.1f V); wideband AFR now %.2f\n",
            b->inj.grams_total, b->bench.inj_dead_ms * inj_dead_factor(b->inj.volts), b->inj.volts, b->bench.inj_dead_ms,
            INJ_DEAD_REF_V, b->afr_live);
  for (int k = 0; k < 2; k++) {
    double hz = s32board_hz(b, SB_E1 + k);
    uint64_t n = b->eo[k].tach_edges, m = n > 32 ? 32 : n;          /* frequency over the newest (up to) 32 pulses */
    double span = m > 1 ? (b->eo[k].tach_at[(n - 1) % 32] - b->eo[k].tach_at[(n - m) % 32]) / hz : 0;
    fprintf(f, "ECU%d outputs: tach %llu pulses (%.1f Hz), fuel-flow edges %llu, LEDs RD7/RG12-15 = %X (%llu changes), test points RF0/RF1/RF3/RG0 edges %llu/%llu/%llu/%llu; Hall edges crank %llu sync %llu, extra pulses %llu\n",
            k + 1, (unsigned long long)n, span > 0 ? (m - 1) / span : 0.0,
            (unsigned long long)b->eo[k].ff_edges, b->eo[k].leds, (unsigned long long)b->eo[k].led_changes,
            (unsigned long long)b->eo[k].tp_edges[0], (unsigned long long)b->eo[k].tp_edges[1], (unsigned long long)b->eo[k].tp_edges[2],
            (unsigned long long)b->eo[k].tp_edges[3], (unsigned long long)b->crank_edges[k][0], (unsigned long long)b->crank_edges[k][1],
            (unsigned long long)b->glitches[k]);
    fprintf(f, "ECU%d fuel-flow output (RD8): %llu edges, last at %.6f s\n", k + 1,
            (unsigned long long)b->eo[k].ff_edges, b->eo[k].ff_edges ? b->eo[k].ff_last_at : -1.0);
    if (b->ff_win_set)
      fprintf(f, "ECU%d fuel-flow output (RD8) at or after %.6f s: %llu edges\n", k + 1,
              b->ff_win_from, (unsigned long long)b->eo[k].ff_edges_win);
    if (b->link_bits_flipped[k] || b->link_bytes_lost[k] || b->link_garbage[k] || b->link_frames_sent[k])
      fprintf(f, "ECU%d cat5 data pair: %llu bytes lost, %llu bits flipped, %llu garbage bytes injected, %llu bench frames sent\n", k + 1,
              (unsigned long long)b->link_bytes_lost[k], (unsigned long long)b->link_bits_flipped[k], (unsigned long long)b->link_garbage[k],
              (unsigned long long)b->link_frames_sent[k]);
  }
  if (b->ct) {
    uint16_t pr2 = s32ct_sfr_value(b->ct, 0x010C), oc1r = s32ct_sfr_value(b->ct, 0x0196), oc1con = s32ct_sfr_value(b->ct, 0x0190);
    double duty = (oc1con & 7u) >= 6u && pr2 ? 100.0 * oc1r / (pr2 + 1.0) : 0.0;
    fprintf(f, "Controller outputs: backlight PWM (OC1 on RP11) %.1f%% duty at %.0f Hz; DIGOUT1-4 = %X (%llu changes); panel controller setup frames %llu (%llu clocks); panel enable RD0 %d",
            duty, pr2 ? SB_FCY / 8.0 / (pr2 + 1.0) : 0.0, b->co.digout, (unsigned long long)b->co.digout_changes,
            (unsigned long long)b->co.tcon_frames, (unsigned long long)b->co.tcon_bits, (int)b->co.panel_en);
    if (b->home_at >= 0) fprintf(f, "; home page at %.4f s", b->home_at);
    fputc('\n', f);
    uint64_t ft = s32_gfx_frame_tcy(&b->ct->gfx);
    if (ft)
      fprintf(f, "Controller panel display: %llu frames, %.3f ms per frame (%u x %u px)\n",
              (unsigned long long)b->ct->gfx.frames, (double)ft / 16e3,
              (unsigned)b->ct->gfx.reg[0x10], (unsigned)b->ct->gfx.reg[0x11]);
  }
  fprintf(f, "power: ECU supplies %.1f / %.1f V, power cycles ECU1 %u ECU2 %u Controller %u, MCLR resets %u/%u/%u, watchdog resets ECU1 %u ECU2 %u\n",
          b->supply_v[0], b->supply_v[1], b->power_cycles[1], b->power_cycles[2], b->power_cycles[0], b->resets[1], b->resets[2], b->resets[0],
          b->wdt_resets[0], b->wdt_resets[1]);
  for (int k = 0; k < 2; k++)                               /* injector drivers left on (a stopped core holds its latch) */
    if (b->ecu[k]) {
      uint32_t lat = ecu_sfr_value(b->ecu[k], 0xBF886020u), tris = ecu_sfr_value(b->ecu[k], 0xBF886000u);
      fprintf(f, "ECU%d injector drivers on at end of run: %02X%s\n", k + 1, (unsigned)(lat & ~tris & 0xFFu),
              b->on[SB_E1 + k] ? "" : " (unpowered)");
    }
}

void s32board_fault_summary(const s32board *b, double secs, FILE *f) {
  for (int k = 0; k < 2; k++) {
    for (unsigned i = 0; i < b->nlf; i++) {
      const sb_lfault *l = &b->lf[i];
      if (l->k != k) continue;
      static const char *const KIND[] = {"link dead (powered, power-sense pin high)", "Controller->ECU pair cut", "ECU->Controller pair cut",
                                         "bit errors", "garbage bytes to the Controller", "garbage bytes to the ECU",
                                         "bench frames on Controller->ECU", "bench frames on ECU->Controller"};
      fprintf(f, "fault: ECU%d %s from t=%.3f s%s\n", k + 1, KIND[l->kind], l->t0, l->t0 <= secs ? "" : " (not reached)");
    }
    for (unsigned i = 0; i < b->nev; i++) {
      const sb_ev *e = &b->ev[i];
      if (e->kind != EV_POWER || (int)e->a != k) continue;
      if (!e->level) fprintf(f, "fault: ECU%d power OFF from t=%.3f s%s\n", k + 1, e->t, e->t <= secs ? "" : " (not reached)");
      else {
        double fi = b->first_inj_after_on[k], fs = b->first_spark_after_on[k];
        fprintf(f, "fault: ECU%d power ON (cold boot) at t=%.3f s; first inj +%.1f ms, first spark +%.1f ms (%.1f rev at %.0f RPM)\n",
                k + 1, e->t, fi >= 0 ? (fi - e->t) * 1e3 : -1.0, fs >= 0 ? (fs - e->t) * 1e3 : -1.0,
                fs >= 0 ? (fs - e->t) * (b->rpm / 60.0) : -1.0, b->rpm);
      }
    }
    for (unsigned i = 0; i < b->naf[SB_E1 + k][9]; i++)
      if (b->af[SB_E1 + k][9][i].mode == AF_RAW)
        fprintf(f, "fault: ECU%d TPS (AN9) = %u +-%u counts from t=%.3f s\n", k + 1, b->af[SB_E1 + k][9][i].val,
                b->af[SB_E1 + k][9][i].noise, b->af[SB_E1 + k][9][i].t0);
  }
}

/* ---------------------------------------------------------------- clocks */
int s32board_clock_check(const uint8_t *bfl, size_t nb, const uint8_t *ct, size_t nct, FILE *f) {
  int bad = 0;
  if (bfl && nb >= 0x3000u) {
    uint32_t c1, c2; memcpy(&c1, bfl + 0x2FF8, 4); memcpy(&c2, bfl + 0x2FF4, 4);
    static const unsigned IDIV[8] = {1, 2, 3, 4, 5, 6, 10, 12}, MUL[8] = {15, 16, 17, 18, 19, 20, 21, 24}, ODIV[8] = {1, 2, 4, 8, 16, 32, 64, 256};
    unsigned fnosc = c1 & 7u, posc = (c1 >> 8) & 3u, pbdiv = 1u << ((c1 >> 12) & 3u), wdt = (c1 >> 23) & 1u;
    double xtal = 8e6;                                      /* INFERRED: the only crystal value that fits the PLL limits */
    double sys = fnosc == 3 ? xtal / IDIV[c2 & 7u] * MUL[(c2 >> 4) & 7u] / ODIV[(c2 >> 16) & 7u] : fnosc == 2 ? xtal : 8e6;
    fprintf(f, "clocks: ECU DEVCFG1 %08X DEVCFG2 %08X: FNOSC %u POSCMOD %u, %.0f MHz crystal /%u x%u /%u = SYSCLK %.3f MHz, FPBDIV /%u = PBCLK %.3f MHz, watchdog %s\n",
            c1, c2, fnosc, posc, xtal / 1e6, IDIV[c2 & 7u], MUL[(c2 >> 4) & 7u], ODIV[(c2 >> 16) & 7u], sys / 1e6, pbdiv, sys / pbdiv / 1e6,
            wdt ? "ENABLED" : "off");
    if (fabs(sys - ECU_SYSCLK_HZ) > 1.0) { fprintf(f, "clocks: MISMATCH: emulator SYSCLK %.3f MHz\n", ECU_SYSCLK_HZ / 1e6); bad = 1; }
  }
  if (ct && nct >= (0x2ABFEu / 2u) * 3u + 3u) {
    size_t i2 = (0x2ABFCu / 2u) * 3u, i1 = (0x2ABFEu / 2u) * 3u;
    unsigned c2 = (unsigned)ct[i2] | ((unsigned)ct[i2 + 1] << 8), c1 = (unsigned)ct[i1] | ((unsigned)ct[i1 + 1] << 8);
    unsigned posc = c2 & 3u, fnosc = (c2 >> 8) & 7u, pll96 = (c2 >> 11) & 1u, plldiv = (c2 >> 12) & 7u, wdt = (c1 >> 7) & 1u;
    static const unsigned PDIV[8] = {1, 2, 3, 4, 5, 6, 10, 12};
    double xtal = 8e6, fosc = (fnosc == 3 && pll96) ? 32e6 : xtal;     /* 96 MHz PLL / 3 = 32 MHz (CPDIV = 1) */
    fprintf(f, "clocks: Controller CONFIG2 %04X CONFIG1 %04X: FNOSC %u POSCMOD %u, %.0f MHz crystal /%u = %.0f MHz PLL input -> 96 MHz PLL -> Fosc %.0f MHz, Fcy %.0f MHz, watchdog %s\n",
            c2, c1, fnosc, posc, xtal / 1e6, PDIV[plldiv], xtal / PDIV[plldiv] / 1e6, fosc / 1e6, fosc / 2e6, wdt ? "ENABLED" : "off");
    if (fabs(fosc / 2 - SB_FCY) > 1.0 || fabs(xtal / PDIV[plldiv] - 4e6) > 1.0) { fprintf(f, "clocks: MISMATCH: emulator Fcy %.0f MHz\n", SB_FCY / 1e6); bad = 1; }
  }
  return bad;
}

/* ---------------------------------------------------------------- snapshots (emu/common/s32snap.h) */
static void zero_pointers(s32board *c) {
  memset(&c->host, 0, sizeof c->host);
  c->ct = NULL; c->ecu[0] = c->ecu[1] = NULL; c->rtc = NULL; c->trace = NULL;
  for (int w = 0; w < SB_WHO; w++) { c->fram[w] = NULL; c->sd[w] = NULL; }
  for (int k = 0; k < 2; k++) for (int d = 0; d < 2; d++) {
    c->lmux[k][d].d.on_frame = NULL; c->lmux[k][d].d.on_text = NULL; c->lmux[k][d].d.ctx = NULL;
  }
}
void s32board_snap_blob(const s32board *b, s32_tbuf *out) {
  s32board *c = malloc(sizeof *c);
  if (!c) { fputs("s32board: out of memory\n", stderr); abort(); }
  memcpy(c, b, sizeof *c);
  zero_pointers(c);
  s32snap_b(out, c, sizeof *c);
  free(c);
}

static bool ev_text(uint8_t kind) { return kind == EV_CONSOLE || kind == EV_SNAP; }
static double af_t(const sb_afault *f) { return f->t0; }
static void item_ev(const s32board *b, const sb_ev *e, s32_tbuf *items) {
  s32_tbuf x = {0};
  s32snap_f64(&x, e->t); s32snap_u32(&x, e->a); s32snap_f64(&x, e->v);
  if (ev_text(e->kind) && e->s != 0xFFFF) s32snap_b(&x, b->text + e->s, strlen(b->text + e->s) + 1);
  else s32snap_u16(&x, e->s);
  uint8_t f[5] = {e->kind, e->who, e->port, e->bit, e->level};
  s32snap_b(&x, f, sizeof f);
  s32snap_item(items, 0, 0, e->t, x.p, x.n);
  s32_tbuf_free(&x);
}
static void item_af(const sb_afault *f, uint16_t sub, s32_tbuf *items) {
  s32_tbuf x = {0};
  s32snap_f64(&x, f->t0); s32snap_f64(&x, f->t1); s32snap_f64(&x, f->period);
  s32snap_u16(&x, f->val); s32snap_u16(&x, f->noise); s32snap_u16(&x, f->frozen); s32snap_u8(&x, f->mode); s32snap_u8(&x, f->latched);
  s32snap_item(items, 1, sub, af_t(f), x.p, x.n);
  s32_tbuf_free(&x);
}
static void item_pf(const sb_pfault *f, s32_tbuf *items) {
  s32_tbuf x = {0};
  s32snap_f64(&x, f->t0); s32snap_f64(&x, f->t1); s32snap_f64(&x, f->period); s32snap_f64(&x, f->us);
  uint8_t v[6] = {f->who, f->port, f->bit, f->mode, f->latched, f->frozen};
  s32snap_b(&x, v, sizeof v);
  s32snap_item(items, 2, 0, f->t0, x.p, x.n);
  s32_tbuf_free(&x);
}
static void item_cf(const sb_cfault *f, s32_tbuf *items) {
  s32_tbuf x = {0};
  s32snap_f64(&x, f->t0); s32snap_f64(&x, f->t1); s32snap_f64(&x, f->us); s32snap_f64(&x, f->rate);
  uint8_t v[3] = {f->k, f->kind, f->elem};
  s32snap_b(&x, v, sizeof v);
  s32snap_item(items, 3, 0, f->t0, x.p, x.n);
  s32_tbuf_free(&x);
}
static void item_lf(const sb_lfault *f, s32_tbuf *items) {
  s32_tbuf x = {0};
  s32snap_f64(&x, f->t0); s32snap_f64(&x, f->t1); s32snap_f64(&x, f->v); s32snap_f64(&x, f->dt); s32snap_f64(&x, f->next_t);
  s32snap_u32(&x, f->n); s32snap_u32(&x, (uint32_t)f->arg1); s32snap_u32(&x, (uint32_t)f->arg2); s32snap_u32(&x, (uint32_t)f->arg3);
  s32snap_b(&x, f->cfg28, sizeof f->cfg28);
  uint8_t v[6] = {f->k, f->kind, f->sub, f->pace, f->burst, f->done};
  s32snap_b(&x, v, sizeof v);
  s32snap_item(items, 4, 0, f->t0, x.p, x.n);
  s32_tbuf_free(&x);
}

/* Fields that only describe the stimulus (timed items and their text, the encoder position the bezel script parser
 * reached, the engine's speed and start time): left out of the static part. Before start_s the engine is stopped
 * whatever S32_RPM says (s32board_rpm_at), so the engine start is a timed item. */
static void zero_stimulus(s32board *c) {
  zero_pointers(c);
  memset(c->err, 0, sizeof c->err);
  memset(c->ev, 0, sizeof c->ev); c->nev = c->iev = 0;
  memset(c->text, 0, sizeof c->text); c->ntext = 0;
  memset(c->af, 0, sizeof c->af); memset(c->naf, 0, sizeof c->naf);
  memset(c->pf, 0, sizeof c->pf); c->npf = 0;
  memset(c->cf, 0, sizeof c->cf); c->ncf = 0;
  memset(c->lf, 0, sizeof c->lf); c->nlf = 0;
  memset(c->anchor, 0, sizeof c->anchor); c->nanchor = 0;
  c->enc_gray = 0; c->rpm = 0; c->start_s = 0;
  c->ff_win_from = 0; c->ff_win_set = false;   /* the statistics window is the timed item (S32_STATS_FROM), not static */
}

void s32board_stimulus(const s32board *b, s32_tbuf *static_out, s32_tbuf *items_out) {
  s32board *c = malloc(sizeof *c);
  if (!c) { fputs("s32board: out of memory\n", stderr); abort(); }
  memcpy(c, b, sizeof *c);
  zero_stimulus(c);
  s32snap_b(static_out, c, sizeof *c);
  free(c);
  for (unsigned i = b->iev; i < b->nev; i++) item_ev(b, &b->ev[i], items_out);
  for (int w = 0; w < SB_WHO; w++) for (int ch = 0; ch < 32; ch++)
    for (unsigned i = 0; i < b->naf[w][ch]; i++) item_af(&b->af[w][ch][i], (uint16_t)(w * 32 + ch), items_out);
  for (unsigned i = 0; i < b->npf; i++) item_pf(&b->pf[i], items_out);
  for (unsigned i = 0; i < b->ncf; i++) item_cf(&b->cf[i], items_out);
  for (unsigned i = 0; i < b->nlf; i++) item_lf(&b->lf[i], items_out);
  double eng[2] = {b->rpm, b->start_s};
  s32snap_item(items_out, 5, 0, b->start_s, eng, sizeof eng);
}

/* Fault arrays: this run's own entries, the ones before t_future carrying the saved run's state (same entries, in the
 * same order: checked by the caller through the stimulus items). */
#define MERGE_FAULTS(T, dst, ndst, src, nsrc, live, nlive, tf, label)                                   \
  do {                                                                                                  \
    unsigned j_ = 0;                                                                                    \
    for (unsigned i_ = 0; i_ < (nlive); i_++) {                                                         \
      (dst)[i_] = (live)[i_];                                                                           \
      if ((live)[i_].t0 >= (tf)) continue;                                                              \
      while (j_ < (nsrc) && (src)[j_].t0 >= (tf)) j_++;                                                 \
      if (j_ >= (nsrc)) { snprintf(why, nwhy, "%s faults before the snapshot differ", label); goto fail; } \
      (dst)[i_] = (src)[j_++];                                                                          \
    }                                                                                                   \
    (ndst) = (nlive);                                                                                   \
  } while (0)

int s32board_snap_restore(s32board *b, const uint8_t *blob, size_t n, double t_future, char *why, size_t nwhy) {
  if (n != sizeof *b) { snprintf(why, nwhy, "board state size %zu, this build %zu", n, sizeof *b); return -1; }
  if (b->nanchor) { snprintf(why, nwhy, "this run has phase-anchored times (~), which a restore does not carry"); return -1; }
  s32board *live = malloc(sizeof *live), *snap = malloc(sizeof *snap);
  if (!live || !snap) { fputs("s32board: out of memory\n", stderr); abort(); }
  memcpy(live, b, sizeof *b);
  memcpy(snap, blob, sizeof *snap);
  /* "{H+x}" times: the saved run had seen (or predicted) the home page by its snapshot point, so this run's own
   * home-page times are measured from that same moment, before they are merged by time (the caller has checked that
   * the items before t_future agree, with the home-page items placed the same way). */
  if (snap->home_at >= 0 && live->home_at < 0) home_resolve(live, snap->home_at);
  memcpy(b, snap, sizeof *b);
  /* pointers: this process's */
  b->host = live->host; b->ct = live->ct; b->ecu[0] = live->ecu[0]; b->ecu[1] = live->ecu[1]; b->rtc = live->rtc;
  b->trace = live->trace;
  for (int w = 0; w < SB_WHO; w++) { b->fram[w] = live->fram[w]; b->sd[w] = live->sd[w]; }
  for (int k = 0; k < 2; k++) for (int d = 0; d < 2; d++) {
    b->lmux[k][d].d.on_frame = live->lmux[k][d].d.on_frame; b->lmux[k][d].d.on_text = live->lmux[k][d].d.on_text;
    b->lmux[k][d].d.ctx = live->lmux[k][d].d.ctx;
  }
  /* events: the saved run's history and pending queue, its own knob events from t_future on replaced by this run's.
   * The queue is ordered by time; at equal times the knob events (added when the knobs were parsed) come before events
   * the running system added (ev_add), so this run's knob events go before saved run-time events of the same time. */
  {
    unsigned nk = 0, ne = b->iev;                    /* history [0, iev) kept as saved */
    sb_ev *out = malloc(sizeof b->ev);
    if (!out) { fputs("s32board: out of memory\n", stderr); abort(); }
    memcpy(out, snap->ev, b->iev * sizeof *out);
    unsigned i = snap->iev, j = 0;
    const sb_ev *L = live->ev;
    unsigned nl = live->nev;
    while (j < nl && L[j].t < t_future) j++;       /* this run's events before t_future: the saved run's (same items) */
    for (;;) {
      while (i < snap->nev && !snap->ev[i].dyn && snap->ev[i].t >= t_future) i++;   /* the saved run's own future */
      bool hs = i < snap->nev, hl = j < nl;
      if (!hs && !hl) break;
      if (ne >= SB_MAX_EV) { free(out); snprintf(why, nwhy, "event queue full after merging this run's events"); goto fail; }
      if (hl && (!hs || L[j].t <= snap->ev[i].t)) {
        sb_ev e = L[j++];
        if (ev_text(e.kind) && e.s != 0xFFFF) {                       /* its text moves into the saved text buffer */
          const char *s = live->text + e.s;
          size_t len = strlen(s) + 1;
          if (b->ntext + len > SB_TEXT) { free(out); snprintf(why, nwhy, "event text buffer full after merging"); goto fail; }
          memcpy(b->text + b->ntext, s, len); e.s = (uint16_t)b->ntext; b->ntext += (unsigned)len;
        }
        out[ne++] = e; nk++;
      } else out[ne++] = snap->ev[i++];
    }
    memcpy(b->ev, out, ne * sizeof *out);
    if (ne < SB_MAX_EV) memset(b->ev + ne, 0, (SB_MAX_EV - ne) * sizeof *out);
    b->nev = ne;
    free(out);
    (void)nk;
  }
  /* faults */
  for (int w = 0; w < SB_WHO; w++) for (int ch = 0; ch < 32; ch++)
    MERGE_FAULTS(sb_afault, b->af[w][ch], b->naf[w][ch], snap->af[w][ch], snap->naf[w][ch], live->af[w][ch], live->naf[w][ch], t_future, "analogue");
  MERGE_FAULTS(sb_pfault, b->pf, b->npf, snap->pf, snap->npf, live->pf, live->npf, t_future, "pin");
  MERGE_FAULTS(sb_cfault, b->cf, b->ncf, snap->cf, snap->ncf, live->cf, live->ncf, t_future, "crank");
  MERGE_FAULTS(sb_lfault, b->lf, b->nlf, snap->lf, snap->nlf, live->lf, live->nlf, t_future, "link");
  /* stimulus-only fields: this run's */
  b->enc_gray = live->enc_gray; b->rpm = live->rpm; b->start_s = live->start_s;
  b->ff_win_from = live->ff_win_from; b->ff_win_set = live->ff_win_set;
  free(live); free(snap);
  return 0;
fail:
  free(live); free(snap);
  return -1;
}
