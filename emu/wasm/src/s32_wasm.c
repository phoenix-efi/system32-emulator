/* s32_wasm.c -- WebAssembly / browser co-simulation wrapper for the target-system emulator.
 *
 * Runs the Cockpit Controller (PIC24FJ256DA210) and two ECUs (PIC32MX795F512L on uc_mips32_shim) in lockstep 100-us
 * quanta on a single thread. Includes a built-in C unpacker for raw CTvvvbbb.bin (Intel HEX) and ECvvvbbb.bin
 * (Intel HEX under a one-byte key, recovered from the file: the value under which its records are valid) as well as
 * flat flash/FRAM/SD images, per-ECU TPS fault injection (S32_TPS_FAULT), and map table inspection/editing through the
 * FRAM chip and the link.
 *
 * Firmware-agnostic by construction: the bench observes only what a real bench could -- output pins (coils,
 * injectors), the analogue inputs it drives, the link bytes (emu/common/link_decode.c), the FRAM chips through the
 * documented layout (emu/common/fram_format.c) and the LCD framebuffer. It never reads a firmware's RAM.
 */
#include "s32ct.h"
#include "s32dev.h"
#include "ecu.h"
#include "link_decode.h"
#include "fram_format.h"
#include "bench_engine.h"
#include "s32board.h"
#include "s32trace.h"
#include "s32panel.h"
#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#else
#define EMSCRIPTEN_KEEPALIVE
#endif

#define FCY 16000000.0
#define QUANTUM_US 100
#define CT_TOP_PC 0x2AC00u
#define CT_PROG_BYTES ((CT_TOP_PC / 2u) * 3u)
#define EC_BFL_BYTES 0x00003000u
#define EC_PFL_BYTES 0x00080000u

static uint8_t CT_PROG[CT_PROG_BYTES];
static uint8_t CT_PFL[CT_PROG_BYTES];      /* the Controller's program flash chip: erases and programming survive a power cycle */
_Static_assert(CT_PROG_BYTES == S32CT_PFL_BYTES, "the bench and the Controller model agree on the flash size");
static uint8_t EC_BFL[EC_BFL_BYTES];
static uint8_t EC_PFL[EC_PFL_BYTES];
static uint8_t EC_PFLK[2][EC_PFL_BYTES];   /* each ECU's program flash chip: what its boot code programmed survives a power cycle */
static bool HAVE_CT_FW = false;
static bool HAVE_EC_BFL = false;
static bool HAVE_EC_PFL = false;
static int LAST_EC_KEY = -1;

static s32ct CT;
static ecu_t ECU[2];
static s32_fram CT_FRAM, ECU_FRAM[2];
static s32_rtc RTC;
static s32_sd CT_SD;
static uint8_t *CT_SD_IMG = NULL;
static size_t CT_SD_LEN = 0;
static bool BOOTED = false;

static long QUANTUM_IDX = 0;
static double NOW_S = 0.0;

/* RGBA8888 320x240 display output buffer */
static uint8_t RGBA_BUF[320 * 240 * 4];

/* Link analysers (emu/common/link_decode.c): one per wire direction per ECU. They are the only source of live engine
 * values (status frames) -- the bench never reads a firmware's RAM. */
static link_deframer DF_CT2E[2], DF_E2CT[2];
static char ECU_CON_BUF[2][2048];
static unsigned ECU_CON_LEN[2];
static link_status LINK_ST[2];          /* newest status frame from each ECU */
static bool LINK_ST_HAVE[2];
static const int ECU_IDX[2] = {0, 1};

static void ecu_console_byte(void *ctx, uint8_t b) {
  int k = *(const int *)ctx;
  if (ECU_CON_LEN[k] + 1 < sizeof(ECU_CON_BUF[0]) && (b >= 0x20 || b == '\n' || b == '\r')) {
    ECU_CON_BUF[k][ECU_CON_LEN[k]++] = (char)b;
    ECU_CON_BUF[k][ECU_CON_LEN[k]] = '\0';
  }
}

/* Bench link injector (map editor): table writes queued per ECU and table, sent on the Controller -> ECU wire at a
 * frame boundary once that ECU is in link mode, one at a time, each until the ECU's ACK (or a 1-s timeout).
 * A write the ECU rejects or does not ACK is reported (last_result), not retried.
 */
typedef struct {
  bool pending, inflight;
  uint8_t table[FRAM_TABLE_BYTES];
  double sent_at;
  int last_result;                      /* 1 accepted, -1 rejected, 0 none yet, -2 timed out */
  uint64_t sent, acked;
} tbl_write_t;
static tbl_write_t TW[2][2];            /* [ecu][is_fuel] */

static void ecu_frame(void *ctx, unsigned type, const uint8_t *p, unsigned len, unsigned wire) {
  int k = *(const int *)ctx;
  (void)wire;
  if (type == LINK_E2C_STATUS && link_decode_status(p, len, &LINK_ST[k])) LINK_ST_HAVE[k] = true;
  else if (type == LINK_E2C_ACK_FUEL || type == LINK_E2C_ACK_IGN) {
    tbl_write_t *w = &TW[k][type == LINK_E2C_ACK_FUEL ? 1 : 0];
    int32_t r = 0;
    if (w->inflight && link_decode_s32(p, len, &r)) {
      w->inflight = false;
      w->acked++;
      w->last_result = r == 1 ? 1 : -1;
    }
  }
}

typedef struct {
  uint8_t b[4096];
  unsigned n;
} outbox_t;

static outbox_t OB_CT2E[2], OB_E2CT[2];

/* The injector is a store-and-forward mux on the Controller -> ECU wire: while it sends a frame, the Controller's
 * bytes are held and follow the frame. The bench paces table frames at half the line rate: one byte per
 * MUX_BYTE_QUANTA quanta (2,000 B/s). */
#define MUX_BYTE_QUANTA 5
static struct {
  uint8_t frame[2 * FRAM_TABLE_BYTES + 16];
  unsigned n, pos;            /* n > 0 while a frame is going out */
  outbox_t hold;              /* Controller bytes held meanwhile */
} MUX[2];

/* The harness (emu/common/s32board.c): engine and crank sensors, every sensor and switch, power, faults, the injectors
 * downstream of the enable gates and the wideband O2 -- the same model and the same knobs as emu/pair. */
static s32board B;
static s32trace TR;                     /* conformance trace (s32_wasm_trace_enable before boot) */
static bool TRACE_WANTED;

static void ct_tx(void *ctx, unsigned uart, uint8_t b) {
  (void)ctx;
  if (B.trace) s32trace_tx(B.trace, SB_CT, CT.cpu.cycles, uart, b);
  if (uart != 3u && uart != 4u) return;
  int k = uart == 3u ? 0 : 1;
  link_deframe_byte(&DF_CT2E[k], b);
  outbox_t *o = MUX[k].n ? &MUX[k].hold : &OB_CT2E[k];
  if (o->n < sizeof(o->b) / 2) o->b[o->n++] = b;
}

static void ecu_tx(void *ctx, ecu_t *e, unsigned uart, uint8_t b) {
  int k = (int)(intptr_t)ctx;
  if (B.trace) s32trace_tx(B.trace, SB_E1 + k, e->cyc, uart, b);
  if (uart == 1u) {
    link_deframe_byte(&DF_E2CT[k], b);
    if (OB_E2CT[k].n < sizeof(OB_E2CT[k].b) / 2) OB_E2CT[k].b[OB_E2CT[k].n++] = b;
  }
}

/* Between quanta, before the outboxes are delivered: move the mux on by one quantum. */
static void mux_step(int k, long q) {
  if (!MUX[k].n) return;
  if (q % MUX_BYTE_QUANTA == 0 && OB_CT2E[k].n < sizeof(OB_CT2E[k].b)) OB_CT2E[k].b[OB_CT2E[k].n++] = MUX[k].frame[MUX[k].pos++];
  if (MUX[k].pos == MUX[k].n) {                       /* done: release what the Controller sent meanwhile */
    unsigned m = MUX[k].hold.n;
    if (OB_CT2E[k].n + m > sizeof(OB_CT2E[k].b)) m = (unsigned)(sizeof(OB_CT2E[k].b) - OB_CT2E[k].n);
    memcpy(OB_CT2E[k].b + OB_CT2E[k].n, MUX[k].hold.b, m);
    OB_CT2E[k].n += m;
    MUX[k].hold.n = 0;
    MUX[k].n = MUX[k].pos = 0;
  }
}

/* Start a queued table write when the Controller is between frames on that wire. */
static void tbl_write_pump(int k) {
  if (MUX[k].n || !LINK_ST_HAVE[k] || !link_deframer_idle(&DF_CT2E[k])) return;   /* busy / not in link mode / mid-frame */
  for (int f = 0; f < 2; f++) {
    tbl_write_t *w = &TW[k][f];
    if (w->inflight) {
      if (NOW_S - w->sent_at > 1.0) { w->inflight = false; w->last_result = -2; }
      else return;                                                    /* one write on the wire at a time */
    }
  }
  for (int f = 1; f >= 0; f--) {
    tbl_write_t *w = &TW[k][f];
    if (!w->pending) continue;
    size_t n = link_encode(f ? LINK_C2E_WRITE_FUEL : LINK_C2E_WRITE_IGN, w->table, FRAM_TABLE_BYTES, MUX[k].frame,
                           sizeof MUX[k].frame);
    if (!n) return;
    MUX[k].n = (unsigned)n;
    MUX[k].pos = 0;
    w->pending = false;
    w->inflight = true;
    w->sent_at = NOW_S;
    w->sent++;
    return;
  }
}

static bool BENCH_DISP_SET;             /* displacement given explicitly; else 90 cu in per cylinder (Lycoming 360/540) */
static struct { uint64_t n; double last_dwell_ms, last_adv_deg, dwell_ms_sum, adv_sum; } COIL[2][4];
static struct { uint64_t n; double last_width_ms, width_ms_sum; uint64_t open_at; } INJ[2][16];
#define MAX_PENDING_KNOBS 64
static char *PENDING_KNOB[MAX_PENDING_KNOBS][2];   /* knobs set before boot, re-applied by boot(): the caller's
                                                    * strings, uncut -- the native front end reads its knobs from the
                                                    * process environment, which has no length limit either */
static unsigned N_PENDING_KNOBS;

static void on_coil(void *ctx, ecu_t *e, unsigned oc, uint64_t on, uint64_t off) {
  int k = (int)(intptr_t)ctx;
  if (oc < 1u || oc > 4u) return;
  double toff = off / e->hz, a = s32board_angle(&B, toff);
  double sp = 720.0 / (B.ncyl > 0 ? B.ncyl : 6);
  double adv = sp - fmod(a, sp);
  if (adv >= sp) adv -= sp;
  double dwell = (off > on ? (off - on) : 0) / e->hz * 1e3;
  if (B.trace) s32trace_coil(B.trace, SB_E1 + k, oc, on, off);
  s32board_on_coil(&B, k, toff);
  COIL[k][oc - 1].n++;
  COIL[k][oc - 1].last_dwell_ms = dwell;
  COIL[k][oc - 1].last_adv_deg = adv;
  COIL[k][oc - 1].dwell_ms_sum += dwell;
  COIL[k][oc - 1].adv_sum += adv;
}

static void on_porta(void *ctx, ecu_t *e, uint64_t cyc, uint16_t old_lat, uint16_t new_lat) {   /* driver pins */
  int k = (int)(intptr_t)ctx;
  for (int b = 0; b < 16; b++) {
    bool was = (old_lat >> b) & 1u, now = (new_lat >> b) & 1u;
    if (!was && now) INJ[k][b].open_at = cyc;
    if (was && !now && INJ[k][b].open_at) {
      double w = (cyc - INJ[k][b].open_at) / e->hz * 1e3;
      INJ[k][b].n++;
      INJ[k][b].last_width_ms = w;
      INJ[k][b].width_ms_sum += w;
      INJ[k][b].open_at = 0;
    }
  }
}

static double BTN_DOWN_TIME[4] = {-1.0, -1.0, -1.0, -1.0};

/* Intel HEX parser helper */
static int hex_nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static int hex_byte(const char *p) {
  int hi = hex_nibble(p[0]), lo = hex_nibble(p[1]);
  return (hi < 0 || lo < 0) ? -1 : ((hi << 4) | lo);
}

/* Does decoding the first line with `key` give a valid Intel HEX record with a matching checksum? Intel HEX has a known
 * plain-text structure, so only the right key passes; the parse after it checks every record's checksum. */
static bool test_ec_key(const uint8_t *raw, size_t len, int key) {
  if (len < 11) return false;
  int k = key & 0xFF;
  char line[80];
  size_t n = 0;
  for (size_t i = 0; i < len && n < sizeof(line) - 1; i++) {
    uint8_t p = (uint8_t)((raw[i] - k) & 0xFF);
    if (k > 0) {
      k = (k - 1) & 0xFF;
      if (k == 0) k = key & 0xFF;
    }
    if (p == '\r' || p == '\n') {
      if (n > 0) break;
      continue;
    }
    line[n++] = (char)p;
  }
  line[n] = '\0';
  if (n < 11 || line[0] != ':') return false;
  int reclen = hex_byte(line + 1);
  if (reclen < 0 || n != (size_t)(1 + (reclen + 5) * 2)) return false;
  unsigned sum = 0;
  for (int i = 0; i < reclen + 5; i++) {
    int b = hex_byte(line + 1 + i * 2);
    if (b < 0) return false;
    sum += (unsigned)b;
  }
  return (sum & 0xFFu) == 0u;
}

typedef void (*hex_sink_fn)(uint32_t addr, uint8_t val);

static void ct_hex_sink(uint32_t addr, uint8_t val) {
  uint32_t pc = addr >> 1;
  uint32_t rem = addr & 3u;
  if (pc < CT_TOP_PC && rem < 3u) {
    size_t idx = (size_t)(pc / 2u) * 3u + rem;
    if (idx < sizeof(CT_PROG)) CT_PROG[idx] = val;
  }
}

static void ec_hex_sink(uint32_t addr, uint8_t val) {
  uint32_t pa = addr & 0x1FFFFFFFu;
  if (pa >= 0x1D000000u && pa < 0x1D000000u + EC_PFL_BYTES) {
    EC_PFL[pa - 0x1D000000u] = val;
  } else if (pa >= 0x1FC00000u && pa < 0x1FC00000u + EC_BFL_BYTES) {
    EC_BFL[pa - 0x1FC00000u] = val;
  }
}

static int parse_intel_hex_stream(const uint8_t *raw, size_t len, int obf_key, hex_sink_fn sink) {
  int k = obf_key & 0xFF;
  uint32_t base = 0;
  char line[600];
  size_t n = 0;
  int records = 0;

  for (size_t i = 0; i <= len; i++) {
    char c = '\n';
    if (i < len) {
      uint8_t p = obf_key > 0 ? (uint8_t)((raw[i] - k) & 0xFF) : raw[i];
      if (obf_key > 0) {
        k = (k - 1) & 0xFF;
        if (k == 0) k = obf_key & 0xFF;
      }
      c = (char)p;
    }
    if (c == '\r' || c == '\n') {
      if (n > 0) {
        line[n] = '\0';
        n = 0;
        if (line[0] != ':') return -1;
        int rl = hex_byte(line + 1);
        int ah = hex_byte(line + 3), al = hex_byte(line + 5);
        int typ = hex_byte(line + 7);
        if (rl < 0 || ah < 0 || al < 0 || typ < 0) return -2;
        unsigned sum = (unsigned)(rl + ah + al + typ);
        uint8_t body[256];
        for (int b = 0; b < rl; b++) {
          int v = hex_byte(line + 9 + b * 2);
          if (v < 0) return -3;
          body[b] = (uint8_t)v;
          sum += (unsigned)v;
        }
        int cks = hex_byte(line + 9 + rl * 2);
        if (cks < 0 || ((sum + (unsigned)cks) & 0xFFu) != 0u) return -4;
        uint32_t addr = ((uint32_t)ah << 8) | (uint32_t)al;
        if (typ == 0) {
          for (int b = 0; b < rl; b++) sink(base + addr + (uint32_t)b, body[b]);
          records++;
        } else if (typ == 4 && rl == 2) {
          base = (((uint32_t)body[0] << 8) | (uint32_t)body[1]) << 16;
        } else if (typ == 2 && rl == 2) {
          base = (((uint32_t)body[0] << 8) | (uint32_t)body[1]) << 4;
        } else if (typ == 1) {
          break;
        }
      }
      continue;
    }
    if (n < sizeof(line) - 1) line[n++] = c;
  }
  return records > 0 ? records : -5;
}

EMSCRIPTEN_KEEPALIVE
int s32_wasm_load_ct_firmware(const uint8_t *data, size_t len) {
  if (!data || len < 16) return -1;
  memset(CT_PROG, 0xFF, sizeof(CT_PROG));
  size_t i = 0;
  while (i < len && (data[i] == ' ' || data[i] == '\r' || data[i] == '\n' || data[i] == '\t')) i++;
  if (i < len && data[i] == ':') {
    int rc = parse_intel_hex_stream(data, len, 0, ct_hex_sink);
    if (rc < 0) return rc;
    HAVE_CT_FW = true;
    return rc;
  }
  size_t copy_len = len < sizeof(CT_PROG) ? len : sizeof(CT_PROG);
  memcpy(CT_PROG, data, copy_len);
  HAVE_CT_FW = true;
  return (int)copy_len;
}

EMSCRIPTEN_KEEPALIVE
int s32_wasm_load_ec_firmware(const uint8_t *data, size_t len, int key_hint) {
  if (!data || len < 16) return -1;
  int key = -1;
  if (key_hint >= 0 && key_hint <= 255 && test_ec_key(data, len, key_hint)) {
    key = key_hint;
  } else if (test_ec_key(data, len, 0)) {
    key = 0;
  } else {
    for (int k = 1; k <= 255; k++) {
      if (test_ec_key(data, len, k)) {
        key = k;
        break;
      }
    }
  }
  if (key < 0) return -2;
  memset(EC_BFL, 0xFF, sizeof(EC_BFL));
  memset(EC_PFL, 0xFF, sizeof(EC_PFL));
  int rc = parse_intel_hex_stream(data, len, key, ec_hex_sink);
  if (rc < 0) return rc;
  HAVE_EC_BFL = true;
  HAVE_EC_PFL = true;
  LAST_EC_KEY = key;
  return key;
}

EMSCRIPTEN_KEEPALIVE
int s32_wasm_load_ec_bflash(const uint8_t *data, size_t len) {
  if (!data || !len) return -1;
  memset(EC_BFL, 0xFF, sizeof(EC_BFL));
  memcpy(EC_BFL, data, len < sizeof(EC_BFL) ? len : sizeof(EC_BFL));
  HAVE_EC_BFL = true;
  return 0;
}

/* A full program-flash image, or an application-only one (ECU_APP_FLAT_BYTES) placed at the application window. With no boot flash loaded the ECUs start the application directly (application start,
 * emu/ecu/src/ecu.c). */
EMSCRIPTEN_KEEPALIVE
int s32_wasm_load_ec_pflash(const uint8_t *data, size_t len) {
  if (!data || !len) return -1;
  (void)ecu_place_pflash(EC_PFL, data, len);
  HAVE_EC_PFL = true;
  return 0;
}

/* Forget the loaded images (the page does this before it loads what the user dropped, so a dropped image replaces the
 * default one instead of mixing with it). */
EMSCRIPTEN_KEEPALIVE
void s32_wasm_clear_ec(void) {
  memset(EC_BFL, 0xFF, sizeof(EC_BFL)); memset(EC_PFL, 0xFF, sizeof(EC_PFL));
  HAVE_EC_BFL = HAVE_EC_PFL = false;
}
EMSCRIPTEN_KEEPALIVE
void s32_wasm_clear_ct(void) { memset(CT_PROG, 0xFF, sizeof(CT_PROG)); HAVE_CT_FW = false; }
/* 1 when the ECUs run without a boot region (application start), 0 with boot code, -1 before boot. */
EMSCRIPTEN_KEEPALIVE
int s32_wasm_ecu_app_start(void) { return BOOTED ? (ECU[0].app_start ? 1 : 0) : -1; }
/* 1 when a Controller image is loaded (with none the bench runs the ECUs alone, the Controller slot empty). */
EMSCRIPTEN_KEEPALIVE
int s32_wasm_have_ct(void) { return HAVE_CT_FW ? 1 : 0; }
/* Simulated time at which this run's Controller first showed its home page (emu/common/s32panel.c, s32board home_at),
 * or < 0 while it has not (or with no Controller). The same moment the cases' "{H}" anchor uses. */
EMSCRIPTEN_KEEPALIVE
double s32_wasm_home_seen(void) { return BOOTED ? B.home_at : -1.0; }

EMSCRIPTEN_KEEPALIVE
int s32_wasm_load_fram(int target, const uint8_t *data, size_t len) {
  if (!data || !len) return -1;
  s32_fram *f = (target == 0) ? &CT_FRAM : (target == 1) ? &ECU_FRAM[0] : (target == 2) ? &ECU_FRAM[1] : NULL;
  if (!f) return -2;
  s32_fram_init(f, 0x00);
  memcpy(f->mem, data, len < sizeof(f->mem) ? len : sizeof(f->mem));
  return 0;
}

EMSCRIPTEN_KEEPALIVE
uint8_t *s32_wasm_get_fram_ptr(int target) {
  if (target == 0) return CT_FRAM.mem;
  if (target == 1) return ECU_FRAM[0].mem;
  if (target == 2) return ECU_FRAM[1].mem;
  return NULL;
}

/* Map editor. Before boot an edit goes into the ECU's FRAM image (model/s32_fram.ksy, emu/common/fram_format.c), as
 * a user would prepare a configuration chip. Once booted it goes the way the Controller itself writes maps: a table
 * write frame (link type 5 fuel / 7 ignition, emu/common/link_decode.h) on the Controller -> ECU wire, which the ECU
 * validates, stores in its own FRAM and ACKs (type 8 / 9). The edit starts from the table the ECU last wrote to its
 * FRAM chip, or from a write still queued. Returns 0 = FRAM image edited, 1 = link write queued, < 0 = error. */
EMSCRIPTEN_KEEPALIVE
int s32_wasm_set_map_cell(int ecu_mask, int is_fuel, int row, int col, int val) {
  if (row < 0 || row >= 16 || col < 0 || col >= 16) return -1;
  uint8_t v = (uint8_t)(val < 0 ? 0 : val > 255 ? 255 : val);
  uint32_t fram_off = is_fuel ? FRAM_ECU_FUEL_TABLE : FRAM_ECU_IGN_TABLE;
  int rc = 0;
  for (int k = 0; k < 2; k++) {
    if (!((ecu_mask >> k) & 1)) continue;
    if (!BOOTED) {
      fram_table_set_cell(ECU_FRAM[k].mem + fram_off, (unsigned)row, (unsigned)col, v);
      continue;
    }
    tbl_write_t *w = &TW[k][is_fuel ? 1 : 0];
    if (!w->pending) memcpy(w->table, ECU_FRAM[k].mem + fram_off, FRAM_TABLE_BYTES);
    fram_table_set_cell(w->table, (unsigned)row, (unsigned)col, v);
    w->pending = true;
    rc = 1;
  }
  return rc;
}

/* One ECU's table as the FRAM chip holds it (decoded through the documented layout), as JSON for the map view. */
static char MAP_JSON[8192];
EMSCRIPTEN_KEEPALIVE
const char *s32_wasm_get_map_json(int ecu_idx, int is_fuel) {
  if (ecu_idx < 0 || ecu_idx >= 2) return "{}";
  const uint8_t *raw = ECU_FRAM[ecu_idx].mem + (is_fuel ? FRAM_ECU_FUEL_TABLE : FRAM_ECU_IGN_TABLE);
  fram_table t;
  fram_table_decode(raw, &t);
  unsigned nr = t.nrows > 16u ? 16u : t.nrows, nc = t.ncols > 16u ? 16u : t.ncols;
  const tbl_write_t *w = &TW[ecu_idx][is_fuel ? 1 : 0];
  size_t o = (size_t)snprintf(MAP_JSON, sizeof MAP_JSON,
                              "{\"nrows\":%u,\"ncols\":%u,\"valid\":%s,\"pending\":%s,\"last_result\":%d,\"sent\":%llu,\"acked\":%llu,\"rpm\":[",
                              nr, nc, fram_table_valid(raw) ? "true" : "false",
                              (w->pending || w->inflight) ? "true" : "false", w->last_result,
                              (unsigned long long)w->sent, (unsigned long long)w->acked);
  for (unsigned r = 0; r < nr && o < sizeof MAP_JSON; r++) o += (size_t)snprintf(MAP_JSON + o, sizeof MAP_JSON - o, "%s%u", r ? "," : "", t.rpm[r]);
  if (o < sizeof MAP_JSON) o += (size_t)snprintf(MAP_JSON + o, sizeof MAP_JSON - o, "],\"map_inhg\":[");
  for (unsigned c = 0; c < nc && o < sizeof MAP_JSON; c++) o += (size_t)snprintf(MAP_JSON + o, sizeof MAP_JSON - o, "%s%.2f", c ? "," : "", t.map_x100[c] / 100.0);
  if (o < sizeof MAP_JSON) o += (size_t)snprintf(MAP_JSON + o, sizeof MAP_JSON - o, "],\"cells\":[");
  for (unsigned r = 0; r < nr && o < sizeof MAP_JSON; r++) {
    o += (size_t)snprintf(MAP_JSON + o, sizeof MAP_JSON - o, "%s[", r ? "," : "");
    for (unsigned c = 0; c < nc && o < sizeof MAP_JSON; c++) o += (size_t)snprintf(MAP_JSON + o, sizeof MAP_JSON - o, "%s%u", c ? "," : "", t.cell[r][c]);
    if (o < sizeof MAP_JSON) o += (size_t)snprintf(MAP_JSON + o, sizeof MAP_JSON - o, "]");
  }
  if (o < sizeof MAP_JSON) snprintf(MAP_JSON + o, sizeof MAP_JSON - o, "]}");
  return MAP_JSON;
}

EMSCRIPTEN_KEEPALIVE
int s32_wasm_load_sd(const uint8_t *data, size_t len) {
  if (!data || !len) return -1;
  free(CT_SD_IMG);
  CT_SD_IMG = (uint8_t *)malloc(len);
  if (!CT_SD_IMG) return -2;
  memcpy(CT_SD_IMG, data, len);
  CT_SD_LEN = len;
  if (BOOTED) {
    s32_sd_init(&CT_SD, CT_SD_IMG, (uint32_t)CT_SD_LEN);
    s32ct_spi_attach(&CT, 1, (s32_pin){4, 2}, s32_sd_dev(&CT_SD));
    s32ct_set_pin(&CT, 4, 3, false); /* RE3 low = card present */
    s32ct_set_pin(&CT, 4, 4, false); /* RE4 low = writable */
  }
  return 0;
}

static uint16_t clamp10(double v) {
  return (uint16_t)(v < 0.0 ? 0 : v > 1023.0 ? 1023 : (int)(v + 0.5));
}

/* Any harness knob by name (emu/common/s32board.h; the same names as s32pair's environment, e.g. "S32_AN_FAULT",
 * "2:9=float@3"). Before boot the knob is remembered -- the name and value strings as given, never cut -- and applied
 * when the bench boots; a knob that fails there fails the boot (s32_wasm_knob_error says why). Returns 0 or < 0. */
EMSCRIPTEN_KEEPALIVE
int s32_wasm_knob(const char *name, const char *value) {
  if (!name) return -1;
  if (!BOOTED) {
    static s32board scratch;                            /* validate now so the caller can report a bad value */
    s32board_init(&scratch, (s32board_host){0});
    int rc = s32board_knob(&scratch, name, value);
    if (rc) { snprintf(B.err, sizeof B.err, "%s", scratch.err); return rc; }
    if (N_PENDING_KNOBS >= MAX_PENDING_KNOBS) {
      snprintf(B.err, sizeof B.err, "too many knobs set before boot (max %d)", MAX_PENDING_KNOBS);
      return -3;
    }
    char *n = strdup(name), *v = strdup(value ? value : "");
    if (!n || !v) {
      free(n); free(v);
      snprintf(B.err, sizeof B.err, "out of memory storing knob %.64s", name);
      return -4;
    }
    PENDING_KNOB[N_PENDING_KNOBS][0] = n;
    PENDING_KNOB[N_PENDING_KNOBS][1] = v;
    N_PENDING_KNOBS++;
    return 0;
  }
  return s32board_knob(&B, name, value);
}
EMSCRIPTEN_KEEPALIVE
const char *s32_wasm_knob_error(void) { return B.err; }
/* The knob table is fixed at build time, so its JSON is rendered once, at the size the names actually need, however
 * long the table grows. Only the one allocation can fail; NULL then, s32_wasm_knob_error() says why. */
static char *KNOB_JSON;
EMSCRIPTEN_KEEPALIVE
const char *s32_wasm_knob_names(void) {
  if (!KNOB_JSON) {
    const char *const *n = s32board_knob_names();
    size_t need = 2;                                         /* '[' and ']' */
    int count = 0;
    for (; n[count]; count++) need += strlen(n[count]) + 3;  /* '"name",' -- one comma too many overall: harmless */
    char *buf = malloc(need + 1);
    if (!buf) {
      snprintf(B.err, sizeof B.err, "out of memory listing the %d knobs", count);
      return NULL;
    }
    size_t o = (size_t)snprintf(buf, need + 1, "[");
    for (int i = 0; n[i]; i++) o += (size_t)snprintf(buf + o, need + 1 - o, "%s\"%s\"", i ? "," : "", n[i]);
    snprintf(buf + o, need + 1 - o, "]");
    KNOB_JSON = buf;
  }
  return KNOB_JSON;
}

EMSCRIPTEN_KEEPALIVE
void s32_wasm_set_engine(double displacement_ci, double ve, double inj_flow_lb_hr, double inj_dead_ms) {
  if (displacement_ci > 0.0) { B.bench.displacement_ci = displacement_ci; BENCH_DISP_SET = true; }
  if (ve > 0.0 && ve <= 1.5) B.bench.ve = ve;
  if (inj_flow_lb_hr > 0.0) B.bench.inj_flow_lb_hr = inj_flow_lb_hr;
  if (inj_dead_ms >= 0.0) B.bench.inj_dead_ms = inj_dead_ms;
  B.inj.dead_s = B.bench.inj_dead_ms / 1e3;
  B.inj.flow_gps = B.bench.inj_flow_lb_hr * (453.59237 / 3600.0);
}

/* One ECU's TPS pin from now on (raw counts +- noise), or back to the shared sensor (raw < 0). */
EMSCRIPTEN_KEEPALIVE
void s32_wasm_set_tps_fault(int ecu_idx, int raw, int noise) {
  if (ecu_idx < 0 || ecu_idx >= 2) return;
  s32board_an_fault_clear(&B, SB_E1 + ecu_idx, 9);
  if (raw < 0) return;
  char v[64];
  snprintf(v, sizeof v, "%d:%d~%d@%.4f", ecu_idx + 1, raw > 1023 ? 1023 : raw, noise > 0 ? (noise > 1023 ? 1023 : noise) : 0, B.now);
  (void)s32board_knob(&B, "S32_TPS_FAULT", v);
}

/* The sensors as the pilot's instruments would read them; each reaches the pins through its transfer curve (the
 * lines below, fitted to the readings the stock firmware displays) and the harness wiring (ET on ECU1 only;
 * batteries on the ECU feeds and on the Controller's DB-15 senses). */
EMSCRIPTEN_KEEPALIVE
void s32_wasm_set_sensors(double map_inhg, double tps_pct, double iat_f, double et_f, double batt1_v, double batt2_v, int tps_noise, double afr_override) {
  for (int k = 0; k < 2; k++) {
    s32board_set_an(&B, SB_E1 + k, 5, clamp10((map_inhg - 3.06) / 0.02674));
    s32board_set_an(&B, SB_E1 + k, 6, clamp10((245.0 - iat_f) * 3.76));
    if (k == 0 || B.et_both) s32board_set_an(&B, SB_E1 + k, 7, clamp10((265.0 - et_f) * 3.76));
    s32board_set_an(&B, SB_E1 + k, 9, clamp10(166.0 + (tps_pct / 100.0) * 800.0));
    B.an_noise[SB_E1 + k][9] = (uint16_t)(tps_noise > 0 ? tps_noise : 0);
    s32board_set_supply(&B, k, k == 0 ? batt1_v : batt2_v, !B.cut[k]);
  }
  B.batt_v[0] = batt1_v;
  B.batt_v[1] = batt2_v;
  B.afr_fixed = afr_override >= 10.0 ? afr_override : -1.0;
}

EMSCRIPTEN_KEEPALIVE
void s32_wasm_set_ecu_an(int ecu_idx, int ch, int raw10) {
  if (ecu_idx >= 0 && ecu_idx < 2 && ch >= 0 && ch < 32) s32board_set_an(&B, SB_E1 + ecu_idx, (unsigned)ch, clamp10(raw10));
}

EMSCRIPTEN_KEEPALIVE
void s32_wasm_set_ct_an(int ch, int raw10) {
  if (ch >= 0 && ch < 32) s32board_set_an(&B, SB_CT, (unsigned)ch, clamp10(raw10));
}

EMSCRIPTEN_KEEPALIVE
void s32_wasm_set_ct_pin(int port, int bit, int level) {
  if (port >= 0 && port < 7 && bit >= 0 && bit < 16) s32board_ct_contact(&B, B.now, (unsigned)port, (unsigned)bit, level != 0);
}

EMSCRIPTEN_KEEPALIVE
void s32_wasm_set_rpm(double rpm) {
  s32board_set_rpm(&B, rpm >= 0.0 ? rpm : 0.0);
}

EMSCRIPTEN_KEEPALIVE
void s32_wasm_set_cylinders(int ncyl, double mag1, double mag2) {
  if (ncyl == 4 || ncyl == 6 || ncyl == 8) B.ncyl = ncyl;
  if (mag1 > 0.0) B.mag[0] = mag1;
  if (mag2 > 0.0) B.mag[1] = mag2;
}

/* One ECU's cockpit wires as the pins see them (P-lead, injector enable), its retard input, and its breaker. */
EMSCRIPTEN_KEEPALIVE
void s32_wasm_set_switches(int ecu_idx, int plead_open, int inj_enabled, int retard_on, int powered) {
  if (ecu_idx < 0 || ecu_idx >= 2) return;
  s32board_set_switches(&B, ecu_idx, plead_open != 0, inj_enabled != 0);
  s32board_set_retard(&B, ecu_idx, retard_on != 0);
  s32board_set_supply(&B, ecu_idx, B.supply_v[ecu_idx], powered != 0);
}

/* Link fault: the ECU stays powered (its power-sense pin high, still firing) but nothing crosses its link either way. */
EMSCRIPTEN_KEEPALIVE
void s32_wasm_set_link(int ecu_idx, int alive) {
  if (ecu_idx >= 0 && ecu_idx < 2) B.link_dead[ecu_idx] = (alive == 0);
}

EMSCRIPTEN_KEEPALIVE
void s32_wasm_button(int btn, int pressed) {
  static const uint8_t BITS[4] = {5, 3, 4, 7};
  if (btn >= 0 && btn < 4) {
    if (pressed) {
      BTN_DOWN_TIME[btn] = NOW_S;
      s32board_ct_contact(&B, NOW_S, 0, BITS[btn], false);
    } else {
      /* A very short press is not registered by the Controller: hold every press at least 45 ms of emulated time so
       * fast clicks are never dropped. */
      double held = NOW_S - BTN_DOWN_TIME[btn];
      double at = (BTN_DOWN_TIME[btn] >= 0.0 && held < 0.045) ? BTN_DOWN_TIME[btn] + 0.045 : NOW_S;
      s32board_ct_contact(&B, at, 0, BITS[btn], true);
      BTN_DOWN_TIME[btn] = -1.0;
    }
  }
}

EMSCRIPTEN_KEEPALIVE
void s32_wasm_button_pulse(int btn_mask, int hold_ms) {
  static const uint8_t BITS[4] = {5, 3, 4, 7};
  double t0 = NOW_S + 0.001;
  double t1 = t0 + (hold_ms > 0 ? hold_ms / 1000.0 : 0.050);
  for (int i = 0; i < 4; i++) {
    if ((btn_mask >> i) & 1) {
      s32board_ct_contact(&B, t0, 0, BITS[i], false);
      s32board_ct_contact(&B, t1, 0, BITS[i], true);
    }
  }
}

static double LAST_ENC_TIME = 0.0;
EMSCRIPTEN_KEEPALIVE
void s32_wasm_encoder_turn(int steps) {
  int dir = steps < 0 ? -1 : 1;
  int n = abs(steps);
  double t = LAST_ENC_TIME > NOW_S ? LAST_ENC_TIME : (NOW_S + 0.001);
  for (int d = 0; d < n; d++) {
    s32board_encoder_edge(&B, t, -dir);   /* one single edge = one +-1 step on the Controller */
    t += 0.135;
  }
  LAST_ENC_TIME = t;
}

static void fram_reset_spi(s32_fram *f) {
  f->sr = 0;
  f->op = 0;
  f->phase = 0;
  f->addr = 0;
  f->asleep = false;
  f->reads = 0;
  f->writes = 0;
}

EMSCRIPTEN_KEEPALIVE
void s32_wasm_clear_fram(int target) {
  if (target == 0) s32_fram_init(&CT_FRAM, 0x00);
  else if (target == 1) s32_fram_init(&ECU_FRAM[0], 0x00);
  else if (target == 2) s32_fram_init(&ECU_FRAM[1], 0x00);
}

static void ct_setup(void) {
  s32ct_spi_attach(&CT, 2, (s32_pin){6, 7}, s32_fram_dev(&CT_FRAM));
  s32ct_spi_attach(&CT, 2, (s32_pin){5, 3}, s32_rtc_dev(&RTC));
  if (CT_SD_IMG && CT_SD_LEN > 0) s32ct_spi_attach(&CT, 1, (s32_pin){4, 2}, s32_sd_dev(&CT_SD));
  for (unsigned u = 1; u <= 4; u++) s32ct_uart_sink(&CT, u, ct_tx, NULL);
  s32ct_idle_skip(&CT, true);                               /* idle loops, found by watching execution */
}
static void ecu_setup(int k) {
  ecu_spi_attach(&ECU[k], 2, 6, 9, s32_fram_dev(&ECU_FRAM[k]));
  ecu_uart_sink(&ECU[k], 1, ecu_tx, (void *)(intptr_t)k);
  ecu_uart_sink(&ECU[k], 4, ecu_tx, (void *)(intptr_t)k);
  ecu_idle_skip(&ECU[k], true);                                     /* idle loops, found by watching execution */
  ECU[k].rng ^= (uint32_t)(k + 1) * 0x85EBCA6Bu;
  ECU[k].oc_sink = on_coil;
  ECU[k].porta_sink = on_porta;
  ECU[k].out_ctx = (void *)(intptr_t)k;
}
/* The harness asks for a (re)boot: cold = power restored, else an MCLR pulse. FRAM, SD and RTC keep their contents. */
static void host_boot(void *ctx, int who, bool cold) {
  (void)ctx;
  if (who == SB_CT) {
    uint64_t cyc = CT.cpu.cycles;
    if (cold) { s32ct_pflash_read(&CT, CT_PFL, sizeof(CT_PFL)); s32ct_init(&CT, CT_PFL, sizeof(CT_PFL)); ct_setup(); }
    else s32ct_reset(&CT);
    CT.cpu.cycles = cyc;
  } else {
    int k = who - 1;
    uint64_t cyc = (uint64_t)(B.now * s32board_hz(&B, who));
    (void)ecu_pflash_read(&ECU[k], EC_PFLK[k], sizeof(EC_PFLK[k]));   /* the flash chip keeps what was programmed */
    ecu_free(&ECU[k]);
    s32_fram_dev(&ECU_FRAM[k]).select(&ECU_FRAM[k], false);
    if (ecu_init(&ECU[k], EC_BFL, sizeof(EC_BFL), EC_PFLK[k], sizeof(EC_PFLK[k]))) return;
    ECU[k].cyc = cyc;
    ecu_setup(k);
  }
  s32board_attach(&B);
}

/* plain = the harness exactly as emu/pair's s32pair starts it (s32board_init defaults, every setting from knobs): the
 * conformance gate compares this front end with s32pair byte for byte. Otherwise the bench page's defaults (sensors
 * at 24 inHg / 55 % TPS / 72 F / 185 F / 14.1 V, engine stopped, the given cylinders and magnet positions). */
static int boot(bool plain, int ncyl, int mag1, int mag2) {
  if (!HAVE_EC_PFL) return -1;                               /* the ECUs need at least their application */
  if (!HAVE_EC_BFL) ecu_app_boot_flash(EC_BFL, sizeof(EC_BFL));   /* no boot region: application start */
  if (!HAVE_CT_FW) memset(CT_PROG, 0xFF, sizeof(CT_PROG));   /* no Controller: its slot stays empty and unpowered */

  for (int k = 0; k < 2; k++) ecu_free(&ECU[k]);
  fram_reset_spi(&CT_FRAM);
  fram_reset_spi(&ECU_FRAM[0]);
  fram_reset_spi(&ECU_FRAM[1]);
  for (int k = 0; k < 2; k++) {
    link_deframer_init(&DF_CT2E[k], NULL, NULL, NULL);
    link_deframer_init(&DF_E2CT[k], ecu_frame, ecu_console_byte, (void *)&ECU_IDX[k]);
  }
  memset(LINK_ST, 0, sizeof(LINK_ST));
  memset(LINK_ST_HAVE, 0, sizeof(LINK_ST_HAVE));
  memset(TW, 0, sizeof(TW));
  memset(MUX, 0, sizeof(MUX));
  memset(&OB_CT2E, 0, sizeof(OB_CT2E));
  memset(&OB_E2CT, 0, sizeof(OB_E2CT));
  memset(&ECU_CON_BUF, 0, sizeof(ECU_CON_BUF));
  memset(&ECU_CON_LEN, 0, sizeof(ECU_CON_LEN));
  memset(COIL, 0, sizeof(COIL));
  memset(INJ, 0, sizeof(INJ));
  memset(RGBA_BUF, 0, sizeof(RGBA_BUF));
  LAST_ENC_TIME = 0.0;
  for (int b = 0; b < 4; b++) BTN_DOWN_TIME[b] = -1.0;
  QUANTUM_IDX = 0;
  NOW_S = 0.0;

  bench_engine bench = B.bench.displacement_ci > 0.0 ? B.bench : bench_engine_six();
  s32board_init(&B, (s32board_host){.boot = host_boot});
  B.ct_absent = !HAVE_CT_FW;
  if (!plain) {
    B.bench = bench;
    if (!BENCH_DISP_SET) B.bench.displacement_ci = 90.0 * (ncyl == 4 ? 4 : 6);
    inj_meter_init(&B.inj, ECU_SYSCLK_HZ, B.bench.inj_dead_ms, B.bench.inj_flow_lb_hr);
    B.ncyl = (ncyl == 4 || ncyl == 6) ? ncyl : 6;
    B.mag[0] = mag1 > 0 ? (double)mag1 : 80.0;
    B.mag[1] = mag2 > 0 ? (double)mag2 : 77.0;
    B.ext_rpm = 0.0;
  }
  s32trace_free(&TR);
  if (TRACE_WANTED) { s32trace_open(&TR, NULL); B.trace = &TR; }
  B.ct = &CT; B.ecu[0] = &ECU[0]; B.ecu[1] = &ECU[1];
  B.fram[SB_CT] = &CT_FRAM; B.fram[SB_E1] = &ECU_FRAM[0]; B.fram[SB_E2] = &ECU_FRAM[1]; B.rtc = &RTC;

  memcpy(CT_PFL, CT_PROG, sizeof(CT_PFL));                  /* a fresh bench: the chip holds the loaded image */
  s32ct_init(&CT, CT_PFL, sizeof(CT_PFL));
  s32_rtc_init(&RTC, &CT.cpu.cycles, SB_FCY, 1791806400u);
  if (CT_SD_IMG && CT_SD_LEN > 0) {
    s32_sd_init(&CT_SD, CT_SD_IMG, (uint32_t)CT_SD_LEN);
    B.sd[SB_CT] = &CT_SD;
    B.lvl[SB_CT][4] &= (uint16_t)~(1u << 3);                 /* RE3 low = card present, RE4 low = writable */
  }
  ct_setup();
  for (int k = 0; k < 2; k++) {
    memcpy(EC_PFLK[k], EC_PFL, sizeof(EC_PFL));                /* a fresh bench: both chips hold the loaded image */
    if (ecu_init(&ECU[k], EC_BFL, sizeof(EC_BFL), EC_PFLK[k], sizeof(EC_PFLK[k]))) return -2 - k;
    ecu_setup(k);
  }
  if (!plain) s32_wasm_set_sensors(24.0, 55.0, 72.0, 185.0, 14.1, 14.1, 0, -1.0);
  int knob_bad = 0;
  for (unsigned i = 0; i < N_PENDING_KNOBS; i++) {              /* the pre-boot knobs; a failure here fails the boot */
    if (s32board_knob(&B, PENDING_KNOB[i][0], PENDING_KNOB[i][1]) && !knob_bad) {
      char msg[sizeof B.err];
      snprintf(msg, sizeof msg, "pre-boot knob %.48s: %s", PENDING_KNOB[i][0], B.err);
      snprintf(B.err, sizeof B.err, "%s", msg);
      knob_bad = 1;
    }
    free(PENDING_KNOB[i][0]);
    free(PENDING_KNOB[i][1]);
    PENDING_KNOB[i][0] = PENDING_KNOB[i][1] = NULL;
  }
  N_PENDING_KNOBS = 0;
  if (knob_bad) return -4;
  s32board_attach(&B);                                       /* after the knobs: crystal error (S32_PPM) sets each ECU's clock */
  RTC.fcy = s32board_hz(&B, SB_CT);
  BOOTED = true;
  return 0;
}

EMSCRIPTEN_KEEPALIVE
int s32_wasm_boot(int ncyl, int mag1, int mag2) { return boot(false, ncyl, mag1, mag2); }
EMSCRIPTEN_KEEPALIVE
int s32_wasm_boot_plain(void) { return boot(true, 0, 0, 0); }

/* Conformance trace (emu/common/s32trace.h): enable before boot; take the bytes recorded so far (pointer, then
 * length, then clear) as often as wanted; finish once at the end (final-state lines; returns the trace's SHA-256). */
EMSCRIPTEN_KEEPALIVE
void s32_wasm_trace_enable(int on) { TRACE_WANTED = on != 0; }
EMSCRIPTEN_KEEPALIVE
const char *s32_wasm_trace_ptr(void) { return TR.out.p ? TR.out.p : ""; }
EMSCRIPTEN_KEEPALIVE
int s32_wasm_trace_len(void) { return (int)TR.out.n; }
EMSCRIPTEN_KEEPALIVE
void s32_wasm_trace_clear(void) { TR.out.n = 0; }
static char TRACE_SHA[65];
EMSCRIPTEN_KEEPALIVE
const char *s32_wasm_trace_finish(void) {
  if (!B.trace) return "";
  const uint8_t *pf[2]; size_t npf[2];
  for (int k = 0; k < 2; k++) { npf[k] = ecu_pflash_read(&ECU[k], EC_PFLK[k], sizeof(EC_PFLK[k])); pf[k] = EC_PFLK[k]; }
  s32trace_finish(&TR, &B, pf, npf, TRACE_SHA);
  return TRACE_SHA;
}

EMSCRIPTEN_KEEPALIVE
int s32_wasm_step_quanta(int n_quanta) {
  if (!BOOTED || n_quanta <= 0) return -1;
  for (int i = 0; i < n_quanta; i++) {
    long q = ++QUANTUM_IDX;
    NOW_S = q * QUANTUM_US / 1e6;
    s32board_step(&B, q);                                     /* events, power, pins, sensors, crank edges */
    uint64_t ct_target = s32board_target(&B, SB_CT, q);
    if (B.on[SB_CT]) s32ct_run_to(&CT, ct_target); else CT.cpu.cycles = ct_target;
    for (int k = 0; k < 2; k++) {
      uint64_t t = s32board_target(&B, SB_E1 + k, q);
      if (B.on[SB_E1 + k] && !ECU[k].last_err) ecu_run_to(&ECU[k], t);
      else ECU[k].cyc = t;
    }
    s32board_watchdogs(&B);                                   /* an expired ECU watchdog resets that core */
    for (int k = 0; k < 2; k++) {
      tbl_write_pump(k);
      mux_step(k, q);
      unsigned n = s32board_link(&B, k, true, OB_CT2E[k].b, OB_CT2E[k].n, sizeof(OB_CT2E[k].b));
      if (n) ecu_uart_feed(&ECU[k], 1, OB_CT2E[k].b, n);
      if (B.trace) s32trace_link(B.trace, q, k, true, OB_CT2E[k].b, n);
      OB_CT2E[k].n = 0;
      n = s32board_link(&B, k, false, OB_E2CT[k].b, OB_E2CT[k].n, sizeof(OB_E2CT[k].b));
      if (n) s32ct_uart_feed(&CT, k == 0 ? 3u : 4u, OB_E2CT[k].b, n);
      if (B.trace) s32trace_link(B.trace, q, k, false, OB_E2CT[k].b, n);
      OB_E2CT[k].n = 0;
    }
    if (B.trace) s32trace_quantum(B.trace, q, &CT, B.on[SB_CT]);
  }
  return 0;
}

/* The panel as it shows (emu/common/s32panel.c: the same renderer as emu/pair's screen dumps and the conformance
 * trace): dark while unpowered, with the graphics module or its display pins off, or while the controller shows its
 * test pattern (the stock firmware selects the black screen while it builds a new screen). */
EMSCRIPTEN_KEEPALIVE
uint8_t *s32_wasm_render_rgba(void) {
  static uint8_t rgb[S32_PANEL_W * S32_PANEL_H * 3];
  s32_panel_render(&CT, BOOTED && B.on[SB_CT], rgb, NULL);
  for (unsigned n = 0; n < S32_PANEL_W * S32_PANEL_H; n++) {
    RGBA_BUF[4 * n] = rgb[3 * n]; RGBA_BUF[4 * n + 1] = rgb[3 * n + 1]; RGBA_BUF[4 * n + 2] = rgb[3 * n + 2];
    RGBA_BUF[4 * n + 3] = 255u;
  }
  return RGBA_BUF;
}

static char TELEM_JSON[8192];

#define AVGAS_G_PER_GAL 2721.6   /* 6.0 lb/US gal */

/* Live values come from the link (each ECU's newest status frame, emu/common/link_decode.h) and from the bench's own
 * instruments (injector pins, wideband sensor); the Controller's displayed numbers are on its screen (framebuffer). */
EMSCRIPTEN_KEEPALIVE
const char *s32_wasm_get_telemetry_json(void) {
  int trim[2] = {0, 0}, link_rpm[2] = {0, 0}, adv[2] = {0, 0}, iat[2] = {0, 0}, et[2] = {0, 0};
  double duty[2] = {0.0, 0.0}, map[2] = {0.0, 0.0}, tps[2] = {0.0, 0.0};
  for (int k = 0; k < 2; k++) {
    if (!LINK_ST_HAVE[k]) continue;
    const link_status *st = &LINK_ST[k];
    trim[k] = (int)st->fuel_trim_pct;
    link_rpm[k] = (int)st->rpm;
    adv[k] = (int)st->ign_adv_deg;
    iat[k] = (int)st->iat_f;
    et[k] = (int)st->et_f;
    duty[k] = isfinite(st->inj_duty_pct) ? (double)st->inj_duty_pct : 0.0;
    map[k] = st->map_x100 / 100.0;
    tps[k] = isfinite(st->tps_pct) ? (double)st->tps_pct : 0.0;
  }
  double fuel_gph = BOOTED ? inj_meter_fuel_gps(&B.inj, (uint64_t)(B.now * ECU_SYSCLK_HZ), 0.5) * 3600.0 / AVGAS_G_PER_GAL : 0.0;
  snprintf(
    TELEM_JSON, sizeof(TELEM_JSON),
    "{"
    "\"booted\":%s,\"time_s\":%.3f,\"rpm\":%.0f,\"ncyl\":%d,\"ec_key\":%d,"
    "\"afr\":%.2f,\"ct_frames\":%llu,\"ct_idle_skip_s\":%.3f,\"trim_pct\":[%d,%d],"
    "\"duty_pct\":[%.2f,%.2f],\"fuel_gph\":%.2f,\"link\":[{\"rpm\":%d,\"map_inhg\":%.2f,\"tps_pct\":%.1f,"
    "\"iat_f\":%d,\"et_f\":%d,\"adv_deg\":%d},{\"rpm\":%d,\"map_inhg\":%.2f,\"tps_pct\":%.1f,\"iat_f\":%d,"
    "\"et_f\":%d,\"adv_deg\":%d}],"
    "\"ecu\":["
      "{\"powered\":%s,\"err\":%d,\"ct_to_ecu\":%llu,\"ecu_to_ct\":%llu,\"bad\":%llu,\"text\":%llu,"
       "\"idle_skip_s\":%.3f,\"warps\":%llu,"
       "\"coils\":[{\"sparks\":%llu,\"dwell_ms\":%.2f,\"adv_deg\":%.1f},"
                  "{\"sparks\":%llu,\"dwell_ms\":%.2f,\"adv_deg\":%.1f},"
                  "{\"sparks\":%llu,\"dwell_ms\":%.2f,\"adv_deg\":%.1f}],"
       "\"inj\":[{\"pulses\":%llu,\"width_ms\":%.3f},{\"pulses\":%llu,\"width_ms\":%.3f},"
                "{\"pulses\":%llu,\"width_ms\":%.3f},{\"pulses\":%llu,\"width_ms\":%.3f},"
                "{\"pulses\":%llu,\"width_ms\":%.3f},{\"pulses\":%llu,\"width_ms\":%.3f}]},"
      "{\"powered\":%s,\"err\":%d,\"ct_to_ecu\":%llu,\"ecu_to_ct\":%llu,\"bad\":%llu,\"text\":%llu,"
       "\"idle_skip_s\":%.3f,\"warps\":%llu,"
       "\"coils\":[{\"sparks\":%llu,\"dwell_ms\":%.2f,\"adv_deg\":%.1f},"
                  "{\"sparks\":%llu,\"dwell_ms\":%.2f,\"adv_deg\":%.1f},"
                  "{\"sparks\":%llu,\"dwell_ms\":%.2f,\"adv_deg\":%.1f}],"
       "\"inj\":[{\"pulses\":%llu,\"width_ms\":%.3f},{\"pulses\":%llu,\"width_ms\":%.3f},"
                "{\"pulses\":%llu,\"width_ms\":%.3f},{\"pulses\":%llu,\"width_ms\":%.3f},"
                "{\"pulses\":%llu,\"width_ms\":%.3f},{\"pulses\":%llu,\"width_ms\":%.3f}]}"
    "]"
    "}",
    BOOTED ? "true" : "false", NOW_S, s32board_rpm_at(&B, NOW_S), B.ncyl, LAST_EC_KEY,
    B.afr_live, (unsigned long long)CT.gfx.frames, CT.idle_skipped / FCY, trim[0], trim[1],
    duty[0], duty[1], fuel_gph,
    link_rpm[0], map[0], tps[0], iat[0], et[0], adv[0], link_rpm[1], map[1], tps[1], iat[1], et[1], adv[1],
    /* ECU 1 */
    B.on[SB_E1] ? "true" : "false", ECU[0].last_err,
    (unsigned long long)DF_CT2E[0].frames, (unsigned long long)DF_E2CT[0].frames,
    (unsigned long long)(DF_CT2E[0].bad + DF_E2CT[0].bad), (unsigned long long)DF_E2CT[0].text,
    ECU[0].idle_skipped / ECU_SYSCLK_HZ, (unsigned long long)ECU[0].warps,
    (unsigned long long)COIL[0][0].n, COIL[0][0].last_dwell_ms, COIL[0][0].last_adv_deg,
    (unsigned long long)COIL[0][1].n, COIL[0][1].last_dwell_ms, COIL[0][1].last_adv_deg,
    (unsigned long long)COIL[0][2].n, COIL[0][2].last_dwell_ms, COIL[0][2].last_adv_deg,
    (unsigned long long)INJ[0][0].n, INJ[0][0].last_width_ms,
    (unsigned long long)INJ[0][1].n, INJ[0][1].last_width_ms,
    (unsigned long long)INJ[0][2].n, INJ[0][2].last_width_ms,
    (unsigned long long)INJ[0][3].n, INJ[0][3].last_width_ms,
    (unsigned long long)INJ[0][4].n, INJ[0][4].last_width_ms,
    (unsigned long long)INJ[0][5].n, INJ[0][5].last_width_ms,
    /* ECU 2 */
    B.on[SB_E2] ? "true" : "false", ECU[1].last_err,
    (unsigned long long)DF_CT2E[1].frames, (unsigned long long)DF_E2CT[1].frames,
    (unsigned long long)(DF_CT2E[1].bad + DF_E2CT[1].bad), (unsigned long long)DF_E2CT[1].text,
    ECU[1].idle_skipped / ECU_SYSCLK_HZ, (unsigned long long)ECU[1].warps,
    (unsigned long long)COIL[1][0].n, COIL[1][0].last_dwell_ms, COIL[1][0].last_adv_deg,
    (unsigned long long)COIL[1][1].n, COIL[1][1].last_dwell_ms, COIL[1][1].last_adv_deg,
    (unsigned long long)COIL[1][2].n, COIL[1][2].last_dwell_ms, COIL[1][2].last_adv_deg,
    (unsigned long long)INJ[1][0].n, INJ[1][0].last_width_ms,
    (unsigned long long)INJ[1][1].n, INJ[1][1].last_width_ms,
    (unsigned long long)INJ[1][2].n, INJ[1][2].last_width_ms,
    (unsigned long long)INJ[1][3].n, INJ[1][3].last_width_ms,
    (unsigned long long)INJ[1][4].n, INJ[1][4].last_width_ms,
    (unsigned long long)INJ[1][5].n, INJ[1][5].last_width_ms
  );
  return TELEM_JSON;
}

EMSCRIPTEN_KEEPALIVE
const char *s32_wasm_get_ecu_console(int ecu_idx) {
  if (ecu_idx < 0 || ecu_idx >= 2) return "";
  return ECU_CON_BUF[ecu_idx];
}
