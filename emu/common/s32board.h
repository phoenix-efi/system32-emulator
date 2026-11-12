/* s32board.h -- the target system's wiring harness and physical world, shared by every front end (emu/pair s32pair,
 * emu/wasm). One canonical model of everything outside the three MCUs: the engine and its two crank/sync Hall sensors,
 * every analogue sensor and its wiring, the cockpit switches (P-lead run-up switches, the ECU Select switch and the
 * injector-enable wires it grounds), the retard input, the ECU DIP switches, the Controller's DB-15 inputs (battery and
 * fuel-pump senses, wideband O2), its bezel buttons and encoder, the power supplies (per-ECU breaker and voltage,
 * brown-out, MCLR, the Controller powered through the cat5 cables), the cat5 data pairs, the console UARTs, the FRAM,
 * RTC and SD card slots, and the injector harness downstream of the drivers. model/io_ledger.json lists every signal,
 * its evidence and the knob below that drives it.
 *
 * The board observes and drives ONLY real interfaces: MCU pins (levels, scheduled edges), ADC input voltages (as raw
 * counts at the pin), UART bytes and SPI devices. It never reads or writes firmware RAM.
 *
 * Knobs: s32board_knob(name, value) -- the same names in every front end (s32pair reads them from the environment,
 * the browser bench passes them through s32_wasm_knob). s32board_knob_help() lists them. Times are emulated seconds.
 */
#ifndef S32BOARD_H
#define S32BOARD_H
#include "s32ct.h"
#include "s32dev.h"
#include "ecu.h"
#include "bench_engine.h"
#include "link_decode.h"
#include "fram_format.h"
#include "s32trace.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

enum { SB_CT = 0, SB_E1 = 1, SB_E2 = 2, SB_WHO = 3 };
#define SB_QUANTUM_S 100e-6           /* co-simulation quantum (emu/pair, emu/wasm) */
#define SB_FCY 16000000.0             /* Controller instruction clock (config words: 8 MHz XT, 96 MHz PLL, CPDIV 1) */
#define SB_HOME_T0 1e6                /* sentinel base for "{H+x}" times before the Controller shows its home page */
/* Phase anchors: a time "<t>~<event>[+x]" is x after the first <event> at or after t (t: seconds, "{H}" or
 * "{H+x}"), so a stimulus reaches every firmware at the same point of its own cycle whatever its boot time. Events, as a
 * protocol analyser or a scope on the panel would see them: "ct1" / "ct2" the end of a complete frame the Controller
 * sends to ECU1 / ECU2, "ecu1" / "ecu2" the end of a complete frame that ECU sends to the Controller (the closing flag on
 * the wire, CRC good), each optionally of one message type ("ct1#8"); "panel" a Controller panel refresh (vblank).
 * Until its event occurs an anchored time is a sentinel SB_ANCHOR_T0 + slot * SB_ANCHOR_SPAN + offset; the times a knob
 * derives from it (the later edges of a detent, contact bounce) stay in its slot and move with it. An anchored time is
 * placed on the quantum grid (the first quantum boundary at or after event + offset). A run that ends with an anchor
 * that never occurred fails (s32board_anchor_check). */
#define SB_ANCHOR_T0 1e7
#define SB_ANCHOR_SPAN 1e4
#define SB_MAX_ANCHOR 256

typedef struct {
  /* (Re)initialise MCU `who` (SB_CT / SB_E1 / SB_E2): CPU and peripherals reset, devices re-attached; FRAM, SD and RTC
   * contents kept. cold = power-on (supply restored); !cold = MCLR reset pulse. Called between quanta. */
  void (*boot)(void *ctx, int who, bool cold);
  void (*snap)(void *ctx, const char *name);   /* bezel script "snap:<name>": dump the screen (may be NULL) */
  void *ctx;
} s32board_host;

/* ---- fault and event records (internal; listed here so front ends can report them) ---- */
typedef struct { double t0, t1, period; uint16_t val, noise, frozen; uint8_t mode; bool latched; } sb_afault;
typedef struct { double t0, t1, period, us; uint8_t who, port, bit, mode; bool latched, frozen; } sb_pfault;
typedef struct { double t0, t1, us, rate; uint8_t k, kind, elem; } sb_cfault;
typedef struct {
  double t0, t1, v, dt, next_t;
  uint32_t n;
  int32_t arg1, arg2, arg3;
  int32_t cfg28[28];                 /* LF_SUB_CFG_BLOCK: the 28 config words an e2c config_block frame carries */
  uint8_t k, kind, sub, pace, burst;
  bool done;
} sb_lfault;
typedef struct { double t; uint32_t a; double v; uint16_t s; uint8_t kind, who, port, bit, level, dyn; } sb_ev;
enum { SB_ANC_CT1, SB_ANC_CT2, SB_ANC_ECU1, SB_ANC_ECU2, SB_ANC_PANEL };
typedef struct { double base, at; int16_t type; uint8_t src; bool fired; char spec[48]; } sb_anchor;
typedef struct { uint16_t crc; uint16_t n; uint8_t t0, t1; bool in, esc; } sb_frame_watch;   /* frame ends on one wire */   /* dyn: added while running */

#define SB_MAX_AF 4
#define SB_MAX_PF 32
#define SB_MAX_CF 64
#define SB_MAX_LF 32
#define SB_MAX_EV 8192
#define SB_TEXT 16384
#define SB_ENG_HIST 8192

typedef struct s32board {
  s32board_host host;
  s32ct *ct;
  ecu_t *ecu[2];
  s32_fram *fram[SB_WHO];                  /* FRAM chip on each board (front end attaches it) */
  s32_sd *sd[SB_WHO];                      /* SD card in each slot (NULL = slot empty) */
  s32_rtc *rtc;                            /* Controller RTC */
  char err[256];
  uint32_t rng;
  double now;                              /* start of the current quantum */
  double home_at;                          /* simulated time when the Controller first showed the completed home page (< 0 = not yet) */
  uint64_t panel_frames;                   /* Controller vblank count last checked for the home page */
  bool splash_seen;                        /* a lit non-home panel frame (the splash) has been shown since Controller boot */
  long q;

  /* power */
  double supply_v[2];                      /* each ECU's 12 V feed (its breaker), volts */
  bool ct_absent;                          /* no Controller on the bench (its slot empty): never powered */
  bool cut[2];                             /* S32_ECU_POWER: breaker pulled */
  double bor_v;                            /* below this supply the board's regulators drop out: MCU held in reset */
  bool on[SB_WHO];                         /* MCU running (powered and out of reset) */
  double ppm[SB_WHO];                      /* crystal error per MCU */
  double first_spark_after_on[2], first_inj_after_on[2], on_at[2];
  unsigned power_cycles[SB_WHO], resets[SB_WHO], wdt_resets[2];

  /* engine and crank/sync sensors (one dual-element Hall sensor per ECU) */
  double rpm, start_s, ext_rpm, sync_before, mag[2], hall_deg, jitter_us, angle;
  int ncyl;
  struct { double t0, a0, rpm; } eh[SB_ENG_HIST];
  long eh_last;
  sb_cfault cf[SB_MAX_CF]; unsigned ncf;
  bool hall_stuck[2][2];                   /* [k][0 crank, 1 sync]: line held by a fault (no edges) */
  uint64_t crank_edges[2][2], glitches[2];

  /* analogue inputs: nominal raw counts at the pin (host or physics), fault overlay, per-sample noise */
  uint16_t an_nom[SB_WHO][32];
  bool an_host[SB_WHO][32];                /* the host set this channel explicitly (wins over the physics default) */
  uint16_t an_noise[SB_WHO][32];
  sb_afault af[SB_WHO][32][SB_MAX_AF]; uint8_t naf[SB_WHO][32];
  int tps_supply;                          /* ECU (0/1) whose 5 V excitation powers the shared TPS */
  int tps_unpowered;                       /* 0 gnd, 1 float, 2 pull-up: the TPS wiper with its supply ECU unpowered */
  bool et_both;                            /* ET sensor wired to both ECUs (documented harness: ECU1 only) */
  double batt_v[2];                        /* the Controller's DB-15 battery senses (Bus Manager) */
  double afr_fixed;                        /* S32_AFR: fixed wideband reading (< 0 = engine model) */
  double afr_live;
  bench_engine bench;
  inj_meter inj;                           /* the injectors, downstream of both ECUs' drivers and the enable gates */

  /* discrete inputs: contact level per pin (what the harness presents), board-owned mask, wire faults */
  uint16_t lvl[SB_WHO][7], own[SB_WHO][7];
  sb_pfault pf[SB_MAX_PF]; unsigned npf;
  bool plead_stop[2], inj_gnd[2], retard[2], dip[2][4];
  int select;                              /* ECU Select switch: 1, 2, 0 = in transit / absent (both open), 3 = both grounded */
  bool sw_legacy;                          /* S32_SWITCHES: enable wires set per ECU, no select switch */
  double select_gap_s;                     /* break-before-make transit time of the select switch */
  double bounce_s; int bounce_edges;       /* contact bounce on every mechanical switch transition */
  bool pump[2], digin_lo[2];
  int enc_gray;

  /* cat5 data pairs, consoles */
  sb_lfault lf[SB_MAX_LF]; unsigned nlf;
  bool link_dead[2];                       /* host switch (browser bench): data pair dead, ECU powered */
  bool link_table_full;                    /* S32_LINK_TABLE_SEAL=full: bench table frames carry the 322-byte sum */
  uint64_t link_bits_flipped[2], link_bytes_lost[2], link_garbage[2], link_frames_sent[2];
  struct {
    link_deframer d;
    uint8_t q[4096], hold[4096];
    unsigned qh, qt, hn, cd, pace, busy_q;
  } lmux[2][2];                            /* [k][0 = CT->ECUk, 1 = ECUk->CT]: frame-boundary link mux */

  /* timed events and their text */
  sb_ev ev[SB_MAX_EV]; unsigned nev, iev;
  char text[SB_TEXT]; unsigned ntext;

  /* harness-level observers (output pins) */
  struct { uint64_t tach_edges, ff_edges, ff_edges_win, tp_edges[4]; uint16_t leds; uint64_t led_changes; uint64_t tach_at[32]; double ff_last_at; } eo[2];
  double ff_win_from;                      /* fuel-flow edges at/after this time also count into eo[].ff_edges_win */
  bool ff_win_set;                         /* the run set a statistics window (S32_STATS_FROM): report the windowed count */
  struct { uint16_t digout; uint64_t digout_changes, tcon_frames, tcon_bits, panel_en_changes; bool panel_en; } co;

  s32trace *trace;                         /* conformance trace (emu/common/s32trace.h): every output-pin change; NULL = off */

  /* phase anchors (SB_ANCHOR_T0) */
  sb_anchor anchor[SB_MAX_ANCHOR]; unsigned nanchor;
  sb_frame_watch fw[2][2];                 /* [k][0 = CT->ECUk, 1 = ECUk->CT]: frame ends as sent (before cable faults) */
  uint64_t anchor_frames;                  /* Controller vblank count last seen by the panel anchor */
} s32board;

/* Initialise with defaults (the documented IO-540-class harness, engine stopped). The host links its MCUs and devices
 * afterwards (b->ct, b->ecu[], b->fram[], b->sd[], b->rtc). */
void s32board_init(s32board *b, s32board_host host);
/* Set one knob (see s32board_knob_help). Returns 0, or -1 unknown name / -2 bad value (b->err says why). */
int s32board_knob(s32board *b, const char *name, const char *value);
/* Every knob from the environment (s32pair). Returns the number applied; exits(2) on a bad value. */
int s32board_knobs_from_env(s32board *b);
/* One line per knob: "NAME  description". */
void s32board_knob_help(FILE *f);
/* Names of all knobs, NULL-terminated (the ledger check compares them with model/io_ledger.json). */
const char *const *s32board_knob_names(void);

/* Before running quantum q (covering [ (q-1)Q, qQ )): due events, power, pins, analogue inputs, crank edges. */
void s32board_step(s32board *b, long q);
/* Bytes one side put on the wire this quantum -> what arrives at the other end (cat5 faults; unpowered transceivers).
 * Returns the count of bytes in `buf` to deliver (may grow by injected garbage, up to cap). */
unsigned s32board_link(s32board *b, int k, bool to_ecu, uint8_t *buf, unsigned n, unsigned cap);
/* After each quantum (every MCU run to the quantum's end): an ECU whose watchdog expired -- enabled by its fuse or by
 * software and not cleared in time, also while its core is stopped on an exception -- is reset through host.boot
 * (RCON.WDTO set). Returns how many ECUs were reset. */
int s32board_watchdogs(s32board *b);
/* Run targets: cycles each MCU should have executed at the end of quantum q (crystal error applied). */
uint64_t s32board_target(const s32board *b, int who, long q);
double s32board_hz(const s32board *b, int who);
/* Crank angle (deg, TDC #1 = 0) at emulated time t. */
double s32board_angle(const s32board *b, double t);
double s32board_rpm_at(const s32board *b, double t);

/* Host inputs (replay, X-Plane, browser UI) -- through the same physical model as the knobs. */
void s32board_set_rpm(s32board *b, double rpm);                         /* < 0: back to S32_RPM / S32_START */
void s32board_set_an(s32board *b, int who, unsigned ch, uint16_t raw);  /* explicit pin counts (wins over physics) */
void s32board_set_switches(s32board *b, int k, bool plead_open, bool inj_open);   /* the two wires as the pins see them */
void s32board_set_retard(s32board *b, int k, bool on);
void s32board_set_plead(s32board *b, int k, bool open);                   /* the P-lead run-up switch: open = RUN */
void s32board_an_fault_clear(s32board *b, int who, unsigned ch);          /* remove every analogue fault on a channel */
void s32board_set_supply(s32board *b, int k, double volts, bool breaker_in);
void s32board_ct_contact(s32board *b, double t, unsigned port, unsigned bit, bool level);   /* bezel contact (bounced) */
void s32board_encoder_edge(s32board *b, double t, int dir);            /* one quadrature edge (+1 / -1) */
int s32board_input_script(s32board *b, const char *spec);             /* S32_INPUT: bezel script */

/* Output observers: the front end forwards each MCU's lat_sink here (s32board_attach does it for it). */
void s32board_attach(s32board *b);
void s32board_on_coil(s32board *b, int k, double t_off);
/* End-of-run harness report (injectors at the engine, tach, fuel flow, LEDs, Controller outputs, faults applied). */
void s32board_report(s32board *b, FILE *f);
/* The fault knobs as applied ("fault: ..." lines, the format emu/pair's tests parse). */
void s32board_fault_summary(const s32board *b, double secs, FILE *f);
/* Snapshots (emu/common/s32snap.h). The board's state with every pointer zeroed. */
void s32board_snap_blob(const s32board *b, s32_tbuf *out);
/* What the knobs set up, taken right after they were applied: the static part (every setting that is not tied to a
 * time, pointers zeroed) and the timed items (events, analogue / pin / crank / link faults, engine start). */
void s32board_stimulus(const s32board *b, s32_tbuf *static_out, s32_tbuf *items_out);
/* Restore a board saved at a quantum boundary into `b`, which holds this run's own knobs (its state right after
 * s32board_knobs_from_env and s32board_attach): the saved state, with this run's timed items from t_future on in
 * place of the saved run's (the caller has checked that the items before t_future are the same). Returns 0, or -1 with
 * `why` set. */
int s32board_snap_restore(s32board *b, const uint8_t *blob, size_t n, double t_future, char *why, size_t nwhy);

/* Clock tree from the images' configuration words vs the emulator's clocks; returns 0 when they agree. */
int s32board_clock_check(const uint8_t *ec_bfl, size_t nb, const uint8_t *ct_prog, size_t nct, FILE *f);
/* Parse a time token: a number, or "{H}" / "{H+x}" with x >= 0 (returns SB_HOME_T0 + x, resolved when the home page
 * appears; "{H-x}" is refused). A phase anchor "~<event>[#type][+x]" after it is accepted only while a knob or bezel
 * script is being parsed for a board (it takes an anchor slot there); elsewhere it is refused. */
bool s32board_parse_time(const char *s, const char **endp, double *out);
/* After the run: one "emulator error: anchor ..." line on f for each anchored time whose event never occurred (its
 * stimulus never acted). Returns how many. */
int s32board_anchor_check(const s32board *b, FILE *f);

#endif
