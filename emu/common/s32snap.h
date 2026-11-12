/* s32snap.h -- whole-system snapshots: the state of the three MCUs, their peripherals and devices, the harness and the
 * front end's observers at a quantum boundary, written to a file and restored in another process, so a run can start
 * from that point instead of from power-on. A restored run is the same run: it gives the same conformance trace, the
 * same logs and the same end state as running straight through (scripts/conformance.py checks this for every case;
 * docs/conformance.md).
 *
 * File: a gzip stream of "S32SNAP" + format version + layout fingerprint, then tagged chunks (tag, length, bytes).
 * Each component writes its own chunk with every pointer field zeroed; restoring copies the chunk over the live
 * component (set up by the normal start-up path of the restoring process, which owns the pointers) and keeps the live
 * pointers. A snapshot is restorable only by a build with the same layout (same emulator sources, same pointer size);
 * the chunk "build" names the build.
 *
 * Stimulus: what the run will do, as the harness parsed it (s32board_stimulus): a static part (every configuration that
 * is not tied to a time) and timed items (events, faults, engine start, each with its time). A snapshot taken at time t
 * can be restored by a run whose static part is identical and whose timed items before t + S32SNAP_GUARD_S are
 * identical; items from then on may differ (that is how one boot serves many cases). */
#ifndef S32SNAP_H
#define S32SNAP_H
#include "s32trace.h"
#include "s32ct.h"
#include "s32dev.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define S32SNAP_VERSION 1u
#define S32SNAP_GUARD_S 0.010   /* timed items this close after the snapshot point must match too */

/* ---- container ---- */
typedef struct { s32_tbuf buf; } s32snap_w;
typedef struct { char tag[48]; const uint8_t *p; size_t n; } s32snap_chunk;
typedef struct { uint8_t *data; size_t n; s32snap_chunk *c; size_t nc; char err[256]; } s32snap_r;

void s32snap_w_init(s32snap_w *w);
void s32snap_put(s32snap_w *w, const char *tag, const void *p, size_t n);
/* Write everything to `path` (gzip, level 1). Returns 0, or -1 (errno set). Frees the buffer. */
int s32snap_w_save(s32snap_w *w, const char *path, uint32_t layout);
/* Read a snapshot file. Returns 0, or -1 with r->err set (missing file, not a snapshot, other layout). */
int s32snap_r_load(s32snap_r *r, const char *path, uint32_t layout);
const s32snap_chunk *s32snap_get(const s32snap_r *r, const char *tag);
void s32snap_r_free(s32snap_r *r);

/* Little helpers to append fixed-size fields to a byte buffer (item bytes, small chunks). */
void s32snap_b(s32_tbuf *b, const void *p, size_t n);
void s32snap_u8(s32_tbuf *b, uint8_t v);
void s32snap_u16(s32_tbuf *b, uint16_t v);
void s32snap_u32(s32_tbuf *b, uint32_t v);
void s32snap_u64(s32_tbuf *b, uint64_t v);
void s32snap_f64(s32_tbuf *b, double v);

/* Timed stimulus items: kind, sub (array index), time, bytes. Equal items before t_future in the same order.
 * Home-page times: an item time t >= home_t0 / 2 is "{H+x}" with x = t - home_t0 (home_t0 = 0: no such times). Such an
 * item counts as before t_future when x < 0 (it acts before the home page: always compared), or when home_at + x <
 * t_future (home_at >= 0: the home page was already seen, or predicted, when the snapshot was taken), or when t_snap + x
 * < t_future (home page not seen yet: it comes after t_snap, so t_snap + x is the earliest the item can act). */
void s32snap_item(s32_tbuf *items, uint8_t kind, uint16_t sub, double t, const void *p, size_t n);
bool s32snap_items_equal_before(const uint8_t *a, size_t na, const uint8_t *b, size_t nb, double t_future, double home_t0,
                                double home_at, double t_snap, char *why, size_t nwhy);
/* JSON array of the items: [[kind, sub, time, "sha256 of the bytes (16 hex digits)"], ...]; a home-page item (time >=
 * home_t0 / 2, home_t0 > 0) is [kind, sub, x, hash, 1] with x its offset from the home page. */
void s32snap_items_json(const uint8_t *p, size_t n, double home_t0, FILE *f);

/* ---- components in the public headers (Controller, devices, trace) ---- */
/* Controller: the SoC (core, SFRs, timers, SPI, UART, graphics, flash) with every pointer zeroed. */
void s32snap_ct_blob(const s32ct *ct, s32_tbuf *out);
int s32snap_ct_restore(s32ct *ct, const uint8_t *p, size_t n);
void s32snap_rtc_blob(const s32_rtc *r, s32_tbuf *out);
int s32snap_rtc_restore(s32_rtc *r, const uint8_t *p, size_t n);
/* SD card: the controller state; the contents go separately as the blocks that differ from `orig` (the image as loaded). */
void s32snap_sd_blob(const s32_sd *c, s32_tbuf *out);
int s32snap_sd_restore(s32_sd *c, const uint8_t *p, size_t n);
void s32snap_sd_diff(const s32_sd *c, const uint8_t *orig, s32_tbuf *out);
int s32snap_sd_apply(s32_sd *c, const uint8_t *p, size_t n);
/* Trace: its running state (SHA-256 so far, counts, panel state); the bytes already written go in their own chunk. */
void s32snap_trace_blob(const s32trace *t, s32_tbuf *out);
int s32snap_trace_restore(s32trace *t, const uint8_t *p, size_t n);

#endif
