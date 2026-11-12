/* s32trace.h -- the canonical conformance trace: what a bench with a logic analyser on every pin and a tap on every
 * wire would record, in one fixed text format, the same bytes from every front end (emu/pair, emu/wasm) and every
 * build (native, WebAssembly). Two runs of the same stimulus are conformant when their traces are byte-identical
 * (scripts/conformance.py; docs/conformance.md).
 *
 * Physical observables only -- never firmware RAM:
 *   output pins   every output-latch change on any port of each MCU (lat_sink: injector drivers, tach, fuel-flow, LEDs,
 *                 test points, chip selects, panel controls ...), each ECU's output-compare coil pulses (on, off cycle);
 *   wires         every byte each MCU's UARTs transmit (link pairs and consoles), stamped with the sender's cycle count,
 *                 and every byte delivered over each cat5 pair after the harness (faults applied), stamped by quantum;
 *   panel         a hash of the picture the panel shows (emu/common/s32panel.c, the renderer every front end uses), its
 *                 backlight duty, lit / dark and why, and the graphics controller's display registers, checked after
 *                 every quantum with a vblank or a Controller power change, written when any of it changed;
 *   at the end    SHA-256 of each FRAM chip, each SD card image, each ECU's program flash chip, and every MCU's cycle count.
 *
 * Time stamps are integers: an MCU's own cycle count (Controller Fcy, ECU SYSCLK; crystal error applied) or the quantum
 * number (100 us each). Within a quantum the lines are in a fixed order -- the quantum's barrier lines, then the
 * Controller's, ECU1's and ECU2's own lines in the order each MCU produced them -- so the trace is the same however
 * the MCUs were scheduled (one thread each, or one after another).
 *
 * Threading: s32trace_pin/coil/tx for source `who` are called only from the thread that runs that MCU during a
 * quantum; s32trace_link/quantum/finish only between quanta (all MCUs parked). */
#ifndef S32TRACE_H
#define S32TRACE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define S32TRACE_VERSION 1

/* A growable text buffer (also used by emu/pair for its per-thread log lines). */
typedef struct { char *p; size_t n, cap; } s32_tbuf;
void s32_tbuf_printf(s32_tbuf *b, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
void s32_tbuf_put(s32_tbuf *b, const char *s, size_t n);
void s32_tbuf_free(s32_tbuf *b);

typedef struct { uint32_t h[8]; uint64_t len; uint8_t blk[64]; unsigned nblk; } s32_sha256;
void s32_sha256_init(s32_sha256 *s);
void s32_sha256_update(s32_sha256 *s, const void *p, size_t n);
void s32_sha256_final(s32_sha256 *s, uint8_t out[32]);
void s32_sha256_hex(const void *p, size_t n, char out[65]);

struct s32board;
struct s32ct;

typedef struct s32trace {
  bool on;
  FILE *f;                     /* sink (native); NULL: lines stay in `out` until the front end takes them (WebAssembly) */
  s32_tbuf src[3];             /* per MCU (SB_CT, SB_E1, SB_E2), filled during a quantum by that MCU's thread */
  s32_tbuf bar;                /* barrier lines of the current quantum */
  s32_tbuf out;                /* canonical bytes not yet written to f (or taken by the front end) */
  s32_sha256 sha;              /* of every byte emitted so far */
  uint64_t lines, bytes;
  uint64_t fb_frames;          /* vblanks seen at the last panel check */
  bool ct_on;                  /* Controller powered at the last panel check */
  char panel_last[200];        /* last panel line written */
  long q;                      /* quantum being recorded */
} s32trace;

/* Start a trace to `f` (may be NULL: memory only). Writes the header. */
void s32trace_open(s32trace *t, FILE *f);
/* Per-MCU events (from that MCU's thread). who = SB_CT / SB_E1 / SB_E2. */
void s32trace_pin(s32trace *t, int who, uint64_t cyc, unsigned port, uint16_t old_pins, uint16_t new_pins);
void s32trace_coil(s32trace *t, int who, unsigned oc, uint64_t on_cyc, uint64_t off_cyc);
void s32trace_tx(s32trace *t, int who, uint64_t cyc, unsigned uart, uint8_t byte);
/* Between quanta: bytes delivered over cat5 pair k this quantum (to_ecu: Controller -> ECUk). */
void s32trace_link(s32trace *t, long q, int k, bool to_ecu, const uint8_t *b, unsigned n);
/* End of quantum q (every MCU run to its end, the quantum's bytes delivered): framebuffer check, then the quantum's
 * lines in canonical order go to the output. ct may be NULL (no panel) or unpowered (ct_on false: no check). */
void s32trace_quantum(s32trace *t, long q, struct s32ct *ct, bool ct_on);
/* End of run: final state lines (FRAM, SD, program flash, cycle counts), flush, and the trace's own SHA-256 (hex) in
 * `sha_hex` when non-NULL. pflash[k] = ECUk's program flash chip as a device programmer reads it (may be NULL). */
void s32trace_finish(s32trace *t, const struct s32board *b, const uint8_t *const pflash[2], const size_t npflash[2],
                     char sha_hex[65]);
/* Write out (or keep, with no sink) everything emitted so far. */
void s32trace_flush(s32trace *t);
void s32trace_free(s32trace *t);

#endif
