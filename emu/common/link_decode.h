/* link_decode.h -- the Controller <-> ECU serial link at the wire: deframing, encoding and payload decoding.
 *
 * Everything here follows the link protocol as it appears on the wire: HDLC-style
 * framing (flag 0x7E, escape 0x7D, xor 0x20), body = type u16le + payload + CRC-16/MCRF4XX u16le, and the ECU status
 * frame (type 2, 48 bytes). It is the ONLY way the emulator front ends (emu/pair, emu/wasm) learn live engine values:
 * they decode what crosses the wires, exactly as a protocol analyser clipped onto the link would, and never look into
 * a firmware's RAM. Any firmware that keeps the same protocol (the stock firmware, for one) works the same.
 */
#ifndef LINK_DECODE_H
#define LINK_DECODE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LINK_FLAG 0x7Eu
#define LINK_ESC 0x7Du
#define LINK_ESC_XOR 0x20u
#define LINK_BODY_MAX 800u   /* deframer buffer; the largest frame measured is 326 bytes */

/* Message types used by the front ends. */
enum {
  LINK_C2E_SET_CONFIG = 3,   /* Controller -> ECU: u32 config index (1..28) + s32 value (ECU replies type 4) */
  LINK_C2E_WRITE_FUEL = 5,   /* Controller -> ECU: 324-byte fuel table (ECU replies type 8) */
  LINK_C2E_WRITE_IGN = 7,    /* Controller -> ECU: 324-byte ignition table (ECU replies type 9) */
  LINK_C2E_TRIM_DELTA = 9,   /* Controller -> ECU: s32 trim delta, one per encoder step */
  LINK_C2E_TRIM_ABS = 10,    /* Controller -> ECU: s32 absolute trim */
  LINK_E2C_STATUS = 2,       /* ECU -> Controller: 48-byte status, ~every 500 ms */
  LINK_E2C_CONFIG_BLOCK = 5, /* ECU -> Controller: 112-byte config block (28 x u32le), on a config_dump request */
  LINK_E2C_ACK_FUEL = 8,     /* ECU -> Controller: s32 result of a fuel-table write (1 = accepted) */
  LINK_E2C_ACK_IGN = 9,      /* ECU -> Controller: s32 result of an ignition-table write (1 = accepted) */
};

uint16_t link_crc16(const uint8_t *b, size_t n);   /* CRC-16/MCRF4XX: reflected poly 0x8408, init 0xFFFF, xorout 0 */

typedef void (*link_frame_fn)(void *ctx, unsigned type, const uint8_t *payload, unsigned len, unsigned wire_bytes);
typedef void (*link_text_fn)(void *ctx, uint8_t b);

typedef struct link_deframer {
  uint8_t buf[LINK_BODY_MAX];
  unsigned n;            /* unescaped body bytes so far */
  unsigned wire;         /* wire bytes since the opening flag (escapes included): the frame's length on the line */
  bool esc, in;          /* in = inside framed traffic (false before the first flag = plain text, e.g. a console) */
  uint64_t frames, bad, text, by_type[16];
  link_frame_fn on_frame;  /* each good frame (CRC residue 0); may be NULL */
  link_text_fn on_text;    /* each byte outside framed traffic; may be NULL */
  void *ctx;
} link_deframer;

void link_deframer_init(link_deframer *d, link_frame_fn on_frame, link_text_fn on_text, void *ctx);
void link_deframe_byte(link_deframer *d, uint8_t b);
/* True when no frame body is in progress on this direction of the wire (just after a flag, nothing pending), so a
 * complete frame can be merged into the byte stream here without splitting one of the sender's frames. */
bool link_deframer_idle(const link_deframer *d);

/* Encode one frame: flag, stuffed (type, payload, CRC), flag. Returns the wire length, 0 if `cap` is too small. */
size_t link_encode(unsigned type, const uint8_t *payload, size_t len, uint8_t *out, size_t cap);

/* ECU status frame (type 2, 48 bytes). Field meanings OBSERVED on the wire with the stock firmware, driving known
 * inputs (2400 RPM, MAP 24.0 inHg, TPS 55 %, ET 185 F, IAT 72 F, trim -3):
 * +40/+42 are s16 ET and IAT in deg F and +24 is MAP in hundredths of inHg; +12, +28, +32 and +36 are kept raw. */
typedef struct {
  uint32_t fw_version;     /* +0 */
  uint16_t rpm;            /* +4 */
  float inj_duty_pct;      /* +8 */
  float word12;            /* +12 f32, meaning open (scales with fuel delivered) */
  float tps_pct;           /* +16 */
  int32_t fuel_trim_pct;   /* +20 */
  uint32_t map_x100;       /* +24 MAP, hundredths of inHg */
  uint32_t word28, word32, word36;   /* raw */
  int16_t et_f, iat_f;     /* +40, +42 */
  uint8_t inj_inhibit;     /* +44 */
  int8_t ign_adv_deg;      /* +45 */
  uint8_t plead_inhibit;   /* +46 */
  uint8_t status_flags;    /* +47 */
} link_status;
bool link_decode_status(const uint8_t *payload, unsigned len, link_status *s);   /* false unless len == 48 */
bool link_decode_s32(const uint8_t *payload, unsigned len, int32_t *v);          /* first 4 bytes, s32le */

/* Fuel-trim tracker: what ECU1's trim is right now, from the wire alone. The status frame reports the trim (+20) about
 * every 500 ms; between reports, the Controller's trim-delta frames (type 9) that reached the ECU after the report was
 * built are added. A delta is taken as already inside a report when it ended more than `lag_s` before the report's
 * first byte (the ECU's processing + status build-to-wire latency; OBSERVED, see emu/pair). */
#define LINK_TRIM_MAX_PENDING 64
typedef struct {
  bool have;               /* a status frame has been seen */
  int32_t base;            /* trim in the newest status frame */
  double base_t;           /* emulated time of that frame's first byte */
  double lag_s;
  struct { double t; int32_t d; } pend[LINK_TRIM_MAX_PENDING];
  unsigned npend;
  uint64_t reports, mismatches;   /* status frames seen; ones that disagreed with the running estimate */
} link_trim_tracker;
void link_trim_init(link_trim_tracker *k, double lag_s);
void link_trim_delta(link_trim_tracker *k, double t_end, int32_t delta);   /* a type-9 frame delivered at t_end */
void link_trim_status(link_trim_tracker *k, double t_start, int32_t trim); /* a status frame that began at t_start */
int32_t link_trim_estimate(const link_trim_tracker *k);

#endif
