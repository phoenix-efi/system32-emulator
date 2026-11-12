/* link_decode.c -- see link_decode.h. Wire-level only: bytes in, frames and decoded fields out. */
#include "link_decode.h"
#include <string.h>

uint16_t link_crc16(const uint8_t *b, size_t n) {
  uint16_t c = 0xFFFFu;
  for (size_t i = 0; i < n; i++) {
    c ^= b[i];
    for (int k = 0; k < 8; k++) c = (c & 1u) ? (uint16_t)((c >> 1) ^ 0x8408u) : (uint16_t)(c >> 1);
  }
  return c;
}

void link_deframer_init(link_deframer *d, link_frame_fn on_frame, link_text_fn on_text, void *ctx) {
  memset(d, 0, sizeof *d);
  d->on_frame = on_frame;
  d->on_text = on_text;
  d->ctx = ctx;
}

void link_deframe_byte(link_deframer *d, uint8_t b) {
  if (b == LINK_FLAG) {
    if (d->in && d->n >= 4u) {
      if (link_crc16(d->buf, d->n) == 0u) {
        unsigned t = (unsigned)d->buf[0] | ((unsigned)d->buf[1] << 8);
        d->frames++;
        d->by_type[t & 15u]++;
        if (d->on_frame) d->on_frame(d->ctx, t, d->buf + 2, d->n - 4u, d->wire + 1u);
      } else {
        d->bad++;
      }
    }
    d->n = 0;
    d->wire = 1;
    d->esc = false;
    d->in = true;
    return;
  }
  if (!d->in) {
    d->text++;
    if (d->on_text) d->on_text(d->ctx, b);
    return;
  }
  d->wire++;
  if (b == LINK_ESC) {
    d->esc = true;
    return;
  }
  if (d->n < sizeof d->buf) d->buf[d->n++] = d->esc ? (uint8_t)(b ^ LINK_ESC_XOR) : b;
  else d->in = false;   /* runaway frame: treat what follows as text until the next flag */
  d->esc = false;
}

bool link_deframer_idle(const link_deframer *d) { return d->in && d->n == 0u && !d->esc; }

static size_t put_stuffed(uint8_t *out, size_t o, size_t cap, uint8_t b) {
  if (b == LINK_FLAG || b == LINK_ESC) {
    if (o + 2 > cap) return 0;
    out[o] = LINK_ESC;
    out[o + 1] = (uint8_t)(b ^ LINK_ESC_XOR);
    return o + 2;
  }
  if (o + 1 > cap) return 0;
  out[o] = b;
  return o + 1;
}

size_t link_encode(unsigned type, const uint8_t *payload, size_t len, uint8_t *out, size_t cap) {
  uint8_t body[LINK_BODY_MAX];
  if (len + 4u > sizeof body || cap < 2) return 0;
  body[0] = (uint8_t)(type & 0xFFu);
  body[1] = (uint8_t)(type >> 8);
  if (len) memcpy(body + 2, payload, len);
  uint16_t c = link_crc16(body, len + 2u);
  body[len + 2] = (uint8_t)(c & 0xFFu);
  body[len + 3] = (uint8_t)(c >> 8);
  size_t o = 0;
  out[o++] = LINK_FLAG;
  for (size_t i = 0; i < len + 4u; i++) {
    o = put_stuffed(out, o, cap, body[i]);
    if (!o) return 0;
  }
  if (o + 1 > cap) return 0;
  out[o++] = LINK_FLAG;
  return o;
}

static uint32_t u32le(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static float f32le(const uint8_t *p) {
  uint32_t u = u32le(p);
  float f;
  memcpy(&f, &u, sizeof f);
  return f;
}

bool link_decode_status(const uint8_t *p, unsigned len, link_status *s) {
  if (len != 48u) return false;
  s->fw_version = u32le(p + 0);
  s->rpm = (uint16_t)(p[4] | (p[5] << 8));
  s->inj_duty_pct = f32le(p + 8);
  s->word12 = f32le(p + 12);
  s->tps_pct = f32le(p + 16);
  s->fuel_trim_pct = (int32_t)u32le(p + 20);
  s->map_x100 = u32le(p + 24);
  s->word28 = u32le(p + 28);
  s->word32 = u32le(p + 32);
  s->word36 = u32le(p + 36);
  s->et_f = (int16_t)((uint16_t)p[40] | ((uint16_t)p[41] << 8));
  s->iat_f = (int16_t)((uint16_t)p[42] | ((uint16_t)p[43] << 8));
  s->inj_inhibit = p[44];
  s->ign_adv_deg = (int8_t)p[45];
  s->plead_inhibit = p[46];
  s->status_flags = p[47];
  return true;
}

bool link_decode_s32(const uint8_t *p, unsigned len, int32_t *v) {
  if (len < 4u) return false;
  *v = (int32_t)u32le(p);
  return true;
}

void link_trim_init(link_trim_tracker *k, double lag_s) {
  memset(k, 0, sizeof *k);
  k->lag_s = lag_s;
}

void link_trim_delta(link_trim_tracker *k, double t_end, int32_t delta) {
  if (k->npend == LINK_TRIM_MAX_PENDING) {   /* full: fold the oldest into the base (only after a long silence) */
    k->base += k->pend[0].d;
    memmove(k->pend, k->pend + 1, (k->npend - 1u) * sizeof k->pend[0]);
    k->npend--;
  }
  k->pend[k->npend].t = t_end;
  k->pend[k->npend].d = delta;
  k->npend++;
}

void link_trim_status(link_trim_tracker *k, double t_start, int32_t trim) {
  /* deltas that ended at least lag_s before this report began are inside it */
  unsigned keep = 0;
  int32_t inside = 0;
  for (unsigned i = 0; i < k->npend; i++) {
    if (k->pend[i].t <= t_start - k->lag_s) inside += k->pend[i].d;
    else k->pend[keep++] = k->pend[i];
  }
  if (k->have && k->base + inside != trim) k->mismatches++;
  k->npend = keep;
  k->base = trim;
  k->base_t = t_start;
  k->have = true;
  k->reports++;
}

int32_t link_trim_estimate(const link_trim_tracker *k) {
  int32_t v = k->base;
  for (unsigned i = 0; i < k->npend; i++) v += k->pend[i].d;
  return v;
}
