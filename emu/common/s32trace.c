/* s32trace.c -- the canonical conformance trace (see s32trace.h). */
#include "s32trace.h"
#include "s32board.h"
#include "s32panel.h"
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

/* ---------------------------------------------------------------- text buffer */
static void tbuf_reserve(s32_tbuf *b, size_t extra) {
  if (b->n + extra + 1 <= b->cap) return;
  size_t cap = b->cap ? b->cap : 4096;
  while (cap < b->n + extra + 1) cap *= 2;
  char *p = realloc(b->p, cap);
  if (!p) { fputs("s32trace: out of memory\n", stderr); abort(); }
  b->p = p; b->cap = cap;
}
void s32_tbuf_put(s32_tbuf *b, const char *s, size_t n) {
  tbuf_reserve(b, n);
  memcpy(b->p + b->n, s, n);
  b->n += n; b->p[b->n] = '\0';
}
void s32_tbuf_printf(s32_tbuf *b, const char *fmt, ...) {
  char tmp[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(tmp, sizeof tmp, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if ((size_t)n < sizeof tmp) { s32_tbuf_put(b, tmp, (size_t)n); return; }
  tbuf_reserve(b, (size_t)n);
  va_start(ap, fmt);
  vsnprintf(b->p + b->n, (size_t)n + 1, fmt, ap);
  va_end(ap);
  b->n += (size_t)n;
}
void s32_tbuf_free(s32_tbuf *b) { free(b->p); b->p = NULL; b->n = b->cap = 0; }

/* ---------------------------------------------------------------- SHA-256 (FIPS 180-4) */
static const uint32_t K256[64] = {
  0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01,
  0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
  0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
  0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
  0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08,
  0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
  0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
#define ROR(x, n) (((x) >> (n)) | ((x) << (32 - (n))))
static void sha_block(s32_sha256 *s, const uint8_t *p) {
  uint32_t w[64];
  for (int i = 0; i < 16; i++) w[i] = (uint32_t)p[4 * i] << 24 | (uint32_t)p[4 * i + 1] << 16 | (uint32_t)p[4 * i + 2] << 8 | p[4 * i + 3];
  for (int i = 16; i < 64; i++) {
    uint32_t s0 = ROR(w[i - 15], 7) ^ ROR(w[i - 15], 18) ^ (w[i - 15] >> 3);
    uint32_t s1 = ROR(w[i - 2], 17) ^ ROR(w[i - 2], 19) ^ (w[i - 2] >> 10);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }
  uint32_t a = s->h[0], b = s->h[1], c = s->h[2], d = s->h[3], e = s->h[4], f = s->h[5], g = s->h[6], h = s->h[7];
  for (int i = 0; i < 64; i++) {
    uint32_t t1 = h + (ROR(e, 6) ^ ROR(e, 11) ^ ROR(e, 25)) + ((e & f) ^ (~e & g)) + K256[i] + w[i];
    uint32_t t2 = (ROR(a, 2) ^ ROR(a, 13) ^ ROR(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
    h = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
  }
  s->h[0] += a; s->h[1] += b; s->h[2] += c; s->h[3] += d; s->h[4] += e; s->h[5] += f; s->h[6] += g; s->h[7] += h;
}
void s32_sha256_init(s32_sha256 *s) {
  static const uint32_t H0[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  memcpy(s->h, H0, sizeof H0); s->len = 0; s->nblk = 0;
}
void s32_sha256_update(s32_sha256 *s, const void *pv, size_t n) {
  const uint8_t *p = pv;
  s->len += n;
  if (s->nblk) {
    size_t take = 64 - s->nblk < n ? 64 - s->nblk : n;
    memcpy(s->blk + s->nblk, p, take); s->nblk += (unsigned)take; p += take; n -= take;
    if (s->nblk == 64) { sha_block(s, s->blk); s->nblk = 0; }
  }
  for (; n >= 64; p += 64, n -= 64) sha_block(s, p);
  if (n) { memcpy(s->blk, p, n); s->nblk = (unsigned)n; }
}
void s32_sha256_final(s32_sha256 *s, uint8_t out[32]) {
  uint64_t bits = s->len * 8;
  uint8_t pad = 0x80, z = 0;
  s32_sha256_update(s, &pad, 1);
  while (s->nblk != 56) s32_sha256_update(s, &z, 1);
  uint8_t l[8];
  for (int i = 0; i < 8; i++) l[i] = (uint8_t)(bits >> (56 - 8 * i));
  s32_sha256_update(s, l, 8);
  for (int i = 0; i < 8; i++) { out[4 * i] = (uint8_t)(s->h[i] >> 24); out[4 * i + 1] = (uint8_t)(s->h[i] >> 16); out[4 * i + 2] = (uint8_t)(s->h[i] >> 8); out[4 * i + 3] = (uint8_t)s->h[i]; }
}
static void hex32(const uint8_t d[32], char out[65]) {
  static const char X[] = "0123456789abcdef";
  for (int i = 0; i < 32; i++) { out[2 * i] = X[d[i] >> 4]; out[2 * i + 1] = X[d[i] & 15]; }
  out[64] = '\0';
}
void s32_sha256_hex(const void *p, size_t n, char out[65]) {
  s32_sha256 s; uint8_t d[32];
  s32_sha256_init(&s); s32_sha256_update(&s, p, n); s32_sha256_final(&s, d); hex32(d, out);
}

/* ---------------------------------------------------------------- trace */
static const char *const WHO[3] = {"CT", "E1", "E2"};

static void emit(s32trace *t, const char *s, size_t n) {
  if (!n) return;
  s32_sha256_update(&t->sha, s, n);
  for (size_t i = 0; i < n; i++) t->lines += s[i] == '\n';
  t->bytes += n;
  s32_tbuf_put(&t->out, s, n);
  if (t->f && t->out.n >= (1u << 16)) s32trace_flush(t);
}

void s32trace_open(s32trace *t, FILE *f) {
  memset(t, 0, sizeof *t);
  t->on = true; t->f = f;
  s32_sha256_init(&t->sha);
  char h[160];
  int n = snprintf(h, sizeof h, "s32-trace %d quantum_us 100 clocks CT=Fcy E1,E2=SYSCLK (cycles, crystal error applied)\n",
                   S32TRACE_VERSION);
  emit(t, h, (size_t)n);
}

void s32trace_pin(s32trace *t, int who, uint64_t cyc, unsigned port, uint16_t old_pins, uint16_t new_pins) {
  if (!t || !t->on || who < 0 || who > 2) return;
  s32_tbuf_printf(&t->src[who], "%s %llu pin R%c %04X %04X\n", WHO[who], (unsigned long long)cyc, 'A' + (int)port,
                  (unsigned)old_pins, (unsigned)new_pins);
}
void s32trace_coil(s32trace *t, int who, unsigned oc, uint64_t on_cyc, uint64_t off_cyc) {
  if (!t || !t->on || who < 0 || who > 2) return;
  s32_tbuf_printf(&t->src[who], "%s %llu coil OC%u on %llu\n", WHO[who], (unsigned long long)off_cyc, oc,
                  (unsigned long long)on_cyc);
}
void s32trace_tx(s32trace *t, int who, uint64_t cyc, unsigned uart, uint8_t byte) {
  if (!t || !t->on || who < 0 || who > 2) return;
  s32_tbuf_printf(&t->src[who], "%s %llu tx U%u %02X\n", WHO[who], (unsigned long long)cyc, uart, (unsigned)byte);
}
void s32trace_link(s32trace *t, long q, int k, bool to_ecu, const uint8_t *b, unsigned n) {
  if (!t || !t->on || !n || k < 0 || k > 1) return;
  static const char X[] = "0123456789ABCDEF";
  s32_tbuf_printf(&t->bar, "q %ld deliver %s %u ", q, to_ecu ? (k ? "CT>E2" : "CT>E1") : (k ? "E2>CT" : "E1>CT"), n);
  char hx[2 * 64];
  for (unsigned i = 0; i < n;) {
    unsigned m = 0;
    for (; m < 64 && i < n; m++, i++) { hx[2 * m] = X[b[i] >> 4]; hx[2 * m + 1] = X[b[i] & 15]; }
    s32_tbuf_put(&t->bar, hx, 2 * m);
  }
  s32_tbuf_put(&t->bar, "\n", 1);
}

/* The picture on the panel (emu/common/s32panel.c, the renderer every front end uses) and the panel's state. */
static void panel_line(s32trace *t, long q, s32ct *ct, bool ct_on) {
  static uint8_t rgb[S32_PANEL_W * S32_PANEL_H * 3];
  s32_panel_state st;
  s32_panel_render(ct, ct_on, rgb, &st);
  uint8_t d[32]; char hx[65], line[200];
  s32_sha256 s; s32_sha256_init(&s); s32_sha256_update(&s, rgb, sizeof rgb); s32_sha256_final(&s, d); hex32(d, hx);
  snprintf(line, sizeof line, "%.16s bl %u %s%s G1CON1 %04X G1CON2 %04X G1CON3 %04X", hx, (unsigned)st.backlight_permille,
           st.lit ? "lit" : "dark:", st.lit ? "" : st.why, st.con1, st.con2, st.con3);
  if (strcmp(line, t->panel_last)) {
    snprintf(t->panel_last, sizeof t->panel_last, "%s", line);
    s32_tbuf_printf(&t->bar, "q %ld panel %llu %s\n", q, (unsigned long long)ct->gfx.frames, line);
  }
}

void s32trace_quantum(s32trace *t, long q, struct s32ct *ct, bool ct_on) {
  if (!t || !t->on) return;
  t->q = q;
  if (ct && (ct->gfx.frames != t->fb_frames || ct_on != t->ct_on || q == 1)) {   /* after each vblank, power change */
    t->fb_frames = ct->gfx.frames; t->ct_on = ct_on;
    panel_line(t, q, ct, ct_on);
  }
  for (int w = 0; w < 3; w++) { emit(t, t->src[w].p, t->src[w].n); t->src[w].n = 0; }
  emit(t, t->bar.p, t->bar.n); t->bar.n = 0;
}

void s32trace_finish(s32trace *t, const struct s32board *b, const uint8_t *const pflash[2], const size_t npflash[2],
                     char sha_hex[65]) {
  if (!t || !t->on) return;
  for (int w = 0; w < 3; w++) { emit(t, t->src[w].p, t->src[w].n); t->src[w].n = 0; }
  emit(t, t->bar.p, t->bar.n); t->bar.n = 0;
  char hx[65], line[200];
  int n;
  for (int w = 0; w < SB_WHO; w++) {
    if (b->fram[w]) { s32_sha256_hex(b->fram[w]->mem, sizeof b->fram[w]->mem, hx); n = snprintf(line, sizeof line, "end fram %s %s\n", WHO[w], hx); }
    else n = snprintf(line, sizeof line, "end fram %s none\n", WHO[w]);
    emit(t, line, (size_t)n);
  }
  for (int w = 0; w < SB_WHO; w++) {
    if (b->sd[w] && b->sd[w]->img) {
      s32_sha256_hex(b->sd[w]->img, (size_t)b->sd[w]->sectors * 512u, hx);
      n = snprintf(line, sizeof line, "end sd %s %s\n", WHO[w], hx);
    } else n = snprintf(line, sizeof line, "end sd %s none\n", WHO[w]);
    emit(t, line, (size_t)n);
  }
  for (int k = 0; k < 2; k++) {
    if (pflash && pflash[k]) { s32_sha256_hex(pflash[k], npflash[k], hx); n = snprintf(line, sizeof line, "end pflash E%d %s\n", k + 1, hx); }
    else n = snprintf(line, sizeof line, "end pflash E%d none\n", k + 1);
    emit(t, line, (size_t)n);
  }
  n = snprintf(line, sizeof line, "end cycles CT %llu E1 %llu E2 %llu\n", b->ct ? (unsigned long long)b->ct->cpu.cycles : 0ull,
               b->ecu[0] ? (unsigned long long)b->ecu[0]->cyc : 0ull, b->ecu[1] ? (unsigned long long)b->ecu[1]->cyc : 0ull);
  emit(t, line, (size_t)n);
  s32trace_flush(t);
  if (sha_hex) {
    s32_sha256 s = t->sha; uint8_t d[32];
    s32_sha256_final(&s, d); hex32(d, sha_hex);
  }
}

void s32trace_flush(s32trace *t) {
  if (!t || !t->f || !t->out.n) return;
  fwrite(t->out.p, 1, t->out.n, t->f);
  t->out.n = 0;
}

void s32trace_free(s32trace *t) {
  if (!t) return;
  for (int w = 0; w < 3; w++) s32_tbuf_free(&t->src[w]);
  s32_tbuf_free(&t->bar); s32_tbuf_free(&t->out);
  t->on = false;
}
