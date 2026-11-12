/* s32snap.c -- whole-system snapshots (see s32snap.h). */
#include "s32snap.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>

static const char MAGIC[8] = "S32SNAP";

/* ---------------------------------------------------------------- byte helpers */
void s32snap_b(s32_tbuf *b, const void *p, size_t n) { s32_tbuf_put(b, (const char *)p, n); }
void s32snap_u8(s32_tbuf *b, uint8_t v) { s32snap_b(b, &v, 1); }
void s32snap_u16(s32_tbuf *b, uint16_t v) { s32snap_b(b, &v, 2); }
void s32snap_u32(s32_tbuf *b, uint32_t v) { s32snap_b(b, &v, 4); }
void s32snap_u64(s32_tbuf *b, uint64_t v) { s32snap_b(b, &v, 8); }
void s32snap_f64(s32_tbuf *b, double v) { s32snap_b(b, &v, 8); }

/* ---------------------------------------------------------------- container */
void s32snap_w_init(s32snap_w *w) { memset(w, 0, sizeof *w); }

void s32snap_put(s32snap_w *w, const char *tag, const void *p, size_t n) {
  uint32_t tl = (uint32_t)strlen(tag);
  s32snap_u32(&w->buf, tl); s32snap_b(&w->buf, tag, tl); s32snap_u64(&w->buf, (uint64_t)n);
  if (n) s32snap_b(&w->buf, p, n);
}

int s32snap_w_save(s32snap_w *w, const char *path, uint32_t layout) {
  gzFile g = gzopen(path, "wb1");
  int rc = -1;
  if (g) {
    uint32_t ver = S32SNAP_VERSION;
    bool ok = gzwrite(g, MAGIC, sizeof MAGIC) == (int)sizeof MAGIC && gzwrite(g, &ver, 4) == 4 && gzwrite(g, &layout, 4) == 4;
    for (size_t off = 0; ok && off < w->buf.n;) {                 /* gzwrite takes an unsigned length */
      unsigned part = w->buf.n - off > (1u << 30) ? (1u << 30) : (unsigned)(w->buf.n - off);
      ok = gzwrite(g, w->buf.p + off, part) == (int)part;
      off += part;
    }
    rc = (gzclose(g) == Z_OK && ok) ? 0 : -1;
  }
  s32_tbuf_free(&w->buf);
  return rc;
}

int s32snap_r_load(s32snap_r *r, const char *path, uint32_t layout) {
  memset(r, 0, sizeof *r);
  gzFile g = gzopen(path, "rb");
  if (!g) { snprintf(r->err, sizeof r->err, "cannot open %s", path); return -1; }
  size_t cap = 1u << 24;
  r->data = malloc(cap);
  for (;;) {
    if (!r->data) { gzclose(g); snprintf(r->err, sizeof r->err, "out of memory"); return -1; }
    if (r->n == cap) { cap *= 2; uint8_t *p = realloc(r->data, cap); if (!p) { free(r->data); r->data = NULL; continue; } r->data = p; }
    int got = gzread(g, r->data + r->n, (unsigned)(cap - r->n > (1u << 30) ? (1u << 30) : cap - r->n));
    if (got < 0) { gzclose(g); snprintf(r->err, sizeof r->err, "%s: not a readable gzip stream", path); return -1; }
    if (got == 0) break;
    r->n += (size_t)got;
  }
  gzclose(g);
  uint32_t ver, lay;
  if (r->n < 16 || memcmp(r->data, MAGIC, sizeof MAGIC)) { snprintf(r->err, sizeof r->err, "%s is not a snapshot", path); return -1; }
  memcpy(&ver, r->data + 8, 4); memcpy(&lay, r->data + 12, 4);
  if (ver != S32SNAP_VERSION) { snprintf(r->err, sizeof r->err, "snapshot format %u, this build reads %u", ver, S32SNAP_VERSION); return -1; }
  if (lay != layout) { snprintf(r->err, sizeof r->err, "snapshot written by a build with another state layout (%08X, this build %08X)", lay, layout); return -1; }
  size_t off = 16, capc = 64;
  r->c = malloc(capc * sizeof *r->c);
  while (off < r->n) {
    uint32_t tl; uint64_t n;
    if (off + 4 > r->n) goto bad;
    memcpy(&tl, r->data + off, 4); off += 4;
    if (tl >= sizeof r->c[0].tag || off + tl + 8 > r->n) goto bad;
    if (r->nc == capc) { capc *= 2; s32snap_chunk *c = realloc(r->c, capc * sizeof *c); if (!c) goto bad; r->c = c; }
    s32snap_chunk *c = &r->c[r->nc++];
    memcpy(c->tag, r->data + off, tl); c->tag[tl] = '\0'; off += tl;
    memcpy(&n, r->data + off, 8); off += 8;
    if (n > r->n - off) goto bad;
    c->p = r->data + off; c->n = (size_t)n; off += (size_t)n;
  }
  return 0;
bad:
  snprintf(r->err, sizeof r->err, "%s: truncated or damaged snapshot", path);
  return -1;
}

const s32snap_chunk *s32snap_get(const s32snap_r *r, const char *tag) {
  for (size_t i = 0; i < r->nc; i++) if (!strcmp(r->c[i].tag, tag)) return &r->c[i];
  return NULL;
}

void s32snap_r_free(s32snap_r *r) { free(r->data); free(r->c); memset(r, 0, sizeof *r); }

/* ---------------------------------------------------------------- stimulus items */
void s32snap_item(s32_tbuf *items, uint8_t kind, uint16_t sub, double t, const void *p, size_t n) {
  s32snap_u8(items, kind); s32snap_u16(items, sub); s32snap_f64(items, t); s32snap_u32(items, (uint32_t)n);
  if (n) s32snap_b(items, p, n);
}

typedef struct { uint8_t kind; uint16_t sub; double t; const uint8_t *p; uint32_t n; } item;
static bool next_item(const uint8_t *p, size_t n, size_t *off, item *it) {
  if (*off + 15 > n) return false;
  const uint8_t *q = p + *off;
  it->kind = q[0]; memcpy(&it->sub, q + 1, 2); memcpy(&it->t, q + 3, 8); memcpy(&it->n, q + 11, 4);
  if (*off + 15 + it->n > n) return false;
  it->p = q + 15; *off += 15 + it->n;
  return true;
}
typedef struct { double home_t0, home_at, t_snap; } home_ctx;
static bool is_home(double t, double home_t0) { return home_t0 > 0 && t >= home_t0 / 2; }
static double when(double t, const home_ctx *h) {            /* the time an item acts, for "before t_future" */
  if (!is_home(t, h->home_t0)) return t;
  double x = t - h->home_t0;
  if (x < 0) return -INFINITY;                               /* "{H-x}": before the home page, always compared */
  return (h->home_at >= 0 ? h->home_at : h->t_snap) + x;
}
static bool next_before(const uint8_t *p, size_t n, size_t *off, double t_future, const home_ctx *h, item *it) {
  while (next_item(p, n, off, it)) if (when(it->t, h) < t_future) return true;
  return false;
}
static const char *KIND[] = {"event", "analogue fault", "pin fault", "crank fault", "link fault", "engine start", "statistics window"};

bool s32snap_items_equal_before(const uint8_t *a, size_t na, const uint8_t *b, size_t nb, double t_future, double home_t0,
                                double home_at, double t_snap, char *why, size_t nwhy) {
  size_t oa = 0, ob = 0;
  home_ctx h = {home_t0, home_at, t_snap};
  for (;;) {
    item x, y;
    bool hx = next_before(a, na, &oa, t_future, &h, &x), hy = next_before(b, nb, &ob, t_future, &h, &y);
    if (!hx && !hy) return true;
    if (hx != hy || x.kind != y.kind || x.sub != y.sub || x.n != y.n || memcmp(x.p, y.p, x.n)) {
      char sa[96] = "nothing", sb[96] = "nothing";
      const item *it[2] = {hx ? &x : NULL, hy ? &y : NULL};
      char *dst[2] = {sa, sb};
      for (int i = 0; i < 2; i++) if (it[i]) {
        const char *k = it[i]->kind < sizeof KIND / sizeof KIND[0] ? KIND[it[i]->kind] : "item";
        if (is_home(it[i]->t, home_t0)) snprintf(dst[i], 96, "%s at H%+.4f s", k, it[i]->t - home_t0);
        else snprintf(dst[i], 96, "%s at t=%.4f s", k, it[i]->t);
      }
      snprintf(why, nwhy, "the timed stimuli before t=%.4f s differ (snapshot: %s; this run: %s)", t_future, sa, sb);
      return false;
    }
  }
}

void s32snap_items_json(const uint8_t *p, size_t n, double home_t0, FILE *f) {
  size_t off = 0; item it; bool first = true;
  fputc('[', f);
  while (next_item(p, n, &off, &it)) {
    char hx[65];
    s32_sha256_hex(it.p, it.n, hx); hx[16] = '\0';
    if (is_home(it.t, home_t0))
      fprintf(f, "%s[%u,%u,%.17g,\"%s\",1]", first ? "" : ",", (unsigned)it.kind, (unsigned)it.sub, it.t - home_t0, hx);
    else fprintf(f, "%s[%u,%u,%.17g,\"%s\"]", first ? "" : ",", (unsigned)it.kind, (unsigned)it.sub, it.t, hx);
    first = false;
  }
  fputc(']', f);
}

/* ---------------------------------------------------------------- Controller */
void s32snap_ct_blob(const s32ct *ct, s32_tbuf *out) {
  s32ct *c = malloc(sizeof *c);
  if (!c) { fputs("s32snap: out of memory\n", stderr); abort(); }
  memcpy(c, ct, sizeof *c);
  bool ill = c->cpu.cfg.illegal_stops, dbg = c->cpu.cfg.debugger;
  memset(&c->cpu.cfg, 0, sizeof c->cpu.cfg);
  c->cpu.cfg.illegal_stops = ill; c->cpu.cfg.debugger = dbg;
  for (int i = 0; i < 3; i++) for (int j = 0; j < S32_MAX_SPI_DEVS; j++) memset(&c->spi[i].devs[j].dev, 0, sizeof c->spi[i].devs[j].dev);
  for (int i = 0; i < 4; i++) { c->uart[i].tx = NULL; c->uart[i].tx_ctx = NULL; }
  c->log = NULL; c->cov = NULL; c->gfx.log = NULL; c->lat_sink = NULL; c->lat_ctx = NULL;
  s32snap_b(out, c, sizeof *c);
  free(c);
}

int s32snap_ct_restore(s32ct *ct, const uint8_t *p, size_t n) {
  if (n != sizeof *ct) return -1;
  pic24_config cfg = ct->cpu.cfg;
  s32_spi_dev dev[3][S32_MAX_SPI_DEVS];
  void (*tx[4])(void *, unsigned, uint8_t); void *txc[4];
  for (int i = 0; i < 3; i++) for (int j = 0; j < S32_MAX_SPI_DEVS; j++) dev[i][j] = ct->spi[i].devs[j].dev;
  for (int i = 0; i < 4; i++) { tx[i] = ct->uart[i].tx; txc[i] = ct->uart[i].tx_ctx; }
  FILE *log = ct->log, *glog = ct->gfx.log; s32_iocov *cov = ct->cov;
  void (*ls)(void *, struct s32ct *, uint64_t, unsigned, uint16_t, uint16_t) = ct->lat_sink;
  void *lc = ct->lat_ctx;
  unsigned ndevs[3] = {ct->spi[0].ndevs, ct->spi[1].ndevs, ct->spi[2].ndevs};
  memcpy(ct, p, sizeof *ct);
  for (int i = 0; i < 3; i++) if (ct->spi[i].ndevs != ndevs[i]) return -2;   /* the devices on the board differ */
  ct->cpu.cfg = cfg;
  for (int i = 0; i < 3; i++) for (int j = 0; j < S32_MAX_SPI_DEVS; j++) ct->spi[i].devs[j].dev = dev[i][j];
  for (int i = 0; i < 4; i++) { ct->uart[i].tx = tx[i]; ct->uart[i].tx_ctx = txc[i]; }
  ct->log = log; ct->gfx.log = glog; ct->cov = cov; ct->lat_sink = ls; ct->lat_ctx = lc;
  return 0;
}

/* ---------------------------------------------------------------- RTC, SD card */
void s32snap_rtc_blob(const s32_rtc *r, s32_tbuf *out) {
  s32_rtc c = *r; c.cycles = NULL;
  s32snap_b(out, &c, sizeof c);
}
int s32snap_rtc_restore(s32_rtc *r, const uint8_t *p, size_t n) {
  if (n != sizeof *r) return -1;
  const uint64_t *cy = r->cycles;
  memcpy(r, p, sizeof *r); r->cycles = cy;
  return 0;
}

void s32snap_sd_blob(const s32_sd *c, s32_tbuf *out) {
  s32_sd x = *c; x.img = NULL;
  s32snap_b(out, &x, sizeof x);
}
int s32snap_sd_restore(s32_sd *c, const uint8_t *p, size_t n) {
  if (n != sizeof *c) return -1;
  uint8_t *img = c->img; uint32_t sectors = c->sectors;
  memcpy(c, p, sizeof *c);
  if (c->sectors != sectors) return -2;
  c->img = img;
  return 0;
}
void s32snap_sd_diff(const s32_sd *c, const uint8_t *orig, s32_tbuf *out) {
  for (uint32_t s = 0; s < c->sectors; s++)
    if (memcmp(c->img + (size_t)s * 512u, orig + (size_t)s * 512u, 512)) { s32snap_u32(out, s); s32snap_b(out, c->img + (size_t)s * 512u, 512); }
}
int s32snap_sd_apply(s32_sd *c, const uint8_t *p, size_t n) {
  if (n % 516) return -1;
  for (size_t off = 0; off < n; off += 516) {
    uint32_t s; memcpy(&s, p + off, 4);
    if (s >= c->sectors) return -1;
    memcpy(c->img + (size_t)s * 512u, p + off + 4, 512);
  }
  return 0;
}

/* ---------------------------------------------------------------- trace */
void s32snap_trace_blob(const s32trace *t, s32_tbuf *out) {
  s32snap_b(out, &t->sha, sizeof t->sha);
  s32snap_u64(out, t->lines); s32snap_u64(out, t->bytes); s32snap_u64(out, t->fb_frames);
  s32snap_u8(out, t->ct_on); s32snap_b(out, t->panel_last, sizeof t->panel_last); s32snap_u64(out, (uint64_t)t->q);
}
int s32snap_trace_restore(s32trace *t, const uint8_t *p, size_t n) {
  if (n != sizeof t->sha + 3 * 8 + 1 + sizeof t->panel_last + 8) return -1;
  for (int w = 0; w < 3; w++) t->src[w].n = 0;                     /* anything the start-up path produced is replaced */
  t->bar.n = 0; t->out.n = 0;
  memcpy(&t->sha, p, sizeof t->sha); p += sizeof t->sha;
  memcpy(&t->lines, p, 8); memcpy(&t->bytes, p + 8, 8); memcpy(&t->fb_frames, p + 16, 8); p += 24;
  t->ct_on = *p++ != 0;
  memcpy(t->panel_last, p, sizeof t->panel_last); p += sizeof t->panel_last;
  uint64_t q; memcpy(&q, p, 8); t->q = (long)q;
  return 0;
}
