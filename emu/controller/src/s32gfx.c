/* s32gfx.c -- graphics controller model (see s32gfx.h). Pixel formats: 1/2/4/8 bpp packed little-endian within a byte
 * (pixel 0 in the low bits), CLUT entries RGB565. Behaviour per DS39969 section 9 / FRM "Graphics Controller Module"
 * and the command use measured on the bench; anything not yet seen is counted in unknown_cmds, not guessed. */
#include "s32gfx.h"
#include <string.h>
#include <zlib.h>

enum { R_CMDL = 0x00, R_CMDH = 0x01, R_CON1 = 0x02, R_STAT = 0x03, R_W1L = 0x06, R_W1H = 0x07, R_W2L = 0x08, R_W2H = 0x09,
       R_PUW = 0x0A, R_PUH = 0x0B, R_DPL = 0x0C, R_DPH_ = 0x0D, R_DPW = 0x0E, R_DPH = 0x0F, R_CON2 = 0x12,
       R_CLUT = 0x18, R_CLUTWR = 0x19, R_CLUTRD = 0x1A, R_CHRX = 0x1C, R_CHRY = 0x1D, R_IPU = 0x1E };

void s32_gfx_init(s32_gfx *g) {
  FILE *log = g->log; unsigned b = g->log_budget;
  memset(g, 0, sizeof *g);
  g->log = log; g->log_budget = b;
}

uint8_t s32_eds_byte(pic24_cpu *cpu, uint32_t a) {
  a &= 0xFFFFFFu;
  if (a < PIC24_DATA_LOW_TOP) return (uint8_t)(cpu->ram[a >> 1] >> ((a & 1u) * 8u));
  if (a >= PIC24_EDS_RAM_BASE && a < PIC24_EDS_RAM_TOP)
    return (uint8_t)(cpu->eds[(a - PIC24_EDS_RAM_BASE) >> 1] >> ((a & 1u) * 8u));
  return 0;
}

static void eds_put(pic24_cpu *cpu, uint32_t a, uint8_t v) {
  a &= 0xFFFFFFu;
  uint16_t *w = NULL;
  if (a < PIC24_DATA_LOW_TOP) w = &cpu->ram[a >> 1];
  else if (a >= PIC24_EDS_RAM_BASE && a < PIC24_EDS_RAM_TOP) w = &cpu->eds[(a - PIC24_EDS_RAM_BASE) >> 1];
  if (!w) return;
  unsigned sh = (a & 1u) * 8u;
  *w = (uint16_t)((*w & ~(0xFFu << sh)) | ((unsigned)v << sh));
}

static unsigned bpp_of(unsigned code) { return 1u << (code & 3u); }            /* 0:1 1:2 2:4 3:8 */
static unsigned pu_bpp(const s32_gfx *g) { return bpp_of((g->reg[R_CON1] >> 5) & 7u); }   /* G1CON1.PUBPP<7:5> */
static uint32_t w1(const s32_gfx *g) { return g->reg[R_W1L] | ((uint32_t)(g->reg[R_W1H] & 0xFF) << 16); }
static uint32_t w2(const s32_gfx *g) { return g->reg[R_W2L] | ((uint32_t)(g->reg[R_W2H] & 0xFF) << 16); }

/* Pixel n (0-based, linear) of a packed buffer at EDS base `b`. */
static unsigned px_get(pic24_cpu *cpu, uint32_t b, uint32_t n, unsigned bpp) {
  uint32_t bit = n * bpp;
  return (s32_eds_byte(cpu, b + (bit >> 3)) >> (bit & 7u)) & ((1u << bpp) - 1u);
}
static void px_put(pic24_cpu *cpu, uint32_t b, uint32_t n, unsigned bpp, unsigned v) {
  uint32_t bit = n * bpp, a = b + (bit >> 3);
  unsigned sh = bit & 7u, m = ((1u << bpp) - 1u) << sh;
  uint8_t old = s32_eds_byte(cpu, a);
  eds_put(cpu, a, (uint8_t)((old & ~m) | ((v << sh) & m)));
}

/* RCC: rectangle w x h at pixel offset rcc_dst in the work area W2 (stride G1PUW). START operand: bit 1 src
 * continuous, bit 2 dst continuous, bits 6:3 ROP, bits 9:7 type (0 solid fill, 1 copy, 6 transparent copy). */
static void rcc_start(s32_gfx *g, pic24_cpu *cpu, uint32_t op) {
  unsigned bpp = pu_bpp(g), stride = g->reg[R_PUW] ? g->reg[R_PUW] : 320u;
  unsigned rop = (op >> 3) & 0xFu, type = (op >> 7) & 7u;
  bool sc = op & 2u, dc = op & 4u;
  bool need_d = !(rop == 0x0u || rop == 0x3u || rop == 0xCu || rop == 0xFu);
  for (uint32_t y = 0; y < g->rcc_h; y++)
    for (uint32_t x = 0; x < g->rcc_w; x++) {
      uint32_t dn = dc ? g->rcc_dst + y * g->rcc_w + x : g->rcc_dst + y * stride + x;
      unsigned s = g->rcc_color, d = 0, v;
      if (type != 0) {
        uint32_t sn = sc ? g->rcc_src + y * g->rcc_w + x : g->rcc_src + y * stride + x;
        s = px_get(cpu, w1(g), sn, bpp);
        if (type == 6 && s == (g->rcc_color & ((1u << bpp) - 1u))) continue;   /* transparent colour skipped */
      }
      if (need_d) d = px_get(cpu, w2(g), dn, bpp);
      switch (rop) {               /* 4-bit ROP over (S, D): bit n of rop = result for (S,D) = n */
      case 0x0: v = 0; break;
      case 0xC: v = s; break;
      case 0xA: v = d; break;
      case 0x5: v = ~d; break;
      case 0x3: v = ~s; break;
      case 0x6: v = s ^ d; break;
      case 0x8: v = s & d; break;
      case 0xE: v = s | d; break;
      case 0xF: v = 0xFFu; break;
      default: v = s; g->unknown_cmds++; break;
      }
      px_put(cpu, w2(g), dn, bpp, v);
    }
}

/* CHR: the character engine's font table (FRM "Graphics Controller Module", character printing): an 8-byte header
 * (bytes 0..1 not used here; u16 first character @+2; u16 last character @+4; u16 height in rows @+6), then one 4-byte
 * entry per character {u8 width, u24 offset of its glyph from the font base}; glyph rows are ceil(width/8) bytes,
 * LSB = leftmost pixel. */
static void chr_print(s32_gfx *g, pic24_cpu *cpu, uint32_t op) {
  uint8_t ch = (uint8_t)op;
  bool transparent = (op >> 23) & 1u;
  uint32_t f = g->chr_font;
  unsigned first = s32_eds_byte(cpu, f + 2) | (s32_eds_byte(cpu, f + 3) << 8);
  unsigned last = s32_eds_byte(cpu, f + 4) | (s32_eds_byte(cpu, f + 5) << 8);
  unsigned h = s32_eds_byte(cpu, f + 6) | (s32_eds_byte(cpu, f + 7) << 8);
  if (ch < first || ch > last || h == 0 || h > 64) { g->unknown_cmds++; return; }
  uint32_t e = f + 8 + 4u * (ch - first);
  unsigned w = s32_eds_byte(cpu, e);
  uint32_t off = s32_eds_byte(cpu, e + 1) | (s32_eds_byte(cpu, e + 2) << 8) | ((uint32_t)s32_eds_byte(cpu, e + 3) << 16);
  unsigned bpp = pu_bpp(g), stride = g->reg[R_PUW] ? g->reg[R_PUW] : 320u, rowbytes = (w + 7u) / 8u;
  for (unsigned y = 0; y < h; y++)
    for (unsigned x = 0; x < w; x++) {
      unsigned bit = (s32_eds_byte(cpu, f + off + y * rowbytes + x / 8u) >> (x & 7u)) & 1u;
      if (!bit && transparent) continue;
      uint32_t px = g->chr_x + x, py = g->chr_y + y;
      if (px > g->chr_ax1 || py > g->chr_ay1 || px < g->chr_ax0 || py < g->chr_ay0) continue;
      px_put(cpu, w2(g), py * stride + px, bpp, bit ? g->chr_fg : g->chr_bg);
    }
  g->chr_x += w;
  g->reg[R_CHRX] = (uint16_t)g->chr_x;
  g->reg[R_CHRY] = (uint16_t)g->chr_y;
}

/* IPU: inflate a DEFLATE stream at W1+src into `len` bytes at W2+dst (internal RAM only). G1IPU: bit0 BFINAL,
 * bit1 IPUDONE, bit3 LENERR, bit4 BLCKERR, bit5 HUFFERR. Raw DEFLATE first (RFC 1951), then zlib-wrapped. */
static void ipu_decompress(s32_gfx *g, pic24_cpu *cpu, uint32_t len) {
  static uint8_t in[0x20000], out[0x20000];
  uint32_t src = w1(g) + g->ipu_src, dst = w2(g) + g->ipu_dst;
  uint32_t n = src < PIC24_EDS_RAM_TOP ? PIC24_EDS_RAM_TOP - src : 0;
  uint32_t max_in = len + 4096u;
  if (max_in > sizeof in) max_in = sizeof in;
  if (n > max_in) n = max_in;
  if (len > sizeof out) len = sizeof out;
  for (uint32_t i = 0; i < n; i++) in[i] = s32_eds_byte(cpu, src + i);
  int ok = 0;
  for (int wbits = -15; wbits <= 15 && !ok; wbits += 30) {
    z_stream z; memset(&z, 0, sizeof z);
    if (inflateInit2(&z, wbits) != Z_OK) continue;
    z.next_in = in; z.avail_in = n; z.next_out = out; z.avail_out = len;
    int r = inflate(&z, Z_FINISH);
    if ((r == Z_STREAM_END || r == Z_BUF_ERROR || r == Z_OK) && z.total_out == len) ok = 1;
    inflateEnd(&z);
  }
  if (ok) { for (uint32_t i = 0; i < len; i++) eds_put(cpu, dst + i, out[i]); g->reg[R_IPU] = 0x0003u; g->ipu_ok++; }
  else { g->reg[R_IPU] = 0x0010u; g->unknown_cmds++; }
}

static void execute(s32_gfx *g, pic24_cpu *cpu, uint32_t cmd) {
  unsigned opc = (cmd >> 24) & 0xFFu;
  uint32_t op = cmd & 0xFFFFFFu;
  g->cmd_count[opc]++;
  if (g->log && g->log_budget) {
    g->log_budget--;
    fprintf(g->log, "%llu GFX %02X %06X  W1=%06X W2=%06X\n", (unsigned long long)cpu->cycles, opc, (unsigned)op, (unsigned)w1(g), (unsigned)w2(g));
  }
  switch (opc) {
  case 0x62: g->rcc_src = op; break;
  case 0x63: g->rcc_dst = op; break;
  case 0x64: g->rcc_w = (op >> 12) & 0xFFFu; g->rcc_h = op & 0xFFFu; break;
  case 0x66: g->rcc_color = op; break;
  case 0x67: rcc_start(g, cpu, op); break;
  case 0x50: g->chr_fg = op; break;
  case 0x51: g->chr_bg = op; break;
  case 0x52: g->chr_font = op; break;
  case 0x53: chr_print(g, cpu, op); break;
  case 0x58: g->chr_ax0 = op & 0xFFFu; g->chr_ay0 = (op >> 12) & 0xFFFu; break;
  case 0x59: g->chr_ax1 = op & 0xFFFu; g->chr_ay1 = (op >> 12) & 0xFFFu; break;
  case 0x5A: g->chr_x = op & 0xFFFu; g->chr_y = (op >> 12) & 0xFFFu; g->reg[R_CHRX] = (uint16_t)g->chr_x; g->reg[R_CHRY] = (uint16_t)g->chr_y; break;
  case 0x71: g->ipu_src = op; break;
  case 0x72: g->ipu_dst = op; break;
  case 0x73: g->ipu_len = op; break;
  case 0x74: ipu_decompress(g, cpu, op); break;
  default: g->unknown_cmds++; break;
  }
}

uint16_t s32_gfx_read(s32_gfx *g, pic24_cpu *cpu, uint16_t addr) {
  (void)cpu;
  unsigned r = (addr - 0x0700u) >> 1;
  if (r == R_STAT) return 0x0001u;                   /* CMDMPT: FIFO empty; every engine idle */
  if (r == R_CLUTRD) return g->clut[g->reg[R_CLUT] & 0xFFu];
  if (r == R_CLUT) return (uint16_t)(g->reg[R_CLUT] & ~0x4000u);    /* CLUTBUSY never set: writes are immediate */
  if (r == R_IPU) return g->reg[R_IPU];
  return r < 0x20 ? g->reg[r] : 0;
}

void s32_gfx_write(s32_gfx *g, pic24_cpu *cpu, uint16_t addr, uint16_t v, uint16_t m) {
  unsigned r = (addr - 0x0700u) >> 1;
  if (r >= 0x20) return;
  g->reg[r] = (uint16_t)((g->reg[r] & ~m) | (v & m));
  if (r == R_CMDH) execute(g, cpu, ((uint32_t)g->reg[R_CMDH] << 16) | g->reg[R_CMDL]);
  else if (r == R_CLUTWR && (g->reg[R_CLUT] & 0x0100u))           /* CLUTRWEN (bit 8): write the entry at CLUTADR */
    g->clut[g->reg[R_CLUT] & 0xFFu] = g->reg[R_CLUTWR];
}

/* One display frame ended: set VMRGNIF; GFX1IF (IRQ 100) follows when VMRGNIE is enabled. */
void s32_gfx_vblank(s32_gfx *g, pic24_cpu *cpu) {
  g->frames++;
  g->reg[0x05] |= 0x0010u;                                          /* G1IR.VMRGNIF */
  if (g->reg[0x04] & g->reg[0x05] & 0x8010u) pic24_set_irq(cpu, 100, true);
}

/* Tcy per frame from G1DPWT x G1DPHT and the display clock, 96 MHz / (1 + GCLKDIV / 4) (DS39969, CLKDIV2): the stock
 * firmware programs 370 x 246 px at 5.907 MHz (96 MHz / 16.25), 15.4 ms per frame (OBSERVED); registers not yet
 * programmed -> 0 (no frames). */
uint64_t s32_gfx_frame_tcy(const s32_gfx *g) {
  unsigned wt = g->reg[0x10], ht = g->reg[0x11];
  if (!wt || !ht) return 0;
  return (uint64_t)((double)wt * ht / 5.907e6 * 16e6);
}

int s32_gfx_dump_ppm(const s32_gfx *g, pic24_cpu *cpu, const char *path) {
  unsigned w = g->reg[R_DPW], h = g->reg[R_DPH];
  unsigned bpp = bpp_of((g->reg[R_CON2] >> 5) & 7u);             /* G1CON2.DPBPP<7:5> */
  uint32_t base = g->reg[R_DPL] | ((uint32_t)(g->reg[R_DPH_] & 0xFF) << 16);
  if (!w || !h || w > 1024 || h > 1024) return -1;
  FILE *f = fopen(path, "wb");
  if (!f) return -1;
  fprintf(f, "P6\n%u %u\n255\n", w, h);
  for (unsigned y = 0; y < h; y++)
    for (unsigned x = 0; x < w; x++) {
      unsigned idx = px_get(cpu, base, y * w + x, bpp);
      uint16_t c = bpp >= 16 ? 0 : g->clut[idx];
      uint8_t rgb[3] = {(uint8_t)(((c >> 11) & 31u) * 255u / 31u), (uint8_t)(((c >> 5) & 63u) * 255u / 63u),
                        (uint8_t)((c & 31u) * 255u / 31u)};
      fwrite(rgb, 1, 3, f);
    }
  fclose(f);
  return 0;
}
