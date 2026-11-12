/* Unit tests for the G1 graphics controller model, driven through the SFRs like the firmware does. */
#include "minitest.h"
#include "s32ct.h"
#include <string.h>
#include <zlib.h>

static s32ct S;
static const uint8_t NOPROG[3] = {0, 0, 0};
static void cmd(uint32_t c) { pic24_poke16(&S.cpu, 0x0700, (uint16_t)c); pic24_poke16(&S.cpu, 0x0702, (uint16_t)(c >> 16)); }

static void setup(void) {
  s32ct_init(&S, NOPROG, sizeof NOPROG);
  pic24_poke16(&S.cpu, 0x0704, 0x8040);          /* G1CON1: G1EN, PUBPP = 2 (4 bpp) */
  pic24_poke16(&S.cpu, 0x0724, 0x0041);          /* G1CON2: DPMODE 1, DPBPP = 2 (4 bpp) */
  pic24_poke16(&S.cpu, 0x0710, 0xD000);          /* G1W2ADR = EDS 0xD000 (a frame buffer address) */
  pic24_poke16(&S.cpu, 0x0718, 0xD000);          /* G1DPADR */
  pic24_poke16(&S.cpu, 0x0714, 320); pic24_poke16(&S.cpu, 0x0716, 240);
  pic24_poke16(&S.cpu, 0x071C, 320); pic24_poke16(&S.cpu, 0x071E, 240);
}

TEST(rcc_solid_fill_writes_4bpp_pixels_in_the_rectangle_only) {
  setup();
  cmd(0x66000007u);                               /* RCC_COLOR 7 */
  cmd(0x63000000u | (10u * 320u + 4u));           /* RCC_DESTADDR (x 4, y 10) */
  cmd(0x64000000u | (3u << 12) | 2u);             /* 3 x 2 */
  cmd(0x67000060u);                               /* START: solid fill, ROP C */
  uint32_t row = 0xD000u + (10u * 320u) / 2u;
  CHECK_EQ(s32_eds_byte(&S.cpu, row + 2), 0x77);  /* pixels 4,5 */
  CHECK_EQ(s32_eds_byte(&S.cpu, row + 3), 0x07);  /* pixel 6 set, pixel 7 untouched */
  CHECK_EQ(s32_eds_byte(&S.cpu, row + 1), 0x00);
  CHECK_EQ(s32_eds_byte(&S.cpu, row + 160 + 2), 0x77);   /* second row */
  CHECK_EQ(s32_eds_byte(&S.cpu, row + 320 + 2), 0x00);   /* no third row */
  CHECK_EQ(pic24_peek16(&S.cpu, 0x0706) & 1, 1);  /* G1STAT: FIFO empty */
}

TEST(clut_write_needs_clutrwen_bit8) {
  setup();
  pic24_poke16(&S.cpu, 0x0730, 0x0003);           /* CLUTADR 3, RWEN off */
  pic24_poke16(&S.cpu, 0x0732, 0x1234);
  CHECK_EQ(S.gfx.clut[3], 0);
  pic24_poke16(&S.cpu, 0x0730, 0x0103);           /* RWEN */
  pic24_poke16(&S.cpu, 0x0732, 0xF800);
  CHECK_EQ(S.gfx.clut[3], 0xF800);
  CHECK_EQ(pic24_peek16(&S.cpu, 0x0730) & 0x4000, 0);   /* never busy */
}

TEST(vblank_sets_vmrgnif_and_raises_irq100_only_when_enabled) {
  setup();
  s32_gfx_vblank(&S.gfx, &S.cpu);
  CHECK_EQ(pic24_peek16(&S.cpu, 0x070A) & 0x10, 0x10);
  CHECK_EQ(pic24_peek16(&S.cpu, 0x0090) & 0x10, 0);     /* IFS6.GFX1IF not set: VMRGNIE off */
  pic24_poke16(&S.cpu, 0x0708, 0x0010);
  s32_gfx_vblank(&S.gfx, &S.cpu);
  CHECK_EQ(pic24_peek16(&S.cpu, 0x0090) & 0x10, 0x10);
}

static void put_bytes(uint32_t a, const uint8_t *b, unsigned n) {
  for (unsigned i = 0; i < n; i++) {
    uint32_t x = a + i; uint16_t *w = x < 0x8000u ? &S.cpu.ram[x >> 1] : &S.cpu.eds[(x - 0x8000u) >> 1];
    unsigned sh = (x & 1u) * 8u; *w = (uint16_t)((*w & ~(0xFFu << sh)) | ((unsigned)b[i] << sh));
  }
}

TEST(rcc_transparent_copy_skips_the_key_colour) {
  /* G1 RCC (DS39969 section 22, graphics controller): a transparent copy (type 6) copies every source pixel except
   * those equal to RCC_COLOR (the key); skipped pixels leave the destination as it was. */
  setup();
  pic24_poke16(&S.cpu, 0x070C, 0xD800);           /* G1W1ADR: the copy's source buffer */
  put_bytes(0xD800, (const uint8_t[]){0x37}, 1);  /* source pixels: key (7), then 3 (4 bpp) */
  put_bytes(0xD000, (const uint8_t[]){0x11}, 1);  /* destination starts 1, 1 */
  cmd(0x62000000u);                               /* RCC_SRCADDR */
  cmd(0x63000000u);                               /* RCC_DESTADDR */
  cmd(0x66000007u);                               /* RCC_COLOR 7: the key */
  cmd(0x64000000u | (1u << 12) | 2u);             /* 1 x 2 pixels */
  cmd(0x67000000u | (6u << 7) | (0xCu << 3) | 6u);/* START: transparent copy, ROP C, source and dest continuous */
  CHECK_EQ(s32_eds_byte(&S.cpu, 0xD000), 0x31);   /* key pixel skipped (stays 1), the other copied (3) */
  CHECK_EQ(S.gfx.unknown_cmds, 0u);
}

TEST(chr_glyph_rows_are_lsb_leftmost) {
  /* The CHR engine's font format (FRM; see the model's CHR notes): an 8-byte header
   * (firstChar at +2, lastChar at +4, height at +6), per-character {width, 24-bit offset}, then the glyph's rows,
   * ceil(width/8) bytes each, LSB = the leftmost pixel. */
  setup();
  uint8_t font[] = {0, 0, 65, 0, 65, 0, 8, 0,    /* header: one glyph, 'A', 8 px high */
                    8, 12, 0, 0,                  /* width 8, glyph at font + 12 */
                    0x01, 0, 0, 0, 0, 0, 0, 0};   /* row 0: only the LEFTMOST pixel set */
  put_bytes(0xE000, font, sizeof font);
  cmd(0x50000005u);                               /* CHR_FG 5 */
  cmd(0x51000002u);                               /* CHR_BG 2 */
  cmd(0x5200E000u);                               /* CHR_FONT */
  cmd(0x58000000u);                               /* clip area x0,y0 = 0,0 */
  cmd(0x590EF13Fu);                               /* x1,y1 = 319,239 */
  cmd(0x5A000000u);                               /* print at x=0, y=0 */
  cmd(0x53000041u);                               /* print 'A' (opaque) */
  CHECK_EQ(s32_eds_byte(&S.cpu, 0xD000), 0x25);   /* pixel 0 = fg, pixel 1 = bg (4 bpp) */
  CHECK_EQ(s32_eds_byte(&S.cpu, 0xD003) & 0xF0, 0x20);  /* pixel 7 = bg (MSB-first would put fg here) */
  CHECK_EQ(S.gfx.unknown_cmds, 0u);
}
TEST(ipu_inflates_raw_deflate_into_w2_and_flags_done) {
  setup();
  uint8_t raw[300], comp[600];
  for (unsigned i = 0; i < sizeof raw; i++) raw[i] = (uint8_t)(i * 7 % 13);
  z_stream z; memset(&z, 0, sizeof z);
  deflateInit2(&z, 9, Z_DEFLATED, -15, 8, Z_DEFAULT_STRATEGY);
  z.next_in = raw; z.avail_in = sizeof raw; z.next_out = comp; z.avail_out = sizeof comp;
  deflate(&z, Z_FINISH); unsigned clen = (unsigned)z.total_out; deflateEnd(&z);
  put_bytes(0x17282u, comp, clen);               /* an EDS staging address */
  pic24_poke16(&S.cpu, 0x070C, 0x7282); pic24_poke16(&S.cpu, 0x070E, 0x0001);
  pic24_poke16(&S.cpu, 0x0710, 0x6600); pic24_poke16(&S.cpu, 0x0712, 0x0001);
  cmd(0x71000000u); cmd(0x72000000u); cmd(0x74000000u | sizeof raw);
  CHECK_EQ(pic24_peek16(&S.cpu, 0x073C) & 3, 3);  /* BFINAL | IPUDONE */
  CHECK_EQ(s32_eds_byte(&S.cpu, 0x16600u + 10), raw[10]);
  CHECK_EQ(s32_eds_byte(&S.cpu, 0x16600u + 299), raw[299]);
  put_bytes(0x17282u, (const uint8_t *)"\xff\xff\xff\xff", 4);   /* garbage: error, not DONE */
  cmd(0x74000000u | sizeof raw);
  CHECK_EQ(pic24_peek16(&S.cpu, 0x073C) & 2, 0);
}

int main(void) {
  RUN(rcc_solid_fill_writes_4bpp_pixels_in_the_rectangle_only);
  RUN(clut_write_needs_clutrwen_bit8);
  RUN(vblank_sets_vmrgnif_and_raises_irq100_only_when_enabled);
  RUN(rcc_transparent_copy_skips_the_key_colour);
  RUN(chr_glyph_rows_are_lsb_leftmost);
  RUN(ipu_inflates_raw_deflate_into_w2_and_flags_done);
  MT_REPORT();
}
