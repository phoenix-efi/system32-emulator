/* Unit tests of the conformance trace (emu/common/s32trace.c), no firmware needed: SHA-256 against FIPS 180-4 test
 * vectors, the canonical line order within a quantum whatever order the MCU threads produced their events in, the
 * line formats, the board's output-pin hook and the end-of-run state lines. */
#include "minitest.h"
#include "s32board.h"
#include "s32trace.h"
#include "s32panel.h"
#include <string.h>

static char *slurp_tmp(FILE *f) {
  fflush(f);
  long n = ftell(f);
  char *s = calloc(1, (size_t)n + 1);
  rewind(f);
  if (fread(s, 1, (size_t)n, f) != (size_t)n) abort();
  return s;
}

TEST(sha256_vectors) {
  char hx[65];
  s32_sha256_hex("", 0, hx);
  CHECK(!strcmp(hx, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"));
  s32_sha256_hex("abc", 3, hx);
  CHECK(!strcmp(hx, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"));
  const char *m = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
  s32_sha256_hex(m, strlen(m), hx);
  CHECK(!strcmp(hx, "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"));
  s32_sha256 s; uint8_t d[32]; static char a[1001];          /* one million 'a', fed in uneven pieces */
  memset(a, 'a', sizeof a);
  s32_sha256_init(&s);
  for (int i = 0; i < 1000; i++) s32_sha256_update(&s, a, i % 2 ? 999 : 1001);
  s32_sha256_final(&s, d);
  CHECK_EQ(d[0], 0xcd); CHECK_EQ(d[1], 0xc7); CHECK_EQ(d[31], 0xd0);
}

/* The same per-MCU events, produced in two different interleavings (as two schedules of the MCU threads would),
 * give the same bytes: the Controller's lines, then ECU1's, then ECU2's, then the barrier's. */
static char *one_quantum(int order) {
  FILE *f = tmpfile();
  s32trace t; s32trace_open(&t, f);
  const uint8_t del[3] = {0xA5, 0x00, 0xFF};
  if (order == 0) {
    s32trace_tx(&t, SB_E2, 900, 1, 0x41);
    s32trace_pin(&t, SB_E1, 800, 0, 0x0000, 0x0001);
    s32trace_coil(&t, SB_E2, 2, 950, 990);
    s32trace_tx(&t, SB_CT, 100, 3, 0x7E);
    s32trace_pin(&t, SB_E1, 805, 3, 0x2000, 0x0000);
  } else {
    s32trace_tx(&t, SB_CT, 100, 3, 0x7E);
    s32trace_pin(&t, SB_E1, 800, 0, 0x0000, 0x0001);
    s32trace_pin(&t, SB_E1, 805, 3, 0x2000, 0x0000);
    s32trace_tx(&t, SB_E2, 900, 1, 0x41);
    s32trace_coil(&t, SB_E2, 2, 950, 990);
  }
  s32trace_link(&t, 7, 1, false, del, 3);
  s32trace_link(&t, 7, 0, true, del, 0);                      /* nothing delivered: no line */
  s32trace_quantum(&t, 7, NULL, false);
  s32trace_flush(&t);
  char *s = slurp_tmp(f);
  fclose(f); s32trace_free(&t);
  return s;
}

TEST(canonical_order_and_formats) {
  char *a = one_quantum(0), *b = one_quantum(1);
  CHECK(!strcmp(a, b));
  const char *want =
    "s32-trace 1 quantum_us 100 clocks CT=Fcy E1,E2=SYSCLK (cycles, crystal error applied)\n"
    "CT 100 tx U3 7E\n"
    "E1 800 pin RA 0000 0001\n"
    "E1 805 pin RD 2000 0000\n"
    "E2 900 tx U1 41\n"
    "E2 990 coil OC2 on 950\n"
    "q 7 deliver E2>CT 3 A500FF\n";
  CHECK(!strcmp(a, want));
  if (strcmp(a, want)) fprintf(stderr, "got:\n%s", a);
  free(a); free(b);
}

/* Without a sink (the WebAssembly bench) the bytes stay in `out` for the front end; the trace's own hash covers them. */
TEST(memory_sink_and_hash) {
  s32trace t; s32trace_open(&t, NULL);
  s32trace_pin(&t, SB_CT, 5, 0, 0, 0x5555);
  s32trace_quantum(&t, 1, NULL, false);
  char hx[65], want[65];
  s32_sha256_hex(t.out.p, t.out.n, want);
  s32_sha256 s = t.sha; uint8_t d[32];
  s32_sha256_final(&s, d);
  for (int i = 0; i < 32; i++) snprintf(hx + 2 * i, 3, "%02x", d[i]);
  CHECK(!strcmp(hx, want));
  CHECK_EQ(t.lines, 2);
  s32trace_free(&t);
}

/* The board forwards every output-pin change of every MCU (s32board_attach installs the observers), and the run's end
 * lines hash each chip as the front end holds it. */
static s32board B;
static s32ct CT;
static ecu_t E[2];
static s32_fram FR[3];
static uint8_t BFL[0x3000], PFL[0x1000], CTP[3 * 64];
static void boot(void *ctx, int who, bool cold) { (void)ctx; (void)who; (void)cold; }

TEST(board_hook_and_end_lines) {
  uint32_t loop[2] = {0x1000FFFFu, 0x00000000u};                /* b . ; nop */
  memcpy(BFL, loop, sizeof loop);
  for (int k = 0; k < 2; k++) if (ecu_init(&E[k], BFL, sizeof BFL, PFL, sizeof PFL)) abort();
  s32ct_init(&CT, CTP, sizeof CTP);
  s32board_init(&B, (s32board_host){.boot = boot});
  B.ct = &CT; B.ecu[0] = &E[0]; B.ecu[1] = &E[1];
  for (int w = 0; w < 3; w++) { s32_fram_init(&FR[w], 0x00); B.fram[w] = &FR[w]; }
  FR[SB_E2].mem[100] = 0x42;
  s32board_attach(&B);
  FILE *f = tmpfile();
  s32trace t; s32trace_open(&t, f);
  B.trace = &t;
  E[0].lat_sink(E[0].lat_ctx, &E[0], 1234, 6, 0x0000, 0x1000);   /* ECU1 RG12 (an LED) on */
  CT.lat_sink(CT.lat_ctx, &CT, 77, 4, 0x0000, 0x0001);            /* Controller RE0 (DIGOUT1) on */
  s32trace_quantum(&t, 1, NULL, false);
  const uint8_t *pf[2] = {PFL, NULL}; const size_t npf[2] = {sizeof PFL, 0};
  char sha[65];
  s32trace_finish(&t, &B, pf, npf, sha);
  char *s = slurp_tmp(f);
  CHECK(strstr(s, "CT 77 pin RE 0000 0001\nE1 1234 pin RG 0000 1000\n") != NULL);
  char hx[65], line[128];
  s32_sha256_hex(FR[SB_E2].mem, sizeof FR[SB_E2].mem, hx);
  snprintf(line, sizeof line, "end fram E2 %s\n", hx);
  CHECK(strstr(s, line) != NULL);
  CHECK(strstr(s, "end sd CT none\n") != NULL);
  CHECK(strstr(s, "end pflash E2 none\n") != NULL);
  CHECK(strstr(s, "end cycles CT 0 E1 0 E2 0\n") != NULL);
  s32_sha256_hex(s, strlen(s), hx);
  CHECK(!strcmp(hx, sha));                                       /* the reported hash is the file's */
  CHECK_EQ(B.eo[0].leds, 0x2);                                   /* the board's own observer still ran */
  free(s); fclose(f); s32trace_free(&t);
  for (int k = 0; k < 2; k++) ecu_free(&E[k]);
}

/* The panel every front end renders (emu/common/s32panel.c): dark unless powered, G1EN, DPPINOE, no test pattern and
 * a 320 x 240 display; lit pixels through the CLUT (RGB565 widened). */
TEST(panel_lit_and_dark) {
  static s32ct ct; static uint8_t rgb[S32_PANEL_W * S32_PANEL_H * 3];
  s32ct_init(&ct, CTP, sizeof CTP);
  s32_gfx *g = &ct.gfx;
  g->reg[0x02] = 0x8000; g->reg[0x13] = 0x0200; g->reg[0x0E] = 320; g->reg[0x0F] = 240;
  g->reg[0x12] = 2u << 5;                                         /* DPBPP = 4 bpp, DPTEST = 0 */
  g->reg[0x0C] = 0x1000; g->reg[0x0D] = 0;                        /* display buffer in RAM (all zero after init) */
  g->clut[0] = 0xF800;                                            /* index 0 = full red */
  s32_panel_state st;
  s32_panel_render(&ct, true, rgb, &st);
  CHECK(st.lit);
  CHECK_EQ(rgb[0], 255); CHECK_EQ(rgb[1], 0); CHECK_EQ(rgb[2], 0);
  CHECK_EQ(rgb[sizeof rgb - 3], 255);
  s32_panel_render(&ct, false, rgb, &st);
  CHECK(!st.lit && !strcmp(st.why, "unpowered") && rgb[0] == 0);
  g->reg[0x12] |= 0x0100;                                         /* DPTEST = 01: black screen */
  s32_panel_render(&ct, true, rgb, &st);
  CHECK(!st.lit && !strcmp(st.why, "test pattern") && rgb[0] == 0);
  g->reg[0x12] &= (uint16_t)~0x0300u; g->reg[0x13] = 0;           /* display pins not driven */
  s32_panel_render(&ct, true, rgb, &st);
  CHECK(!st.lit && !strcmp(st.why, "pins off"));
  g->reg[0x13] = 0x0200; g->reg[0x02] = 0;                        /* graphics module off */
  s32_panel_render(&ct, true, rgb, &st);
  CHECK(!st.lit && !strcmp(st.why, "G1EN off"));
  g->reg[0x02] = 0x8000; g->reg[0x0E] = 480;
  s32_panel_render(&ct, true, rgb, &st);
  CHECK(!st.lit && !strcmp(st.why, "size"));
}

int main(void) {
  RUN(sha256_vectors);
  RUN(canonical_order_and_formats);
  RUN(memory_sink_and_hash);
  RUN(board_hook_and_end_lines);
  RUN(panel_lit_and_dark);
  MT_REPORT();
}
