/* test_core.c -- unit tests per instruction class for the PIC24F core.  Expected values come from the PRM
 * (DS70000157G) text/examples (page cited) or from first-principles arithmetic stated in the test. */
#include "minitest.h"
#include "pic24.h"

#include <stdint.h>
#include <string.h>

/* ------------------------------------------------------------------------------------------- encoders --- */
enum { T_SUBR = 0x02, T_SUBBR = 0x03, T_ADD = 0x08, T_ADDC = 0x09, T_SUB = 0x0A, T_SUBB = 0x0B, T_AND = 0x0C,
       T_XOR = 0x0D, T_IOR = 0x0E, T_MOV = 0x0F };
static uint32_t e3(unsigned t5, unsigned B, unsigned wb, unsigned p, unsigned s, unsigned q, unsigned d) {
  return (t5 << 19) | (wb << 15) | (B << 14) | (q << 11) | (d << 7) | (p << 4) | s;
}
static uint32_t e3k(unsigned t5, unsigned B, unsigned wb, unsigned k, unsigned q, unsigned d) {
  return (t5 << 19) | (wb << 15) | (B << 14) | (q << 11) | (d << 7) | 0x60u | (k & 0x1Fu);
}
static uint32_t ef(unsigned op8, unsigned hi, unsigned B, unsigned D, unsigned f) {
  return (op8 << 16) | (hi << 15) | (B << 14) | (D << 13) | (f & 0x1FFFu);
}
static uint32_t elit10(unsigned op8, unsigned hi, unsigned B, unsigned k, unsigned d) {
  return (op8 << 16) | (hi << 15) | (B << 14) | ((k & 0x3FFu) << 4) | d;
}
#define MOVLIT(k, d) (0x200000u | (((k) & 0xFFFFu) << 4) | (d))
#define BRA(cc, off) ((uint32_t)(0x30u | (cc)) << 16 | ((uint32_t)(off) & 0xFFFFu))
#define REPEAT(n) (0x090000u | (n))
#define REPEATW(s) (0x098000u | (s))
#define DIVI(u, t, v, w, s) (0xD80000u | ((u) << 15) | ((t) << 11) | ((v) << 7) | ((w) << 6) | (s))
#define RETURN_ 0x060000u
#define RETFIE_ 0x064000u
#define NOP_ 0x000000u
#define MOVF_W(f, d) (0x800000u | ((((f) >> 1) & 0x7FFFu) << 4) | (d))
#define MOVW_F(s, f) (0x880000u | ((((f) >> 1) & 0x7FFFu) << 4) | (s))
static uint32_t movoff(bool store, unsigned B, int k, unsigned d, unsigned s) {
  unsigned u = (unsigned)k & 0x3FFu;
  return (store ? 0x980000u : 0x900000u) | (((u >> 6) & 0xFu) << 15) | (B << 14) | (((u >> 3) & 7u) << 11) | (d << 7) |
         ((u & 7u) << 4) | s;
}

static pic24_cpu *mk(void) {
  pic24_cpu *c = calloc(1, sizeof *c);
  pic24_init(c, NULL);
  return c;
}
static void put(pic24_cpu *c, uint32_t pc, const uint32_t *w, size_t n) {
  for (size_t i = 0; i < n; i++) c->prog[(pc >> 1) + i] = w[i] & 0xFFFFFFu;
}
#define PUT(c, pc, ...) do { const uint32_t _w[] = {__VA_ARGS__}; put(c, pc, _w, sizeof _w / sizeof _w[0]); } while (0)
/* run one instruction at 0x200 from a given state; return cycles */
static unsigned one(pic24_cpu *c, uint32_t insn) {
  c->prog[0x100] = insn;
  c->prog[0x101] = 0;
  c->pc = 0x200;
  uint64_t c0 = c->cycles;
  pic24_status st = pic24_step(c);
  CHECK_EQ(st, PIC24_OK);
  return (unsigned)(c->cycles - c0);
}
static uint16_t flags(pic24_cpu *c) { return c->sr & (PIC24_SR_C | PIC24_SR_Z | PIC24_SR_OV | PIC24_SR_N | PIC24_SR_DC); }
#define C_ PIC24_SR_C
#define Z_ PIC24_SR_Z
#define OV_ PIC24_SR_OV
#define N_ PIC24_SR_N
#define DC_ PIC24_SR_DC

/* ======================================================================================== arithmetic === */

TEST(add_flags) {
  pic24_cpu *c = mk();
  /* word: 0x7FFF + 1 = 0x8000: OV (pos+pos=neg), N, DC (carry out of bit 7), no C */
  c->ram[1] = 0x7FFF; c->ram[2] = 1;
  one(c, e3(T_ADD, 0, 1, 0, 2, 0, 3));
  CHECK_EQ(c->ram[3], 0x8000);
  CHECK_EQ(flags(c), OV_ | N_ | DC_);
  /* 0xFFFF + 1 = 0: C, Z, DC */
  c->ram[1] = 0xFFFF; c->ram[2] = 1;
  one(c, e3(T_ADD, 0, 1, 0, 2, 0, 3));
  CHECK_EQ(c->ram[3], 0);
  CHECK_EQ(flags(c), C_ | Z_ | DC_);
  /* 0x0080 + 0x0080: no DC? carry into bit 8 from bit 7 -> DC=1 (word DC = carry out of bit 7) */
  c->ram[1] = 0x0080; c->ram[2] = 0x0080;
  one(c, e3(T_ADD, 0, 1, 0, 2, 0, 3));
  CHECK_EQ(c->ram[3], 0x0100);
  CHECK_EQ(flags(c), DC_);
  /* 0x000F + 0x0001: digit carry out of bit 3 does NOT set word DC */
  c->ram[1] = 0x000F; c->ram[2] = 1;
  one(c, e3(T_ADD, 0, 1, 0, 2, 0, 3));
  CHECK_EQ(flags(c), 0);
  /* byte: 0x0F + 0x01 = 0x10 -> DC (carry out of bit 3), high byte of Wd preserved */
  c->ram[1] = 0xAA0F; c->ram[2] = 0x5501; c->ram[3] = 0x1234;
  one(c, e3(T_ADD, 1, 1, 0, 2, 0, 3));
  CHECK_EQ(c->ram[3], 0x1210);
  CHECK_EQ(flags(c), DC_);
  /* byte 0x80 + 0x80 = 0x00: C, Z, OV (neg+neg=pos) */
  c->ram[1] = 0x0080; c->ram[2] = 0x0080;
  one(c, e3(T_ADD, 1, 1, 0, 2, 0, 3));
  CHECK_EQ(c->ram[3] & 0xFF, 0);
  CHECK_EQ(flags(c), C_ | Z_ | OV_);
  /* ADD #lit10,Wn (p.103): 0x3FF + 1 */
  c->ram[4] = 1;
  one(c, elit10(0xB0, 0, 0, 0x3FF, 4));
  CHECK_EQ(c->ram[4], 0x400);
  free(c);
}

/* PRM Example 4-15 (p.79): ADD/ADDC 32-bit. The PRM prints SR = 0x0201 "(DC,C=1)" for the first case: 0x0200 is
 * bit 9 (DA on dsPIC), DC is bit 8 -- a typo in the PRM; the stated flags DC,C=1 give 0x0101. Second case 0x0103. */
TEST(prm_example_4_15) {
  pic24_cpu *c = mk();
  PUT(c, 0x200, e3(T_ADD, 0, 0, 0, 2, 0, 4), e3(T_ADDC, 0, 1, 0, 3, 0, 5));
  c->ram[0] = 0x2342; c->ram[1] = 0xFFF0; c->ram[2] = 0x39AA; c->ram[3] = 0x0010; c->sr = 0; c->pc = 0x200;
  pic24_step(c); pic24_step(c);
  CHECK_EQ(c->ram[4], 0x5CEC);
  CHECK_EQ(c->ram[5], 0x0000);
  CHECK_EQ(c->sr, 0x0101);
  c->ram[0] = 0xB76E; c->ram[1] = 0xFB7B; c->ram[2] = 0x4892; c->ram[3] = 0x0484; c->sr = 0; c->pc = 0x200;
  pic24_step(c); pic24_step(c);
  CHECK_EQ(c->ram[4], 0x0000);
  CHECK_EQ(c->ram[5], 0x0000);
  CHECK_EQ(c->sr, 0x0103);
  free(c);
}

TEST(addc_sticky_z_and_ov) {
  pic24_cpu *c = mk();
  /* ADDC: Z can only be cleared (PRM 4.10). Z=0 in, result 0 -> Z stays 0 */
  c->ram[1] = 0xFFFF; c->ram[2] = 0; c->sr = C_;
  one(c, e3(T_ADDC, 0, 1, 0, 2, 0, 3));
  CHECK_EQ(c->ram[3], 0);
  CHECK(!(c->sr & Z_));
  CHECK(c->sr & C_);
  /* Z=1 in, result non-zero -> Z cleared */
  c->ram[1] = 1; c->ram[2] = 1; c->sr = Z_;
  one(c, e3(T_ADDC, 0, 1, 0, 2, 0, 3));
  CHECK(!(c->sr & Z_));
  /* OV on ADDC uses the full a+b+c: 0x8000 + 0xFFFF + 1 = 0x8000 (true -32768): NO overflow */
  c->ram[1] = 0x8000; c->ram[2] = 0xFFFF; c->sr = C_;
  one(c, e3(T_ADDC, 0, 1, 0, 2, 0, 3));
  CHECK_EQ(c->ram[3], 0x8000);
  CHECK(!(c->sr & OV_));
  CHECK(c->sr & C_);
  /* 0x7FFF + 0 + 1 = 0x8000: overflow */
  c->ram[1] = 0x7FFF; c->ram[2] = 0; c->sr = C_;
  one(c, e3(T_ADDC, 0, 1, 0, 2, 0, 3));
  CHECK(c->sr & OV_);
  free(c);
}

TEST(sub_family) {
  pic24_cpu *c = mk();
  /* SUB: C = NOT borrow. 5 - 3 = 2: C=1 */
  c->ram[1] = 5; c->ram[2] = 3;
  one(c, e3(T_SUB, 0, 1, 0, 2, 0, 3));
  CHECK_EQ(c->ram[3], 2);
  CHECK(c->sr & C_);
  /* 3 - 5: borrow -> C=0, N */
  c->ram[1] = 3; c->ram[2] = 5;
  one(c, e3(T_SUB, 0, 1, 0, 2, 0, 3));
  CHECK_EQ(c->ram[3], 0xFFFE);
  CHECK(!(c->sr & C_));
  CHECK(c->sr & N_);
  /* 0x8000 - 1 = 0x7FFF: OV (neg - pos = pos) */
  c->ram[1] = 0x8000; c->ram[2] = 1;
  one(c, e3(T_SUB, 0, 1, 0, 2, 0, 3));
  CHECK_EQ(c->ram[3], 0x7FFF);
  CHECK(c->sr & OV_);
  /* DC as digit borrow: 0x0100 - 0x0001: borrow from bit 8 into bit 7 -> DC=0 */
  c->ram[1] = 0x0100; c->ram[2] = 1;
  one(c, e3(T_SUB, 0, 1, 0, 2, 0, 3));
  CHECK(!(c->sr & DC_));
  c->ram[1] = 0x0180; c->ram[2] = 1;
  one(c, e3(T_SUB, 0, 1, 0, 2, 0, 3));
  CHECK(c->sr & DC_);
  /* SUBB with C=0 (borrow pending): 5 - 3 - 1 = 1 */
  c->ram[1] = 5; c->ram[2] = 3; c->sr = 0;
  one(c, e3(T_SUBB, 0, 1, 0, 2, 0, 3));
  CHECK_EQ(c->ram[3], 1);
  /* SUBB OV uses full a-b-!C: 0x8000 - 0x7FFF - 1 = 0 (true -32768-32767-1 = -65536 -> wraps 0; OV=1) */
  c->ram[1] = 0x8000; c->ram[2] = 0x7FFF; c->sr = 0;
  one(c, e3(T_SUBB, 0, 1, 0, 2, 0, 3));
  CHECK_EQ(c->ram[3], 0);
  CHECK(c->sr & OV_);
  /* SUBB borrow edge: a - 0xFFFF - 1 always borrows (C=0) even for a=0xFFFF */
  c->ram[1] = 0xFFFF; c->ram[2] = 0xFFFF; c->sr = 0;
  one(c, e3(T_SUBB, 0, 1, 0, 2, 0, 3));
  CHECK_EQ(c->ram[3], 0xFFFF);
  CHECK(!(c->sr & C_));
  /* SUBB sticky Z: result 0, Z=0 in -> stays 0 */
  c->ram[1] = 5; c->ram[2] = 5; c->sr = C_;
  one(c, e3(T_SUBB, 0, 1, 0, 2, 0, 3));
  CHECK_EQ(c->ram[3], 0);
  CHECK(!(c->sr & Z_));
  /* SUBR Wb,Ws,Wd = Ws - Wb (p.437) */
  c->ram[1] = 3; c->ram[2] = 10;
  one(c, e3(T_SUBR, 0, 1, 0, 2, 0, 3));
  CHECK_EQ(c->ram[3], 7);
  /* SUBR Wb,#lit5,Wd = lit5 - Wb (p.436) */
  c->ram[1] = 3;
  one(c, e3k(T_SUBR, 0, 1, 10, 0, 3));
  CHECK_EQ(c->ram[3], 7);
  /* SUBBR Wb,Ws,Wd = Ws - Wb - !C (p.433), C=0 */
  c->ram[1] = 3; c->ram[2] = 10; c->sr = 0;
  one(c, e3(T_SUBBR, 0, 1, 0, 2, 0, 3));
  CHECK_EQ(c->ram[3], 6);
  /* SUB #lit10,Wn = Wn - lit10 (p.419) */
  c->ram[4] = 100;
  one(c, elit10(0xB1, 0, 0, 1, 4));
  CHECK_EQ(c->ram[4], 99);
  /* SUB f: f - WREG (p.418) to WREG; SUBR f: WREG - f (p.435) */
  c->ram[0x1000 >> 1] = 50; c->ram[0] = 8;
  one(c, ef(0xB5, 0, 0, 0, 0x1000));
  CHECK_EQ(c->ram[0], 42);
  c->ram[0] = 8;
  one(c, ef(0xBD, 0, 0, 1, 0x1000));
  CHECK_EQ(c->ram[0x1000 >> 1], (uint16_t)(8 - 50));
  /* NEG: 0x8000 -> 0x8000 with OV; 0 -> 0 with C, Z (p.351) */
  c->ram[1] = 0x8000;
  one(c, 0xEA0000u | (0u << 11) | (2u << 7) | (0u << 4) | 1u);
  CHECK_EQ(c->ram[2], 0x8000);
  CHECK(c->sr & OV_);
  c->ram[1] = 0;
  one(c, 0xEA0000u | (2u << 7) | 1u);
  CHECK_EQ(flags(c), C_ | Z_ | DC_);
  /* CP0: C=1, DC=1 always (Table 7-2) */
  c->ram[1] = 0x8000;
  one(c, 0xE00000u | 1u);
  CHECK_EQ(flags(c), C_ | DC_ | N_);
  /* CP.B Wb,#lit5 */
  c->ram[1] = 0x1205;
  one(c, 0xE10000u | (1u << 11) | (1u << 10) | 0x60u | 5u);
  CHECK_EQ(flags(c), C_ | Z_ | DC_);
  free(c);
}

TEST(inc_dec) {
  pic24_cpu *c = mk();
  c->ram[1] = 0x7FFF;
  one(c, 0xE80000u | (2u << 7) | 1u); /* INC W1,W2 */
  CHECK_EQ(c->ram[2], 0x8000);
  CHECK_EQ(flags(c), OV_ | N_ | DC_);
  c->ram[1] = 1;
  one(c, 0xE98000u | (2u << 7) | 1u); /* DEC2 W1,W2 */
  CHECK_EQ(c->ram[2], 0xFFFF);
  CHECK(!(c->sr & C_));
  c->ram[0x900 >> 1] = 0x00FF;
  one(c, ef(0xEC, 0, 1, 1, 0x900)); /* INC.B 0x900 */
  CHECK_EQ(c->ram[0x900 >> 1], 0x0000);
  CHECK_EQ(flags(c), C_ | Z_ | DC_);
  free(c);
}

/* ============================================================================================= logic === */

TEST(logic_nz_only) {
  pic24_cpu *c = mk();
  c->sr = C_ | OV_ | DC_;
  c->ram[1] = 0xF0F0; c->ram[2] = 0x0F0F;
  one(c, e3(T_AND, 0, 1, 0, 2, 0, 3));
  CHECK_EQ(c->ram[3], 0);
  CHECK_EQ(flags(c), C_ | OV_ | DC_ | Z_);
  one(c, e3(T_IOR, 0, 1, 0, 2, 0, 3));
  CHECK_EQ(c->ram[3], 0xFFFF);
  CHECK_EQ(flags(c), C_ | OV_ | DC_ | N_);
  one(c, e3k(T_XOR, 0, 1, 0x1F, 0, 3));
  CHECK_EQ(c->ram[3], 0xF0EF);
  /* COM.B W1,W3 */
  c->ram[1] = 0x12F0; c->ram[3] = 0xAA00;
  one(c, 0xEA8000u | (1u << 14) | (3u << 7) | 1u);
  CHECK_EQ(c->ram[3], 0xAA0F);
  /* IOR #lit10 / AND.B #lit10 / XOR #lit10 */
  c->ram[4] = 0x1000;
  one(c, elit10(0xB3, 0, 0, 0x3FF, 4));
  CHECK_EQ(c->ram[4], 0x13FF);
  one(c, elit10(0xB2, 0, 1, 0x0F, 4));
  CHECK_EQ(c->ram[4], 0x130F);
  /* MOV.B #lit8 keeps high byte (p.303) */
  one(c, 0xB3C000u | (0xABu << 4) | 4u);
  CHECK_EQ(c->ram[4], 0x13AB);
  /* SETM.B / CLR */
  one(c, 0xEB8000u | (1u << 14) | (4u << 7));
  CHECK_EQ(c->ram[4], 0x13FF);
  one(c, 0xEB0000u | (4u << 7));
  CHECK_EQ(c->ram[4], 0);
  free(c);
}

/* ========================================================================================= shifts === */

TEST(shifts_rotates) {
  pic24_cpu *c = mk();
  /* SL W1,W2: C = msb, N/Z; OV unchanged (PRM p.414, Table 7-2) */
  c->ram[1] = 0xC001; c->sr = OV_;
  one(c, 0xD00000u | (2u << 7) | 1u);
  CHECK_EQ(c->ram[2], 0x8002);
  CHECK_EQ(flags(c), C_ | N_ | OV_);
  /* ASR W1,W2 (p.123) */
  c->ram[1] = 0x8001; c->sr = 0;
  one(c, 0xD18000u | (2u << 7) | 1u);
  CHECK_EQ(c->ram[2], 0xC000);
  CHECK_EQ(flags(c), C_ | N_);
  /* LSR.B W1,W2 */
  c->ram[1] = 0x1281; c->ram[2] = 0x5500;
  one(c, 0xD10000u | (1u << 14) | (2u << 7) | 1u);
  CHECK_EQ(c->ram[2], 0x5540);
  CHECK_EQ(flags(c), C_);
  /* RLC W1,W2 with C=1 */
  c->ram[1] = 0x8000; c->sr = C_;
  one(c, 0xD28000u | (2u << 7) | 1u);
  CHECK_EQ(c->ram[2], 0x0001);
  CHECK_EQ(flags(c), C_);
  /* RRNC: C untouched */
  c->ram[1] = 0x0001; c->sr = 0;
  one(c, 0xD30000u | (2u << 7) | 1u);
  CHECK_EQ(c->ram[2], 0x8000);
  CHECK_EQ(flags(c), N_);
  /* RRC.B with C=1 */
  c->ram[1] = 0x0002; c->sr = C_;
  one(c, 0xD38000u | (1u << 14) | (2u << 7) | 1u);
  CHECK_EQ(c->ram[2] & 0xFF, 0x81);
  CHECK(!(c->sr & C_));
  /* multi-bit: SL W1,#4,W2; ASR W1,W3,W2 with W3=20 (>15: sign fill, p.126 note 2); LSR by 16 -> 0 */
  c->ram[1] = 0x1234;
  one(c, 0xDD0000u | (1u << 11) | (2u << 7) | 0x40u | 4u);
  CHECK_EQ(c->ram[2], 0x2340);
  c->ram[1] = 0x8000; c->ram[3] = 20;
  one(c, 0xDE8000u | (1u << 11) | (2u << 7) | 3u);
  CHECK_EQ(c->ram[2], 0xFFFF);
  c->ram[3] = 16;
  one(c, 0xDE0000u | (1u << 11) | (2u << 7) | 3u);
  CHECK_EQ(c->ram[2], 0);
  CHECK(c->sr & Z_);
  /* Wns<4:0>: 0x21 -> shift 1 */
  c->ram[1] = 0x0002; c->ram[3] = 0x21;
  one(c, 0xDE0000u | (1u << 11) | (2u << 7) | 3u);
  CHECK_EQ(c->ram[2], 1);
  free(c);
}

/* ========================================================================================= multiply === */

TEST(multiply) {
  pic24_cpu *c = mk();
  /* MUL.SS W1,W2,W4: -2 * 3 = -6 */
  c->ram[1] = 0xFFFE; c->ram[2] = 3;
  one(c, 0xB98000u | (1u << 11) | (4u << 7) | 2u);
  CHECK_EQ(c->ram[4], 0xFFFA); CHECK_EQ(c->ram[5], 0xFFFF);
  /* MUL.UU: 0xFFFF*0xFFFF = 0xFFFE0001 */
  c->ram[1] = 0xFFFF; c->ram[2] = 0xFFFF;
  one(c, 0xB80000u | (1u << 11) | (4u << 7) | 2u);
  CHECK_EQ(c->ram[4], 0x0001); CHECK_EQ(c->ram[5], 0xFFFE);
  /* MUL.SU: signed(0xFFFF=-1) * unsigned(0xFFFF) = -65535 = 0xFFFF0001 */
  one(c, 0xB90000u | (1u << 11) | (4u << 7) | 2u);
  CHECK_EQ(c->ram[4], 0x0001); CHECK_EQ(c->ram[5], 0xFFFF);
  /* MUL.US: unsigned(0xFFFF) * signed(-1) = -65535 */
  one(c, 0xB88000u | (1u << 11) | (4u << 7) | 2u);
  CHECK_EQ(c->ram[4], 0x0001); CHECK_EQ(c->ram[5], 0xFFFF);
  /* MUL.SU W1,#31,W6: -1*31 */
  one(c, 0xB90000u | (1u << 11) | (6u << 7) | 0x60u | 31u);
  CHECK_EQ(c->ram[6], (uint16_t)-31); CHECK_EQ(c->ram[7], 0xFFFF);
  /* PRM Example 4-17 (p.82): MUL.B 0x100 with WREG=0x7705, [0x100]=0x1255 -> W2=0x01A9, W3 unchanged;
   * MUL 0x100 -> W2=0xDEA9, W3=0x0885. Flags unaffected. */
  c->ram[0] = 0x7705; c->ram[2] = 0x1235; c->ram[3] = 0x1000; c->ram[0x100 >> 1] = 0x1255; c->sr = 0x000F;
  /* 0x100 is an SFR address on this part: use RAM 0x0900 for the same arithmetic */
  c->ram[0x900 >> 1] = 0x1255;
  one(c, 0xBC0000u | (1u << 14) | 0x900u);
  CHECK_EQ(c->ram[2], 0x01A9); CHECK_EQ(c->ram[3], 0x1000);
  one(c, 0xBC0000u | 0x900u);
  CHECK_EQ(c->ram[2], 0xDEA9); CHECK_EQ(c->ram[3], 0x0885);
  CHECK_EQ(flags(c), 0x000F);
  free(c);
}

/* ============================================================================================ divide === */

static void div_ref(bool uns, bool dbl, uint16_t lo, uint16_t hi, uint16_t dv, uint16_t *q, uint16_t *r, bool *ov) {
  if (uns) {
    uint32_t x = dbl ? ((uint32_t)hi << 16 | lo) : lo;
    uint32_t qq = x / dv, rr = x % dv;
    *q = (uint16_t)qq; *r = (uint16_t)rr; *ov = qq > 0xFFFF;
  } else {
    int64_t x = dbl ? (int32_t)((uint32_t)hi << 16 | lo) : (int16_t)lo;
    int64_t d = (int16_t)dv;
    int64_t qq = x / d, rr = x % d;
    *q = (uint16_t)qq; *r = (uint16_t)rr; *ov = qq > 32767 || qq < -32768;
  }
}

static unsigned run_div(pic24_cpu *c, uint32_t div_insn) {
  PUT(c, 0x200, REPEAT(17), div_insn, NOP_);
  c->pc = 0x200;
  uint64_t c0 = c->cycles;
  while (c->pc != 0x204) CHECK_EQ(pic24_step(c), PIC24_OK);
  return (unsigned)(c->cycles - c0);
}

TEST(divide_against_reference) {
  pic24_cpu *c = mk();
  uint32_t seed = 12345;
  int bad = 0;
  for (int n = 0; n < 40000; n++) {
    seed = seed * 1103515245u + 12345u;
    uint32_t r1 = seed;
    seed = seed * 1103515245u + 12345u;
    uint32_t r2 = seed;
    bool uns = r1 & 1, dbl = (r1 >> 1) & 1;
    uint16_t lo = (uint16_t)(r2 >> 8), hi = (uint16_t)(r1 >> 16), dv = (uint16_t)(r2 >> 20 | r1 << 4);
    if (n % 7 == 0) dv = (uint16_t)((r1 >> 9) & 0xF) + 1;          /* small divisors */
    if (n % 11 == 0) dv = (uint16_t)-((int)((r1 >> 9) & 0xF) + 1); /* small negative */
    if (dv == 0) dv = 3;
    /* keep the quotient representable for .D (overflow behaviour is "should not be used") */
    if (dbl) {
      if (uns) hi = (uint16_t)(hi % dv);
      else { int m = abs((int16_t)dv) / 2; if (m == 0) m = 1; hi = (uint16_t)(int16_t)((int16_t)hi % m); }
    }
    c->ram[4] = lo; c->ram[5] = hi; c->ram[6] = dv; c->sr = 0; c->rcount = 0;
    unsigned cyc = run_div(c, DIVI(uns, dbl ? 5u : 0u, 4u, dbl, 6u));
    uint16_t q, r; bool ov;
    div_ref(uns, dbl, lo, hi, dv, &q, &r, &ov);
    if (ov) continue;
    if (c->ram[0] != q || c->ram[1] != r || cyc != 19 || ((c->sr & OV_) != 0) != ov ||
        ((c->sr & Z_) != 0) != (r == 0) || ((c->sr & N_) != 0) != (!uns && (r & 0x8000))) {
      if (bad++ < 5)
        fprintf(stderr, "div %s%s %04X:%04X / %04X -> q=%04X r=%04X sr=%04X cyc=%u (want q=%04X r=%04X)\n",
                uns ? "U" : "S", dbl ? "D" : "W", hi, lo, dv, c->ram[0], c->ram[1], c->sr, cyc, q, r);
    }
  }
  CHECK_EQ(bad, 0);
  free(c);
}

TEST(divide_edges) {
  pic24_cpu *c = mk();
  /* DIV.SW 0x8000 / 0xFFFF: the one word overflow; OV=1 and W0 = 0x8000 (PRM p.233 note 2) */
  c->ram[4] = 0x8000; c->ram[6] = 0xFFFF;
  run_div(c, DIVI(0, 0, 4, 0, 6));
  CHECK(c->sr & OV_);
  CHECK_EQ(c->ram[0], 0x8000);
  CHECK_EQ(c->ram[1], 0);
  /* remainder takes the dividend's sign: -7/2 = -3 r -1; 7/-2 = -3 r 1; -7/-2 = 3 r -1 */
  c->ram[4] = (uint16_t)-7; c->ram[6] = 2; run_div(c, DIVI(0, 0, 4, 0, 6));
  CHECK_EQ((int16_t)c->ram[0], -3); CHECK_EQ((int16_t)c->ram[1], -1); CHECK(c->sr & N_);
  c->ram[4] = 7; c->ram[6] = (uint16_t)-2; run_div(c, DIVI(0, 0, 4, 0, 6));
  CHECK_EQ((int16_t)c->ram[0], -3); CHECK_EQ((int16_t)c->ram[1], 1); CHECK(!(c->sr & N_));
  c->ram[4] = (uint16_t)-7; c->ram[6] = (uint16_t)-2; run_div(c, DIVI(0, 0, 4, 0, 6));
  CHECK_EQ((int16_t)c->ram[0], 3); CHECK_EQ((int16_t)c->ram[1], -1);
  /* DIV.UD overflow: 0x00020000 / 1 -> OV */
  c->ram[4] = 0; c->ram[5] = 2; c->ram[6] = 1; run_div(c, DIVI(1, 5, 4, 1, 6));
  CHECK(c->sr & OV_);
  /* DIV.UW: OV always 0 (Table 7-2), N always 0 */
  c->ram[4] = 0xFFFF; c->ram[6] = 1; c->sr = OV_ | N_; run_div(c, DIVI(1, 0, 4, 0, 6));
  CHECK_EQ(c->ram[0], 0xFFFF); CHECK(!(c->sr & (OV_ | N_)));
  /* dividend register = W0 (overwritten during the divide), divisor W2 */
  c->ram[0] = 1000; c->ram[2] = 7; run_div(c, DIVI(0, 0, 0, 0, 2));
  CHECK_EQ(c->ram[0], 142); CHECK_EQ(c->ram[1], 6);
  free(c);
}

TEST(divide_by_zero_trap) {
  pic24_cpu *c = mk();
  /* math-error trap vector (IVT 0x00000C) -> handler at 0x300 */
  c->prog[0x0C >> 1] = 0x000300;
  PUT(c, 0x300, NOP_);
  PUT(c, 0x200, REPEAT(17), DIVI(0, 0, 4, 0, 6), NOP_);
  c->ram[4] = 5; c->ram[6] = 0; c->pc = 0x200;
  pic24_step(c); /* REPEAT */
  pic24_step(c); /* first DIV cycle executes (FRM2 2.6.3) */
  CHECK(c->intcon1 & PIC24_INTCON1_MATHERR);
  pic24_step(c); /* exception entry */
  CHECK_EQ(c->pc, 0x300);
  CHECK_EQ(pic24_cpu_ipl(c), 11u);
  /* stacked: PC = DIV address (loop resumes there), RA = 1 */
  CHECK_EQ(c->ram[0x800 >> 1], 0x202);
  CHECK(c->ram[0x802 >> 1] & (PIC24_SR_RA << 8));
  CHECK_EQ(c->rcount, 16);
  free(c);
}

/* An interrupt in the middle of a DIV whose ISR performs its own DIV (and saves/restores W0, W1, RCOUNT, SR as
 * an ISR prologue does) must not disturb the foreground result: all inter-iteration state is architectural (FRM2 2.6.3). */
TEST(divide_interrupted_by_divide) {
  for (int at = 1; at <= 18; at++) {
    pic24_cpu *c = mk();
    /* INT0 (IRQ 0) vector -> ISR at 0x400 */
    c->prog[0x14 >> 1] = 0x000400;
    PUT(c, 0x400,
        0xF80000u | 0x0036u,                     /* push RCOUNT */
        0x781F80u,                               /* push w0 */
        0x781F81u,                               /* push w1 */
        MOVLIT(30000, 8), MOVLIT(7, 9),
        REPEAT(17), DIVI(1, 0, 8, 0, 9),         /* div.uw w8,w9 */
        0xA90084u,                               /* bclr IFS0,#0 */
        0x78004Fu | (1u << 7),                   /* pop w1  = mov [--w15],w1 */
        0x78004Fu,                               /* pop w0 */
        0xF90036u,                               /* pop RCOUNT */
        RETFIE_);
    PUT(c, 0x200, REPEAT(17), DIVI(0, 0, 4, 0, 6), NOP_, NOP_);
    c->ram[4] = (uint16_t)-12345; c->ram[6] = 77; c->pc = 0x200; c->ram[15] = 0x1000;
    c->iec[0] = 1; c->ipc[0] = 0x0001; /* INT0 priority 1 */
    int steps = 0;
    while (c->pc != 0x204 && steps < 200) {
      if (steps == at) pic24_set_irq(c, 0, true);
      pic24_step(c);
      steps++;
    }
    CHECK_EQ((int16_t)c->ram[0], -12345 / 77);
    CHECK_EQ((int16_t)c->ram[1], -12345 % 77);
    CHECK_EQ(c->ram[15], 0x1000);
    CHECK(c->ram[8] == 30000 && c->ram[9] == 7);
    free(c);
  }
}

/* ============================================================================================ REPEAT === */

TEST(repeat_semantics) {
  pic24_cpu *c = mk();
  /* REPEAT #2; INC W1,W1 -> 3 executions; RA during loop, cleared before the last (FRM2 Example 2-13) */
  PUT(c, 0x200, REPEAT(2), 0xE80000u | (1u << 7) | 1u, NOP_);
  c->pc = 0x200; c->ram[1] = 0;
  pic24_step(c);
  CHECK_EQ(c->rcount, 2); CHECK(c->sr & PIC24_SR_RA); CHECK_EQ(c->pc, 0x202);
  pic24_step(c);
  CHECK_EQ(c->rcount, 1); CHECK(c->sr & PIC24_SR_RA); CHECK_EQ(c->pc, 0x202);
  pic24_step(c);
  CHECK_EQ(c->rcount, 0); CHECK(!(c->sr & PIC24_SR_RA)); CHECK_EQ(c->pc, 0x202);
  pic24_step(c);
  CHECK_EQ(c->pc, 0x204); CHECK_EQ(c->ram[1], 3);
  /* REPEAT #0: like a NOP, RA never set, target executes once (PRM p.375) */
  PUT(c, 0x200, REPEAT(0), 0xE80000u | (1u << 7) | 1u, NOP_);
  c->pc = 0x200; c->ram[1] = 0;
  pic24_step(c);
  CHECK(!(c->sr & PIC24_SR_RA));
  pic24_step(c);
  CHECK_EQ(c->pc, 0x204); CHECK_EQ(c->ram[1], 1);
  /* REPEAT Wn uses Wn<13:0>: W2 = 0xC004 -> 5 executions */
  PUT(c, 0x200, REPEATW(2), 0xE80000u | (1u << 7) | 1u, NOP_);
  c->pc = 0x200; c->ram[1] = 0; c->ram[2] = 0xC004;
  while (c->pc != 0x204) pic24_step(c);
  CHECK_EQ(c->ram[1], 5);
  /* memcpy idiom: REPEAT #9; MOV.B [W1++],[W2++] copies 10 bytes */
  for (int i = 0; i < 10; i++) ((uint8_t *)c->ram)[0x900 + i] = (uint8_t)(0x40 + i);
  PUT(c, 0x200, REPEAT(9), e3(T_MOV, 1, 0, 3, 1, 3, 2), NOP_);
  c->pc = 0x200; c->ram[1] = 0x900; c->ram[2] = 0xA01;
  uint64_t c0 = c->cycles;
  while (c->pc != 0x204) pic24_step(c);
  CHECK_EQ(c->cycles - c0, 11u);
  CHECK_EQ(((uint8_t *)c->ram)[0xA01], 0x40); CHECK_EQ(((uint8_t *)c->ram)[0xA0A], 0x49);
  CHECK_EQ(c->ram[1], 0x90A); CHECK_EQ(c->ram[2], 0xA0B);
  free(c);
}

TEST(repeat_interrupted) {
  pic24_cpu *c = mk();
  c->prog[0x14 >> 1] = 0x000400;
  PUT(c, 0x400, 0xA90084u /* bclr IFS0,#0 */, RETFIE_);
  PUT(c, 0x200, REPEAT(9), 0xE80000u | (1u << 7) | 1u, NOP_);
  c->pc = 0x200; c->ram[1] = 0; c->iec[0] = 1; c->ipc[0] = 3;
  pic24_step(c); pic24_step(c); pic24_step(c);
  pic24_set_irq(c, 0, true);
  pic24_step(c); /* exception entry: RA stacked, RA cleared in ISR */
  CHECK_EQ(c->pc, 0x400);
  CHECK(!(c->sr & PIC24_SR_RA));
  CHECK_EQ(pic24_cpu_ipl(c), 3u);
  pic24_step(c); pic24_step(c); /* ISR */
  CHECK_EQ(c->pc, 0x202);
  CHECK(c->sr & PIC24_SR_RA);
  while (c->pc != 0x204) pic24_step(c);
  CHECK_EQ(c->ram[1], 10);
  free(c);
}

/* ================================================================================ addressing modes === */

TEST(addressing_modes) {
  pic24_cpu *c = mk();
  /* PRM Example 4-4 (p.57): MOV.B [W0++],[W13--] */
  c->ram[0] = 0x2300; c->ram[13] = 0x2708; c->ram[0x2300 >> 1] = 0x7783; c->ram[0x2708 >> 1] = 0x904E;
  one(c, e3(T_MOV, 1, 0, 3, 0, 2, 13));
  CHECK_EQ(c->ram[0], 0x2301); CHECK_EQ(c->ram[13], 0x2707); CHECK_EQ(c->ram[0x2708 >> 1], 0x9083);
  /* ADD W1,[--W5],[++W8] */
  c->ram[1] = 0x0800; c->ram[5] = 0x2200; c->ram[8] = 0x2400; c->ram[0x21FE >> 1] = 0x7783; c->ram[0x2402 >> 1] = 0xAACC;
  one(c, e3(T_ADD, 0, 1, 4, 5, 5, 8));
  CHECK_EQ(c->ram[5], 0x21FE); CHECK_EQ(c->ram[8], 0x2402); CHECK_EQ(c->ram[0x2402 >> 1], 0x7F83);
  /* Example 4-5: MOV.B [W0+W1],[W7++] */
  c->ram[0] = 0x2300; c->ram[1] = 0x01FE; c->ram[7] = 0x1000; c->ram[0x24FE >> 1] = 0x7783; c->ram[0x1000 >> 1] = 0x11DC;
  one(c, e3(T_MOV, 1, 1, 6, 0, 3, 7));
  CHECK_EQ(c->ram[7], 0x1001); CHECK_EQ(c->ram[0x1000 >> 1], 0x1183);
  /* Example 4-6: MOV [W0+0x20],W1 ; MOV W4,[W8-0x300] */
  c->ram[0] = 0x1200; c->ram[0x1220 >> 1] = 0xFD27;
  one(c, movoff(false, 0, 0x20 / 2, 1, 0));
  CHECK_EQ(c->ram[1], 0xFD27);
  c->ram[4] = 0x3411; c->ram[8] = 0x2944;
  one(c, movoff(true, 0, -0x300 / 2, 8, 4));
  CHECK_EQ(c->ram[0x2644 >> 1], 0x3411);
  /* byte offset mode: MOV.B [W0-1],W2 (offset not scaled in byte mode) */
  c->ram[0] = 0x1221; c->ram[2] = 0xFFFF;
  one(c, movoff(false, 1, -1, 2, 0));
  CHECK_EQ(c->ram[2], 0xFF27);
  /* Example 4-8 (p.66): MOV.B W0,0x1001 via MOV.B WREG,f */
  c->ram[0] = 0x1234; c->ram[0x1000 >> 1] = 0x5555;
  one(c, ef(0xB7, 1, 1, 1, 0x1001));
  CHECK_EQ(c->ram[0x1000 >> 1], 0x3455);
  /* MOV f,Wnd / MOV Wns,f across all of data space */
  c->ram[0x7FFE >> 1] = 0xBEEF;
  one(c, MOVF_W(0x7FFE, 3));
  CHECK_EQ(c->ram[3], 0xBEEF);
  one(c, MOVW_F(3, 0x0802));
  CHECK_EQ(c->ram[0x802 >> 1], 0xBEEF);
  /* W registers are memory mapped: MOV [W1],W2 with W1 = &W3 */
  c->ram[1] = 6; c->ram[3] = 0x4242;
  one(c, e3(T_MOV, 0, 0, 1, 1, 0, 2));
  CHECK_EQ(c->ram[2], 0x4242);
  /* W15<0> is always clear */
  one(c, MOVLIT(0x0901, 15));
  CHECK_EQ(c->ram[15], 0x0900);
  /* MOV f{,WREG} sets N,Z; MOV WREG,f doesn't */
  c->ram[0x900 >> 1] = 0x8000; c->sr = 0;
  one(c, ef(0xBF, 1, 0, 0, 0x900));
  CHECK_EQ(c->ram[0], 0x8000); CHECK_EQ(flags(c), N_);
  free(c);
}

/* ===================================================================== byte ops / address error trap === */

TEST(odd_address_and_address_error) {
  pic24_cpu *c = mk();
  c->prog[0x08 >> 1] = 0x000300; /* address error trap vector */
  PUT(c, 0x300, NOP_);
  /* byte ops on odd addresses are fine (PRM 4.4) */
  c->ram[1] = 0x1001; c->ram[0x1000 >> 1] = 0x5555; c->ram[0] = 0x1234;
  one(c, e3(T_MOV, 1, 0, 0, 0, 3, 1)); /* MOV.B W0,[W1++] */
  CHECK_EQ(c->ram[0x1000 >> 1], 0x3455); CHECK_EQ(c->ram[1], 0x1002);
  CHECK(!(c->intcon1 & PIC24_INTCON1_ADDRERR));
  /* PRM Example 4-11 (p.70): MOV 0x1001,W0 -- encoding cannot express an odd f; use [W1] */
  c->ram[1] = 0x1001; c->ram[0x1000 >> 1] = 0x1234; c->ram[0] = 0x5555;
  one(c, e3(T_MOV, 0, 0, 1, 1, 0, 0));
  CHECK_EQ(c->ram[0], 0x1234); /* read completes with EA<0> cleared */
  CHECK(c->intcon1 & PIC24_INTCON1_ADDRERR);
  pic24_step(c); /* trap taken */
  CHECK_EQ(c->pc, 0x300); CHECK_EQ(pic24_cpu_ipl(c), 13u);
  /* misaligned write is inhibited: MOV W0,[W1] with W1 odd */
  c->intcon1 = 0; c->sr = 0; c->ipl3 = 0;
  c->ram[0] = 0x1234; c->ram[1] = 0x1001; c->ram[0x1000 >> 1] = 0x5555; c->ram[0x1002 >> 1] = 0x6666;
  one(c, e3(T_MOV, 0, 0, 0, 0, 1, 1));
  CHECK_EQ(c->ram[0x1000 >> 1], 0x5555); CHECK_EQ(c->ram[0x1002 >> 1], 0x6666);
  CHECK(c->intcon1 & PIC24_INTCON1_ADDRERR);
  /* Example 4-11 third case: MOV [W0],[W1++] with W0 odd: the write still happens, W1 incremented */
  c->intcon1 = 0;
  c->ram[0] = 0x1235; c->ram[1] = 0x1000; c->ram[0x1234 >> 1] = 0xAAAA;
  one(c, e3(T_MOV, 0, 0, 1, 0, 3, 1));
  CHECK_EQ(c->ram[0x1000 >> 1], 0xAAAA); CHECK_EQ(c->ram[1], 0x1002);
  CHECK(c->intcon1 & PIC24_INTCON1_ADDRERR);
  /* bit instruction, word mode, odd indirect EA -> address error (FRM8 8.2.2.3 item 2) */
  c->intcon1 = 0; c->ram[1] = 0x1001;
  one(c, 0xA00000u | (3u << 12) | (1u << 4) | 1u);
  CHECK(c->intcon1 & PIC24_INTCON1_ADDRERR);
  free(c);
}

/* ============================================================================ PSV / EDS data paging === */

TEST(psv_and_eds) {
  pic24_cpu *c = mk();
  c->prog[0x10010 >> 1] = 0xA1B2C3; /* program word at PC 0x010010 */
  /* PSV lower word: DSRPAG = 0x200 | (0x010010 >> 15) = 0x202, EA = 0x8000 | (0x010010 & 0x7FFF) = 0x8010 */
  c->dsrpag = 0x202; c->ram[1] = 0x8010;
  unsigned cyc = one(c, e3(T_MOV, 0, 0, 1, 1, 0, 2));
  CHECK_EQ(c->ram[2], 0xB2C3);
  CHECK_EQ(cyc, 2u); /* MOV + 1 (DS 4.3.3) */
  /* byte read of the odd byte */
  c->ram[1] = 0x8011;
  one(c, e3(T_MOV, 1, 0, 1, 1, 0, 2));
  CHECK_EQ(c->ram[2] & 0xFF, 0xB2);
  /* upper word (DSRPAG<8>=1): phantom byte reads 0 */
  c->dsrpag = 0x302; c->ram[1] = 0x8010;
  cyc = one(c, e3(T_ADD, 0, 3, 1, 1, 0, 2)); /* ADD W3,[W1],W2 with W3 = 0 */
  CHECK_EQ(c->ram[2], 0x00A1 + c->ram[3]);
  CHECK_EQ(cyc, 3u); /* non-MOV + 2 */
  /* EDS RAM: write through DSWPAG, read back through DSRPAG */
  c->dswpag = 2; c->dsrpag = 2; c->ram[1] = 0x8800; c->ram[4] = 0x1357;
  one(c, e3(T_MOV, 0, 0, 0, 4, 1, 1));
  CHECK_EQ(c->eds[(0x10800 - PIC24_EDS_RAM_BASE) >> 1], 0x1357);
  cyc = one(c, e3(T_MOV, 0, 0, 1, 1, 0, 5));
  CHECK_EQ(c->ram[5], 0x1357);
  CHECK_EQ(cyc, 2u);
  /* lower 32 KB ignores the page registers */
  c->dsrpag = 0; c->ram[1] = 0x7000; c->ram[0x7000 >> 1] = 0x2468;
  one(c, e3(T_MOV, 0, 0, 1, 1, 0, 5));
  CHECK_EQ(c->ram[5], 0x2468); CHECK(!(c->intcon1 & PIC24_INTCON1_ADDRERR));
  /* page 0 in the EDS window -> address error (DS Table 4-35 note 3) */
  c->ram[1] = 0x8000;
  one(c, e3(T_MOV, 0, 0, 1, 1, 0, 5));
  CHECK(c->intcon1 & PIC24_INTCON1_ADDRERR);
  /* RMW bit instruction on EDS writes with DSRPAG (DS Table 4-35 note 3) */
  c->intcon1 = 0; c->dsrpag = 1; c->dswpag = 2; c->ram[1] = 0x8000; c->eds[0] = 0;
  one(c, 0xA00000u | (5u << 12) | (1u << 4) | 1u); /* BSET [W1],#5 */
  CHECK_EQ(c->eds[0], 0x0020);
  free(c);
}

/* ========================================================================================= TBLRD/WT === */

static uint32_t g_tw_addr; static uint16_t g_tw_val, g_tw_mask; static bool g_tw_high;
static void tw_cb(void *ctx, pic24_cpu *cpu, uint32_t a, uint16_t v, uint16_t m, bool hi) {
  g_tw_addr = a; g_tw_val = v; g_tw_mask = m; g_tw_high = hi;
}

TEST(table_ops) {
  pic24_config cfg = {0};
  cfg.tblwt = tw_cb;
  pic24_cpu *c = calloc(1, sizeof *c);
  pic24_init(c, &cfg);
  c->prog[0x012340 >> 1] = 0x7F8E9D;
  c->tblpag = 0x01; c->ram[1] = 0x2340;
  unsigned cyc = one(c, 0xBA0000u | (2u << 7) | (1u << 4) | 1u); /* TBLRDL [W1],W2 */
  CHECK_EQ(c->ram[2], 0x8E9D); CHECK_EQ(cyc, 2u);
  one(c, 0xBA8000u | (2u << 7) | (1u << 4) | 1u); /* TBLRDH [W1],W2 */
  CHECK_EQ(c->ram[2], 0x007F);
  c->ram[2] = 0xFFFF;
  one(c, 0xBA0000u | (1u << 14) | (2u << 7) | (3u << 4) | 1u); /* TBLRDL.B [W1++],W2 : byte, +1 */
  CHECK_EQ(c->ram[2], 0xFF9D); CHECK_EQ(c->ram[1], 0x2341);
  one(c, 0xBA0000u | (1u << 14) | (2u << 7) | (1u << 4) | 1u); /* odd -> PM<15:8> */
  CHECK_EQ(c->ram[2], 0xFF8E);
  one(c, 0xBA8000u | (1u << 14) | (2u << 7) | (1u << 4) | 1u); /* TBLRDH.B odd -> 0 */
  CHECK_EQ(c->ram[2], 0xFF00);
  /* configuration space: DEVID at 0xFF0000 */
  c->tblpag = 0xFF; c->ram[1] = 0;
  one(c, 0xBA0000u | (2u << 7) | (1u << 4) | 1u);
  CHECK_EQ(c->ram[2], 0x410E);
  /* TBLWTL/H -> callback (NVM latches are a peripheral) */
  c->tblpag = 0x02; c->ram[1] = 0x0100; c->ram[3] = 0xCAFE;
  cyc = one(c, 0xBB0000u | (1u << 11) | (1u << 7) | 3u); /* TBLWTL W3,[W1] */
  CHECK_EQ(g_tw_addr, 0x020100u); CHECK_EQ(g_tw_val, 0xCAFE); CHECK_EQ(g_tw_mask, 0xFFFF); CHECK(!g_tw_high);
  CHECK_EQ(cyc, 2u);
  one(c, 0xBB8000u | (1u << 11) | (1u << 7) | 3u); /* TBLWTH W3,[W1] */
  CHECK_EQ(g_tw_val, 0x00FE); CHECK(g_tw_high);
  free(c);
}

/* ================================================================================ flow / cycles ===== */

TEST(branches_all_conditions) {
  /* every BRA cc over all 16 N/OV/Z/C combinations vs PRM Table 4-8 */
  pic24_cpu *c = mk();
  for (unsigned cc = 0; cc < 15; cc++) {
    for (unsigned f = 0; f < 16; f++) {
      bool C = f & 1, Z = f & 2, V = f & 4, N = f & 8, want;
      switch (cc) {
      case 0: want = V; break; case 1: want = C; break; case 2: want = Z; break; case 3: want = N; break;
      case 4: want = Z || (N && !V) || (!N && V); break; case 5: want = (N && !V) || (!N && V); break;
      case 6: want = !C || Z; break; case 7: want = true; break; case 8: want = !V; break; case 9: want = !C; break;
      case 10: want = !Z; break; case 11: want = !N; break; case 12: want = (!Z && N && V) || (!Z && !N && !V); break;
      case 13: want = (N && V) || (!N && !V); break; default: want = C && !Z; break;
      }
      c->sr = (uint16_t)f;
      unsigned cyc = one(c, BRA(cc, 5));
      CHECK_EQ(c->pc, want ? 0x200u + 2u + 10u : 0x202u);
      CHECK_EQ(cyc, want ? 2u : 1u);
    }
  }
  free(c);
}

TEST(call_return_stack) {
  pic24_cpu *c = mk();
  /* CALL 0x012340 at 0x01FFFC: returns to 0x020000 (PC<22:16> stacked) */
  c->prog[0x1FFFC >> 1] = 0x020000u | 0x2340u;
  c->prog[0x1FFFE >> 1] = 0x000001u;
  c->prog[0x12340 >> 1] = RETURN_;
  c->pc = 0x1FFFC; c->ram[15] = 0x0900;
  uint64_t c0 = c->cycles;
  pic24_step(c);
  CHECK_EQ(c->pc, 0x012340u); CHECK_EQ(c->cycles - c0, 2u);
  CHECK_EQ(c->ram[0x900 >> 1], 0x0000); CHECK_EQ(c->ram[0x902 >> 1], 0x0002); CHECK_EQ(c->ram[15], 0x904);
  c0 = c->cycles;
  pic24_step(c);
  CHECK_EQ(c->pc, 0x020000u); CHECK_EQ(c->cycles - c0, 3u); CHECK_EQ(c->ram[15], 0x900);
  /* RCALL backwards, RETLW.B */
  PUT(c, 0x300, 0x050000u | (1u << 14) | (0x5Au << 4) | 3u); /* RETLW.B #0x5A,W3 */
  PUT(c, 0x400, 0x070000u | (uint16_t)(-((0x402 - 0x300) / 2)));
  c->pc = 0x400; c->ram[3] = 0x1200;
  pic24_step(c);
  CHECK_EQ(c->pc, 0x300u);
  pic24_step(c);
  CHECK_EQ(c->pc, 0x402u); CHECK_EQ(c->ram[3], 0x125A);
  /* GOTO Wn / CALL Wn / BRA Wn */
  c->ram[4] = 0x0601;
  one(c, 0x014004u);
  CHECK_EQ(c->pc, 0x600u);
  c->ram[4] = (uint16_t)-3;
  unsigned cyc = one(c, 0x016004u); /* BRA W4: PC+2 + 2*(-3) */
  CHECK_EQ(c->pc, 0x1FCu); CHECK_EQ(cyc, 2u);
  free(c);
}

TEST(skips_and_cycles) {
  pic24_cpu *c = mk();
  /* BTSC f over a 2-word GOTO: 3 cycles (PRM 3.2.1) */
  c->ram[0x900 >> 1] = 0x0000;
  PUT(c, 0x200, 0xAF0000u | (0x900u & 0x1FFEu) | ((5u & 7u) << 13), 0x040000u | 0x1234u, 0x000000u, NOP_);
  c->pc = 0x200;
  uint64_t c0 = c->cycles;
  pic24_step(c);
  CHECK_EQ(c->pc, 0x206u); CHECK_EQ(c->cycles - c0, 3u);
  /* BTSS Ws over 1-word: 2 cycles */
  c->ram[1] = 0x8000;
  PUT(c, 0x200, 0xA60000u | (15u << 12) | 1u, NOP_, NOP_);
  c->pc = 0x200; c0 = c->cycles;
  pic24_step(c);
  CHECK_EQ(c->pc, 0x204u); CHECK_EQ(c->cycles - c0, 2u);
  /* not skipped: 1 cycle */
  c->ram[1] = 0;
  c->pc = 0x200; c0 = c->cycles;
  pic24_step(c);
  CHECK_EQ(c->pc, 0x202u); CHECK_EQ(c->cycles - c0, 1u);
  /* CPSEQ / CPSGT (signed) / CPSLT.B */
  c->ram[1] = 0xFFFF; c->ram[2] = 1;
  PUT(c, 0x200, 0xE60000u | (1u << 11) | 2u, NOP_, NOP_); /* CPSGT W1,W2: -1 > 1? no */
  c->pc = 0x200; pic24_step(c); CHECK_EQ(c->pc, 0x202u);
  PUT(c, 0x200, 0xE68000u | (1u << 11) | 2u, NOP_, NOP_); /* CPSLT: yes */
  c->pc = 0x200; pic24_step(c); CHECK_EQ(c->pc, 0x204u);
  c->ram[1] = 0x12AB; c->ram[2] = 0x34AB;
  PUT(c, 0x200, 0xE78000u | (1u << 11) | (1u << 10) | 2u, NOP_, NOP_); /* CPSEQ.B: equal low bytes */
  c->pc = 0x200; pic24_step(c); CHECK_EQ(c->pc, 0x204u);
  /* RAW stall: MOV #x,W2 then MOV [W2],W3 -> +1 cycle (PRM Table 4-6) */
  c->ram[0x900 >> 1] = 0x77;
  PUT(c, 0x200, MOVLIT(0x900, 2), e3(T_MOV, 0, 0, 1, 2, 0, 3), e3(T_MOV, 0, 0, 1, 2, 0, 4));
  c->pc = 0x200; c0 = c->cycles;
  pic24_step(c); pic24_step(c);
  CHECK_EQ(c->cycles - c0, 3u);
  c0 = c->cycles; pic24_step(c);
  CHECK_EQ(c->cycles - c0, 1u); /* W2 not written by the previous instruction */
  CHECK_EQ(c->ram[3], 0x77);
  /* MOV.D / PUSH.D / POP.D: 2 cycles */
  c->ram[6] = 0x1111; c->ram[7] = 0x2222; c->ram[15] = 0x0A00;
  unsigned cyc = one(c, 0xBE9F80u | 6u); /* PUSH.D W6 */
  CHECK_EQ(cyc, 2u); CHECK_EQ(c->ram[0xA00 >> 1], 0x1111); CHECK_EQ(c->ram[0xA02 >> 1], 0x2222);
  CHECK_EQ(c->ram[15], 0xA04);
  one(c, 0xBE004Fu | (4u << 8)); /* POP.D W8 (ddd = 4 -> W8) */
  CHECK_EQ(c->ram[8], 0x1111); CHECK_EQ(c->ram[9], 0x2222); CHECK_EQ(c->ram[15], 0xA00);
  free(c);
}

/* ======================================================================================= interrupts === */

TEST(nested_interrupts_and_retfie) {
  pic24_cpu *c = mk();
  /* T1 (IRQ 3, vector 0x1A) priority 2 -> ISR A at 0x400; U1RX (IRQ 11, vector 0x2A) priority 5 -> ISR B */
  c->prog[0x1A >> 1] = 0x000400; c->prog[0x2A >> 1] = 0x000500;
  PUT(c, 0x400, NOP_, NOP_, 0xA90084u | (3u & 7u) << 13, RETFIE_); /* bclr IFS0,#3 */
  PUT(c, 0x500, 0xA90084u | (11u & 7u) << 13 | 1u, RETFIE_);        /* bclr IFS0,#11 */
  PUT(c, 0x200, NOP_, NOP_, NOP_, NOP_, NOP_, NOP_);
  c->iec[0] = (1u << 3) | (1u << 11);
  c->ipc[0] = 0x2000; /* T1IP = 2 (IPC0<14:12>) */
  c->ipc[2] = 0x5000; /* U1RXIP = 5 (IPC2<14:12>) */
  c->pc = 0x200; c->ram[15] = 0x0800; c->sr = PIC24_SR_C | PIC24_SR_N; /* flags to be preserved */
  pic24_set_irq(c, 3, true);
  uint64_t c0 = c->cycles;
  pic24_step(c);
  CHECK_EQ(c->pc, 0x400u); CHECK_EQ(pic24_cpu_ipl(c), 2u); CHECK_EQ(c->cycles - c0, 3u);
  CHECK_EQ(c->ram[0x800 >> 1], 0x0200); CHECK_EQ(c->ram[0x802 >> 1], (uint16_t)((PIC24_SR_C | PIC24_SR_N) << 8));
  pic24_step(c); /* NOP in A */
  c->sr |= PIC24_SR_Z;
  pic24_set_irq(c, 11, true); /* higher priority: nests */
  pic24_step(c);
  CHECK_EQ(c->pc, 0x500u); CHECK_EQ(pic24_cpu_ipl(c), 5u);
  pic24_step(c); pic24_step(c); /* bclr, retfie: back into A where B interrupted it */
  CHECK_EQ(c->pc, 0x402u); CHECK_EQ(pic24_cpu_ipl(c), 2u); CHECK(c->sr & PIC24_SR_Z);
  pic24_step(c); /* second NOP in A */
  /* a priority-2 request while at IPL 2 does not nest */
  pic24_set_irq(c, 3, true);
  pic24_step(c); /* bclr IFS0,#3 executes (clears it) */
  CHECK_EQ(c->pc, 0x406u);
  pic24_step(c); /* RETFIE */
  CHECK_EQ(c->pc, 0x200u); CHECK_EQ(pic24_cpu_ipl(c), 0u);
  CHECK_EQ(c->sr & 0xFF, PIC24_SR_C | PIC24_SR_N);
  CHECK_EQ(c->ram[15], 0x0800);
  free(c);
}

TEST(nstdis_altivt_disi) {
  pic24_cpu *c = mk();
  c->prog[0x1A >> 1] = 0x000400;  /* IVT T1 */
  c->prog[0x11A >> 1] = 0x000600; /* AIVT T1 */
  PUT(c, 0x200, NOP_, NOP_, NOP_, NOP_);
  c->iec[0] = 1u << 3; c->ipc[0] = 0x1000; c->pc = 0x200;
  c->intcon1 = PIC24_INTCON1_NSTDIS; c->intcon2 = PIC24_INTCON2_ALTIVT;
  pic24_set_irq(c, 3, true);
  pic24_step(c);
  CHECK_EQ(c->pc, 0x600u);
  CHECK_EQ(pic24_cpu_ipl(c), 7u); /* NSTDIS forces IPL 7 */
  /* DISI #2 blocks priority 1-6 for 3 cycles (incl. its own) but not level 7 */
  pic24_cpu *d = mk();
  d->prog[0x1A >> 1] = 0x000400;
  PUT(d, 0x200, 0xFC0002u, NOP_, NOP_, NOP_, NOP_);
  d->iec[0] = 1u << 3; d->ipc[0] = 0x6000; d->pc = 0x200;
  pic24_step(d); /* DISI */
  CHECK(d->intcon2 & PIC24_INTCON2_DISI);
  pic24_set_irq(d, 3, true);
  pic24_step(d); CHECK_EQ(d->pc, 0x204u);
  pic24_step(d); CHECK_EQ(d->pc, 0x206u);
  pic24_step(d); CHECK_EQ(d->pc, 0x400u); /* DISICNT reached 0 */
  CHECK(!(d->intcon2 & PIC24_INTCON2_DISI));
  free(d);
  free(c);
}

TEST(stack_error_trap) {
  pic24_cpu *c = mk();
  c->prog[0x0A >> 1] = 0x000300; /* stack error vector */
  PUT(c, 0x300, NOP_);
  /* SPLIM armed by a word write; push at EA == SPLIM is OK, the next push traps (DS 4.2.6) */
  PUT(c, 0x200, MOVLIT(0x0904, 0), MOVW_F(0, 0x0020), 0x781F80u, 0x781F80u, 0x781F80u, NOP_);
  c->pc = 0x200; c->ram[15] = 0x0900;
  pic24_step(c); pic24_step(c);
  CHECK(c->splim_armed); CHECK_EQ(c->splim, 0x0904);
  pic24_step(c); pic24_step(c); pic24_step(c); /* pushes at 0x900, 0x902, 0x904: fine */
  CHECK(!(c->intcon1 & PIC24_INTCON1_STKERR));
  PUT(c, 0x20A, 0x781F80u);
  pic24_step(c); /* push at EA 0x906 > SPLIM */
  CHECK(c->intcon1 & PIC24_INTCON1_STKERR);
  pic24_step(c);
  CHECK_EQ(c->pc, 0x300u); CHECK_EQ(pic24_cpu_ipl(c), 12u);
  /* underflow: POP with W15 = 0x0800 (EA 0x07FE < 0x0800) */
  pic24_cpu *d = mk();
  d->ram[15] = 0x0800;
  one(d, 0x78004Fu); /* pop w0 */
  CHECK(d->intcon1 & PIC24_INTCON1_STKERR);
  /* negative register offset into the frame is NOT a wrap (a negative Wb, PRM addressing modes) */
  pic24_cpu *e = mk();
  PUT(e, 0x200, MOVLIT(0x7FF0, 0), MOVW_F(0, 0x0020), NOP_);
  e->pc = 0x200; pic24_step(e); pic24_step(e);
  e->ram[15] = 0x6FC6; e->ram[2] = 0xFBD0; e->ram[4] = 0xBEEF;
  one(e, 0x793784u);                    /* mov w4, [w15+w2] */
  CHECK(!(e->intcon1 & PIC24_INTCON1_STKERR));
  CHECK_EQ(e->ram[0x6B96 >> 1], 0xBEEF);
  /* a positive register offset that really wraps past 0xFFFF still traps */
  e->ram[2] = 0x9100;                    /* +0x9100: 0x6FC6 + 0x9100 = 0x100C6 */
  one(e, 0x793784u);
  CHECK(e->intcon1 & PIC24_INTCON1_STKERR);
  free(e);
  free(d);
  free(c);
}

/* ===================================================================================== misc ops ===== */

TEST(misc_ops) {
  pic24_cpu *c = mk();
  /* PUSH.S / POP.S: W0-W3 + DC,N,OV,Z,C */
  c->ram[0] = 1; c->ram[1] = 2; c->ram[2] = 3; c->ram[3] = 4; c->sr = 0x011F & ~PIC24_SR_RA;
  one(c, 0xFEA000u);
  c->ram[0] = c->ram[1] = c->ram[2] = c->ram[3] = 0; c->sr = 0;
  one(c, 0xFE8000u);
  CHECK(c->ram[0] == 1 && c->ram[1] == 2 && c->ram[2] == 3 && c->ram[3] == 4);
  CHECK_EQ(c->sr, 0x010F);
  /* LNK #4 / ULNK (PRM 4.8.2) */
  c->ram[14] = 0x1234; c->ram[15] = 0x0A00;
  one(c, 0xFA0004u);
  CHECK_EQ(c->ram[0xA00 >> 1], 0x1234); CHECK_EQ(c->ram[14], 0x0A02); CHECK_EQ(c->ram[15], 0x0A06);
  one(c, 0xFA8000u);
  CHECK_EQ(c->ram[14], 0x1234); CHECK_EQ(c->ram[15], 0x0A00);
  /* SE / ZE flags (p.406, 456) */
  c->ram[1] = 0x1280;
  one(c, 0xFB0000u | (2u << 7) | 1u);
  CHECK_EQ(c->ram[2], 0xFF80); CHECK_EQ(flags(c) & (N_ | Z_ | C_), N_);
  one(c, 0xFB8000u | (2u << 7) | 1u);
  CHECK_EQ(c->ram[2], 0x0080); CHECK_EQ(flags(c) & (N_ | Z_ | C_), C_);
  c->ram[1] = 0x1200;
  one(c, 0xFB0000u | (2u << 7) | 1u);
  CHECK_EQ(flags(c) & (N_ | Z_ | C_), Z_ | C_);
  /* FF1L / FF1R (p.257, 259) */
  c->ram[1] = 0x0400;
  one(c, 0xCF8000u | (2u << 7) | 1u);
  CHECK_EQ(c->ram[2], 6); CHECK(!(c->sr & C_));
  one(c, 0xCF0000u | (2u << 7) | 1u);
  CHECK_EQ(c->ram[2], 11);
  c->ram[1] = 0;
  one(c, 0xCF0000u | (2u << 7) | 1u);
  CHECK_EQ(c->ram[2], 0); CHECK(c->sr & C_);
  /* EXCH / SWAP / SWAP.B */
  c->ram[1] = 0x1234; c->ram[2] = 0xABCD;
  one(c, 0xFD0000u | (2u << 7) | 1u);
  CHECK(c->ram[1] == 0xABCD && c->ram[2] == 0x1234);
  one(c, 0xFD8000u | 1u);
  CHECK_EQ(c->ram[1], 0xCDAB);
  one(c, 0xFDC000u | 1u);
  CHECK_EQ(c->ram[1], 0xCDBA);
  /* BSW examples (PRM p.163-164) */
  c->ram[2] = 0xF234; c->ram[3] = 0x111F; c->sr = Z_;
  one(c, 0xAD0000u | (3u << 11) | 2u);
  CHECK_EQ(c->ram[2], 0x7234);
  c->ram[2] = 0xE235; c->ram[3] = 0x0550; c->sr = Z_;
  one(c, 0xAD8000u | (3u << 11) | 2u);
  CHECK_EQ(c->ram[2], 0xE234);
  c->ram[0] = 0x1000; c->ram[6] = 0x34A3; c->ram[0x1002 >> 1] = 0x2380; c->sr = C_;
  one(c, 0xAD0000u | (6u << 11) | (5u << 4) | 0u); /* BSW.C [++W0],W6 */
  CHECK_EQ(c->ram[0], 0x1002); CHECK_EQ(c->ram[0x1002 >> 1], 0x2388);
  /* BTST.Z / BTST.C / BTSTS.C / BTST f / BTSTS f */
  c->ram[1] = 0x0008; c->sr = 0;
  one(c, 0xA30800u | (3u << 12) | 1u);
  CHECK(!(c->sr & Z_));
  one(c, 0xA30000u | (2u << 12) | 1u);
  CHECK(!(c->sr & C_));
  one(c, 0xA40000u | (2u << 12) | 1u); /* BTSTS.C W1,#2 */
  CHECK(!(c->sr & C_)); CHECK_EQ(c->ram[1], 0x000C);
  c->ram[0x900 >> 1] = 0x0000;
  one(c, 0xAC0000u | 0x900u | ((9u & 7u) << 13) | (9u >> 3)); /* BTSTS 0x900,#9 */
  CHECK(c->sr & Z_); CHECK_EQ(c->ram[0x900 >> 1], 0x0200);
  /* bit ops on file register (word RMW): BSET/BCLR/BTG 0x900,#15 */
  one(c, 0xA80000u | 0x900u | (7u << 13) | 1u);
  CHECK_EQ(c->ram[0x900 >> 1], 0x8200);
  one(c, 0xAA0000u | 0x900u | (1u << 13) | 1u); /* BTG #9 */
  CHECK_EQ(c->ram[0x900 >> 1], 0x8000);
  /* BCLR.B [W1],#3 on odd byte */
  c->ram[1] = 0x0901; c->ram[0x900 >> 1] = 0xFFFF;
  one(c, 0xA10000u | (3u << 12) | (1u << 10) | (1u << 4) | 1u);
  CHECK_EQ(c->ram[0x900 >> 1], 0xF7FF);
  /* DAW.B (p.225): 0x19 + 0x28 = 0x41 with DC -> 0x47 */
  c->ram[1] = 0x0041; c->sr = DC_;
  one(c, 0xFD4000u | 1u);
  CHECK_EQ(c->ram[1], 0x0047); CHECK(!(c->sr & C_));
  c->ram[1] = 0x009A; c->sr = 0;
  one(c, 0xFD4000u | 1u);
  CHECK_EQ(c->ram[1] & 0xFF, 0x00); CHECK(c->sr & C_);
  /* PUSH f / POP f */
  c->ram[0x900 >> 1] = 0x5A5A; c->ram[15] = 0xA00;
  one(c, 0xF80900u);
  one(c, 0xF90902u);
  CHECK_EQ(c->ram[0x902 >> 1], 0x5A5A); CHECK_EQ(c->ram[15], 0xA00);
  /* MOV.D [W1++],W4 and MOV.D W4,[W2] */
  c->ram[1] = 0x900; c->ram[0x900 >> 1] = 0xA319; c->ram[0x902 >> 1] = 0x9927;
  one(c, 0xBE0000u | (2u << 8) | (3u << 4) | 1u);
  CHECK(c->ram[4] == 0xA319 && c->ram[5] == 0x9927 && c->ram[1] == 0x904);
  c->ram[2] = 0xA10;
  one(c, 0xBE8000u | (1u << 11) | (2u << 7) | 4u);
  CHECK(c->ram[0xA10 >> 1] == 0xA319 && c->ram[0xA12 >> 1] == 0x9927);
  free(c);
}

TEST(pwrsav_and_wake) {
  pic24_cpu *c = mk();
  c->prog[0x1A >> 1] = 0x000400;
  PUT(c, 0x200, 0xFE4001u, NOP_);
  c->pc = 0x200; c->iec[0] = 1u << 3; c->ipc[0] = 0x1000;
  pic24_step(c);
  CHECK_EQ(c->power, PIC24_IDLE);
  CHECK_EQ(pic24_step(c), PIC24_STOP_SLEEP);
  pic24_set_irq(c, 3, true);
  pic24_step(c);
  CHECK_EQ(c->pc, 0x400u); CHECK_EQ(c->power, PIC24_RUNNING);
  /* wake with priority <= IPL: continue after PWRSAV */
  pic24_cpu *d = mk();
  PUT(d, 0x200, 0xFE4000u, NOP_);
  d->pc = 0x200; d->iec[0] = 1u << 3; d->ipc[0] = 0x1000; d->sr = 0x00E0; /* IPL 7 */
  pic24_step(d);
  pic24_set_irq(d, 3, true);
  pic24_step(d); /* wakes and executes the instruction after PWRSAV */
  CHECK_EQ(d->pc, 0x204u); CHECK_EQ(d->power, PIC24_RUNNING);
  free(d);
  free(c);
}

/* Decoder test: these encodings must decode as illegal on a PIC24F. Uses the debug knob (cfg.illegal_stops) so the
 * opcode is identified by the stop status with nothing else changed; the silicon reset is tested below. */
TEST(illegal_opcodes) {
  pic24_config cfg = {0};
  cfg.illegal_stops = true;
  pic24_cpu *c = calloc(1, sizeof *c);
  pic24_init(c, &cfg);
  const uint32_t ill[] = {0x080000u /* DO */, 0xC30000u /* CLR A (DSP) */, 0xFEC000u /* MOVPAG */, 0x0C0000u /* BRA OA */,
                          0xD90000u /* DIVF */, 0xDF0000u /* FBCL */, 0x3F0000u, 0xDA4000u /* BREAK, no debugger */,
                          0xDA4001u /* not BREAK */};
  for (size_t i = 0; i < sizeof ill / sizeof ill[0]; i++) {
    c->prog[0x100] = ill[i]; c->pc = 0x200;
    CHECK_EQ(pic24_step(c), PIC24_STOP_ILLEGAL);
    CHECK_EQ(c->pc, 0x200u);
    CHECK_EQ(c->stop_insn, ill[i]);
  }
  CHECK_EQ(c->resets, 0u);
  free(c);
}

/* on_reset recorder: the host side of a core-performed reset (RCON lives with the host). */
typedef struct { unsigned n; pic24_status why; uint32_t pc_at_cb; uint16_t rcon; } rst_log;
static void rst_cb(void *ctx, pic24_cpu *c, pic24_status why) {
  rst_log *l = ctx;
  l->n++; l->why = why; l->pc_at_cb = c->pc;
  if (why == PIC24_STOP_ILLEGAL) l->rcon |= 0x4000u;      /* IOPUWR (DS39969 Register 6-1) */
  if (why == PIC24_STOP_RESET) l->rcon |= 0x0040u;        /* SWR */
}
static pic24_cpu *mk_rst(rst_log *l, bool debugger) {
  pic24_config cfg = {0};
  cfg.ctx = l; cfg.on_reset = rst_cb; cfg.debugger = debugger;
  pic24_cpu *c = calloc(1, sizeof *c);
  pic24_init(c, &cfg);
  return c;
}
/* dirty the core so a reset is visible: W0, W15, SR, IPL, interrupt enables, a REPEAT count, PSV page */
static void dirty(pic24_cpu *c) {
  c->ram[0] = 0x1234; c->ram[15] = 0x1000; c->sr = PIC24_SR_C | PIC24_SR_Z | 0x00E0u; c->iec[0] = 0xFFFF;
  c->rcount = 7; c->dsrpag = 0x200; c->intcon1 = PIC24_INTCON1_NSTDIS; c->splim = 0x2000;
}
static void check_reset_state(pic24_cpu *c) {
  CHECK_EQ(c->pc, 0u);                 /* reset vector (DS39969 4.1: GOTO at 0x000000) */
  CHECK_EQ(c->ram[0], 0u);
  CHECK_EQ(c->ram[15], 0x0800u);       /* W15 = 0x0800 at reset (DS Table 4-4) */
  CHECK_EQ(c->sr, 0u);
  CHECK_EQ(c->iec[0], 0u);
  CHECK_EQ(c->rcount, 0u);
  CHECK_EQ(c->dsrpag, 1u);
  CHECK_EQ(c->intcon1, 0u);
  CHECK_EQ(c->splim, 0u);
}

/* Silicon: an illegal opcode resets the device with RCON.IOPUWR set (DS39969 Register 6-1), through the same path as
 * the RESET instruction: core registers to reset values, PC at the reset vector, host told the cause. */
TEST(illegal_opcode_resets) {
  rst_log l = {0};
  pic24_cpu *c = mk_rst(&l, false);
  c->prog[0x100] = 0xD90000u; /* DIVF: dsPIC only */
  c->pc = 0x200; dirty(c);
  CHECK_EQ(pic24_step(c), PIC24_STOP_RESET);
  check_reset_state(c);
  CHECK_EQ(l.n, 1u); CHECK_EQ(l.why, PIC24_STOP_ILLEGAL); CHECK_EQ(l.pc_at_cb, 0u);
  CHECK(l.rcon & 0x4000u);             /* IOPUWR */
  CHECK(!(l.rcon & 0x0040u));          /* not SWR */
  CHECK_EQ(c->last_reset, PIC24_STOP_ILLEGAL); CHECK_EQ(c->last_reset_pc, 0x200u); CHECK_EQ(c->resets, 1u);
  CHECK_EQ(c->stop_insn, 0xD90000u);
  /* illegal address mode (DS: "an illegal address mode ... caused a Reset"): ADD with destination mode qqq = 110,
   * which PRM Table 5-3 says forces a RESET */
  c->prog[0x100] = e3(T_ADD, 0, 1, 0, 2, 6, 3); c->pc = 0x200; dirty(c);
  CHECK_EQ(pic24_step(c), PIC24_STOP_RESET);
  check_reset_state(c);
  CHECK_EQ(l.n, 2u); CHECK_EQ(l.why, PIC24_STOP_ILLEGAL);
  /* after the reset, execution continues from the reset vector: GOTO 0x200 at 0x000000 runs normally */
  c->prog[0] = 0x040200u; c->prog[1] = 0;
  c->prog[0x100] = NOP_;
  CHECK_EQ(pic24_step(c), PIC24_OK); CHECK_EQ(c->pc, 0x200u);
  CHECK_EQ(pic24_step(c), PIC24_OK); CHECK_EQ(c->pc, 0x202u);
  /* the RESET instruction takes the same path, cause SWR */
  c->prog[0x101] = 0xFE0000u;
  CHECK_EQ(pic24_step(c), PIC24_STOP_RESET);
  CHECK_EQ(c->pc, 0u); CHECK_EQ(l.why, PIC24_STOP_RESET); CHECK(l.rcon & 0x0040u);
  CHECK_EQ(c->last_reset_pc, 0x202u); CHECK_EQ(c->resets, 3u);
  free(c);
}

/* BREAK (0xDA4000) is a debugger breakpoint, not in the user instruction set (DS70000157G). Without a debugger it is
 * an illegal opcode -> reset with IOPUWR (INFERRED-high); with cfg.debugger it halts with PIC24_STOP_BREAK. */
TEST(break_instruction) {
  rst_log l = {0};
  pic24_cpu *c = mk_rst(&l, false);
  c->prog[0x100] = 0xDA4000u; c->pc = 0x200; dirty(c);
  CHECK_EQ(pic24_step(c), PIC24_STOP_RESET);
  check_reset_state(c);
  CHECK_EQ(l.n, 1u); CHECK_EQ(l.why, PIC24_STOP_ILLEGAL); CHECK(l.rcon & 0x4000u);
  CHECK_EQ(c->last_reset_pc, 0x200u); CHECK_EQ(c->stop_insn, 0xDA4000u);
  /* pic24_run returns at the reset (the host restarts it) */
  c->prog[0] = 0x040200u; c->prog[1] = 0;
  CHECK_EQ(pic24_run(c, 1000), PIC24_STOP_RESET);
  CHECK_EQ(c->pc, 0u); CHECK_EQ(l.n, 2u);
  free(c);

  rst_log l2 = {0};
  pic24_cpu *d = mk_rst(&l2, true);  /* debugger attached */
  PUT(d, 0x200, 0xDA4000u, NOP_);
  d->pc = 0x200; d->ram[0] = 0x1234;
  uint64_t c0 = d->cycles;
  CHECK_EQ(pic24_step(d), PIC24_STOP_BREAK);
  CHECK_EQ(d->pc, 0x202u);           /* halted after the BREAK; resuming continues */
  CHECK_EQ(d->ram[0], 0x1234u);      /* no reset */
  CHECK_EQ(l2.n, 0u); CHECK_EQ(d->resets, 0u);
  CHECK_EQ(d->cycles - c0, 1u);
  CHECK_EQ(pic24_step(d), PIC24_OK); CHECK_EQ(d->pc, 0x204u);
  /* only the exact BREAK encoding: 0xDA4001 is illegal even with a debugger */
  d->prog[0x100] = 0xDA4001u; d->pc = 0x200;
  CHECK_EQ(pic24_step(d), PIC24_STOP_RESET);
  CHECK_EQ(l2.why, PIC24_STOP_ILLEGAL);
  free(d);
}

/* Undocumented corner the core must decide (a modelling decision): Wb is read before the source EA side effect. */
TEST(documented_choices) {
  pic24_cpu *c = mk();
  c->ram[1] = 0x0900; c->ram[0x900 >> 1] = 5;
  one(c, e3(T_ADD, 0, 1, 3, 1, 0, 2)); /* ADD W1,[W1++],W2 */
  CHECK_EQ(c->ram[2], 0x0905); CHECK_EQ(c->ram[1], 0x0902);
  /* PRM 4.3.1 note: register-offset destination with Ws == Wd uses old Ws, offset ignored */
  c->ram[1] = 0x0900; c->ram[2] = 0x0010; c->ram[0x900 >> 1] = 0x1111; c->ram[0x902 >> 1] = 0;
  one(c, e3(T_MOV, 0, 2, 3, 1, 6, 1)); /* MOV [W1++],[W1+W2] */
  CHECK_EQ(c->ram[0x900 >> 1], 0x1111);
  CHECK_EQ(c->ram[0x910 >> 1], 0);
  free(c);
}

int main(void) {
  RUN(add_flags);
  RUN(prm_example_4_15);
  RUN(addc_sticky_z_and_ov);
  RUN(sub_family);
  RUN(inc_dec);
  RUN(logic_nz_only);
  RUN(shifts_rotates);
  RUN(multiply);
  RUN(divide_against_reference);
  RUN(divide_edges);
  RUN(divide_by_zero_trap);
  RUN(divide_interrupted_by_divide);
  RUN(repeat_semantics);
  RUN(repeat_interrupted);
  RUN(addressing_modes);
  RUN(odd_address_and_address_error);
  RUN(psv_and_eds);
  RUN(table_ops);
  RUN(branches_all_conditions);
  RUN(call_return_stack);
  RUN(skips_and_cycles);
  RUN(nested_interrupts_and_retfie);
  RUN(nstdis_altivt_disi);
  RUN(stack_error_trap);
  RUN(misc_ops);
  RUN(pwrsav_and_wake);
  RUN(illegal_opcodes);
  RUN(illegal_opcode_resets);
  RUN(break_instruction);
  RUN(documented_choices);
  MT_REPORT();
}
