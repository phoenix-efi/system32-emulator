/*
 * pic24_core.c -- PIC24F CPU core (PIC24FJ256DA210). See include/pic24.h.
 *
 * Citations in comments: PRM = DS70000157G, DS = DS39969B, FRM2/FRM8/FRM45 = DS39703A/DS39707A/DS39733A.
 * "p.NNN" = PRM page.
 */
#include "pic24.h"

#include <string.h>

#define SR_C PIC24_SR_C
#define SR_Z PIC24_SR_Z
#define SR_OV PIC24_SR_OV
#define SR_N PIC24_SR_N
#define SR_RA PIC24_SR_RA
#define SR_IPL PIC24_SR_IPL
#define SR_DC PIC24_SR_DC
#define SR_ARITH (SR_C | SR_Z | SR_OV | SR_N | SR_DC)

/* per-instruction access flags */
enum { ACC_PSV = 1, ACC_EDS = 2 };

/* core SFR addresses (DS Table 4-4, 4-6) */
enum {
  A_SPLIM = 0x20, A_PCL = 0x2E, A_PCH = 0x30, A_DSRPAG = 0x32, A_DSWPAG = 0x34, A_RCOUNT = 0x36, A_SR = 0x42,
  A_CORCON = 0x44, A_DISICNT = 0x52, A_TBLPAG = 0x54, A_INTCON1 = 0x80, A_INTCON2 = 0x82, A_IFS0 = 0x84,
  A_IEC0 = 0x94, A_IPC0 = 0xA4, A_INTTREG = 0xE0
};

/* implemented-bit masks of the interrupt controller registers (impl_mask in model/pic24fj256da210_sfr.json, from
 * Microchip's EDC). IFSx and IECx share masks. IPC14/17/24 do not exist on this part. */
static const uint16_t IFS_IMPL[7] = {0x3FEF, 0xFEDF, 0x3FE3, 0x4066, 0x210E, 0x3FFE, 0x0010};
static const uint16_t IPC_IMPL[26] = {0x7777, 0x7770, 0x7777, 0x0077, 0x7777, 0x7707, 0x7770, 0x7777, 0x0077,
                                      0x7770, 0x7777, 0x0077, 0x0770, 0x0770, 0x0000, 0x0700, 0x7770, 0x0000,
                                      0x0007, 0x0070, 0x7770, 0x7777, 0x7777, 0x0077, 0x0000, 0x0007};

/* ======================================================================================== small helpers === */

static inline uint32_t prog_rd(const pic24_cpu *c, uint32_t pc) {
  pc &= 0xFFFFFEu;
  return pc < PIC24_PROG_TOP ? c->prog[pc >> 1] : 0u; /* unimplemented reads '0' (DS Fig 4-1) */
}

static inline unsigned cur_ipl(const pic24_cpu *c) { return ((unsigned)c->ipl3 << 3) | ((c->sr >> 5) & 7u); }

static inline void set_w(pic24_cpu *c, unsigned n, uint16_t v) {
  if (n == 15) v &= 0xFFFEu; /* W15<0> always clear (PRM 4.8.1 note) */
  c->ram[n] = v;
}

/* ============================================================================= exception arbitration === */

static const struct {
  uint16_t bit;
  uint8_t level, vec;
} TRAPS[] = {
    /* FRM8 8.2: oscillator fail 14, address error 13, stack error 12, math error 11; DS Table 7-1 vectors */
    {PIC24_INTCON1_OSCFAIL, 14, 1},
    {PIC24_INTCON1_ADDRERR, 13, 2},
    {PIC24_INTCON1_STKERR, 12, 3},
    {PIC24_INTCON1_MATHERR, 11, 4},
};

static void arbitrate(pic24_cpu *c) {
  int va = -1, la = 0, v7 = -1, l7 = 0;
  for (unsigned t = 0; t < 4; t++) {
    if (c->intcon1 & TRAPS[t].bit) {
      va = v7 = TRAPS[t].vec;
      la = l7 = TRAPS[t].level;
      break;
    }
  }
  if (va < 0) {
    for (unsigned w = 0; w < 7; w++) {
      uint16_t p = c->ifs[w] & c->iec[w];
      while (p) {
        unsigned b = (unsigned)__builtin_ctz(p);
        p &= (uint16_t)(p - 1);
        unsigned irq = w * 16 + b;
        if (irq / 4 >= 26) continue;
        int lvl = (c->ipc[irq / 4] >> ((irq & 3) * 4)) & 7;
        if (lvl == 0) continue; /* priority 0 = disabled (FRM8 8.2.5 note) */
        /* natural order: lower IRQ wins ties -> strict '>' while scanning ascending (FRM8 8.1.2) */
        if (lvl > la) { la = lvl; va = 8 + (int)irq; }
        if (lvl >= 7 && lvl > l7) { l7 = lvl; v7 = 8 + (int)irq; }
      }
    }
  }
  c->pend_vec_all = va;
  c->pend_lvl_all = la;
  c->pend_vec_ge7 = v7;
  c->pend_lvl_ge7 = l7;
  c->pend_valid = true;
}

/* Is an exception able to interrupt right now?  Returns vector index or -1. */
static inline int exception_ready(pic24_cpu *c, int *level) {
  if (!c->pend_valid) arbitrate(c);
  unsigned ipl = cur_ipl(c);
  /* DISI blocks priority 1..6 only (FRM8 8.2.3) */
  int v = c->disicnt ? c->pend_vec_ge7 : c->pend_vec_all;
  int l = c->disicnt ? c->pend_lvl_ge7 : c->pend_lvl_all;
  if (v >= 0 && l > (int)ipl) {
    *level = l;
    return v;
  }
  return -1;
}

/* ================================================================================== data memory access === */

static void raise_addrerr(pic24_cpu *c) {
  c->intcon1 |= PIC24_INTCON1_ADDRERR;
  c->hard_trap_pending = 1;
  c->pend_valid = false;
}

/* --- core SFRs ---------------------------------------------------------------------------------------- */
static bool core_sfr_get(const pic24_cpu *c, uint16_t a, uint16_t *v) {
  switch (a) {
  case A_SPLIM: *v = c->splim; return true;
  case A_PCL: *v = (uint16_t)c->pc; return true;
  case A_PCH: *v = (uint16_t)((c->pc >> 16) & 0x7F); return true;
  case A_DSRPAG: *v = c->dsrpag; return true;
  case A_DSWPAG: *v = c->dswpag; return true;
  case A_RCOUNT: *v = c->rcount; return true;
  case A_SR: *v = c->sr; return true;
  case A_CORCON: *v = (uint16_t)((c->ipl3 << 3) | 0x0004u); return true; /* bit2 reserved, reads 1 (DS Reg 3-2) */
  case A_DISICNT: *v = c->disicnt; return true;
  case A_TBLPAG: *v = c->tblpag; return true;
  case A_INTCON1: *v = c->intcon1; return true;
  case A_INTCON2: *v = c->intcon2; return true;
  case A_INTTREG: *v = c->inttreg; return true;
  default: break;
  }
  if (a >= A_IFS0 && a < A_IFS0 + 14) { *v = c->ifs[(a - A_IFS0) >> 1]; return true; }
  if (a >= A_IEC0 && a < A_IEC0 + 14) { *v = c->iec[(a - A_IEC0) >> 1]; return true; }
  if (a >= A_IPC0 && a < A_IPC0 + 52) { *v = c->ipc[(a - A_IPC0) >> 1]; return true; }
  /* unimplemented words inside the core / interrupt blocks read 0 */
  if (a < 0x56 && a >= 0x20) { *v = 0; return true; } /* core block ends at TBLPAG; ICN starts 0x56 (DS Tab 4-5) */
  if (a >= 0x80 && a < 0xE2) { *v = 0; return true; }
  return false;
}

/* write `v` under lane mask `m` */
static bool core_sfr_put(pic24_cpu *c, uint16_t a, uint16_t v, uint16_t m) {
  uint16_t old;
  if (!core_sfr_get(c, a, &old)) return false;
  uint16_t n = (uint16_t)((old & ~m) | (v & m));
  switch (a) {
  case A_SPLIM:
    c->splim = n & 0xFFFEu;
    if (m == 0xFFFF) c->splim_armed = true; /* "until a word write to SPLIM occurs" (PRM 4.8.3) */
    return true;
  case A_PCL: case A_PCH: return true; /* read-only view of the PC in this model (modelling decision) */
  case A_DSRPAG: c->dsrpag = n & 0x3FFu; return true;
  case A_DSWPAG: c->dswpag = n & 0x1FFu; return true;
  case A_RCOUNT: c->rcount = n & 0x3FFFu; return true;
  case A_SR: {
    uint16_t keep = SR_RA; /* RA read-only (DS Reg 3-1) */
    if (c->intcon1 & PIC24_INTCON1_NSTDIS) keep |= SR_IPL; /* IPL read-only when NSTDIS=1 */
    c->sr = (uint16_t)((c->sr & keep) | (n & 0x01FFu & ~keep));
    c->pend_valid = false;
    return true;
  }
  case A_CORCON:
    /* IPL3 is clear-only from software (DS Reg 3-2 "R/C") */
    if (!(n & 0x0008u)) c->ipl3 = 0;
    c->pend_valid = false;
    return true;
  case A_DISICNT:
    /* writable; extending only meaningful while DISI is active (FRM8 8.2.3) */
    c->disicnt = n & 0x3FFFu;
    if (!c->disicnt) c->intcon2 &= (uint16_t)~PIC24_INTCON2_DISI;
    return true;
  case A_TBLPAG: c->tblpag = n & 0xFFu; return true;
  case A_INTCON1: c->intcon1 = n & 0x801Eu; c->pend_valid = false; return true;
  case A_INTCON2:
    c->intcon2 = (uint16_t)((c->intcon2 & PIC24_INTCON2_DISI) | (n & 0x801Fu)); /* DISI bit read-only */
    c->pend_valid = false;
    return true;
  case A_INTTREG: c->inttreg = (uint16_t)((c->inttreg & ~0x2000u) | (n & 0x2000u)); return true; /* VHOLD only */
  default: break;
  }
  if (a >= A_IFS0 && a < A_IFS0 + 14) { unsigned i = (a - A_IFS0) >> 1; c->ifs[i] = n & IFS_IMPL[i]; c->pend_valid = false; return true; }
  if (a >= A_IEC0 && a < A_IEC0 + 14) { unsigned i = (a - A_IEC0) >> 1; c->iec[i] = n & IFS_IMPL[i]; c->pend_valid = false; return true; }
  if (a >= A_IPC0 && a < A_IPC0 + 52) { unsigned i = (a - A_IPC0) >> 1; c->ipc[i] = n & IPC_IMPL[i]; c->pend_valid = false; return true; }
  return true; /* unimplemented: write ignored */
}

/* Write log (pic24_wlog_start): the first write to a word records the value it held. Keys: ram[] index, or WLOG_EDS +
 * eds[] index. */
#define WLOG_EDS (PIC24_DATA_LOW_TOP / 2u)
static inline void wlog(pic24_cpu *c, uint32_t key, uint16_t old) {
  if (!c->wlog_on) return;
  for (unsigned i = 0; i < c->wlog_n; i++) if (c->wlog_key[i] == key) return;
  if (c->wlog_n == PIC24_WLOG) { c->wlog_full = true; return; }
  c->wlog_key[c->wlog_n] = key; c->wlog_old[c->wlog_n] = old; c->wlog_n++;
}
void pic24_wlog_start(pic24_cpu *c) { c->wlog_on = true; c->wlog_full = false; c->wlog_n = 0; }
void pic24_wlog_stop(pic24_cpu *c) { c->wlog_on = false; c->wlog_full = false; c->wlog_n = 0; }
bool pic24_wlog_restored(const pic24_cpu *c) {
  if (c->wlog_full) return false;
  for (unsigned i = 0; i < c->wlog_n; i++) {
    uint32_t k = c->wlog_key[i];
    uint16_t now = k < WLOG_EDS ? c->ram[k] : c->eds[k - WLOG_EDS];
    if (now != c->wlog_old[i]) return false;
  }
  return true;
}

/* SFR region 0x0000-0x07FF (word aligned address). */
static uint16_t sfr_rd(pic24_cpu *c, uint16_t a) {
  if (a < 0x20) return c->ram[a >> 1];
  uint16_t v;
  if (!core_sfr_get(c, a, &v)) v = c->cfg.sfr_read ? c->cfg.sfr_read(c->cfg.ctx, c, a) : c->ram[a >> 1];
  if (c->cfg.sfr_watch) c->cfg.sfr_watch(c->cfg.ctx, c, a, v, 0xFFFFu, false);
  return v;
}

static void sfr_wr(pic24_cpu *c, uint16_t a, uint16_t v, uint16_t m) {
  if (a < 0x20) {
    uint16_t n = (uint16_t)((c->ram[a >> 1] & ~m) | (v & m));
    set_w(c, a >> 1, n);
    return;
  }
  if (c->cfg.sfr_watch) c->cfg.sfr_watch(c->cfg.ctx, c, a, v, m, true);
  if (core_sfr_put(c, a, v, m)) return;
  if (c->cfg.sfr_write) { c->cfg.sfr_write(c->cfg.ctx, c, a, v, m); return; }
  wlog(c, a >> 1, c->ram[a >> 1]);
  c->ram[a >> 1] = (uint16_t)((c->ram[a >> 1] & ~m) | (v & m));
}

/* EDS window read (EA >= 0x8000), word-aligned EA.  FRM45 45.3, DS 4.2.5 / 4.3.3. */
static uint16_t eds_rd(pic24_cpu *c, uint16_t ea, uint16_t page) {
  if (page & 0x200u) { /* PSV: DSRPAG<9>=1 (DS Table 4-36) */
    c->acc_flags |= ACC_PSV;
    uint32_t pa = ((uint32_t)(page & 0xFFu) << 15) | (ea & 0x7FFEu);
    uint32_t w = prog_rd(c, pa);
    return (page & 0x100u) ? (uint16_t)((w >> 16) & 0xFFu) /* upper word: phantom byte reads 0 */ : (uint16_t)w;
  }
  if (page == 0) { raise_addrerr(c); return 0; } /* page 0 invalid (DS Table 4-35 note 3) */
  c->acc_flags |= ACC_EDS;
  uint32_t eda = ((uint32_t)page << 15) | (ea & 0x7FFEu);
  if (eda < PIC24_EDS_RAM_TOP) return c->eds[(eda - PIC24_EDS_RAM_BASE) >> 1];
  uint16_t v = 0;
  if (c->cfg.eds_ext_read && c->cfg.eds_ext_read(c->cfg.ctx, c, eda, &v)) return v;
  raise_addrerr(c); /* EPMP disabled / no device (FRM45 45.3.1.2 note 1) */
  return 0;
}

static void eds_wr(pic24_cpu *c, uint16_t ea, uint16_t page, uint16_t v, uint16_t m) {
  page &= 0x1FFu; /* DSWPAG is 9 bits; PSV is read-only */
  if (page == 0) { raise_addrerr(c); return; }
  uint32_t eda = ((uint32_t)page << 15) | (ea & 0x7FFEu);
  if (eda < PIC24_EDS_RAM_TOP) {
    uint16_t *p = &c->eds[(eda - PIC24_EDS_RAM_BASE) >> 1];
    wlog(c, WLOG_EDS + ((eda - PIC24_EDS_RAM_BASE) >> 1), *p);
    *p = (uint16_t)((*p & ~m) | (v & m));
    return;
  }
  if (c->cfg.eds_ext_write && c->cfg.eds_ext_write(c->cfg.ctx, c, eda, v, m)) return;
  raise_addrerr(c);
}

/* Word read. Misaligned: address error trap, the read completes with EA<0> cleared (PRM 4.5). */
static inline uint16_t rd16(pic24_cpu *c, uint16_t ea) {
  if (ea & 1u) { raise_addrerr(c); ea &= 0xFFFEu; }
  if (ea >= 0x0800u && ea < 0x8000u) return c->ram[ea >> 1];
  if (ea < 0x0800u) return sfr_rd(c, ea);
  return eds_rd(c, ea, c->dsrpag);
}

static inline uint8_t rd8(pic24_cpu *c, uint16_t ea) {
  uint16_t a = ea & 0xFFFEu;
  uint16_t w;
  if (a >= 0x0800u && a < 0x8000u) w = c->ram[a >> 1];
  else if (a < 0x0800u) w = sfr_rd(c, a);
  else w = eds_rd(c, a, c->dsrpag);
  return (uint8_t)((ea & 1u) ? (w >> 8) : w);
}

/* `wpage` = page register used for EDS-window writes: DSWPAG normally, DSRPAG for read-modify-write bit
 * instructions (DS Table 4-35 note 3 / FRM45 45.3.2 note 2). */
static inline void wr16p(pic24_cpu *c, uint16_t ea, uint16_t v, uint16_t wpage) {
  if (ea & 1u) { raise_addrerr(c); return; } /* misaligned write inhibited (PRM 4.5) */
  if (ea >= 0x0800u && ea < 0x8000u) { wlog(c, ea >> 1, c->ram[ea >> 1]); c->ram[ea >> 1] = v; return; }
  if (ea < 0x0800u) { sfr_wr(c, ea, v, 0xFFFFu); return; }
  eds_wr(c, ea, wpage, v, 0xFFFFu);
}
static inline void wr16(pic24_cpu *c, uint16_t ea, uint16_t v) { wr16p(c, ea, v, c->dswpag); }

static inline void wr8p(pic24_cpu *c, uint16_t ea, uint8_t v, uint16_t wpage) {
  uint16_t a = ea & 0xFFFEu;
  uint16_t m = (ea & 1u) ? 0xFF00u : 0x00FFu;
  uint16_t val = (ea & 1u) ? (uint16_t)(v << 8) : v;
  if (a >= 0x0800u && a < 0x8000u) { wlog(c, a >> 1, c->ram[a >> 1]); c->ram[a >> 1] = (uint16_t)((c->ram[a >> 1] & ~m) | val); return; }
  if (a < 0x0800u) { sfr_wr(c, a, val, m); return; }
  eds_wr(c, a, wpage, val, m);
}
static inline void wr8(pic24_cpu *c, uint16_t ea, uint8_t v) { wr8p(c, ea, v, c->dswpag); }

/* ======================================================================================= stack checks === */

/* Every EA generated with W15 as pointer (PRM 4.8.3 / 4.8.4, FRM8 8.2.1.1). `wrapped`: EA calc carried past
 * 0xFFFF. */
static inline void stack_check(pic24_cpu *c, uint32_t ea_unwrapped) {
  uint16_t ea = (uint16_t)ea_unwrapped;
  bool err = ea < 0x0800u;
  if (c->splim_armed && (ea > c->splim || ea_unwrapped > 0xFFFFu)) err = true;
  if (err) { c->intcon1 |= PIC24_INTCON1_STKERR; c->pend_valid = false; }
}

/* ================================================================== effective-address generation (EA) === */

/* Instruction-scoped scratch kept in the CPU struct would cost cache; use a small context on the stack. */
typedef struct {
  pic24_cpu *c;
  uint16_t src_ptr_mask; /* W regs used as source address pointers (RAW stall detection, PRM 4.3.1) */
  uint16_t dst_wmask;    /* W regs written on the destination side */
  unsigned cyc;          /* cycles of this instruction */
} ix;

/* Source EA for modes 1..5 (PRM Table 5-2), with pre/post modification by `sz` bytes (1, 2 or 4). */
static inline uint16_t ea_src(ix *x, unsigned mode, unsigned r, unsigned sz) {
  pic24_cpu *c = x->c;
  uint16_t w = c->ram[r];
  uint32_t ea;
  x->src_ptr_mask |= (uint16_t)(1u << r);
  switch (mode) {
  case 1: ea = w; break;
  case 2: ea = w; set_w(c, r, (uint16_t)(w - sz)); break;      /* [Ws--] */
  case 3: ea = w; set_w(c, r, (uint16_t)(w + sz)); break;      /* [Ws++] */
  case 4: ea = (uint16_t)(w - sz); set_w(c, r, (uint16_t)ea); break; /* [--Ws] */
  default: ea = (uint32_t)w + sz; set_w(c, r, (uint16_t)ea); break; /* [++Ws] */
  }
  if (r == 15) stack_check(c, ea);
  return (uint16_t)ea;
}

static inline uint16_t ea_dst(ix *x, unsigned mode, unsigned r, unsigned sz) {
  pic24_cpu *c = x->c;
  uint16_t w = c->ram[r];
  uint32_t ea;
  switch (mode) {
  case 1: ea = w; break;
  case 2: ea = w; set_w(c, r, (uint16_t)(w - sz)); x->dst_wmask |= (uint16_t)(1u << r); break;
  case 3: ea = w; set_w(c, r, (uint16_t)(w + sz)); x->dst_wmask |= (uint16_t)(1u << r); break;
  case 4: ea = (uint16_t)(w - sz); set_w(c, r, (uint16_t)ea); x->dst_wmask |= (uint16_t)(1u << r); break;
  default: ea = (uint32_t)w + sz; set_w(c, r, (uint16_t)ea); x->dst_wmask |= (uint16_t)(1u << r); break;
  }
  if (r == 15) stack_check(c, ea);
  return (uint16_t)ea;
}

/* memory writes landing on W0..W15 count as destination-side register writes for the RAW stall rule */
static inline void note_mem_wr(ix *x, uint16_t ea) {
  if (ea < 0x20u) x->dst_wmask |= (uint16_t)(1u << (ea >> 1));
}

/* Source operand, modes 0..5 (Ws / [Ws] / [Ws--] / [Ws++] / [--Ws] / [++Ws]).  Byte: zero-extended byte. */
static inline uint16_t rd_src(ix *x, unsigned mode, unsigned r, bool byte) {
  if (mode == 0) return byte ? (uint8_t)x->c->ram[r] : x->c->ram[r];
  uint16_t ea = ea_src(x, mode, r, byte ? 1u : 2u);
  return byte ? rd8(x->c, ea) : rd16(x->c, ea);
}

static inline void wr_dst(ix *x, unsigned mode, unsigned r, bool byte, uint16_t v) {
  pic24_cpu *c = x->c;
  if (mode == 0) {
    x->dst_wmask |= (uint16_t)(1u << r);
    if (byte) set_w(c, r, (uint16_t)((c->ram[r] & 0xFF00u) | (v & 0xFFu)));
    else set_w(c, r, v);
    return;
  }
  uint16_t ea = ea_dst(x, mode, r, byte ? 1u : 2u);
  note_mem_wr(x, ea);
  if (byte) wr8(c, ea, (uint8_t)v);
  else wr16(c, ea, v);
}

/* direct register write (destination side) */
static inline void wr_reg(ix *x, unsigned r, uint16_t v) {
  x->dst_wmask |= (uint16_t)(1u << r);
  set_w(x->c, r, v);
}

/* ============================================================================================== ALU === */

/* a + b + cin at width 8/16; full flag set DC,N,OV,Z,C. Subtraction = a + ~b + cin (C = NOT borrow). */
static inline uint16_t alu_add(pic24_cpu *c, unsigned a, unsigned b, unsigned cin, bool byte, bool sticky_z) {
  unsigned msk = byte ? 0xFFu : 0xFFFFu, sh = byte ? 8u : 16u;
  a &= msk;
  b &= msk;
  unsigned r = a + b + cin;
  unsigned res = r & msk;
  uint16_t sr = c->sr & (uint16_t)~(SR_C | SR_OV | SR_N | SR_DC);
  if (r >> sh) sr |= SR_C;
  if ((a ^ b ^ r) & (byte ? 0x10u : 0x100u)) sr |= SR_DC; /* carry out of bit 3 (byte) / bit 7 (word) */
  if (res >> (sh - 1)) sr |= SR_N;
  if ((~(a ^ b) & (a ^ r)) >> (sh - 1) & 1u) sr |= SR_OV;
  if (sticky_z) { if (res) sr &= (uint16_t)~SR_Z; } /* ADDC/SUBB/SUBBR/CPB: Z can only be cleared (PRM 4.10) */
  else { sr &= (uint16_t)~SR_Z; if (!res) sr |= SR_Z; }
  c->sr = sr;
  return (uint16_t)res;
}

static inline void flags_nz(pic24_cpu *c, unsigned res, bool byte) {
  uint16_t sr = c->sr & (uint16_t)~(SR_N | SR_Z);
  if (byte) { res &= 0xFFu; if (res & 0x80u) sr |= SR_N; }
  else { res &= 0xFFFFu; if (res & 0x8000u) sr |= SR_N; }
  if (!res) sr |= SR_Z;
  c->sr = sr;
}

static inline void set_flag(pic24_cpu *c, uint16_t f, bool v) {
  if (v) c->sr |= f;
  else c->sr &= (uint16_t)~f;
}

/* Binary ALU op shared by Wb,Ws,Wd / #lit5 / f / #lit10 forms. op codes: */
enum { OP_ADD, OP_ADDC, OP_SUB, OP_SUBB, OP_SUBR, OP_SUBBR, OP_AND, OP_IOR, OP_XOR };

static inline uint16_t alu_op(pic24_cpu *c, int op, unsigned a, unsigned b, bool byte) {
  /* a = "first" operand (Wb / f / Wn), b = "second" (Ws / lit / WREG) */
  unsigned cin = (c->sr & SR_C) ? 1u : 0u;
  unsigned msk = byte ? 0xFFu : 0xFFFFu;
  uint16_t r;
  switch (op) {
  case OP_ADD: return alu_add(c, a, b, 0, byte, false);
  case OP_ADDC: return alu_add(c, a, b, cin, byte, true);
  case OP_SUB: return alu_add(c, a, ~b, 1, byte, false);            /* a - b        */
  case OP_SUBB: return alu_add(c, a, ~b, cin, byte, true);          /* a - b - !C   */
  case OP_SUBR: return alu_add(c, b, ~a, 1, byte, false);           /* b - a        */
  case OP_SUBBR: return alu_add(c, b, ~a, cin, byte, true);         /* b - a - !C   */
  case OP_AND: r = (uint16_t)(a & b & msk); break;
  case OP_IOR: r = (uint16_t)((a | b) & msk); break;
  default: r = (uint16_t)((a ^ b) & msk); break;
  }
  flags_nz(c, r, byte);
  return r;
}

/* ========================================================================================== DIVIDE === */
/*
 * DIV.S/DIV.U (W and D) as the hardware runs it: one iteration per execution, 18 executions under REPEAT #17,
 * "the instruction flow function is conditional on RCOUNT" and "all intermediate data is saved in W1:W0 after each
 * iteration. The N, C and Z Status flags are used to convey control information between iterations" (FRM2 2.6.3).
 * Model (all inter-iteration state architectural, so the loop survives interrupts, nested DIVs and context
 * switches exactly as silicon must):
 *   step = 17 - RCOUNT (RCOUNT counts 17..0 across the 18 executions of REPEAT #17; PRM p.375, FRM2 Ex 2-13)
 *   step 0      : divisor==0 -> math error trap after this cycle (FRM2 2.6.3). W1:W0 = |dividend|, C = 0,
 *                 N = dividend sign (signed), Z = quotient-magnitude overflow (|hi| >= |divisor|).
 *   steps 1..16 : one non-restoring step on the 17-bit partial remainder {C,W1} and quotient/dividend W0.
 *   step 17     : remainder correction, sign fix-up, final N/Z/OV per PRM p.233/235. C = algorithm residue.
 * The intermediate W0/W1/C/N/Z values are a MODEL (not silicon-verified); end results follow the PRM exactly.
 */
static void exec_div(ix *x, uint32_t insn) {
  pic24_cpu *c = x->c;
  bool uns = (insn >> 15) & 1u;
  unsigned t = (insn >> 11) & 0xFu, v = (insn >> 7) & 0xFu, s = insn & 0xFu;
  bool dbl = (insn >> 6) & 1u;
  unsigned rc = c->rcount;
  unsigned step = rc >= 17u ? 0u : 17u - rc;
  uint16_t dv = c->ram[s];
  uint32_t mag_d = uns ? dv : (uint32_t)((dv & 0x8000u) ? (uint16_t)(0u - dv) : dv);
  if (dv == 0x8000u && !uns) mag_d = 0x8000u;
  if (step == 0) {
    if (dv == 0) { c->intcon1 |= PIC24_INTCON1_MATHERR; c->pend_valid = false; }
    uint32_t xv;
    if (dbl) xv = ((uint32_t)c->ram[t] << 16) | c->ram[v];
    else xv = uns ? c->ram[v] : (uint32_t)(int32_t)(int16_t)c->ram[v];
    bool sx = !uns && (xv & 0x80000000u);
    uint32_t mag = sx ? 0u - xv : xv;
    bool ovf = (mag >> 16) >= mag_d;
    wr_reg(x, 1, (uint16_t)(mag >> 16));
    wr_reg(x, 0, (uint16_t)mag);
    set_flag(c, SR_C, false);
    set_flag(c, SR_N, sx);
    set_flag(c, SR_Z, ovf);
    return;
  }
  int32_t p = (int32_t)(((c->sr & SR_C) ? 0xFFFF0000u : 0u) | c->ram[1]); /* 17-bit signed partial remainder */
  uint16_t a = c->ram[0];
  if (step <= 16) {
    int32_t p2 = 2 * p + (int32_t)(a >> 15);
    a = (uint16_t)(a << 1);
    p2 = (p >= 0) ? p2 - (int32_t)mag_d : p2 + (int32_t)mag_d;
    if (p2 >= 0) a |= 1u;
    wr_reg(x, 1, (uint16_t)p2);
    wr_reg(x, 0, a);
    set_flag(c, SR_C, p2 < 0);
    return;
  }
  /* step 17: final */
  bool sx = (c->sr & SR_N) != 0, ovf = (c->sr & SR_Z) != 0;
  bool resid_c = p < 0;
  if (p < 0) p += (int32_t)mag_d;
  uint32_t q = a, r = (uint32_t)p & 0xFFFFu;
  uint16_t qo, ro;
  if (uns) {
    qo = (uint16_t)q;
    ro = (uint16_t)r;
    set_flag(c, SR_N, false); /* "The N flag will always be cleared" (PRM p.235) */
  } else {
    bool sd = (dv & 0x8000u) != 0;
    bool neg = sx != sd;
    if (neg ? q > 0x8000u : q > 0x7FFFu) ovf = true;
    qo = (uint16_t)(neg ? 0u - q : q);
    ro = (uint16_t)(sx ? 0u - r : r);
    set_flag(c, SR_N, (ro & 0x8000u) != 0); /* "N set if the remainder is negative" (PRM p.233) */
  }
  wr_reg(x, 0, qo);
  wr_reg(x, 1, ro);
  set_flag(c, SR_Z, ro == 0);
  set_flag(c, SR_OV, ovf);
  set_flag(c, SR_C, resid_c);
}

/* ============================================================================== exception processing === */

static void push_word_raw(pic24_cpu *c, uint16_t v) {
  uint16_t sp = c->ram[15];
  /* exception stacking is a hardware sequence; SPLIM is not re-checked here (modelling decision) */
  if (sp >= 0x0800u && sp < 0x8000u) { wlog(c, sp >> 1, c->ram[sp >> 1]); c->ram[sp >> 1] = v; }
  else if (sp < 0x0800u) sfr_wr(c, sp, v, 0xFFFFu);
  else eds_wr(c, sp, c->dswpag, v, 0xFFFFu);
  set_w(c, 15, (uint16_t)(sp + 2u));
}

static unsigned take_exception(pic24_cpu *c, int vec, int level) {
  uint32_t pc = c->pc;
  c->exceptions++;
  /* FRM8 8.2.4 / PRM 4.8.1: PC<15:0>, then {SRL, IPL3, PC<22:16>} */
  push_word_raw(c, (uint16_t)pc);
  push_word_raw(c, (uint16_t)(((c->sr & 0xFFu) << 8) | ((unsigned)c->ipl3 << 7) | ((pc >> 16) & 0x7Fu)));
  c->sr &= (uint16_t)~SR_RA; /* "After SRL is stacked, the RA Status bit is cleared" (FRM2 2.10.1.2) */
  unsigned newipl;
  if (vec < 8) { c->ipl3 = 1; newipl = (unsigned)level & 7u; }
  else newipl = (c->intcon1 & PIC24_INTCON1_NSTDIS) ? 7u : (unsigned)level; /* FRM8 8.2.4.2 */
  c->sr = (uint16_t)((c->sr & ~SR_IPL) | (newipl << 5));
  uint32_t va = ((c->intcon2 & PIC24_INTCON2_ALTIVT) ? 0x100u : 0u) + 4u + 2u * (uint32_t)vec;
  c->pc = prog_rd(c, va) & 0x7FFFFEu;
  c->inttreg = (uint16_t)((c->inttreg & 0x2000u) | ((unsigned)(level & 0xF) << 8) | ((unsigned)vec & 0x7Fu));
  c->pend_valid = false;
  c->power = PIC24_RUNNING;
  c->raw_wmask = 0;
  c->repeat_first = false;
  c->repeat_final = false;
  /* FRM8 Fig 8-3: cycles 2-4 of the interrupt process (FNOP, vector fetch, FNOP) */
  return 3;
}

/* ============================================================================================ reset === */

void pic24_reset(pic24_cpu *c) {
  /* core registers to POR values (DS Table 4-4, 4-6; EDC por strings) */
  memset(c->ram, 0, 0x20);
  c->ram[15] = 0x0800u;
  c->pc = 0;
  c->sr = 0;
  c->ipl3 = 0;
  c->rcount = 0;
  c->splim = 0;
  c->splim_armed = false;
  c->dsrpag = 1;
  c->dswpag = 1;
  c->tblpag = 0;
  c->disicnt = 0;
  c->intcon1 = 0;
  c->intcon2 = 0;
  c->inttreg = 0;
  memset(c->ifs, 0, sizeof c->ifs);
  memset(c->iec, 0, sizeof c->iec);
  for (unsigned i = 0; i < 26; i++) c->ipc[i] = (uint16_t)(0x4444u & IPC_IMPL[i]); /* priority 4 at reset (DS 7.4.1) */
  memset(c->shadow_w, 0, sizeof c->shadow_w);
  c->shadow_sr = 0;
  c->power = PIC24_RUNNING;
  c->pend_valid = false;
  c->hard_trap_pending = 0;
  c->acc_flags = 0;
  c->raw_wmask = 0;
  c->repeat_first = false;
  c->repeat_final = false;
  c->repeat_iter = 0;
}

size_t pic24_cpu_size(void) { return sizeof(pic24_cpu); }

void pic24_init(pic24_cpu *c, const pic24_config *cfg) {
  memset(c, 0, sizeof *c);
  if (cfg) c->cfg = *cfg;
  for (size_t i = 0; i < PIC24_PROG_WORDS; i++) c->prog[i] = 0xFFFFFFu; /* erased flash */
  pic24_reset(c);
}

void pic24_load_prog(pic24_cpu *c, const uint8_t *b, size_t n) {
  size_t words = n / 3u;
  if (words > PIC24_PROG_WORDS) words = PIC24_PROG_WORDS;
  for (size_t i = 0; i < words; i++)
    c->prog[i] = (uint32_t)b[3 * i] | ((uint32_t)b[3 * i + 1] << 8) | ((uint32_t)b[3 * i + 2] << 16);
}

void pic24_set_sfr_reset(pic24_cpu *c, const uint16_t *w, size_t n) {
  for (size_t i = 0x10; i < n && i < 0x400; i++) {
    uint16_t a = (uint16_t)(i * 2u), dummy;
    if (core_sfr_get(c, a, &dummy)) continue;
    c->ram[i] = w[i];
  }
}

/* A reset the core performs itself (RESET instruction, illegal opcode, trap conflict): core registers to their reset
 * values (the same path as MCLR, DS39969 Table 6-3), then the host resets its peripherals and sets the RCON flag. */
static void device_reset(pic24_cpu *c, pic24_status why) {
  c->last_reset = why;
  c->last_reset_pc = c->pc;
  c->resets++;
  pic24_reset(c);
  if (c->cfg.on_reset) c->cfg.on_reset(c->cfg.ctx, c, why);
}

/* ====================================================================================== execute one === */

/* Branch condition, opcode bits 19:16 of 0x3x (PRM Table 4-8). */
static inline bool cond_true(uint16_t sr, unsigned cc) {
  bool C = sr & SR_C, Z = sr & SR_Z, N = sr & SR_N, V = sr & SR_OV;
  switch (cc) {
  case 0x0: return V;                    /* OV  */
  case 0x1: return C;                    /* C / GEU */
  case 0x2: return Z;                    /* Z   */
  case 0x3: return N;                    /* N   */
  case 0x4: return Z || (N != V);        /* LE  */
  case 0x5: return N != V;               /* LT  */
  case 0x6: return !C || Z;              /* LEU */
  case 0x7: return true;                 /* BRA */
  case 0x8: return !V;                   /* NOV */
  case 0x9: return !C;                   /* NC / LTU */
  case 0xA: return !Z;                   /* NZ  */
  case 0xB: return !N;                   /* NN  */
  case 0xC: return !Z && (N == V);       /* GT  */
  case 0xD: return N == V;               /* GE  */
  case 0xE: return C && !Z;              /* GTU */
  default: return false;
  }
}

int pic24_insn_words(uint32_t insn) {
  unsigned op = (insn >> 16) & 0xFFu;
  return (op == 0x02u || op == 0x04u) ? 2 : 1;
}

/* push / pop through W15 with stack checks (instruction-generated EAs) */
static inline void push16(ix *x, uint16_t v) {
  pic24_cpu *c = x->c;
  uint16_t ea = ea_dst(x, 3, 15, 2);
  wr16(c, ea, v);
}
static inline uint16_t pop16(ix *x) {
  uint16_t ea = ea_src(x, 4, 15, 2);
  return rd16(x->c, ea);
}

/* skip the next instruction: returns extra cycles (1 or 2) and advances *npc */
static inline unsigned do_skip(pic24_cpu *c, uint32_t *npc) {
  uint32_t nxt = prog_rd(c, *npc);
  unsigned w = (unsigned)pic24_insn_words(nxt);
  *npc += 2u * w;
  return w; /* PRM 3.2.1: skip over 1-word = +1 cycle, over 2-word = +2 cycles */
}

/* bit number for the f,#bit4 forms: bits 15:13 = bit<2:0>, bit0 = bit<3> (DS70157F; binutils) */
static inline unsigned fbit4(uint32_t i) { return ((i >> 13) & 7u) | ((i & 1u) << 3); }

#define ILLEGAL() do { goto illegal; } while (0)

static pic24_status execute(pic24_cpu *c, uint32_t insn, uint32_t *npc_out, ix *x) {
  const uint32_t pc = c->pc;
  uint32_t npc = pc + 2u;
  const unsigned op = (insn >> 16) & 0xFFu;
  const bool byte = (insn >> 14) & 1u; /* 'B' at bit 14 for most formats */
  const unsigned s = insn & 0xFu, p = (insn >> 4) & 7u, d = (insn >> 7) & 0xFu, q = (insn >> 11) & 7u;
  const unsigned wb = (insn >> 15) & 0xFu;
  x->cyc = 1;

  switch (op) {
  /* ----------------------------------------------------------------------------------- 0x00..0x0F --- */
  case 0x00: break; /* NOP (p.354) */
  case 0x01: {
    unsigned sub = (insn >> 4) & 0xFFFu;
    uint16_t wn = c->ram[s];
    if (sub == 0x000u) { /* CALL Wn (p.187) */
      uint32_t ret = pc + 2u;
      push16(x, (uint16_t)ret);
      push16(x, (uint16_t)((ret >> 16) & 0x7Fu));
      npc = wn & 0xFFFEu;
      x->cyc = 2;
    } else if (sub == 0x200u) { /* RCALL Wn (p.371): PC = PC+2 + 2*Wn */
      uint32_t ret = pc + 2u;
      push16(x, (uint16_t)ret);
      push16(x, (uint16_t)((ret >> 16) & 0x7Fu));
      npc = (ret + 2u * (uint32_t)(int32_t)(int16_t)wn) & 0x7FFFFEu;
      x->cyc = 2;
    } else if (sub == 0x400u) { /* GOTO Wn (p.264) */
      npc = wn & 0xFFFEu;
      x->cyc = 2;
    } else if (sub == 0x600u) { /* BRA Wn (p.137) */
      npc = (pc + 2u + 2u * (uint32_t)(int32_t)(int16_t)wn) & 0x7FFFFEu;
      x->cyc = 2;
    } else ILLEGAL();
    break;
  }
  case 0x02: { /* CALL lit23 (p.183), 2 words */
    uint32_t w2 = prog_rd(c, pc + 2u);
    if ((w2 >> 16) & 0xFFu) ILLEGAL(); /* second word must have 8 MSbs = 0 (DS 29.0) */
    uint32_t tgt = (insn & 0xFFFEu) | ((w2 & 0x7Fu) << 16);
    uint32_t ret = pc + 4u;
    push16(x, (uint16_t)ret);
    push16(x, (uint16_t)((ret >> 16) & 0x7Fu));
    npc = tgt;
    x->cyc = 2;
    break;
  }
  case 0x04: { /* GOTO lit23 (p.263) */
    uint32_t w2 = prog_rd(c, pc + 2u);
    if ((w2 >> 16) & 0xFFu) ILLEGAL();
    npc = (insn & 0xFFFEu) | ((w2 & 0x7Fu) << 16);
    x->cyc = 2;
    break;
  }
  case 0x05: { /* RETLW{.B} #lit10,Wn (p.382) */
    if (insn & 0x8000u) ILLEGAL();
    uint16_t hi = pop16(x), lo = pop16(x);
    npc = ((uint32_t)(hi & 0x7Fu) << 16) | (lo & 0xFFFEu);
    uint16_t k = (uint16_t)((insn >> 4) & 0x3FFu);
    if (byte) wr_reg(x, s, (uint16_t)((c->ram[s] & 0xFF00u) | (k & 0xFFu)));
    else wr_reg(x, s, k);
    x->cyc = 3;
    break;
  }
  case 0x06:
    if ((insn & 0xFFFFu) == 0x0000u) { /* RETURN (p.386) */
      uint16_t hi = pop16(x), lo = pop16(x);
      npc = ((uint32_t)(hi & 0x7Fu) << 16) | (lo & 0xFFFEu);
      x->cyc = 3;
    } else if ((insn & 0xFFFFu) == 0x4000u) { /* RETFIE (p.380) */
      uint16_t hi = pop16(x), lo = pop16(x);
      c->sr = (uint16_t)((c->sr & 0xFF00u) | (hi >> 8)); /* SRL incl. IPL<2:0>, RA */
      c->ipl3 = (hi >> 7) & 1u;
      npc = ((uint32_t)(hi & 0x7Fu) << 16) | (lo & 0xFFFEu);
      c->pend_valid = false;
      if (c->sr & SR_RA) c->repeat_first = true; /* re-entering a REPEAT loop (FRM2 2.10.1.2 PSV note) */
      x->cyc = 3;
    } else ILLEGAL();
    break;
  case 0x07: /* RCALL Expr (p.367) */
  {
    uint32_t ret = pc + 2u;
    push16(x, (uint16_t)ret);
    push16(x, (uint16_t)((ret >> 16) & 0x7Fu));
    npc = (ret + 2u * (uint32_t)(int32_t)(int16_t)(insn & 0xFFFFu)) & 0x7FFFFEu;
    x->cyc = 2;
    break;
  }
  case 0x09: { /* REPEAT #lit14 (p.375) / REPEAT Wn (p.377) */
    unsigned cnt = 0;
    if ((insn & 0xC000u) == 0x0000u) cnt = insn & 0x3FFFu;
    else if ((insn & 0xFFF0u) == 0x8000u) cnt = c->ram[s] & 0x3FFFu;
    else ILLEGAL();
    c->rcount = (uint16_t)cnt;
    if (cnt) c->sr |= SR_RA; /* RA set only for a non-zero count (FRM2 2.10.1.1) */
    c->repeat_first = true;
    c->repeat_iter = 0;
    break;
  }
  /* ------------------------------------------------------------------- 0x10..0x7F: Wb,Ws,Wd ALU ops --- */
  case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17:
  case 0x18: case 0x19: case 0x1A: case 0x1B: case 0x1C: case 0x1D: case 0x1E: case 0x1F:
  case 0x40: case 0x41: case 0x42: case 0x43: case 0x44: case 0x45: case 0x46: case 0x47:
  case 0x48: case 0x49: case 0x4A: case 0x4B: case 0x4C: case 0x4D: case 0x4E: case 0x4F:
  case 0x50: case 0x51: case 0x52: case 0x53: case 0x54: case 0x55: case 0x56: case 0x57:
  case 0x58: case 0x59: case 0x5A: case 0x5B: case 0x5C: case 0x5D: case 0x5E: case 0x5F:
  case 0x60: case 0x61: case 0x62: case 0x63: case 0x64: case 0x65: case 0x66: case 0x67:
  case 0x68: case 0x69: case 0x6A: case 0x6B: case 0x6C: case 0x6D: case 0x6E: case 0x6F:
  case 0x70: case 0x71: case 0x72: case 0x73: case 0x74: case 0x75: case 0x76: case 0x77: {
    static const signed char OPS[16] = {
        /* index = op>>3 (bits 23:19) - 2..15 */
        -1, -1, OP_SUBR, OP_SUBBR, -1, -1, -1, -1, OP_ADD, OP_ADDC, OP_SUB, OP_SUBB, OP_AND, OP_XOR, OP_IOR, -1};
    int alu = OPS[op >> 3];
    if (q >= 6) ILLEGAL(); /* qqq=11x forces RESET (PRM Table 5-3) */
    uint16_t a = c->ram[wb]; /* Wb read before the source EA side effects (modelling decision) */
    uint16_t b;
    if (p >= 6) {
      b = (uint16_t)(insn & 0x1Fu); /* #lit5 form: bits 6:5 = 11 (PRM p.104) */
    } else b = rd_src(x, p, s, byte);
    uint16_t r = alu_op(c, alu, a, b, byte);
    wr_dst(x, q, d, byte, r);
    break;
  }
  case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27:
  case 0x28: case 0x29: case 0x2A: case 0x2B: case 0x2C: case 0x2D: case 0x2E: case 0x2F:
    wr_reg(x, s, (uint16_t)((insn >> 4) & 0xFFFFu)); /* MOV #lit16,Wnd (p.304) */
    break;
  case 0x30: case 0x31: case 0x32: case 0x33: case 0x34: case 0x35: case 0x36: case 0x37:
  case 0x38: case 0x39: case 0x3A: case 0x3B: case 0x3C: case 0x3D: case 0x3E: { /* BRA cc (p.136-159) */
    if (cond_true(c->sr, op & 0xFu)) {
      npc = (pc + 2u + 2u * (uint32_t)(int32_t)(int16_t)(insn & 0xFFFFu)) & 0x7FFFFEu;
      x->cyc = 2;
    }
    break;
  }
  case 0x78: case 0x79: case 0x7A: case 0x7B: case 0x7C: case 0x7D: case 0x7E: case 0x7F: {
    /* MOV{.B} Ws,Wd with register-offset modes (p.307); PUSH/POP Wd are aliases (p.358, p.362) */
    unsigned g = p, h = q;
    uint16_t wbv = c->ram[wb];
    uint16_t old_ws = c->ram[s];
    uint16_t v;
    if (g >= 6) {
      uint16_t ea = (uint16_t)(old_ws + wbv);
      x->src_ptr_mask |= (uint16_t)((1u << s) | (1u << wb));
      /* Wb is added as a 16-bit two's-complement offset: [W15+Wb] with Wb negative addresses below W15 (PRM
       * DS70000157G); only a signed result outside 0..0xFFFF wraps. */
      if (s == 15) stack_check(c, (uint32_t)((int32_t)old_ws + (int16_t)wbv) & 0x1FFFFu);
      v = byte ? rd8(c, ea) : rd16(c, ea);
    } else v = rd_src(x, g, s, byte);
    if (h >= 6) {
      uint16_t ea;
      /* PRM 4.3.1 note: register-offset destination with Ws==Wd uses the OLD Ws, offset ignored */
      if (s == d && g != 0) ea = old_ws;
      else {
        ea = (uint16_t)(c->ram[d] + c->ram[wb]);
        if (d == 15) stack_check(c, (uint32_t)((int32_t)c->ram[d] + (int16_t)c->ram[wb]) & 0x1FFFFu);   /* signed Wb, see above */
      }
      note_mem_wr(x, ea);
      if (byte) wr8(c, ea, (uint8_t)v);
      else wr16(c, ea, v);
    } else wr_dst(x, h, d, byte, v);
    break;
  }
  /* ------------------------------------------------------------------------------- 0x80..0x9F: MOV --- */
  case 0x80: case 0x81: case 0x82: case 0x83: case 0x84: case 0x85: case 0x86: case 0x87:
    wr_reg(x, s, rd16(c, (uint16_t)((insn >> 3) & 0xFFFEu))); /* MOV f,Wnd (p.301) */
    break;
  case 0x88: case 0x89: case 0x8A: case 0x8B: case 0x8C: case 0x8D: case 0x8E: case 0x8F: {
    uint16_t ea = (uint16_t)((insn >> 3) & 0xFFFEu); /* MOV Wns,f (p.302) */
    note_mem_wr(x, ea);
    wr16(c, ea, c->ram[s]);
    break;
  }
  case 0x90: case 0x91: case 0x92: case 0x93: case 0x94: case 0x95: case 0x96: case 0x97:
  case 0x98: case 0x99: case 0x9A: case 0x9B: case 0x9C: case 0x9D: case 0x9E: case 0x9F: {
    /* MOV [Ws+Slit10],Wnd (p.305) / MOV Wns,[Wd+Slit10] (p.306); k9..6 = b18..15, k5..3 = b13..11, k2..0 = b6..4 */
    int k = (int)((((insn >> 15) & 0xFu) << 6) | (((insn >> 11) & 7u) << 3) | ((insn >> 4) & 7u));
    if (k & 0x200) k -= 0x400;
    if (!byte) k *= 2; /* word mode: offset is in words, range -1024..1022 */
    if (!(op & 0x08u)) {
      uint16_t base = c->ram[s];
      uint16_t ea = (uint16_t)(base + k);
      x->src_ptr_mask |= (uint16_t)(1u << s);
      if (s == 15) stack_check(c, (uint32_t)((int32_t)base + k) & 0x1FFFFu);
      if (byte) wr_reg(x, d, (uint16_t)((c->ram[d] & 0xFF00u) | rd8(c, ea)));
      else wr_reg(x, d, rd16(c, ea));
    } else {
      uint16_t base = c->ram[d];
      uint16_t ea = (uint16_t)(base + k);
      if (d == 15) stack_check(c, (uint32_t)((int32_t)base + k) & 0x1FFFFu);
      note_mem_wr(x, ea);
      if (byte) wr8(c, ea, (uint8_t)c->ram[s]);
      else wr16(c, ea, c->ram[s]);
    }
    break;
  }
  /* --------------------------------------------------------------------- 0xA0..0xAF: bit operations --- */
  case 0xA0: case 0xA1: case 0xA2: case 0xA4: { /* BSET/BCLR/BTG/BTSTS Ws,#bit4 (p.161,128,166,181) */
    unsigned bit = (insn >> 12) & 0xFu;
    bool bmode, zform = false;
    if (op == 0xA4) { /* BTSTS: bbbb Z000 */
      if (insn & 0x0700u) ILLEGAL();
      zform = (insn >> 11) & 1u;
      bmode = false;
    } else {
      if (insn & 0x0B80u) ILLEGAL(); /* bbbb 0B00 0ppp */
      bmode = (insn >> 10) & 1u;
    }
    if (insn & 0x80u) ILLEGAL();
    if (p >= 6) ILLEGAL();
    if (bmode) bit &= 7u;
    if (p == 0) {
      uint16_t v = c->ram[s];
      uint16_t m = (uint16_t)(1u << bit);
      if (op == 0xA4) { if (zform) set_flag(c, SR_Z, !(v & m)); else set_flag(c, SR_C, (v & m) != 0); }
      if (op == 0xA0 || op == 0xA4) v |= m;
      else if (op == 0xA1) v &= (uint16_t)~m;
      else v ^= m;
      if (bmode) v = (uint16_t)((c->ram[s] & 0xFF00u) | (v & 0xFFu));
      wr_reg(x, s, v);
    } else {
      /* indirect: read-modify-write of the EA; EDS writes use DSRPAG (DS Table 4-35 note 3) */
      uint16_t ea = ea_src(x, p, s, bmode ? 1u : 2u);
      if (bmode) {
        uint8_t v = rd8(c, ea);
        uint8_t m = (uint8_t)(1u << bit);
        if (op == 0xA0) v |= m; else if (op == 0xA1) v &= (uint8_t)~m; else v ^= m;
        note_mem_wr(x, ea);
        wr8p(c, ea, v, c->dsrpag);
      } else {
        uint16_t v = rd16(c, ea); /* odd EA: address error (FRM8 8.2.2.3 item 2) */
        uint16_t m = (uint16_t)(1u << bit);
        if (op == 0xA4) { if (zform) set_flag(c, SR_Z, !(v & m)); else set_flag(c, SR_C, (v & m) != 0); }
        if (op == 0xA0 || op == 0xA4) v |= m;
        else if (op == 0xA1) v &= (uint16_t)~m;
        else v ^= m;
        note_mem_wr(x, ea);
        wr16p(c, ea, v, c->dsrpag);
      }
    }
    break;
  }
  case 0xA3: { /* BTST.C/.Z Ws,#bit4 (p.176): bbbb Z000 0ppp ssss */
    if (insn & 0x0780u) ILLEGAL();
    if (p >= 6) ILLEGAL();
    unsigned bit = (insn >> 12) & 0xFu;
    uint16_t v = rd_src(x, p, s, false);
    if ((insn >> 11) & 1u) set_flag(c, SR_Z, !((v >> bit) & 1u));
    else set_flag(c, SR_C, (v >> bit) & 1u);
    break;
  }
  case 0xA5: { /* BTST.C/.Z Ws,Wb (p.178): Zwww w000 0ppp ssss */
    if (insn & 0x0780u) ILLEGAL();
    if (p >= 6) ILLEGAL();
    unsigned bit = c->ram[(insn >> 11) & 0xFu] & 0xFu;
    uint16_t v = rd_src(x, p, s, false);
    if ((insn >> 15) & 1u) set_flag(c, SR_Z, !((v >> bit) & 1u));
    else set_flag(c, SR_C, (v >> bit) & 1u);
    break;
  }
  case 0xAD: { /* BSW.C/.Z Ws,Wb (p.163): Zwww w000 0ppp ssss; .Z writes NOT Z (PRM examples p.164) */
    if (insn & 0x0780u) ILLEGAL();
    if (p >= 6) ILLEGAL();
    unsigned bit = c->ram[(insn >> 11) & 0xFu] & 0xFu;
    bool val = ((insn >> 15) & 1u) ? !(c->sr & SR_Z) : (c->sr & SR_C) != 0;
    uint16_t m = (uint16_t)(1u << bit);
    if (p == 0) wr_reg(x, s, val ? (uint16_t)(c->ram[s] | m) : (uint16_t)(c->ram[s] & ~m));
    else {
      uint16_t ea = ea_src(x, p, s, 2);
      uint16_t v = rd16(c, ea);
      v = val ? (uint16_t)(v | m) : (uint16_t)(v & ~m);
      note_mem_wr(x, ea);
      wr16p(c, ea, v, c->dsrpag);
    }
    break;
  }
  case 0xA6: case 0xA7: { /* BTSS/BTSC Ws,#bit4 (p.173,170): bbbb 0000 0ppp ssss */
    if (insn & 0x0F80u) ILLEGAL();
    if (p >= 6) ILLEGAL();
    unsigned bit = (insn >> 12) & 0xFu;
    uint16_t v = rd_src(x, p, s, false);
    bool set = (v >> bit) & 1u;
    if ((op == 0xA6) ? set : !set) x->cyc += do_skip(c, &npc);
    break;
  }
  case 0xA8: case 0xA9: case 0xAA: case 0xAB: case 0xAC: case 0xAE: case 0xAF: {
    /* f,#bit4 forms (word RMW on the word-aligned file register; p.160,127,165,175,180,172,168) */
    unsigned bit = fbit4(insn);
    uint16_t f = (uint16_t)(insn & 0x1FFEu);
    uint16_t v = rd16(c, f);
    uint16_t m = (uint16_t)(1u << bit);
    switch (op) {
    case 0xA8: note_mem_wr(x, f); wr16(c, f, (uint16_t)(v | m)); break;
    case 0xA9: note_mem_wr(x, f); wr16(c, f, (uint16_t)(v & ~m)); break;
    case 0xAA: note_mem_wr(x, f); wr16(c, f, (uint16_t)(v ^ m)); break;
    case 0xAB: set_flag(c, SR_Z, !(v & m)); break;
    case 0xAC: set_flag(c, SR_Z, !(v & m)); note_mem_wr(x, f); wr16(c, f, (uint16_t)(v | m)); break;
    case 0xAE:
      if (v & m) x->cyc += do_skip(c, &npc);
      break;
    default:
      if (!(v & m)) x->cyc += do_skip(c, &npc);
      break;
    }
    break;
  }
  /* ------------------------------------------------------------------------------- 0xB0..0xBF --- */
  case 0xB0: case 0xB1: case 0xB2: case 0xB3: { /* op #lit10,Wn (p.103,111,419,425,117,452,272); MOV.B #lit8 */
    if (op == 0xB3 && (insn & 0x8000u)) {
      if ((insn & 0xF000u) != 0xC000u) ILLEGAL();
      wr_reg(x, s, (uint16_t)((c->ram[s] & 0xFF00u) | ((insn >> 4) & 0xFFu))); /* MOV.B #lit8,Wnd (p.303) */
      break;
    }
    static const int L10[4][2] = {{OP_ADD, OP_ADDC}, {OP_SUB, OP_SUBB}, {OP_AND, OP_XOR}, {OP_IOR, -1}};
    int alu = L10[op & 3u][(insn >> 15) & 1u];
    unsigned k = (insn >> 4) & 0x3FFu;
    uint16_t r = alu_op(c, alu, c->ram[s], k, byte); /* Wn op lit (SUB: Wn - lit) */
    if (byte) wr_reg(x, s, (uint16_t)((c->ram[s] & 0xFF00u) | (r & 0xFFu)));
    else wr_reg(x, s, r);
    break;
  }
  case 0xB4: case 0xB5: case 0xB6: case 0xB7: case 0xBD: { /* op f {,WREG} (p.102,110,418,424,116,451,271,435,430) */
    static const int FOPS[5][2] = {{OP_ADD, OP_ADDC}, {OP_SUB, OP_SUBB}, {OP_AND, OP_XOR}, {OP_IOR, -2}, {OP_SUBR, OP_SUBBR}};
    int alu = FOPS[op == 0xBD ? 4 : (op & 3u)][(insn >> 15) & 1u];
    uint16_t f = (uint16_t)(insn & 0x1FFFu);
    bool dfile = (insn >> 13) & 1u;
    if (alu == -2) { /* MOV{.B} WREG,f (p.300): 1011 0111 1B1f */
      if (!dfile) ILLEGAL();
      note_mem_wr(x, f);
      if (byte) wr8(c, f, (uint8_t)c->ram[0]);
      else wr16(c, f, c->ram[0]);
      break;
    }
    uint16_t fv = byte ? rd8(c, f) : rd16(c, f);
    uint16_t r = alu_op(c, alu, fv, c->ram[0], byte); /* SUB f: f - WREG; SUBR f: WREG - f */
    if (dfile) {
      note_mem_wr(x, f);
      if (byte) wr8(c, f, (uint8_t)r);
      else wr16(c, f, r); /* lands after the flag latch: a write to SR itself wins (modelling decision) */
    } else wr_dst(x, 0, 0, byte, r);
    break;
  }
  case 0xB8: case 0xB9: { /* MUL.UU/US/SU/SS Wb,Ws,Wnd; MUL.SU/UU Wb,#lit5,Wnd (p.325-337) */
    bool sb = op == 0xB9 ? true : false;       /* signed Wb: 0xB9 = MUL.SU / MUL.SS */
    bool ss = (insn >> 15) & 1u;               /* bit 15: 0xB8 -> UU(0)/US(1); 0xB9 -> SU(0)/SS(1) */
    bool sws = ss;                             /* signed Ws */
    uint16_t a = c->ram[(insn >> 11) & 0xFu];  /* Wb = bits 14:11 */
    unsigned dd = d;
    uint32_t bval;
    bool lit = ((insn >> 5) & 3u) == 3u;
    if (lit) {
      if (ss) ILLEGAL(); /* no MUL.SS/US with literal */
      bval = insn & 0x1Fu;
      sws = false;
    } else {
      if (p >= 6) ILLEGAL();
      uint16_t bv = rd_src(x, p, s, false);
      bval = sws ? (uint32_t)(int32_t)(int16_t)bv : bv;
    }
    int64_t av = sb ? (int64_t)(int16_t)a : (int64_t)a;
    int64_t bv2 = sws ? (int64_t)(int32_t)bval : (int64_t)(uint32_t)bval;
    uint32_t prod = (uint32_t)(av * bv2);
    wr_reg(x, dd, (uint16_t)prod);
    wr_reg(x, (dd + 1u) & 0xFu, (uint16_t)(prod >> 16));
    break;
  }
  case 0xBA: case 0xBB: { /* TBLRDL/H (p.442,440), TBLWTL/H (p.446,444) */
    bool high = (insn >> 15) & 1u;
    x->cyc = 2;
    if (op == 0xBA) {
      if (p == 0 || p >= 6 || q >= 6) ILLEGAL();
      uint16_t ea = ea_src(x, p, s, byte ? 1u : 2u);
      uint32_t addr = ((uint32_t)c->tblpag << 16) | ea;
      uint32_t wd;
      if (c->tblpag & 0x80u) wd = c->cfg.cfg_read ? c->cfg.cfg_read(c->cfg.ctx, c, addr & 0xFFFFFEu)
                                                  : ((addr & 0xFFFFFEu) == 0xFF0000u ? 0x410Eu : 0u);
      else wd = prog_rd(c, addr);
      uint16_t v;
      if (!high) v = byte ? (uint16_t)((ea & 1u) ? (wd >> 8) & 0xFFu : wd & 0xFFu) : (uint16_t)wd;
      else v = byte ? (uint16_t)((ea & 1u) ? 0u : (wd >> 16) & 0xFFu) : (uint16_t)((wd >> 16) & 0xFFu);
      wr_dst(x, q, d, byte, v);
    } else {
      if (q == 0 || q >= 6 || p >= 6) ILLEGAL();
      uint16_t v = rd_src(x, p, s, byte);
      uint16_t ea = ea_dst(x, q, d, byte ? 1u : 2u);
      uint32_t addr = ((uint32_t)c->tblpag << 16) | (ea & 0xFFFEu);
      uint16_t m, val;
      if (!high) {
        if (byte) { m = (ea & 1u) ? 0xFF00u : 0x00FFu; val = (ea & 1u) ? (uint16_t)(v << 8) : (uint16_t)(v & 0xFFu); }
        else { m = 0xFFFFu; val = v; }
      } else {
        if (byte && (ea & 1u)) break; /* TBLWTH.B to odd address: no operation (p.444) */
        m = 0x00FFu;
        val = v & 0xFFu;
      }
      if (c->cfg.tblwt) c->cfg.tblwt(c->cfg.ctx, c, addr, val, m, high);
    }
    break;
  }
  case 0xBC: { /* MUL{.B} f (p.323): 1011 1100 0B0f */
    if (insn & 0xA000u) ILLEGAL();
    uint16_t f = (uint16_t)(insn & 0x1FFFu);
    if (byte) wr_reg(x, 2, (uint16_t)((uint16_t)(c->ram[0] & 0xFFu) * (uint16_t)rd8(c, f)));
    else {
      uint32_t prod = (uint32_t)c->ram[0] * rd16(c, f);
      wr_reg(x, 2, (uint16_t)prod);
      wr_reg(x, 3, (uint16_t)(prod >> 16));
    }
    break;
  }
  case 0xBE: { /* MOV.D (p.309; store form: PUSH.D p.364) */
    x->cyc = 2;
    if ((insn & 0xF880u) == 0x0000u) { /* 1011 1110 0000 0ddd 0ppp ssss : Ws -> Wnd:Wnd+1 */
      unsigned dn = (insn >> 7) & 0xEu;
      uint16_t lo, hi;
      if (p == 0) { lo = c->ram[s]; hi = c->ram[(s + 1u) & 0xFu]; }
      else {
        if (p >= 6) ILLEGAL();
        uint16_t ea = ea_src(x, p, s, 4);
        lo = rd16(c, ea);
        hi = rd16(c, (uint16_t)(ea + 2u));
      }
      wr_reg(x, dn, lo);
      wr_reg(x, dn + 1u, hi);
    } else if ((insn & 0xC071u) == 0x8000u) { /* 1011 1110 10qq qddd d000 sss0 : Wns:Wns+1 -> Wd */
      unsigned sn = insn & 0xEu;
      uint16_t lo = c->ram[sn], hi = c->ram[sn + 1u];
      if (q == 0) { wr_reg(x, d, lo); wr_reg(x, (d + 1u) & 0xFu, hi); }
      else {
        if (q >= 6) ILLEGAL();
        uint16_t ea = ea_dst(x, q, d, 4);
        note_mem_wr(x, ea);
        note_mem_wr(x, (uint16_t)(ea + 2u));
        wr16(c, ea, lo);
        wr16(c, (uint16_t)(ea + 2u), hi);
      }
    } else ILLEGAL();
    break;
  }
  case 0xBF: { /* MOV{.B} f {,WREG} (p.299): 1011 1111 1BDf, flags N,Z */
    if (!(insn & 0x8000u)) ILLEGAL();
    uint16_t f = (uint16_t)(insn & 0x1FFFu);
    uint16_t v = byte ? rd8(c, f) : rd16(c, f);
    flags_nz(c, v, byte);
    if ((insn >> 13) & 1u) { note_mem_wr(x, f); if (byte) wr8(c, f, (uint8_t)v); else wr16(c, f, v); }
    else wr_dst(x, 0, 0, byte, v);
    break;
  }
  /* ------------------------------------------------------------------------------- 0xC0..0xDF --- */
  case 0xCF: { /* FF1R (p.259) 1100 1111 0000 0ddd dppp ssss / FF1L (p.257) 1100 1111 1000 ... */
    if ((insn & 0x7800u) != 0) ILLEGAL();
    if (p >= 6) ILLEGAL();
    uint16_t v = rd_src(x, p, s, false);
    uint16_t r = 0;
    if (v) r = (insn & 0x8000u) ? (uint16_t)(__builtin_clz((unsigned)v) - 16 + 1) : (uint16_t)(__builtin_ctz(v) + 1);
    wr_reg(x, d, r);
    set_flag(c, SR_C, v == 0);
    break;
  }
  case 0xD0: case 0xD1: case 0xD2: case 0xD3: { /* SL / LSR,ASR / RLNC,RLC / RRNC,RRC  Ws,Wd */
    if (op == 0xD0 && (insn & 0x8000u)) ILLEGAL();
    if (q >= 6 || p >= 6) ILLEGAL();
    uint16_t v = rd_src(x, p, s, byte);
    uint16_t r = 0;
    unsigned top = byte ? 7u : 15u;
    uint16_t msk = byte ? 0xFFu : 0xFFFFu;
    bool hi = (insn >> 15) & 1u, cin = (c->sr & SR_C) != 0;
    switch (op & 3u) {
    case 0:
      r = (uint16_t)((v << 1) & msk);
      set_flag(c, SR_C, (v >> top) & 1u);
      break;
    case 1:
      r = hi ? (uint16_t)(((v >> 1) | (v & (1u << top))) & msk) : (uint16_t)(v >> 1);
      set_flag(c, SR_C, v & 1u);
      break;
    case 2:
      r = (uint16_t)(((v << 1) | (hi ? (unsigned)cin : (v >> top) & 1u)) & msk);
      if (hi) set_flag(c, SR_C, (v >> top) & 1u);
      break;
    default:
      r = (uint16_t)(((v >> 1) | ((hi ? (unsigned)cin : (v & 1u)) << top)) & msk);
      if (hi) set_flag(c, SR_C, v & 1u);
      break;
    }
    flags_nz(c, r, byte);
    wr_dst(x, q, d, byte, r);
    break;
  }
  case 0xD4: case 0xD5: case 0xD6: case 0xD7: { /* same ops on f {,WREG} */
    if (op == 0xD4 && (insn & 0x8000u)) ILLEGAL();
    uint16_t f = (uint16_t)(insn & 0x1FFFu);
    uint16_t v = byte ? rd8(c, f) : rd16(c, f);
    uint16_t r = 0;
    unsigned top = byte ? 7u : 15u;
    uint16_t msk = byte ? 0xFFu : 0xFFFFu;
    bool hi = (insn >> 15) & 1u, cin = (c->sr & SR_C) != 0;
    switch (op & 3u) {
    case 0:
      r = (uint16_t)((v << 1) & msk);
      set_flag(c, SR_C, (v >> top) & 1u);
      break;
    case 1:
      r = hi ? (uint16_t)(((v >> 1) | (v & (1u << top))) & msk) : (uint16_t)(v >> 1);
      set_flag(c, SR_C, v & 1u);
      break;
    case 2:
      r = (uint16_t)(((v << 1) | (hi ? (unsigned)cin : (v >> top) & 1u)) & msk);
      if (hi) set_flag(c, SR_C, (v >> top) & 1u);
      break;
    default:
      r = (uint16_t)(((v >> 1) | ((hi ? (unsigned)cin : (v & 1u)) << top)) & msk);
      if (hi) set_flag(c, SR_C, v & 1u);
      break;
    }
    flags_nz(c, r, byte);
    if ((insn >> 13) & 1u) { note_mem_wr(x, f); if (byte) wr8(c, f, (uint8_t)r); else wr16(c, f, r); }
    else wr_dst(x, 0, 0, byte, r);
    break;
  }
  case 0xDA: /* BREAK (0xDA4000): software breakpoint for an attached debugger; not in the user instruction set
               * (DS70000157G). Without a debugger it is treated as an illegal opcode -> device reset (cfg.debugger). */
    if (insn != 0xDA4000u) ILLEGAL();
    if (!c->cfg.debugger) ILLEGAL();
    c->stop_insn = insn;
    *npc_out = npc;
    return PIC24_STOP_BREAK;
  case 0xD8: /* DIV.S / DIV.U  1101 1000 Uttt tvvv vW00 ssss (p.233, 235) */
    if (insn & 0x30u) ILLEGAL();
    exec_div(x, insn);
    break;
  case 0xDD: case 0xDE: { /* SL (p.416/417), LSR/ASR (p.286/287, 125/126) Wb,#lit4|Wns,Wnd */
    if (op == 0xDD && (insn & 0x8000u)) ILLEGAL();
    unsigned form = (insn >> 4) & 7u;
    if (form != 0 && form != 4) ILLEGAL();
    uint16_t v = c->ram[(insn >> 11) & 0xFu];
    unsigned sh = form == 4 ? (insn & 0xFu) : (c->ram[s] & 0x1Fu); /* Wns<4:0> (p.287, 417) */
    uint16_t r;
    if (op == 0xDD) r = sh > 15 ? 0 : (uint16_t)(v << sh);
    else if (insn & 0x8000u) r = (uint16_t)((int16_t)v >> (sh > 15 ? 15 : sh)); /* ASR: >15 -> sign fill (p.126 note 2) */
    else r = sh > 15 ? 0 : (uint16_t)(v >> sh);
    wr_reg(x, d, r);
    flags_nz(c, r, false);
    break;
  }
  /* ------------------------------------------------------------------------------- 0xE0..0xEF --- */
  case 0xE0: { /* CP0{.B} Ws (p.205): 1110 0000 0000 0B00 0ppp ssss */
    if (insn & 0xFB80u) ILLEGAL();
    if (p >= 6) ILLEGAL();
    bool bm = (insn >> 10) & 1u;
    uint16_t v = rd_src(x, p, s, bm);
    alu_add(c, v, 0xFFFFu, 1, bm, false); /* v - 0 */
    break;
  }
  case 0xE1: { /* CP/CPB Wb,Ws (p.203,209) / Wb,#lit5 (p.201,207): 1110 0001 Xwww wB00 0ppp|011k */
    if (insn & 0x0300u) ILLEGAL();
    bool bm = (insn >> 10) & 1u;
    bool borrow = (insn >> 15) & 1u;
    uint16_t a = c->ram[(insn >> 11) & 0xFu];
    uint16_t b;
    if (((insn >> 5) & 7u) == 3u) b = (uint16_t)(insn & 0x1Fu);
    else {
      if (insn & 0x80u) ILLEGAL();
      if (p >= 6) ILLEGAL();
      b = rd_src(x, p, s, bm);
    }
    alu_op(c, borrow ? OP_SUBB : OP_SUB, a, b, bm);
    break;
  }
  case 0xE2: { /* CP0{.B} f (p.204): 1110 0010 0B0f */
    if (insn & 0xA000u) ILLEGAL();
    uint16_t f = (uint16_t)(insn & 0x1FFFu);
    uint16_t v = byte ? rd8(c, f) : rd16(c, f);
    alu_add(c, v, 0xFFFFu, 1, byte, false);
    break;
  }
  case 0xE3: { /* CP/CPB f (p.200,206): 1110 0011 XB0f -> f - WREG [- !C] */
    if (insn & 0x2000u) ILLEGAL();
    uint16_t f = (uint16_t)(insn & 0x1FFFu);
    uint16_t v = byte ? rd8(c, f) : rd16(c, f);
    alu_op(c, (insn & 0x8000u) ? OP_SUBB : OP_SUB, v, c->ram[0], byte);
    break;
  }
  case 0xE6: case 0xE7: { /* CPSGT/CPSLT (E6, bit15) / CPSNE/CPSEQ (E7) Wb,Wn (p.217-221): Xwww wB00 0000 ssss */
    if (insn & 0x03F0u) ILLEGAL();
    bool bm = (insn >> 10) & 1u;
    uint16_t a = c->ram[(insn >> 11) & 0xFu], b = c->ram[s];
    int32_t sa = bm ? (int8_t)a : (int16_t)a, sb2 = bm ? (int8_t)b : (int16_t)b;
    uint16_t m = bm ? 0xFFu : 0xFFFFu;
    bool hi = (insn >> 15) & 1u, take;
    if (op == 0xE6) take = hi ? (sa < sb2) : (sa > sb2);
    else take = hi ? ((a & m) == (b & m)) : ((a & m) != (b & m));
    if (take) x->cyc += do_skip(c, &npc);
    break;
  }
  case 0xE8: case 0xE9: case 0xEA: { /* INC/INC2, DEC/DEC2, NEG/COM  Ws,Wd */
    if (q >= 6 || p >= 6) ILLEGAL();
    uint16_t v = rd_src(x, p, s, byte);
    bool hi = (insn >> 15) & 1u;
    uint16_t r;
    if (op == 0xE8) r = alu_add(c, v, hi ? 2u : 1u, 0, byte, false);
    else if (op == 0xE9) r = alu_add(c, v, ~(hi ? 2u : 1u), 1, byte, false);
    else if (!hi) r = alu_add(c, 0, ~(unsigned)v, 1, byte, false); /* NEG = 0 - v */
    else { r = (uint16_t)(~v & (byte ? 0xFFu : 0xFFFFu)); flags_nz(c, r, byte); } /* COM */
    wr_dst(x, q, d, byte, r);
    break;
  }
  case 0xEB: { /* CLR / SETM Wd (p.193, 409): XBqq qddd d000 0000 */
    if (insn & 0x7Fu) ILLEGAL();
    if (q >= 6) ILLEGAL();
    wr_dst(x, q, d, byte, (insn & 0x8000u) ? 0xFFFFu : 0u);
    break;
  }
  case 0xEC: case 0xED: case 0xEE: case 0xEF: { /* INC/INC2, DEC/DEC2, NEG/COM, CLR/SETM  f {,WREG} */
    uint16_t f = (uint16_t)(insn & 0x1FFFu);
    bool hi = (insn >> 15) & 1u, dfile = (insn >> 13) & 1u;
    uint16_t r;
    if (op == 0xEF) r = hi ? 0xFFFFu : 0u; /* CLR/SETM: no read, no flags */
    else {
      uint16_t v = byte ? rd8(c, f) : rd16(c, f);
      if (op == 0xEC) r = alu_add(c, v, hi ? 2u : 1u, 0, byte, false);
      else if (op == 0xED) r = alu_add(c, v, ~(hi ? 2u : 1u), 1, byte, false);
      else if (!hi) r = alu_add(c, 0, ~(unsigned)v, 1, byte, false);
      else { r = (uint16_t)(~v & (byte ? 0xFFu : 0xFFFFu)); flags_nz(c, r, byte); }
    }
    if (dfile) { note_mem_wr(x, f); if (byte) wr8(c, f, (uint8_t)r); else wr16(c, f, r); }
    else wr_dst(x, 0, 0, byte, r);
    break;
  }
  /* ------------------------------------------------------------------------------- 0xF8..0xFF --- */
  case 0xF8: case 0xF9: { /* PUSH f (p.361) / POP f (p.357): ffff ffff ffff fff0 */
    if (insn & 1u) ILLEGAL();
    uint16_t f = (uint16_t)(insn & 0xFFFEu);
    if (op == 0xF8) push16(x, rd16(c, f));
    else { uint16_t v = pop16(x); note_mem_wr(x, f); wr16(c, f, v); }
    break;
  }
  case 0xFA:
    if ((insn & 0xC001u) == 0x0000u) { /* LNK #lit14 (p.280) */
      push16(x, c->ram[14]);
      wr_reg(x, 14, c->ram[15]);
      wr_reg(x, 15, (uint16_t)(c->ram[15] + (insn & 0x3FFEu)));
    } else if ((insn & 0xFFFFu) == 0x8000u) { /* ULNK (p.448) */
      wr_reg(x, 15, c->ram[14]);
      wr_reg(x, 14, pop16(x));
    } else ILLEGAL();
    break;
  case 0xFB: { /* SE (p.406) 1111 1011 0000 0ddd dppp ssss / ZE (p.456) 1111 1011 1000 ... */
    if (insn & 0x7800u) ILLEGAL();
    if (p >= 6) ILLEGAL();
    uint8_t v = (uint8_t)rd_src(x, p, s, true);
    uint16_t r;
    if (insn & 0x8000u) { r = v; set_flag(c, SR_N, false); set_flag(c, SR_Z, r == 0); set_flag(c, SR_C, true); }
    else { r = (uint16_t)(int16_t)(int8_t)v; set_flag(c, SR_N, (r & 0x8000u) != 0); set_flag(c, SR_Z, r == 0); set_flag(c, SR_C, !(r & 0x8000u)); }
    wr_reg(x, d, r);
    break;
  }
  case 0xFC: { /* DISI #lit14 (p.232) */
    if (insn & 0xC000u) ILLEGAL();
    c->disicnt = (uint16_t)(insn & 0x3FFFu);
    if (c->disicnt) c->intcon2 |= PIC24_INTCON2_DISI;
    /* the DISI cycle itself is covered: the per-cycle decrement starts with the next instruction */
    c->disicnt = (uint16_t)(c->disicnt + 1u); /* compensates the decrement applied for this instruction's cycle */
    break;
  }
  case 0xFD: {
    if ((insn & 0xF870u) == 0x0000u) { /* EXCH Wns,Wnd (p.254): 0000 0ddd d000 ssss */
      uint16_t a = c->ram[s], b = c->ram[d];
      wr_reg(x, d, a);
      wr_reg(x, s, b);
    } else if ((insn & 0xFFF0u) == 0x4000u) { /* DAW.B Wn (p.225) */
      unsigned v = c->ram[s] & 0xFFu;
      bool cy = (c->sr & SR_C) != 0;
      if ((v & 0xFu) > 9u || (c->sr & SR_DC)) v += 6u;
      if (((v >> 4) & 0x1Fu) > 9u || cy) { v += 0x60u; cy = true; }
      if (v > 0xFFu) cy = true;
      wr_reg(x, s, (uint16_t)((c->ram[s] & 0xFF00u) | (v & 0xFFu)));
      set_flag(c, SR_C, cy);
    } else if ((insn & 0xBFF0u) == 0x8000u) { /* SWAP{.B} Wn (p.439): 1B00 0000 0000 ssss */
      uint16_t v = c->ram[s];
      if (byte) v = (uint16_t)((v & 0xFF00u) | ((v & 0xFu) << 4) | ((v >> 4) & 0xFu));
      else v = (uint16_t)((v << 8) | (v >> 8));
      wr_reg(x, s, v);
    } else ILLEGAL();
    break;
  }
  case 0xFE: {
    unsigned lo = insn & 0xFFFFu;
    if (lo == 0x0000u) { /* RESET (p.379) */
      device_reset(c, PIC24_STOP_RESET);
      *npc_out = c->pc;
      return PIC24_STOP_RESET;
    } else if ((lo & 0xFFFEu) == 0x4000u) { /* PWRSAV #lit1 (p.366): 0 = Sleep, 1 = Idle */
      c->power = (lo & 1u) ? PIC24_IDLE : PIC24_SLEEP;
    } else if (lo == 0x6000u) { /* CLRWDT (p.196): WDT is a peripheral -> nothing in the core */
    } else if (lo == 0x8000u) { /* POP.S (p.360) */
      for (unsigned i = 0; i < 4; i++) wr_reg(x, i, c->shadow_w[i]);
      c->sr = (uint16_t)((c->sr & ~SR_ARITH) | (c->shadow_sr & SR_ARITH));
    } else if (lo == 0xA000u) { /* PUSH.S (p.365) */
      for (unsigned i = 0; i < 4; i++) c->shadow_w[i] = c->ram[i];
      c->shadow_sr = c->sr & SR_ARITH;
    } else ILLEGAL(); /* BOOTSWP, MOVPAG, CTXTSWP: not on PIC24FJ256DA210 (DS Table 29-2) */
    break;
  }
  case 0xFF: break; /* NOPR (p.355) */

  default:
    ILLEGAL(); /* DO, DSP, MOVPAG, LDSLV, BFEXT/BFINS, FBCL, DIVF, FLIM, CPBxx, BRA OA/OB/SA/SB: dsPIC/E only */
  }
  *npc_out = npc;
  return PIC24_OK;

illegal:
  /* Illegal opcode or illegal address mode. Silicon resets the device with RCON.IOPUWR set (DS39969 Register 6-1:
   * "an illegal opcode detection, an illegal address mode or Uninitialized W register used as an Address Pointer
   * caused a Reset"); pic24_step performs that reset unless cfg.illegal_stops (debug) is set. The uninitialized-W
   * cause is not detected by this core (W registers are not tracked for first write). */
  c->stop_insn = insn;
  *npc_out = pc;
  return PIC24_STOP_ILLEGAL;
}

/* ========================================================================================== step / run === */

/* Sleep/Idle wake-up: any enabled source with priority >= 1 (FRM8 8.2.5) */
static bool wake_pending(pic24_cpu *c) {
  if (c->intcon1 & (PIC24_INTCON1_OSCFAIL | PIC24_INTCON1_ADDRERR | PIC24_INTCON1_STKERR | PIC24_INTCON1_MATHERR)) return true;
  for (unsigned w = 0; w < 7; w++) {
    uint16_t pnd = c->ifs[w] & c->iec[w];
    while (pnd) {
      unsigned b = (unsigned)__builtin_ctz(pnd);
      pnd &= (uint16_t)(pnd - 1);
      unsigned irq = w * 16 + b;
      if (irq / 4 < 26 && ((c->ipc[irq / 4] >> ((irq & 3) * 4)) & 7)) return true;
    }
  }
  return false;
}

static inline void consume(pic24_cpu *c, unsigned cyc) {
  c->cycles += cyc;
  if (c->disicnt) {
    c->disicnt = (uint16_t)(c->disicnt > cyc ? c->disicnt - cyc : 0u);
    if (!c->disicnt) { c->intcon2 &= (uint16_t)~PIC24_INTCON2_DISI; }
  }
}

pic24_status pic24_step(pic24_cpu *c) {
  /* 1. exceptions are taken at instruction boundaries (FRM8 8.2.4) */
  int lvl;
  int vec = exception_ready(c, &lvl);
  if (vec >= 0) {
    unsigned cyc = take_exception(c, vec, lvl);
    consume(c, cyc);
    return PIC24_OK;
  }
  if (c->power != PIC24_RUNNING) {
    if (wake_pending(c)) c->power = PIC24_RUNNING; /* priority <= IPL: continue after PWRSAV (FRM8 8.2.5) */
    else { consume(c, 1); return PIC24_STOP_SLEEP; }
  }

  /* 2. fetch */
  uint32_t pc = c->pc;
  if (pc >= PIC24_PROG_TOP) { /* executing unimplemented program memory (FRM8 8.2.2.3 item 5) */
    raise_addrerr(c);
    c->hard_trap_pending = 0;
    consume(c, 1);
    return PIC24_OK;
  }
  uint32_t insn = c->prog[pc >> 1];

  /* 3. execute */
  ix x = {c, 0, 0, 1};
  bool ra = (c->sr & SR_RA) != 0;
  bool rep_first = c->repeat_first;
  c->acc_flags = 0;
  c->hard_trap_pending = 0;
  unsigned ipl_before = cur_ipl(c);
  uint32_t npc;
  pic24_status st = execute(c, insn, &npc, &x);
  if (st != PIC24_OK) {
    if (st == PIC24_STOP_ILLEGAL && !c->cfg.illegal_stops) { /* silicon: illegal-opcode reset, RCON.IOPUWR */
      device_reset(c, PIC24_STOP_ILLEGAL);
      return PIC24_STOP_RESET;
    }
    if (st == PIC24_STOP_BREAK) { c->pc = npc; c->instret++; consume(c, 1); } /* debugger halt: resume after BREAK */
    return st;
  }
  unsigned cyc = x.cyc;

  /* RAW stall: source pointer written on the destination side by the previous instruction (PRM 4.3.1).
   * GOTO / RETURN / RETLW / RETFIE are never stalled (PRM 4.3.3). */
  unsigned op8 = (insn >> 16) & 0xFFu;
  bool nostall = op8 == 0x04u || op8 == 0x05u || op8 == 0x06u || (op8 == 0x01u && (insn & 0xFFF0u) == 0x4000u);
  if (!nostall && (x.src_ptr_mask & c->raw_wmask)) cyc += 1;
  c->raw_wmask = x.dst_wmask;

  /* PSV / EDS read wait states (DS 4.3.3 + 4.2.5.1 note; FRM45 45.3.2; FRM2 2.10.1.2) */
  if (c->acc_flags) {
    bool loop = ra || rep_first || c->repeat_final; /* executing as a REPEAT target */
    if (c->acc_flags & ACC_PSV) {
      bool is_mov = (op8 >= 0x78u && op8 <= 0x87u) || (op8 >= 0x90u && op8 <= 0x97u) || op8 == 0xBEu || op8 == 0xBFu;
      if (!loop) cyc += is_mov ? 1u : 2u;          /* MOV/MOV.D +1, all others +2 */
      else if (rep_first || c->repeat_final) cyc += 2u; /* first / last iteration (and re-entry) +2 */
    } else {
      if (!loop) cyc += 1u;                         /* EDS RAM read: one extra cycle */
      else if (c->repeat_iter < 2u) cyc += 2u;      /* first two accesses take three cycles */
    }
  }

  /* 4. REPEAT sequencing (FRM2 Example 2-13): target re-executes while RA; RCOUNT-- per execution */
  c->repeat_final = false;
  if (ra) {
    c->repeat_first = false;
    if (c->repeat_iter < 255u) c->repeat_iter++;
    if (c->rcount) c->rcount--;
    if (!c->rcount) { c->sr &= (uint16_t)~SR_RA; c->repeat_final = true; }
    /* PC stays on the target */
  } else {
    if (rep_first && op8 != 0x09u) c->repeat_first = false;
    c->pc = npc;
  }
  /* RETURN/RETLW/RETFIE: 3 cycles, 2 if an exception is pending (PRM p.380-386) */
  if (x.cyc == 3 && (op8 == 0x06u || op8 == 0x05u)) {
    int l2;
    if (exception_ready(c, &l2) >= 0) cyc = 2;
  }
  /* hard-trap conflict: new address error while a higher-priority hard trap is being processed (FRM8 8.2.2.1) */
  if (c->hard_trap_pending && ipl_before >= 14) {
    device_reset(c, PIC24_STOP_TRAP_CONFLICT);
    return PIC24_STOP_TRAP_CONFLICT;
  }
  c->instret++;
  consume(c, cyc);
  return PIC24_OK;
}

pic24_status pic24_run(pic24_cpu *c, uint64_t cycles) {
  uint64_t end = c->cycles + cycles;
  while (c->cycles < end) {
    pic24_status st = pic24_step(c);
    if (st != PIC24_OK) return st;
    if (c->stop_request) { c->stop_request = false; return PIC24_STOP_BREAK; }
  }
  return PIC24_STOP_CYCLES;
}

/* ============================================================================================== API === */

void pic24_set_irq(pic24_cpu *c, unsigned irq, bool level) {
  if (irq >= 112) return;
  uint16_t m = (uint16_t)(1u << (irq & 15u));
  if (level) c->ifs[irq >> 4] |= (uint16_t)(m & IFS_IMPL[irq >> 4]); /* unimplemented sources cannot be raised */
  else c->ifs[irq >> 4] &= (uint16_t)~m;
  c->pend_valid = false;
}

void pic24_raise_trap(pic24_cpu *c, uint16_t bit) {
  c->intcon1 |= (uint16_t)(bit & 0x001Eu);
  c->pend_valid = false;
}

unsigned pic24_cpu_ipl(const pic24_cpu *c) { return cur_ipl(c); }

uint16_t pic24_get_w(const pic24_cpu *c, unsigned n) { return c->ram[n & 15u]; }
void pic24_set_w(pic24_cpu *c, unsigned n, uint16_t v) { set_w(c, n & 15u, v); }

uint16_t pic24_peek16(pic24_cpu *c, uint16_t ea) {
  uint16_t ic1 = c->intcon1;
  uint16_t v = rd16(c, ea & 0xFFFEu);
  c->intcon1 = ic1;
  return v;
}
void pic24_poke16(pic24_cpu *c, uint16_t ea, uint16_t v) {
  uint16_t ic1 = c->intcon1;
  wr16(c, ea & 0xFFFEu, v);
  c->intcon1 = ic1;
}

bool pic24_core_sfr(const pic24_cpu *c, uint16_t a, uint16_t *v) {
  if (a < 0x20) { *v = c->ram[a >> 1]; return true; }
  return core_sfr_get(c, a & 0xFFFEu, v);
}

uint32_t pic24_prog_word(const pic24_cpu *c, uint32_t pc) { return prog_rd(c, pc); }
