/* uc_mips32_shim.c -- Pure-C11 MIPS32 M4K interpreter with the subset of the Unicorn engine API that emu/ecu uses.
 *
 * Plain C, so emu/ecu and emu/pair compile unchanged to WebAssembly (emcc) and to native host binaries. Implements the PIC32MX795F512L MIPS32 M4K / M14K instruction set, Fixed Mapping Translation
 * (phys = vaddr & 0x1FFFFFFF), CP0 registers, and External Interrupt Controller (EIC) vectoring.
 */
#include <unicorn/unicorn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RAM_BASE 0x00000000u
#define RAM_SIZE 0x00020000u
#define PFL_BASE 0x1D000000u
#define PFL_SIZE 0x00080000u
#define BFL_BASE 0x1FC00000u
#define BFL_SIZE 0x00003000u
#define SFR_BASE 0x1F800000u
#define SFR_SIZE 0x00100000u

#define MAX_CODE_HOOKS 256   /* the ECU model hooks each CP0 site address once; an image programmed over another adds its own */
#define MAX_BLOCK_HOOKS 8

typedef struct {
  uint64_t begin, end;
  uc_cb_hookcode_t cb;
  void *ud;
} code_hook_t;

struct uc_struct {
  uint32_t gpr[32];
  uint32_t srs_gpr[32];
  uint32_t pc;
  uint32_t hi, lo;

  /* CP0 registers */
  uint32_t cp0_status;
  uint32_t cp0_cause;
  uint32_t cp0_epc;
  uint32_t cp0_ebase;
  uint32_t cp0_intctl;
  uint32_t cp0_srsctl;
  uint32_t cp0_config0;
  uint32_t cp0_config3;
  uint32_t cp0_userlocal;
  uint32_t cp0_count;
  uint32_t cp0_compare;
  int32_t  eic_vec;

  /* Memory regions */
  uint8_t ram[RAM_SIZE];
  uint8_t pfl[PFL_SIZE];
  uint8_t bfl[BFL_SIZE];

  /* Cached basic-block lengths (in instructions) for PFL and BFL */
  uint8_t pfl_tblen[PFL_SIZE / 4];
  uint8_t bfl_tblen[BFL_SIZE / 4];
  /* Bitmap of PFL word indices that have a UC_HOOK_CODE attached */
  uint32_t pfl_code_hook_bm[(PFL_SIZE / 4 + 31) / 32];

  /* MMIO callback for SFR_BASE..SFR_BASE+SFR_SIZE */
  uint64_t mmio_base, mmio_size;
  uc_cb_mmio_read_t mmio_read;
  void *mmio_read_ud;
  uc_cb_mmio_write_t mmio_write;
  void *mmio_write_ud;

  /* Hooks */
  code_hook_t code_hooks[MAX_CODE_HOOKS];
  unsigned n_code_hooks;
  bool any_non_pfl_code_hook;
  code_hook_t block_hooks[MAX_BLOCK_HOOKS];
  unsigned n_block_hooks;
  size_t next_hook_id;

  bool stop_req;
  uc_err err;
  /* write log (uc_shim_wlog_start): the RAM words guest stores wrote, with the value each held before the first one */
  uint32_t exceptions;                 /* exceptions taken (uc_shim_exceptions) */
  bool wlog_on, wlog_full;
  unsigned wlog_n;
  uint32_t wlog_key[UC_SHIM_WLOG], wlog_old[UC_SHIM_WLOG];
};

static inline uint32_t load_u32_le(const uint8_t *p) {
  uint32_t v;
  memcpy(&v, p, 4);
  return v;
}

static inline void store_u32_le(uint8_t *p, uint32_t v) {
  memcpy(p, &v, 4);
}

static inline uint16_t load_u16_le(const uint8_t *p) {
  uint16_t v;
  memcpy(&v, p, 2);
  return v;
}

static inline void store_u16_le(uint8_t *p, uint16_t v) {
  memcpy(p, &v, 2);
}

uc_err uc_open(uc_arch arch, uc_mode mode, uc_engine **uch) {
  (void)mode;
  if (arch != UC_ARCH_MIPS || !uch) return UC_ERR_ARG;
  uc_engine *uc = (uc_engine *)calloc(1, sizeof(uc_engine));
  if (!uc) return UC_ERR_NOMEM;
  /* Reset CP0 state matching QEMU MIPS32 M14K cpu_state_reset */
  uc->pc = 0xBFC00000u;
  uc->cp0_status = (1u << 22) | (1u << 2); /* BEV=1, ERL=1 */
  uc->cp0_ebase = 0x80000000u;
  uc->cp0_intctl = 0xE0000000u;
  uc->cp0_config0 = 0x80008482u;           /* K0=2, Fixed Mapping MMU */
  uc->cp0_config3 = 0x00000060u;           /* VEIC + VInt */
  uc->next_hook_id = 1;
  *uch = uc;
  return UC_ERR_OK;
}

uc_err uc_close(uc_engine *uc) {
  free(uc);
  return UC_ERR_OK;
}

uc_err uc_ctl_set_cpu_model(uc_engine *uc, int model) {
  (void)uc;
  (void)model;
  return UC_ERR_OK;
}

uc_err uc_mem_map(uc_engine *uc, uint64_t address, size_t size, uint32_t perms) {
  (void)uc;
  (void)address;
  (void)size;
  (void)perms;
  return UC_ERR_OK;
}

uc_err uc_mmio_map(uc_engine *uc, uint64_t address, size_t size,
                   uc_cb_mmio_read_t read_cb, void *user_data_read,
                   uc_cb_mmio_write_t write_cb, void *user_data_write) {
  if (!uc) return UC_ERR_ARG;
  uc->mmio_base = address & 0x1FFFFFFFull;
  uc->mmio_size = size;
  uc->mmio_read = read_cb;
  uc->mmio_read_ud = user_data_read;
  uc->mmio_write = write_cb;
  uc->mmio_write_ud = user_data_write;
  return UC_ERR_OK;
}

uc_err uc_mem_read(uc_engine *uc, uint64_t address, void *bytes, size_t size) {
  if (!uc || !bytes) return UC_ERR_ARG;
  uint32_t pa = (uint32_t)(address & 0x1FFFFFFFull);
  if (pa < RAM_SIZE && pa + size <= RAM_SIZE) {
    memcpy(bytes, uc->ram + pa, size);
    return UC_ERR_OK;
  }
  if (pa >= PFL_BASE && pa < PFL_BASE + PFL_SIZE && (pa - PFL_BASE) + size <= PFL_SIZE) {
    memcpy(bytes, uc->pfl + (pa - PFL_BASE), size);
    return UC_ERR_OK;
  }
  if (pa >= BFL_BASE && pa < BFL_BASE + BFL_SIZE && (pa - BFL_BASE) + size <= BFL_SIZE) {
    memcpy(bytes, uc->bfl + (pa - BFL_BASE), size);
    return UC_ERR_OK;
  }
  return UC_ERR_READ_UNMAPPED;
}

uc_err uc_mem_write(uc_engine *uc, uint64_t address, const void *bytes, size_t size) {
  if (!uc || !bytes) return UC_ERR_ARG;
  uint32_t pa = (uint32_t)(address & 0x1FFFFFFFull);
  if (pa < RAM_SIZE && pa + size <= RAM_SIZE) {
    memcpy(uc->ram + pa, bytes, size);
    return UC_ERR_OK;
  }
  if (pa >= PFL_BASE && pa < PFL_BASE + PFL_SIZE && (pa - PFL_BASE) + size <= PFL_SIZE) {
    memcpy(uc->pfl + (pa - PFL_BASE), bytes, size);
    memset(uc->pfl_tblen, 0, sizeof(uc->pfl_tblen));
    return UC_ERR_OK;
  }
  if (pa >= BFL_BASE && pa < BFL_BASE + BFL_SIZE && (pa - BFL_BASE) + size <= BFL_SIZE) {
    memcpy(uc->bfl + (pa - BFL_BASE), bytes, size);
    memset(uc->bfl_tblen, 0, sizeof(uc->bfl_tblen));
    return UC_ERR_OK;
  }
  return UC_ERR_WRITE_UNMAPPED;
}

uc_err uc_reg_read(uc_engine *uc, int regid, void *value) {
  if (!uc || !value) return UC_ERR_ARG;
  uint32_t *dst = (uint32_t *)value;
  if (regid >= UC_MIPS_REG_0 && regid <= UC_MIPS_REG_31) {
    int idx = regid - UC_MIPS_REG_0;
    *dst = idx == 0 ? 0u : uc->gpr[idx];
    return UC_ERR_OK;
  }
  switch (regid) {
    case UC_MIPS_REG_PC:            *dst = uc->pc; break;
    case UC_MIPS_REG_HI:            *dst = uc->hi; break;
    case UC_MIPS_REG_LO:            *dst = uc->lo; break;
    case UC_MIPS_REG_CP0_CONFIG3:   *dst = uc->cp0_config3; break;
    case UC_MIPS_REG_CP0_USERLOCAL: *dst = uc->cp0_userlocal; break;
    case UC_MIPS_REG_CP0_STATUS:    *dst = uc->cp0_status; break;
    case UC_MIPS_REG_CP0_CAUSE:     *dst = uc->cp0_cause; break;
    case UC_MIPS_REG_CP0_EPC:       *dst = uc->cp0_epc; break;
    case UC_MIPS_REG_CP0_EBASE:     *dst = uc->cp0_ebase; break;
    case UC_MIPS_REG_CP0_INTCTL:    *dst = uc->cp0_intctl; break;
    case UC_MIPS_REG_CP0_EICVEC:    *dst = (uint32_t)uc->eic_vec; break;
    case UC_MIPS_REG_CP0_CONFIG0:   *dst = uc->cp0_config0; break;
    default: return UC_ERR_ARG;
  }
  return UC_ERR_OK;
}

uc_err uc_reg_write(uc_engine *uc, int regid, const void *value) {
  if (!uc || !value) return UC_ERR_ARG;
  uint32_t v = *(const uint32_t *)value;
  if (regid >= UC_MIPS_REG_0 && regid <= UC_MIPS_REG_31) {
    int idx = regid - UC_MIPS_REG_0;
    if (idx != 0) uc->gpr[idx] = v;
    return UC_ERR_OK;
  }
  switch (regid) {
    case UC_MIPS_REG_PC:            uc->pc = v; break;
    case UC_MIPS_REG_HI:            uc->hi = v; break;
    case UC_MIPS_REG_LO:            uc->lo = v; break;
    case UC_MIPS_REG_CP0_CONFIG3:   uc->cp0_config3 = v; break;
    case UC_MIPS_REG_CP0_USERLOCAL: uc->cp0_userlocal = v; break;
    case UC_MIPS_REG_CP0_STATUS:    uc->cp0_status = v; break;
    case UC_MIPS_REG_CP0_CAUSE:     uc->cp0_cause = v; break;
    case UC_MIPS_REG_CP0_EPC:       uc->cp0_epc = v; break;
    case UC_MIPS_REG_CP0_EBASE:     uc->cp0_ebase = v; break;
    case UC_MIPS_REG_CP0_INTCTL:    uc->cp0_intctl = v; break;
    case UC_MIPS_REG_CP0_EICVEC:    uc->eic_vec = (int32_t)v; break;
    case UC_MIPS_REG_CP0_CONFIG0:   uc->cp0_config0 = (uc->cp0_config0 & ~7u) | (v & 7u); break;   /* K0 only, as mtc0 */
    default: return UC_ERR_ARG;
  }
  return UC_ERR_OK;
}

uc_err uc_hook_add(uc_engine *uc, uc_hook *hh, int type, void *callback,
                   void *user_data, uint64_t begin, uint64_t end, ...) {
  if (!uc || !callback) return UC_ERR_ARG;
  if (type == UC_HOOK_CODE) {
    if (uc->n_code_hooks >= MAX_CODE_HOOKS) return UC_ERR_RESOURCE;
    uc->code_hooks[uc->n_code_hooks++] = (code_hook_t){begin, end, (uc_cb_hookcode_t)callback, user_data};
    uint32_t pa = (uint32_t)(begin & 0x1FFFFFFFull);
    if (begin == end && pa >= PFL_BASE && pa < PFL_BASE + PFL_SIZE) {
      uint32_t idx = (pa - PFL_BASE) >> 2;
      uc->pfl_code_hook_bm[idx >> 5] |= 1u << (idx & 31u);
    } else {
      uc->any_non_pfl_code_hook = true;
    }
    memset(uc->pfl_tblen, 0, sizeof(uc->pfl_tblen));
    memset(uc->bfl_tblen, 0, sizeof(uc->bfl_tblen));
  } else if (type == UC_HOOK_BLOCK) {
    if (uc->n_block_hooks >= MAX_BLOCK_HOOKS) return UC_ERR_RESOURCE;
    uc->block_hooks[uc->n_block_hooks++] = (code_hook_t){begin, end, (uc_cb_hookcode_t)callback, user_data};
  }
  if (hh) *hh = uc->next_hook_id++;
  return UC_ERR_OK;
}

uc_err uc_emu_stop(uc_engine *uc) {
  if (!uc) return UC_ERR_ARG;
  uc->stop_req = true;
  return UC_ERR_OK;
}

const char *uc_strerror(uc_err code) {
  switch (code) {
    case UC_ERR_OK: return "OK";
    case UC_ERR_NOMEM: return "Out of memory";
    case UC_ERR_ARCH: return "Invalid architecture";
    case UC_ERR_HANDLE: return "Invalid handle";
    case UC_ERR_MODE: return "Invalid mode";
    case UC_ERR_VERSION: return "Unsupported version";
    case UC_ERR_READ_UNMAPPED: return "Read unmapped";
    case UC_ERR_WRITE_UNMAPPED: return "Write unmapped";
    case UC_ERR_FETCH_UNMAPPED: return "Fetch unmapped";
    case UC_ERR_HOOK: return "Invalid hook";
    case UC_ERR_INSN_INVALID: return "Invalid instruction";
    case UC_ERR_MAP: return "Invalid memory mapping";
    case UC_ERR_WRITE_PROT: return "Write protected";
    case UC_ERR_READ_PROT: return "Read protected";
    case UC_ERR_FETCH_PROT: return "Fetch protected";
    case UC_ERR_ARG: return "Invalid argument";
    case UC_ERR_READ_UNALIGNED: return "Read unaligned";
    case UC_ERR_WRITE_UNALIGNED: return "Write unaligned";
    case UC_ERR_FETCH_UNALIGNED: return "Fetch unaligned";
    case UC_ERR_HOOK_EXIST: return "Hook already exists";
    case UC_ERR_RESOURCE: return "Insufficient resources";
    case UC_ERR_EXCEPTION: return "Unhandled CPU exception";
    default: return "Unicorn error";
  }
}

/* Fast physical instruction fetch */
static inline bool fetch_insn(const uc_engine *uc, uint32_t vaddr, uint32_t *out) {
  uint32_t pa = vaddr & 0x1FFFFFFFu;
  if (pa >= PFL_BASE && pa < PFL_BASE + PFL_SIZE) {
    *out = load_u32_le(uc->pfl + (pa - PFL_BASE));
    return true;
  }
  if (pa >= BFL_BASE && pa < BFL_BASE + BFL_SIZE) {
    *out = load_u32_le(uc->bfl + (pa - BFL_BASE));
    return true;
  }
  if (pa < RAM_SIZE) {
    *out = load_u32_le(uc->ram + pa);
    return true;
  }
  return false;
}

/* Classify how an instruction affects QEMU MIPS32 translation block termination:
 * 0 = normal sequential instruction
 * 1 = branch/jump with 1 delay slot (TB ends after the next instruction)
 * 2 = immediate TB stop (MTC0, EI, DI, ERET, DERET, WAIT, SYSCALL, BREAK, TEQ)
 */
static int classify_tb_insn(uint32_t w) {
  uint32_t op = w >> 26;
  switch (op) {
    case 0: { /* SPECIAL */
      uint32_t fn = w & 63u;
      if (fn == 8u || fn == 9u) return 1;                    /* JR, JALR */
      if (fn == 12u || fn == 13u || fn == 52u) return 2;     /* SYSCALL, BREAK, TEQ */
      return 0;
    }
    case 1: { /* REGIMM */
      uint32_t rt = (w >> 16) & 31u;
      if (rt == 0u || rt == 1u || rt == 16u || rt == 17u) return 1; /* BLTZ, BGEZ, BLTZAL, BGEZAL */
      return 0;
    }
    case 2: case 3:                                          /* J, JAL */
    case 4: case 5: case 6: case 7:                          /* BEQ, BNE, BLEZ, BGTZ */
    case 20: case 21: case 22: case 23:                      /* BEQL, BNEL, BLEZL, BGTZL */
      return 1;
    case 16: { /* COP0 */
      uint32_t rs = (w >> 21) & 31u;
      if (rs == 4u || rs == 11u || rs >= 16u) return 2;      /* MTC0, EI/DI, ERET/DERET/WAIT */
      return 0;
    }
    default:
      return 0;
  }
}

static uint8_t compute_tb_info(uc_engine *uc, uint32_t pc) {
  uint32_t page_start = pc & ~0xFFFu;
  uint32_t n = 0;
  bool has_hook = uc->n_code_hooks && uc->any_non_pfl_code_hook;
  while (n < 126u) {
    uint32_t cur = pc + n * 4u, w = 0;
    if (!fetch_insn(uc, cur, &w)) break;
    uint32_t pa = cur & 0x1FFFFFFFu;
    if (uc->n_code_hooks && !has_hook && pa >= PFL_BASE && pa < PFL_BASE + PFL_SIZE) {
      uint32_t idx = (pa - PFL_BASE) >> 2;
      if ((uc->pfl_code_hook_bm[idx >> 5] >> (idx & 31u)) & 1u) has_hook = true;
    }
    n++;
    int kind = classify_tb_insn(w);
    if (kind == 1) {
      /* Include branch delay slot */
      uint32_t dpa = (cur + 4u) & 0x1FFFFFFFu;
      if (uc->n_code_hooks && !has_hook && dpa >= PFL_BASE && dpa < PFL_BASE + PFL_SIZE) {
        uint32_t idx = (dpa - PFL_BASE) >> 2;
        if ((uc->pfl_code_hook_bm[idx >> 5] >> (idx & 31u)) & 1u) has_hook = true;
      }
      n++;
      break;
    }
    if (kind == 2) break;
    if ((cur + 4u) - page_start >= 0x1000u) break;
  }
  uint8_t len = (uint8_t)(n ? n : 1u);
  return has_hook ? (uint8_t)(len | 0x80u) : len;
}

static inline uint8_t get_tb_info(uc_engine *uc, uint32_t pc) {
  uint32_t pa = pc & 0x1FFFFFFFu;
  if (pa >= PFL_BASE && pa < PFL_BASE + PFL_SIZE) {
    uint32_t idx = (pa - PFL_BASE) >> 2;
    uint8_t info = uc->pfl_tblen[idx];
    if (!info) {
      info = compute_tb_info(uc, pc);
      uc->pfl_tblen[idx] = info;
    }
    return info;
  }
  if (pa >= BFL_BASE && pa < BFL_BASE + BFL_SIZE) {
    uint32_t idx = (pa - BFL_BASE) >> 2;
    uint8_t info = uc->bfl_tblen[idx];
    if (!info) {
      info = compute_tb_info(uc, pc);
      uc->bfl_tblen[idx] = info;
    }
    return info;
  }
  return compute_tb_info(uc, pc);
}

/* Guest data bus load/store */
static inline bool guest_load(uc_engine *uc, uint32_t vaddr, unsigned size, uint32_t *out) {
  uint32_t pa = vaddr & 0x1FFFFFFFu;
  if (pa < RAM_SIZE) {
    if (size == 4) *out = load_u32_le(uc->ram + pa);
    else if (size == 2) *out = load_u16_le(uc->ram + pa);
    else *out = uc->ram[pa];
    return true;
  }
  if (pa >= SFR_BASE && pa < SFR_BASE + SFR_SIZE && uc->mmio_read) {
    *out = (uint32_t)uc->mmio_read(uc, pa - SFR_BASE, size, uc->mmio_read_ud);
    return true;
  }
  if (pa >= PFL_BASE && pa < PFL_BASE + PFL_SIZE) {
    const uint8_t *p = uc->pfl + (pa - PFL_BASE);
    if (size == 4) *out = load_u32_le(p);
    else if (size == 2) *out = load_u16_le(p);
    else *out = *p;
    return true;
  }
  if (pa >= BFL_BASE && pa < BFL_BASE + BFL_SIZE) {
    const uint8_t *p = uc->bfl + (pa - BFL_BASE);
    if (size == 4) *out = load_u32_le(p);
    else if (size == 2) *out = load_u16_le(p);
    else *out = *p;
    return true;
  }
  fprintf(stderr, "[uc_shim] UNMAPPED READ vaddr=%08X pa=%08X size=%u pc=%08X ra=%08X\n",
          vaddr, pa, size, uc->pc, uc->gpr[31]);
  uc->err = UC_ERR_READ_UNMAPPED;
  uc->stop_req = true;
  return false;
}

static void wlog(uc_engine *uc, uint32_t w) {
  for (unsigned i = 0; i < uc->wlog_n; i++) if (uc->wlog_key[i] == w) return;
  if (uc->wlog_n == UC_SHIM_WLOG) { uc->wlog_full = true; return; }
  uc->wlog_key[uc->wlog_n] = w; uc->wlog_old[uc->wlog_n] = load_u32_le(uc->ram + w); uc->wlog_n++;
}
static inline bool guest_store(uc_engine *uc, uint32_t vaddr, unsigned size, uint32_t val) {
  uint32_t pa = vaddr & 0x1FFFFFFFu;
  if (pa < RAM_SIZE) {
    if (uc->wlog_on) wlog(uc, pa & ~3u);
    if (size == 4) store_u32_le(uc->ram + pa, val);
    else if (size == 2) store_u16_le(uc->ram + pa, (uint16_t)val);
    else uc->ram[pa] = (uint8_t)val;
    return true;
  }
  if (pa >= SFR_BASE && pa < SFR_BASE + SFR_SIZE && uc->mmio_write) {
    uc->mmio_write(uc, pa - SFR_BASE, size, val, uc->mmio_write_ud);
    return true;
  }
  fprintf(stderr, "[uc_shim] UNMAPPED WRITE vaddr=%08X pa=%08X size=%u val=%08X pc=%08X ra=%08X\n",
          vaddr, pa, size, val, uc->pc, uc->gpr[31]);
  uc->err = UC_ERR_WRITE_UNMAPPED;
  uc->stop_req = true;
  return false;
}

static uint32_t cp0_read(uc_engine *uc, uint32_t rd, uint32_t sel) {
  switch ((rd << 3) | sel) {
    case (9 << 3) | 0:  return uc->cp0_count;
    case (11 << 3) | 0: return uc->cp0_compare;
    case (12 << 3) | 0: return uc->cp0_status;
    case (12 << 3) | 1: return uc->cp0_intctl;
    case (12 << 3) | 2: return uc->cp0_srsctl;
    case (13 << 3) | 0: return uc->cp0_cause;
    case (14 << 3) | 0: return uc->cp0_epc;
    case (15 << 3) | 0: return 0x00019500u; /* PRId: MIPS32 M4K/M14K */
    case (15 << 3) | 1: return uc->cp0_ebase;
    case (16 << 3) | 0: return uc->cp0_config0;
    case (16 << 3) | 1: return 0x00000000u;
    case (16 << 3) | 2: return 0x00000000u;
    case (16 << 3) | 3: return uc->cp0_config3;
    default: return 0u;
  }
}

static void cp0_write(uc_engine *uc, uint32_t rd, uint32_t sel, uint32_t val) {
  switch ((rd << 3) | sel) {
    case (9 << 3) | 0:  uc->cp0_count = val; break;
    case (11 << 3) | 0: uc->cp0_compare = val; break;
    case (12 << 3) | 0: {
      const uint32_t mask = 0x1258FF17u;
      uc->cp0_status = (uc->cp0_status & ~mask) | (val & mask);
      break;
    }
    case (12 << 3) | 1:
      uc->cp0_intctl = (uc->cp0_intctl & ~0x3E0u) | (val & 0x3E0u);
      break;
    case (12 << 3) | 2: {
      const uint32_t mask = 0x003CF000u;
      uc->cp0_srsctl = (uc->cp0_srsctl & ~mask) | (val & mask);
      break;
    }
    case (13 << 3) | 0: {
      /* Guest MTC0 Cause only modifies writable bits (DC, IV, WP, IP1:0); RIPL 15:10 is driven by EIC */
      const uint32_t mask = 0x08C00300u;
      uc->cp0_cause = (uc->cp0_cause & ~mask) | (val & mask);
      break;
    }
    case (14 << 3) | 0:
      uc->cp0_epc = val;
      break;
    case (15 << 3) | 1:
      if (val & (1u << 11)) {
        uc->cp0_ebase = (uc->cp0_ebase & ~0xFFFFF800u) | (val & 0xFFFFF800u);
      } else {
        uc->cp0_ebase = 0x80000000u | (val & 0x3FFFF000u);
      }
      break;
    case (16 << 3) | 0:
      uc->cp0_config0 = (uc->cp0_config0 & ~7u) | (val & 7u);
      break;
    default:
      break;
  }
}

/* Execute a single instruction at uc->pc; updates uc->pc to the next PC (or sets *has_branch_target). */
static inline void exec_one_insn(uc_engine *uc, uint32_t w, bool check_code_hooks, bool *branch_pending, uint32_t *branch_target, bool *nullify_slot) {
  /* Check UC_HOOK_CODE if registered for this TB */
  if (check_code_hooks) {
    uint32_t pa = uc->pc & 0x1FFFFFFFu;
    bool check = uc->any_non_pfl_code_hook;
    if (!check && pa >= PFL_BASE && pa < PFL_BASE + PFL_SIZE) {
      uint32_t idx = (pa - PFL_BASE) >> 2;
      check = (uc->pfl_code_hook_bm[idx >> 5] >> (idx & 31u)) & 1u;
    }
    if (check) {
      for (unsigned i = 0; i < uc->n_code_hooks; i++) {
        if (uc->pc >= uc->code_hooks[i].begin && uc->pc <= uc->code_hooks[i].end) {
          uc->code_hooks[i].cb(uc, uc->pc, 4, uc->code_hooks[i].ud);
        }
      }
      if (uc->stop_req) return;
    }
  }

  uint32_t next_pc = *branch_pending ? *branch_target : (uc->pc + 4u);
  *branch_pending = false;

  uint32_t op = w >> 26;
  uint32_t rs = (w >> 21) & 31u;
  uint32_t rt = (w >> 16) & 31u;
  uint32_t rd = (w >> 11) & 31u;
  uint32_t sa = (w >> 6) & 31u;
  uint32_t fn = w & 63u;
  uint32_t uimm = w & 0xFFFFu;
  uint32_t simm = (uint32_t)(int32_t)(int16_t)uimm;
  uint32_t *gpr = uc->gpr;

  switch (op) {
    case 0: /* SPECIAL */
      switch (fn) {
        case 0: /* SLL */
          if (rd) gpr[rd] = gpr[rt] << sa;
          break;
        case 2: /* SRL / ROTR */
          if (rd) {
            uint32_t v = gpr[rt];
            if ((rs & 1u) && sa) gpr[rd] = (v >> sa) | (v << (32u - sa));
            else gpr[rd] = v >> sa;
          }
          break;
        case 3: /* SRA */
          if (rd) gpr[rd] = (uint32_t)((int32_t)gpr[rt] >> sa);
          break;
        case 4: /* SLLV */
          if (rd) gpr[rd] = gpr[rt] << (gpr[rs] & 31u);
          break;
        case 6: /* SRLV / ROTRV */
          if (rd) {
            uint32_t s = gpr[rs] & 31u, v = gpr[rt];
            if ((sa & 1u) && s) gpr[rd] = (v >> s) | (v << (32u - s));
            else gpr[rd] = v >> s;
          }
          break;
        case 7: /* SRAV */
          if (rd) gpr[rd] = (uint32_t)((int32_t)gpr[rt] >> (gpr[rs] & 31u));
          break;
        case 8: /* JR */
          *branch_pending = true;
          *branch_target = gpr[rs];
          break;
        case 9: /* JALR */
          *branch_pending = true;
          *branch_target = gpr[rs];
          if (rd) gpr[rd] = uc->pc + 8u;
          break;
        case 10: /* MOVZ */
          if (gpr[rt] == 0u && rd) gpr[rd] = gpr[rs];
          break;
        case 11: /* MOVN */
          if (gpr[rt] != 0u && rd) gpr[rd] = gpr[rs];
          break;
        case 12: /* SYSCALL */
        case 13: /* BREAK */
          uc->err = UC_ERR_EXCEPTION;
          uc->stop_req = true;
          return;
        case 15: /* SYNC */
          break;
        case 16: /* MFHI */
          if (rd) gpr[rd] = uc->hi;
          break;
        case 17: /* MTHI */
          uc->hi = gpr[rs];
          break;
        case 18: /* MFLO */
          if (rd) gpr[rd] = uc->lo;
          break;
        case 19: /* MTLO */
          uc->lo = gpr[rs];
          break;
        case 24: { /* MULT */
          int64_t p = (int64_t)(int32_t)gpr[rs] * (int64_t)(int32_t)gpr[rt];
          uc->hi = (uint32_t)((uint64_t)p >> 32);
          uc->lo = (uint32_t)p;
          break;
        }
        case 25: { /* MULTU */
          uint64_t p = (uint64_t)gpr[rs] * (uint64_t)gpr[rt];
          uc->hi = (uint32_t)(p >> 32);
          uc->lo = (uint32_t)p;
          break;
        }
        case 26: { /* DIV */
          int32_t num = (int32_t)gpr[rs], den = (int32_t)gpr[rt];
          if (den != 0) {
            if (num == (int32_t)0x80000000u && den == -1) {
              uc->lo = 0x80000000u;
              uc->hi = 0u;
            } else {
              uc->lo = (uint32_t)(num / den);
              uc->hi = (uint32_t)(num % den);
            }
          }
          break;
        }
        case 27: { /* DIVU */
          uint32_t num = gpr[rs], den = gpr[rt];
          if (den != 0u) {
            uc->lo = num / den;
            uc->hi = num % den;
          }
          break;
        }
        case 32: /* ADD */
        case 33: /* ADDU */
          if (rd) gpr[rd] = gpr[rs] + gpr[rt];
          break;
        case 34: /* SUB */
        case 35: /* SUBU */
          if (rd) gpr[rd] = gpr[rs] - gpr[rt];
          break;
        case 36: /* AND */
          if (rd) gpr[rd] = gpr[rs] & gpr[rt];
          break;
        case 37: /* OR */
          if (rd) gpr[rd] = gpr[rs] | gpr[rt];
          break;
        case 38: /* XOR */
          if (rd) gpr[rd] = gpr[rs] ^ gpr[rt];
          break;
        case 39: /* NOR */
          if (rd) gpr[rd] = ~(gpr[rs] | gpr[rt]);
          break;
        case 42: /* SLT */
          if (rd) gpr[rd] = (int32_t)gpr[rs] < (int32_t)gpr[rt] ? 1u : 0u;
          break;
        case 43: /* SLTU */
          if (rd) gpr[rd] = gpr[rs] < gpr[rt] ? 1u : 0u;
          break;
        case 52: /* TEQ */
          if (gpr[rs] == gpr[rt]) {
            uc->err = UC_ERR_EXCEPTION;
            uc->stop_req = true;
            return;
          }
          break;
        default:
          uc->err = UC_ERR_INSN_INVALID;
          uc->stop_req = true;
          return;
      }
      break;

    case 1: { /* REGIMM */
      bool cond = false;
      switch (rt) {
        case 0:  /* BLTZ */
          cond = (int32_t)gpr[rs] < 0;
          break;
        case 1:  /* BGEZ */
          cond = (int32_t)gpr[rs] >= 0;
          break;
        case 16: /* BLTZAL */
          cond = (int32_t)gpr[rs] < 0;
          gpr[31] = uc->pc + 8u;
          break;
        case 17: /* BGEZAL */
          cond = (int32_t)gpr[rs] >= 0;
          gpr[31] = uc->pc + 8u;
          break;
        default:
          break;
      }
      if (cond) {
        *branch_pending = true;
        *branch_target = uc->pc + 4u + (simm << 2);
      }
      break;
    }

    case 2: /* J */
      *branch_pending = true;
      *branch_target = ((uc->pc + 4u) & 0xF0000000u) | ((w & 0x03FFFFFFu) << 2);
      break;

    case 3: /* JAL */
      gpr[31] = uc->pc + 8u;
      *branch_pending = true;
      *branch_target = ((uc->pc + 4u) & 0xF0000000u) | ((w & 0x03FFFFFFu) << 2);
      break;

    case 4: /* BEQ */
      if (gpr[rs] == gpr[rt]) {
        *branch_pending = true;
        *branch_target = uc->pc + 4u + (simm << 2);
      }
      break;

    case 5: /* BNE */
      if (gpr[rs] != gpr[rt]) {
        *branch_pending = true;
        *branch_target = uc->pc + 4u + (simm << 2);
      }
      break;

    case 6: /* BLEZ */
      if ((int32_t)gpr[rs] <= 0) {
        *branch_pending = true;
        *branch_target = uc->pc + 4u + (simm << 2);
      }
      break;

    case 7: /* BGTZ */
      if ((int32_t)gpr[rs] > 0) {
        *branch_pending = true;
        *branch_target = uc->pc + 4u + (simm << 2);
      }
      break;

    case 8: /* ADDI */
    case 9: /* ADDIU */
      if (rt) gpr[rt] = gpr[rs] + simm;
      break;

    case 10: /* SLTI */
      if (rt) gpr[rt] = (int32_t)gpr[rs] < (int32_t)simm ? 1u : 0u;
      break;

    case 11: /* SLTIU */
      if (rt) gpr[rt] = gpr[rs] < simm ? 1u : 0u;
      break;

    case 12: /* ANDI */
      if (rt) gpr[rt] = gpr[rs] & uimm;
      break;

    case 13: /* ORI */
      if (rt) gpr[rt] = gpr[rs] | uimm;
      break;

    case 14: /* XORI */
      if (rt) gpr[rt] = gpr[rs] ^ uimm;
      break;

    case 15: /* LUI */
      if (rt) gpr[rt] = uimm << 16;
      break;

    case 16: /* COP0 */
      switch (rs) {
        case 0: /* MFC0 */
          if (rt) gpr[rt] = cp0_read(uc, rd, w & 7u);
          break;
        case 4: /* MTC0 */
          cp0_write(uc, rd, w & 7u, gpr[rt]);
          break;
        case 11: { /* MFMC0 (EI / DI) */
          uint32_t old = uc->cp0_status;
          if (w & (1u << 5)) uc->cp0_status |= 1u;
          else uc->cp0_status &= ~1u;
          if (rt) gpr[rt] = old;
          break;
        }
        case 10: /* RDPGPR */
          if (rd) gpr[rd] = ((uc->cp0_srsctl >> 6) & 15u) ? uc->srs_gpr[rt] : gpr[rt];
          break;
        case 14: /* WRPGPR */
          if (((uc->cp0_srsctl >> 6) & 15u) == 0u) {
            if (rd) gpr[rd] = gpr[rt];
          } else {
            if (rd) uc->srs_gpr[rd] = gpr[rt];
          }
          break;
        default:
          if (rs >= 16u) {
            if (fn == 24u) { /* ERET */
              if (uc->cp0_status & (1u << 2)) {
                uc->cp0_status &= ~(1u << 2);
              } else {
                uc->cp0_status &= ~(1u << 1);
              }
              next_pc = uc->cp0_epc;
            } else if (fn == 31u) { /* DERET */
              next_pc = uc->cp0_epc;
            }
          }
          break;
      }
      break;

    case 20: /* BEQL */
      if (gpr[rs] == gpr[rt]) {
        *branch_pending = true;
        *branch_target = uc->pc + 4u + (simm << 2);
      } else {
        *nullify_slot = true;
      }
      break;

    case 21: /* BNEL */
      if (gpr[rs] != gpr[rt]) {
        *branch_pending = true;
        *branch_target = uc->pc + 4u + (simm << 2);
      } else {
        *nullify_slot = true;
      }
      break;

    case 22: /* BLEZL */
      if ((int32_t)gpr[rs] <= 0) {
        *branch_pending = true;
        *branch_target = uc->pc + 4u + (simm << 2);
      } else {
        *nullify_slot = true;
      }
      break;

    case 23: /* BGTZL */
      if ((int32_t)gpr[rs] > 0) {
        *branch_pending = true;
        *branch_target = uc->pc + 4u + (simm << 2);
      } else {
        *nullify_slot = true;
      }
      break;

    case 28: /* SPECIAL2 */
      switch (fn) {
        case 0: { /* MADD */
          int64_t acc = ((int64_t)(uint64_t)uc->hi << 32) | (uint64_t)uc->lo;
          acc += (int64_t)(int32_t)gpr[rs] * (int64_t)(int32_t)gpr[rt];
          uc->hi = (uint32_t)((uint64_t)acc >> 32);
          uc->lo = (uint32_t)acc;
          break;
        }
        case 1: { /* MADDU */
          uint64_t acc = ((uint64_t)uc->hi << 32) | (uint64_t)uc->lo;
          acc += (uint64_t)gpr[rs] * (uint64_t)gpr[rt];
          uc->hi = (uint32_t)(acc >> 32);
          uc->lo = (uint32_t)acc;
          break;
        }
        case 2: /* MUL */
          if (rd) gpr[rd] = (uint32_t)((int32_t)gpr[rs] * (int32_t)gpr[rt]);
          break;
        case 4: { /* MSUB */
          int64_t acc = ((int64_t)(uint64_t)uc->hi << 32) | (uint64_t)uc->lo;
          acc -= (int64_t)(int32_t)gpr[rs] * (int64_t)(int32_t)gpr[rt];
          uc->hi = (uint32_t)((uint64_t)acc >> 32);
          uc->lo = (uint32_t)acc;
          break;
        }
        case 5: { /* MSUBU */
          uint64_t acc = ((uint64_t)uc->hi << 32) | (uint64_t)uc->lo;
          acc -= (uint64_t)gpr[rs] * (uint64_t)gpr[rt];
          uc->hi = (uint32_t)(acc >> 32);
          uc->lo = (uint32_t)acc;
          break;
        }
        case 32: /* CLZ */
          if (rd) gpr[rd] = gpr[rs] ? (uint32_t)__builtin_clz(gpr[rs]) : 32u;
          break;
        case 33: /* CLO */
          if (rd) {
            uint32_t inv = ~gpr[rs];
            gpr[rd] = inv ? (uint32_t)__builtin_clz(inv) : 32u;
          }
          break;
        case 63: /* SDBBP: EJTAG debug exception (vector 0xBFC00480 with no probe). Not modelled: the core stops, as
                  * for SYSCALL / BREAK / TEQ, instead of executing it as a no-op. */
          uc->err = UC_ERR_EXCEPTION;
          uc->stop_req = true;
          return;
        default: /* undefined SPECIAL2 function: Reserved Instruction exception on silicon; the core stops (as for
                  * every other undefined opcode here) instead of executing it as a no-op */
          uc->err = UC_ERR_INSN_INVALID;
          uc->stop_req = true;
          return;
      }
      break;

    case 31: /* SPECIAL3 */
      switch (fn) {
        case 0: { /* EXT */
          uint32_t size = rd + 1u;
          uint32_t mask = size >= 32u ? 0xFFFFFFFFu : ((1u << size) - 1u);
          if (rt) gpr[rt] = (gpr[rs] >> sa) & mask;
          break;
        }
        case 4: { /* INS */
          uint32_t size = rd - sa + 1u;
          uint32_t mask = size >= 32u ? 0xFFFFFFFFu : ((1u << size) - 1u);
          if (rt) gpr[rt] = (gpr[rt] & ~(mask << sa)) | ((gpr[rs] & mask) << sa);
          break;
        }
        case 32: /* BSHFL */
          if (rd) {
            if (sa == 2u) { /* WSBH */
              uint32_t v = gpr[rt];
              gpr[rd] = ((v & 0x00FF00FFu) << 8) | ((v & 0xFF00FF00u) >> 8);
            } else if (sa == 16u) { /* SEB */
              gpr[rd] = (uint32_t)(int32_t)(int8_t)gpr[rt];
            } else if (sa == 24u) { /* SEH */
              gpr[rd] = (uint32_t)(int32_t)(int16_t)gpr[rt];
            }
          }
          break;
        default: /* undefined SPECIAL3 function (RDHWR not modelled): Reserved Instruction -> the core stops */
          uc->err = UC_ERR_INSN_INVALID;
          uc->stop_req = true;
          return;
      }
      break;

    case 32: { /* LB */
      uint32_t v = 0;
      if (!guest_load(uc, gpr[rs] + simm, 1, &v)) return;
      if (rt) gpr[rt] = (uint32_t)(int32_t)(int8_t)v;
      break;
    }
    case 33: { /* LH */
      uint32_t v = 0;
      if (!guest_load(uc, gpr[rs] + simm, 2, &v)) return;
      if (rt) gpr[rt] = (uint32_t)(int32_t)(int16_t)v;
      break;
    }
    case 34: { /* LWL */
      uint32_t addr = gpr[rs] + simm, wmem = 0;
      if (!guest_load(uc, addr & ~3u, 4, &wmem)) return;
      if (rt) {
        uint32_t shift = (3u - (addr & 3u)) * 8u;
        uint32_t mask = shift ? ((1u << shift) - 1u) : 0u;
        gpr[rt] = (gpr[rt] & mask) | (wmem << shift);
      }
      break;
    }
    case 35: { /* LW */
      uint32_t v = 0;
      if (!guest_load(uc, gpr[rs] + simm, 4, &v)) return;
      if (rt) gpr[rt] = v;
      break;
    }
    case 36: { /* LBU */
      uint32_t v = 0;
      if (!guest_load(uc, gpr[rs] + simm, 1, &v)) return;
      if (rt) gpr[rt] = v & 0xFFu;
      break;
    }
    case 37: { /* LHU */
      uint32_t v = 0;
      if (!guest_load(uc, gpr[rs] + simm, 2, &v)) return;
      if (rt) gpr[rt] = v & 0xFFFFu;
      break;
    }
    case 38: { /* LWR */
      uint32_t addr = gpr[rs] + simm, wmem = 0;
      if (!guest_load(uc, addr & ~3u, 4, &wmem)) return;
      if (rt) {
        uint32_t shift = (addr & 3u) * 8u;
        uint32_t mask = shift ? (0xFFFFFFFFu << (32u - shift)) : 0u;
        gpr[rt] = (gpr[rt] & mask) | (wmem >> shift);
      }
      break;
    }
    case 40: /* SB */
      if (!guest_store(uc, gpr[rs] + simm, 1, gpr[rt] & 0xFFu)) return;
      break;
    case 41: /* SH */
      if (!guest_store(uc, gpr[rs] + simm, 2, gpr[rt] & 0xFFFFu)) return;
      break;
    case 42: { /* SWL */
      uint32_t addr = gpr[rs] + simm, wmem = 0;
      if (!guest_load(uc, addr & ~3u, 4, &wmem)) return;
      uint32_t shift = (3u - (addr & 3u)) * 8u;
      uint32_t mask = shift ? (0xFFFFFFFFu << (32u - shift)) : 0u;
      if (!guest_store(uc, addr & ~3u, 4, (wmem & mask) | (gpr[rt] >> shift))) return;
      break;
    }
    case 43: /* SW */
      if (!guest_store(uc, gpr[rs] + simm, 4, gpr[rt])) return;
      break;
    case 46: { /* SWR */
      uint32_t addr = gpr[rs] + simm, wmem = 0;
      if (!guest_load(uc, addr & ~3u, 4, &wmem)) return;
      uint32_t shift = (addr & 3u) * 8u;
      uint32_t mask = shift ? ((1u << shift) - 1u) : 0u;
      if (!guest_store(uc, addr & ~3u, 4, (wmem & mask) | (gpr[rt] << shift))) return;
      break;
    }
    case 47: /* CACHE (no-op) */
      break;

    default:
      uc->err = UC_ERR_INSN_INVALID;
      uc->stop_req = true;
      return;
  }

  gpr[0] = 0u;
  if (*nullify_slot) {
    uc->pc = next_pc + 4u;
  } else {
    uc->pc = next_pc;
  }
}

static inline bool check_and_take_eic_irq(uc_engine *uc) {
  uint32_t st = uc->cp0_status, ca = uc->cp0_cause;
  if ((st & 1u) && !(st & 6u) && ((ca >> 10) & 0x3Fu) > ((st >> 10) & 0x3Fu)) {
    uc->cp0_epc = uc->pc;
    uc->cp0_cause &= ~((1u << 31) | (0x1Fu << 2));
    uc->cp0_status |= (1u << 1); /* EXL = 1 */
    uc->exceptions++;
    uint32_t offset = 0x200u;
    uint32_t spacing = (uc->cp0_intctl >> 5) & 0x1Fu;
    if ((ca & (1u << 23)) && !(st & (1u << 22)) && spacing) {
      uint32_t vec = uc->eic_vec ? (uint32_t)(uc->eic_vec - 1) : ((ca >> 8) & 0xFFu);
      offset = 0x200u + vec * (spacing << 5);
    }
    uint32_t base = (st & (1u << 22)) ? 0xBFC00000u : (uc->cp0_ebase & ~0xFFFu);
    uc->pc = base + offset;
    return true;
  }
  return false;
}

uc_err uc_emu_start(uc_engine *uc, uint64_t begin, uint64_t until, uint64_t timeout, size_t count) {
  (void)timeout;
  (void)count;
  if (!uc) return UC_ERR_ARG;

  uc->pc = (uint32_t)begin;
  uc->stop_req = false;
  uc->err = UC_ERR_OK;

  while (!uc->stop_req) {
    /* 1. Check pending EIC interrupt before starting the next basic block */
    check_and_take_eic_irq(uc);

    if (uc->pc == (uint32_t)until) break;

    /* 2. Determine basic-block length and invoke UC_HOOK_BLOCK */
    uint8_t tb_info = get_tb_info(uc, uc->pc);
    uint32_t n_insns = tb_info & 0x7Fu;
    bool tb_has_hook = (tb_info & 0x80u) != 0u;
    for (unsigned i = 0; i < uc->n_block_hooks; i++) {
      uc->block_hooks[i].cb(uc, uc->pc, n_insns * 4u, uc->block_hooks[i].ud);
    }
    if (uc->stop_req) break;

    /* If hook_block (e.g. after idle warp + service_events) raised a deliverable EIC interrupt, vector immediately */
    if (check_and_take_eic_irq(uc)) continue;

    /* 3. Execute instructions in the basic block */
    uint32_t pa0 = uc->pc & 0x1FFFFFFFu;
    const uint8_t *tb_ptr = NULL;
    if (pa0 >= PFL_BASE && pa0 + n_insns * 4u <= PFL_BASE + PFL_SIZE) {
      tb_ptr = uc->pfl + (pa0 - PFL_BASE);
    } else if (pa0 >= BFL_BASE && pa0 + n_insns * 4u <= BFL_BASE + BFL_SIZE) {
      tb_ptr = uc->bfl + (pa0 - BFL_BASE);
    }
    bool branch_pending = false, nullify_slot = false;
    uint32_t branch_target = 0, i = 0;
    for (; i < n_insns && !uc->stop_req; i++) {
      uint32_t w = 0;
      if (tb_ptr) {
        w = load_u32_le(tb_ptr + i * 4u);
      } else if (!fetch_insn(uc, uc->pc, &w)) {
        uc->err = UC_ERR_FETCH_UNMAPPED;
        uc->stop_req = true;
        break;
      }
      bool skipped_slot = false;
      exec_one_insn(uc, w, tb_has_hook, &branch_pending, &branch_target, &skipped_slot);
      if (skipped_slot) {
        /* Branch-likely not taken: delay slot was skipped, block ends now */
        i++;
        break;
      }
      /* If an MMIO write inside this instruction raised a deliverable EIC interrupt (and we are not in a delay slot), exit TB */
      if (!branch_pending && ((uc->cp0_status & 7u) == 1u) &&
          ((uc->cp0_cause >> 10) & 0x3Fu) > ((uc->cp0_status >> 10) & 0x3Fu)) {
        i++;
        break;
      }
      (void)nullify_slot;
    }
    if (!(uc->cp0_cause & (1u << 27))) uc->cp0_count += i >> 1;
  }

  return uc->err;
}

unsigned uc_shim_core_regs(const uc_engine *uc, uint32_t *out) {
  unsigned n = 0;
  for (unsigned i = 0; i < 32; i++) out[n++] = uc->gpr[i];
  for (unsigned i = 0; i < 32; i++) out[n++] = uc->srs_gpr[i];
  const uint32_t r[] = {uc->pc, uc->hi, uc->lo, uc->cp0_status, uc->cp0_cause, uc->cp0_epc, uc->cp0_ebase,
                        uc->cp0_intctl, uc->cp0_srsctl, uc->cp0_config0, uc->cp0_config3, uc->cp0_userlocal,
                        uc->cp0_compare, (uint32_t)uc->eic_vec};
  for (size_t i = 0; i < sizeof r / sizeof r[0]; i++) out[n++] = r[i];
  return n;
}
void uc_shim_wlog_start(uc_engine *uc) { uc->wlog_on = true; uc->wlog_full = false; uc->wlog_n = 0; }
void uc_shim_wlog_stop(uc_engine *uc) { uc->wlog_on = false; uc->wlog_full = false; uc->wlog_n = 0; }
bool uc_shim_wlog_full(const uc_engine *uc) { return uc->wlog_full; }
uint32_t uc_shim_exceptions(const uc_engine *uc) { return uc->exceptions; }
bool uc_shim_wlog_restored(const uc_engine *uc) {
  if (uc->wlog_full) return false;
  for (unsigned i = 0; i < uc->wlog_n; i++) if (load_u32_le(uc->ram + uc->wlog_key[i]) != uc->wlog_old[i]) return false;
  return true;
}

/* ---- snapshots (emu/common/s32snap.h): the whole engine state, its callback pointers written as indices into the
 * caller's table `fns` and its user-data pointers as 1 (= `ud`), so a snapshot does not depend on where this process
 * placed code and data. */
size_t uc_shim_snap_size(void) { return sizeof(struct uc_struct); }

static int fn_index(const void *f, void *const *fns, size_t nfns) {
  if (!f) return 0;
  for (size_t i = 0; i < nfns; i++) if (fns[i] == f) return (int)i + 1;
  return -1;
}
static int ud_index(const void *p, const void *ud) { return !p ? 0 : p == ud ? 1 : -1; }

int uc_shim_snap_save(const uc_engine *uc, uint8_t *out, void *const *fns, size_t nfns, const void *ud) {
  struct uc_struct *c = (struct uc_struct *)out;        /* out holds uc_shim_snap_size() bytes */
  memcpy(c, uc, sizeof *c);
  int a = fn_index((const void *)c->mmio_read, fns, nfns), b = fn_index((const void *)c->mmio_write, fns, nfns);
  int ua = ud_index(c->mmio_read_ud, ud), ub = ud_index(c->mmio_write_ud, ud);
  if (a < 0 || b < 0 || ua < 0 || ub < 0) return -1;
  c->mmio_read = (uc_cb_mmio_read_t)(uintptr_t)a; c->mmio_write = (uc_cb_mmio_write_t)(uintptr_t)b;
  c->mmio_read_ud = (void *)(uintptr_t)ua; c->mmio_write_ud = (void *)(uintptr_t)ub;
  for (unsigned i = 0; i < MAX_CODE_HOOKS; i++) {
    int f = fn_index((const void *)c->code_hooks[i].cb, fns, nfns), u = ud_index(c->code_hooks[i].ud, ud);
    if (f < 0 || u < 0) return -1;
    c->code_hooks[i].cb = (uc_cb_hookcode_t)(uintptr_t)f; c->code_hooks[i].ud = (void *)(uintptr_t)u;
  }
  for (unsigned i = 0; i < MAX_BLOCK_HOOKS; i++) {
    int f = fn_index((const void *)c->block_hooks[i].cb, fns, nfns), u = ud_index(c->block_hooks[i].ud, ud);
    if (f < 0 || u < 0) return -1;
    c->block_hooks[i].cb = (uc_cb_hookcode_t)(uintptr_t)f; c->block_hooks[i].ud = (void *)(uintptr_t)u;
  }
  return 0;
}

int uc_shim_snap_restore(uc_engine *uc, const uint8_t *in, void *const *fns, size_t nfns, void *ud) {
  struct uc_struct *c = malloc(sizeof *c);
  if (!c) return -1;
  memcpy(c, in, sizeof *c);
  int rc = -1;
#define FN(T, x) do { uintptr_t i_ = (uintptr_t)(x); if (i_ > nfns) goto out; (x) = i_ ? (T)fns[i_ - 1] : NULL; } while (0)
#define UD(x) do { uintptr_t i_ = (uintptr_t)(x); if (i_ > 1) goto out; (x) = i_ ? ud : NULL; } while (0)
  FN(uc_cb_mmio_read_t, c->mmio_read); FN(uc_cb_mmio_write_t, c->mmio_write); UD(c->mmio_read_ud); UD(c->mmio_write_ud);
  for (unsigned i = 0; i < MAX_CODE_HOOKS; i++) { FN(uc_cb_hookcode_t, c->code_hooks[i].cb); UD(c->code_hooks[i].ud); }
  for (unsigned i = 0; i < MAX_BLOCK_HOOKS; i++) { FN(uc_cb_hookcode_t, c->block_hooks[i].cb); UD(c->block_hooks[i].ud); }
#undef FN
#undef UD
  memcpy(uc, c, sizeof *c);
  rc = 0;
out:
  free(c);
  return rc;
}
