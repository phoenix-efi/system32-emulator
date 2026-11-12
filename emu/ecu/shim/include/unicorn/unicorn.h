/* Minimal Unicorn API header for the pure-C11 MIPS32 M4K interpreter shim (uc_mips32_shim.c).
 * Matches the subset of <unicorn/unicorn.h> + <unicorn/mips.h> used by emu/ecu and emu/pair.
 */
#ifndef UNICORN_SHIM_H
#define UNICORN_SHIM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#undef mips

typedef struct uc_struct uc_engine;
typedef size_t uc_hook;

typedef enum uc_err {
  UC_ERR_OK = 0,
  UC_ERR_NOMEM,
  UC_ERR_ARCH,
  UC_ERR_HANDLE,
  UC_ERR_MODE,
  UC_ERR_VERSION,
  UC_ERR_READ_UNMAPPED,
  UC_ERR_WRITE_UNMAPPED,
  UC_ERR_FETCH_UNMAPPED,
  UC_ERR_HOOK,
  UC_ERR_INSN_INVALID,
  UC_ERR_MAP,
  UC_ERR_WRITE_PROT,
  UC_ERR_READ_PROT,
  UC_ERR_FETCH_PROT,
  UC_ERR_ARG,
  UC_ERR_READ_UNALIGNED,
  UC_ERR_WRITE_UNALIGNED,
  UC_ERR_FETCH_UNALIGNED,
  UC_ERR_HOOK_EXIST,
  UC_ERR_RESOURCE,
  UC_ERR_EXCEPTION
} uc_err;

typedef enum uc_arch {
  UC_ARCH_MIPS = 5
} uc_arch;

typedef enum uc_mode {
  UC_MODE_LITTLE_ENDIAN = 0,
  UC_MODE_MIPS32 = 4
} uc_mode;

typedef enum uc_prot {
  UC_PROT_NONE = 0,
  UC_PROT_READ = 1,
  UC_PROT_WRITE = 2,
  UC_PROT_EXEC = 4,
  UC_PROT_ALL = 7
} uc_prot;

typedef enum uc_hook_type {
  UC_HOOK_INTR = 1 << 0,
  UC_HOOK_INSN = 1 << 1,
  UC_HOOK_CODE = 1 << 2,
  UC_HOOK_BLOCK = 1 << 3
} uc_hook_type;

typedef enum uc_cpu_mips32 {
  UC_CPU_MIPS32_4KC = 0,
  UC_CPU_MIPS32_M14K = 11
} uc_cpu_mips32;

typedef enum uc_mips_reg {
  UC_MIPS_REG_INVALID = 0,
  UC_MIPS_REG_PC,
  UC_MIPS_REG_0,
  UC_MIPS_REG_1,
  UC_MIPS_REG_2,
  UC_MIPS_REG_3,
  UC_MIPS_REG_4,
  UC_MIPS_REG_5,
  UC_MIPS_REG_6,
  UC_MIPS_REG_7,
  UC_MIPS_REG_8,
  UC_MIPS_REG_9,
  UC_MIPS_REG_10,
  UC_MIPS_REG_11,
  UC_MIPS_REG_12,
  UC_MIPS_REG_13,
  UC_MIPS_REG_14,
  UC_MIPS_REG_15,
  UC_MIPS_REG_16,
  UC_MIPS_REG_17,
  UC_MIPS_REG_18,
  UC_MIPS_REG_19,
  UC_MIPS_REG_20,
  UC_MIPS_REG_21,
  UC_MIPS_REG_22,
  UC_MIPS_REG_23,
  UC_MIPS_REG_24,
  UC_MIPS_REG_25,
  UC_MIPS_REG_26,
  UC_MIPS_REG_27,
  UC_MIPS_REG_28,
  UC_MIPS_REG_29,
  UC_MIPS_REG_30,
  UC_MIPS_REG_31,
  UC_MIPS_REG_HI,
  UC_MIPS_REG_LO,
  UC_MIPS_REG_CP0_CONFIG3,
  UC_MIPS_REG_CP0_USERLOCAL,
  UC_MIPS_REG_CP0_STATUS,
  UC_MIPS_REG_CP0_CAUSE,
  UC_MIPS_REG_CP0_EPC,
  UC_MIPS_REG_CP0_EBASE,
  UC_MIPS_REG_CP0_INTCTL,
  UC_MIPS_REG_CP0_EICVEC,
  UC_MIPS_REG_CP0_CONFIG0,      /* shim only (UC_SHIM_HAS_CP0_CONFIG0) */
  UC_MIPS_REG_ENDING
} uc_mips_reg;

#define UC_MIPS_REG_RA UC_MIPS_REG_31
#define UC_SHIM_HAS_CP0_CONFIG0 1

typedef void (*uc_cb_hookcode_t)(uc_engine *uc, uint64_t address, uint32_t size, void *user_data);
typedef uint64_t (*uc_cb_mmio_read_t)(uc_engine *uc, uint64_t offset, unsigned size, void *user_data);
typedef void (*uc_cb_mmio_write_t)(uc_engine *uc, uint64_t offset, unsigned size, uint64_t value, void *user_data);

uc_err uc_open(uc_arch arch, uc_mode mode, uc_engine **uch);
uc_err uc_close(uc_engine *uc);
uc_err uc_ctl_set_cpu_model(uc_engine *uc, int model);
uc_err uc_mem_map(uc_engine *uc, uint64_t address, size_t size, uint32_t perms);
uc_err uc_mem_read(uc_engine *uc, uint64_t address, void *bytes, size_t size);
uc_err uc_mem_write(uc_engine *uc, uint64_t address, const void *bytes, size_t size);
uc_err uc_mmio_map(uc_engine *uc, uint64_t address, size_t size,
                   uc_cb_mmio_read_t read_cb, void *user_data_read,
                   uc_cb_mmio_write_t write_cb, void *user_data_write);
uc_err uc_reg_read(uc_engine *uc, int regid, void *value);
uc_err uc_reg_write(uc_engine *uc, int regid, const void *value);
uc_err uc_hook_add(uc_engine *uc, uc_hook *hh, int type, void *callback,
                   void *user_data, uint64_t begin, uint64_t end, ...);
uc_err uc_emu_start(uc_engine *uc, uint64_t begin, uint64_t until, uint64_t timeout, size_t count);
uc_err uc_emu_stop(uc_engine *uc);
/* Shim extensions for the ECU model's idle-loop watch: the core's registers except the cycle-driven Count (general
 * registers and the shadow set, PC, HI/LO, coprocessor 0) into out[UC_SHIM_CORE_REGS], returning the count; and a
 * write log of guest stores to RAM: start (clears it), stop, whether it overflowed, and whether every word written
 * since the start holds its value from before the first write again. */
#define UC_SHIM_CORE_REGS 78
#define UC_SHIM_WLOG 32
/* Snapshots (emu/common/s32snap.h): the engine's whole state in uc_shim_snap_size() bytes, its callback pointers written
 * as indices into `fns` and its user-data pointers as 1 (= `ud`). Save and restore return 0, or -1. */
size_t uc_shim_snap_size(void);
int uc_shim_snap_save(const uc_engine *uc, uint8_t *out, void *const *fns, size_t nfns, const void *ud);
int uc_shim_snap_restore(uc_engine *uc, const uint8_t *in, void *const *fns, size_t nfns, void *ud);
unsigned uc_shim_core_regs(const uc_engine *uc, uint32_t *out);
void uc_shim_wlog_start(uc_engine *uc);
void uc_shim_wlog_stop(uc_engine *uc);
bool uc_shim_wlog_full(const uc_engine *uc);
bool uc_shim_wlog_restored(const uc_engine *uc);
uint32_t uc_shim_exceptions(const uc_engine *uc);       /* interrupts taken so far */
const char *uc_strerror(uc_err code);

#ifdef __cplusplus
}
#endif

#endif /* UNICORN_SHIM_H */
