/*
 * pic24.h -- PIC24F CPU core (PIC24FJ256DA210 flavour: PIC24F ISA + EDS data paging).
 *
 * Written from Microchip's documentation only:
 *   PRM  = "16-bit MCU and DSC Programmer's Reference Manual", DS70000157G
 *   DS   = PIC24FJ256DA210 family data sheet, DS39969B
 *   FRM2 = PIC24F FRM Section 2 "CPU", DS39703A
 *   FRM8 = PIC24F FRM Section 8 "Interrupts", DS39707A
 *   FRM45= PIC24F FRM Section 45 "Data Memory with EDS", DS39733A
 * Non-obvious behaviours cite the PIC24FJ256DA210 data sheet (DS39969) and the dsPIC33/PIC24 Family Reference
 * Manual sections next to the code that implements them.
 *
 * Design rules: no globals (any number of pic24_cpu instances may coexist), no allocation in the hot path,
 * peripherals attach through the SFR bus callbacks (data addresses 0x0000-0x07FF that the core does not own).
 */
#ifndef PIC24_H
#define PIC24_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- device geometry (DS Fig 4-1, 4-3, Table 4-2) ---------------------------------------------------------- */
#define PIC24_PROG_TOP       0x02AC00u /* first unimplemented PC address (PIC24FJ256DA2xx: 87,552 words)       */
#define PIC24_PROG_WORDS     (PIC24_PROG_TOP / 2u)
#define PIC24_WLOG           32        /* write-log entries (pic24_wlog_start)                                    */
#define PIC24_DATA_LOW_TOP   0x8000u   /* SFR 0x0000-0x07FF + 30 KB RAM 0x0800-0x7FFF                          */
#define PIC24_EDS_RAM_BASE   0x008000u /* EDS address of the first extended-RAM byte (DSxPAG=1, EA 0x8000)       */
#define PIC24_EDS_RAM_TOP    0x018800u /* 66 KB extended RAM: EDS 0x008000-0x0187FF (DS Table 4-35)             */

/* ---- SR bits (DS Register 3-1) -------------------------------------------------------------------------------- */
#define PIC24_SR_C   0x0001u
#define PIC24_SR_Z   0x0002u
#define PIC24_SR_OV  0x0004u
#define PIC24_SR_N   0x0008u
#define PIC24_SR_RA  0x0010u
#define PIC24_SR_IPL 0x00E0u
#define PIC24_SR_DC  0x0100u

/* ---- INTCON1 trap status bits (DS Register 7-3) ------------------------------------------------------------- */
#define PIC24_INTCON1_OSCFAIL 0x0002u
#define PIC24_INTCON1_STKERR  0x0004u
#define PIC24_INTCON1_ADDRERR 0x0008u
#define PIC24_INTCON1_MATHERR 0x0010u
#define PIC24_INTCON1_NSTDIS  0x8000u
#define PIC24_INTCON2_ALTIVT  0x8000u
#define PIC24_INTCON2_DISI    0x4000u

/* number of interrupt-controller sources (IRQ numbers 0..117, DS Table 7-2) */
#define PIC24_NUM_IRQ 118

/* Why pic24_step / pic24_run stopped (0 = normal). */
typedef enum {
  PIC24_OK = 0,
  PIC24_STOP_ILLEGAL,      /* cfg.illegal_stops only: illegal opcode/address mode; stop_insn holds it, PC kept */
  PIC24_STOP_RESET,        /* device reset executed: RESET instruction, or illegal opcode / BREAK (silicon default)  */
  PIC24_STOP_TRAP_CONFLICT,/* lower-priority hard trap while a hard trap was pending -> device reset     */
  PIC24_STOP_SLEEP,        /* PWRSAV #0 with no wake-up source (pic24_run returns; state = sleeping)     */
  PIC24_STOP_BREAK,        /* a callback requested a stop (cpu->stop_request), or BREAK with cfg.debugger set     */
  PIC24_STOP_CYCLES        /* pic24_run budget exhausted (not an error)                                   */
} pic24_status;

typedef enum { PIC24_RUNNING = 0, PIC24_SLEEP = 1, PIC24_IDLE = 2 } pic24_power;

struct pic24_cpu;

/* Bus callbacks. All optional (NULL = core default). `ctx` is pic24_config.ctx. */
typedef struct pic24_config {
  void *ctx;
  /* SFR space not owned by the core (see docs). addr is even, 0x0020..0x07FE. Byte accesses are word reads /
   * lane-masked word writes (DS 4.2.2: "data byte reads will read the complete word"). Default: plain storage in
   * cpu->sfr_shadow (initialised by pic24_set_sfr_reset). */
  uint16_t (*sfr_read)(void *ctx, struct pic24_cpu *cpu, uint16_t addr);
  void (*sfr_write)(void *ctx, struct pic24_cpu *cpu, uint16_t addr, uint16_t value, uint16_t lane_mask);
  /* EDS window pages DSxPAG 0x004-0x1FF and page 3 above 0x0187FF (EPMP external bus). eds_addr is the 24-bit
   * EDS address. Return false = no device (address error trap, FRM45 45.3.1.2 note 1). Default: address error. */
  bool (*eds_ext_read)(void *ctx, struct pic24_cpu *cpu, uint32_t eds_addr, uint16_t *value);
  bool (*eds_ext_write)(void *ctx, struct pic24_cpu *cpu, uint32_t eds_addr, uint16_t value, uint16_t lane_mask);
  /* TBLWTL/TBLWTH (flash write latches -- the NVM controller is a peripheral). addr24 = TBLPAG:EA (even),
   * high = TBLWTH. Default: ignored. */
  void (*tblwt)(void *ctx, struct pic24_cpu *cpu, uint32_t addr24, uint16_t value, uint16_t lane_mask, bool high);
  /* TBLRD from configuration space (TBLPAG<7> = 1). Return the 24-bit word. Default: DEVID table. */
  uint32_t (*cfg_read)(void *ctx, struct pic24_cpu *cpu, uint32_t addr24);
  /* Observer for every SFR-space access 0x0020-0x07FF, core-owned or not (NULL = none). Reads report the value
   * returned, writes the value and lane mask written. For boot traces / peripheral to-do lists. */
  void (*sfr_watch)(void *ctx, struct pic24_cpu *cpu, uint16_t addr, uint16_t value, uint16_t lane_mask, bool write);
  /* Device reset performed by the core, called after the core registers are at their reset values. `why`:
   * PIC24_STOP_RESET = RESET instruction (RCON.SWR), PIC24_STOP_ILLEGAL = illegal opcode, illegal address mode or BREAK
   * without a debugger (RCON.IOPUWR), PIC24_STOP_TRAP_CONFLICT = hard-trap conflict (RCON.TRAPR). RCON is not
   * core-owned: the host sets the flag bit and resets its peripherals here. */
  void (*on_reset)(void *ctx, struct pic24_cpu *cpu, pic24_status why);
  /* Behaviour switches. All false = silicon behaviour. */
  bool illegal_stops; /* DEBUG ONLY. Silicon: an illegal opcode or illegal address mode resets the device with
                       * RCON.IOPUWR set (DS39969 Register 6-1). true: stop with PIC24_STOP_ILLEGAL instead (PC kept,
                       * stop_insn = opcode), for emulator debugging and decoder tests.
                       * Hosts set it from PIC24_ILLEGAL_STOPS=1. */
  bool debugger;      /* A debugger is attached: BREAK (0xDA4000) halts with PIC24_STOP_BREAK, PC past it. false (no
                       * debugger, the flying case): BREAK is not a user instruction (absent from DS70000157G) and is
                       * treated as an illegal opcode (INFERRED-high; a dev board would confirm).
                       * Hosts set it from PIC24_DEBUGGER=1. */
} pic24_config;

typedef struct pic24_cpu {
  /* Lower data space 0x0000-0x7FFF, word-addressed. ram[0..15] ARE W0..W15 (memory-mapped, DS Table 4-4). The
   * SFR words 0x0020-0x07FF in this array are the default storage for non-core SFRs (cpu->cfg.sfr_read NULL). */
  uint16_t ram[PIC24_DATA_LOW_TOP / 2];
  uint16_t eds[(PIC24_EDS_RAM_TOP - PIC24_EDS_RAM_BASE) / 2]; /* extended RAM, EDS 0x008000-0x0187FF */
  uint32_t prog[PIC24_PROG_WORDS];                              /* 24-bit program words, index = PC/2      */

  uint32_t pc;      /* 23-bit, bit0 = 0 */
  uint16_t sr;      /* DC..C, bits 0-8 */
  uint8_t ipl3;     /* CORCON<3> */
  uint16_t rcount;  /* 14-bit */
  uint16_t splim;
  bool splim_armed; /* stack-overflow check armed by first word write to SPLIM (PRM 4.8.3) */
  uint16_t dsrpag, dswpag, tblpag, disicnt;

  /* interrupt controller (core-owned: DS Table 4-6) */
  uint16_t intcon1, intcon2, inttreg;
  uint16_t ifs[7], iec[7], ipc[26];

  /* PUSH.S / POP.S shadows: W0-W3 and DC,N,OV,Z,C (PRM Table 2-2) */
  uint16_t shadow_w[4];
  uint16_t shadow_sr;

  pic24_power power;
  uint64_t cycles;      /* instruction cycles (Tcy) since reset */
  uint64_t instret;     /* instructions retired (each REPEAT iteration counts) */
  uint32_t stop_insn;   /* opcode that caused PIC24_STOP_ILLEGAL or the last illegal-opcode reset, or the BREAK */
  pic24_status last_reset; /* cause of the last core-performed device reset (PIC24_OK = none since pic24_init)   */
  uint32_t last_reset_pc;  /* PC of the instruction that caused it                                              */
  uint32_t resets;         /* core-performed device resets since pic24_init (RESET, illegal opcode, trap conflict) */
  bool stop_request;    /* set by a callback to stop pic24_run at the next boundary */
  /* Optional write log (pic24_wlog_start / pic24_wlog_restored): the data-memory words written (RAM, extended RAM, SFR
   * words the core stores itself, exception stacking) with the value each held before the first write. Register
   * writes (W0-W15 and the core registers) are not logged. */
  bool wlog_on, wlog_full;
  uint16_t wlog_n;
  uint32_t wlog_key[PIC24_WLOG];   /* word index: ram[] index, or PIC24_DATA_LOW_TOP / 2 + eds[] index */
  uint16_t wlog_old[PIC24_WLOG];
  uint32_t exceptions;  /* exceptions (traps and interrupts) taken since pic24_init */

  /* --- internal (do not touch from outside) --- */
  bool pend_valid;      /* cached exception arbitration is current */
  int pend_vec_all;     /* best pending exception: vector index (0-7 trap, 8+irq) or -1 */
  int pend_lvl_all;     /* its priority level (1..15) */
  int pend_vec_ge7;     /* best pending exception with level >= 7 (not blocked by DISI) */
  int pend_lvl_ge7;
  uint8_t hard_trap_pending; /* ADDRERR/OSCFAIL raised by the current instruction (trap-conflict detection) */
  uint8_t acc_flags;    /* per-instruction memory-access flags (PSV / EDS read, address error) */
  uint16_t raw_wmask;   /* W regs written (destination side) by the previous instruction: RAW stall detect */
  bool repeat_first;    /* next REPEAT-target execution is the first of a loop (PSV/EDS timing) */
  bool repeat_final;    /* next execution is the last iteration of a REPEAT loop (RA already clear) */
  uint8_t repeat_iter;  /* iterations of the current REPEAT loop executed (saturating; timing only) */

  pic24_config cfg;
} pic24_cpu;

/* ---- lifecycle ------------------------------------------------------------------------------------------- */
/* Zero the structure, install callbacks, power-on reset. `cfg` may be NULL. */
void pic24_init(pic24_cpu *cpu, const pic24_config *cfg);
/* Device reset (POR/MCLR semantics for the core; program memory and EDS/RAM contents are kept). */
void pic24_reset(pic24_cpu *cpu);
/* sizeof(pic24_cpu) as the core library was compiled. A host compares it with its own sizeof(pic24_cpu): a library
 * built from an older pic24.h would otherwise run with every field after the first changed one at a different
 * offset. An older library has no such symbol, so the link fails instead. */
size_t pic24_cpu_size(void);
/* Load program memory from a flat program image (scripts/fetch_firmware.py ct_prog): little-endian 3 bytes per PC word pair, starting at PC 0.
 * Words beyond nbytes/3 keep their previous value (pic24_init fills program memory with 0xFFFFFF = erased). */
void pic24_load_prog(pic24_cpu *cpu, const uint8_t *bytes, size_t nbytes);
/* Initial value of the non-core SFR shadow (used when cfg.sfr_read is NULL): words[i] for address 2*i. */
void pic24_set_sfr_reset(pic24_cpu *cpu, const uint16_t *words, size_t nwords);

/* ---- execution ------------------------------------------------------------------------------------------- */
/* Execute one instruction (or one REPEAT iteration, or one exception entry). Returns status; cycles consumed are
 * added to cpu->cycles. */
pic24_status pic24_step(pic24_cpu *cpu);
/* Run until at least `cycles` more Tcy elapsed or a stop condition. */
pic24_status pic24_run(pic24_cpu *cpu, uint64_t cycles);

/* ---- interrupts ------------------------------------------------------------------------------------------ */
/* Set / clear the IFS flag of interrupt source `irq` (0..117, DS Table 7-2). This is how peripherals raise IRQs. */
void pic24_set_irq(pic24_cpu *cpu, unsigned irq, bool level);
/* Raise a trap source directly (sets the INTCON1 status bit): PIC24_INTCON1_OSCFAIL etc. */
void pic24_raise_trap(pic24_cpu *cpu, uint16_t intcon1_bit);
/* Current CPU priority level 0..15 (IPL3:IPL<2:0>). */
unsigned pic24_cpu_ipl(const pic24_cpu *cpu);

/* ---- register / memory access (debugger view: no side effects, no traps) ----------------------------------- */
uint16_t pic24_get_w(const pic24_cpu *cpu, unsigned n);
void pic24_set_w(pic24_cpu *cpu, unsigned n, uint16_t v);
/* Data-space read/write through the CPU's address decode (core SFRs, SFR callbacks, RAM, EDS window per the
 * current DSRPAG/DSWPAG). Peripheral callbacks ARE invoked. */
uint16_t pic24_peek16(pic24_cpu *cpu, uint16_t ea);
void pic24_poke16(pic24_cpu *cpu, uint16_t ea, uint16_t v);
/* Core SFR value by data address (SR 0x42, CORCON 0x44, ...); returns false if `addr` is not core-owned. */
bool pic24_core_sfr(const pic24_cpu *cpu, uint16_t addr, uint16_t *value);
/* Program memory word (24-bit) at PC address; 0 for unimplemented addresses. */
uint32_t pic24_prog_word(const pic24_cpu *cpu, uint32_t pc);

/* Write log: start logging data-memory writes (clears the log); true when every word written since holds its value
 * from before the first write again (and the log did not overflow). Logging stops at pic24_wlog_stop. */
void pic24_wlog_start(pic24_cpu *cpu);
bool pic24_wlog_restored(const pic24_cpu *cpu);
void pic24_wlog_stop(pic24_cpu *cpu);

/* 2 for CALL/GOTO lit23 (PRM 3.2.2), else 1. */
int pic24_insn_words(uint32_t insn);

#ifdef __cplusplus
}
#endif
#endif /* PIC24_H */
