/* s32gfx.h -- PIC24FJ256DA210 graphics controller (G1, SFR 0x0700-0x073E).
 *
 * Commands execute synchronously when G1CMDH is written, so G1STAT always reads "FIFO empty, engines idle". Command
 * encodings per the PIC24F DA graphics FRM: opcode = G1CMDH bits 15:8, operand = 24 bits.
 *   0x62 RCC_SRCADDR  0x63 RCC_DESTADDR  0x64 RCC_RECTSIZE (w<<12 | h)  0x66 RCC_COLOR  0x67 RCC_STARTCOPY
 *   0x50 CHR_FGCOLOR  0x51 CHR_BGCOLOR  0x52 CHR_FONTBASE  0x53 CHR_PRINTCHAR (bit 23 transparent)
 *   0x58 CHR_TXTAREASTART  0x59 CHR_TXTAREAEND  0x5A CHR_PRINTPOS (x | y<<12)
 *   0x71 IPU_SRCADDR  0x72 IPU_DESTADDR  0x73 IPU_WRITELENGTH  0x74 IPU_DECOMPRESS
 */
#ifndef S32GFX_H
#define S32GFX_H
#include "pic24.h"
#include <stdint.h>
#include <stdio.h>

typedef struct {
  uint16_t reg[0x20];              /* G1 SFR words 0x0700-0x073E, index = (addr - 0x700) / 2 */
  uint16_t clut[256];
  uint32_t cmd_count[256];         /* commands seen, by opcode */
  uint32_t unknown_cmds;
  /* RCC */
  uint32_t rcc_src, rcc_dst, rcc_w, rcc_h, rcc_color;
  /* CHR */
  uint32_t chr_fg, chr_bg, chr_font, chr_x, chr_y, chr_ax0, chr_ay0, chr_ax1, chr_ay1;
  /* IPU */
  uint32_t ipu_src, ipu_dst, ipu_len;
  uint64_t frames;                 /* vblanks signalled */
  uint32_t ipu_ok;                 /* successful inflates */
  FILE *log;                       /* optional: one line per command */
  unsigned log_budget;             /* lines left */
} s32_gfx;

void s32_gfx_init(s32_gfx *g);
uint16_t s32_gfx_read(s32_gfx *g, pic24_cpu *cpu, uint16_t addr);
void s32_gfx_write(s32_gfx *g, pic24_cpu *cpu, uint16_t addr, uint16_t v, uint16_t mask);
void s32_gfx_vblank(s32_gfx *g, pic24_cpu *cpu);
uint64_t s32_gfx_frame_tcy(const s32_gfx *g);
/* Render the display buffer (G1DPADR, G1DPW x G1DPH, G1CON2 DPBPP) through the CLUT to a binary PPM. */
int s32_gfx_dump_ppm(const s32_gfx *g, pic24_cpu *cpu, const char *path);
/* Byte at a 24-bit EDS data address (internal RAM only; 0 elsewhere). */
uint8_t s32_eds_byte(pic24_cpu *cpu, uint32_t a);

#endif
