/* s32_iocov.h -- I/O coverage recorder shared by both target-system SoC models (emu/ecu, emu/controller).
 *
 * When a front end enables it (S32_IOCOV in emu/pair), the SoC model records what the firmware touched, at the level a
 * logic analyser plus a register-level bus monitor would see it: every SFR word read or written, every port pin the
 * firmware sampled as a digital input (a PORTx read with the pin's TRIS bit set and the pin not analog), every pin it
 * ever configured as an output (TRIS bit clear), and every ADC channel converted. `make io-coverage` compares the
 * union over a scripted run set with model/io_ledger.json. Recording costs one bit-set per SFR access; off (NULL) by
 * default. Nothing here reads firmware RAM.
 */
#ifndef S32_IOCOV_H
#define S32_IOCOV_H
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  uint32_t nwords;            /* SFR words covered (bitmap length in bits) */
  uint32_t base;              /* bus address of SFR word 0 (for the dump) */
  unsigned stride;            /* bytes per SFR word (PIC32: 4, PIC24: 2) */
  uint8_t *rd, *wr;           /* bitmaps: SFR word read / written */
  uint16_t pin_in[7];         /* digital input pins sampled, per port A..G */
  uint16_t pin_out[7];        /* pins ever configured as outputs (TRIS clear), per port A..G */
  uint32_t an_conv;           /* ADC channels converted (bit n = ANn) */
  uint64_t an_samples[32];    /* conversions per channel */
} s32_iocov;

static inline s32_iocov *s32_iocov_new(uint32_t base, uint32_t nwords, unsigned stride) {
  s32_iocov *c = calloc(1, sizeof *c);
  if (!c) return NULL;
  c->base = base; c->nwords = nwords; c->stride = stride;
  c->rd = calloc((nwords + 7) / 8, 1); c->wr = calloc((nwords + 7) / 8, 1);
  if (!c->rd || !c->wr) { free(c->rd); free(c->wr); free(c); return NULL; }
  return c;
}
static inline void s32_iocov_free(s32_iocov *c) { if (c) { free(c->rd); free(c->wr); free(c); } }
static inline void s32_iocov_sfr(s32_iocov *c, uint32_t word, bool write) {
  if (c && word < c->nwords) (write ? c->wr : c->rd)[word >> 3] |= (uint8_t)(1u << (word & 7u));
}
static inline void s32_iocov_an(s32_iocov *c, unsigned ch) {
  if (c && ch < 32) { c->an_conv |= 1u << ch; c->an_samples[ch]++; }
}
/* JSON object (no trailing newline): {"sfr_read":["0x...",...],"sfr_write":[...],"pin_in":{"A":"0x..."},...}.
 * `val` (optional) reads an SFR's current value by bus address: every written SFR is then listed with its final value
 * under "sfr_final" (TRIS/ODC/pull-up/PPS/ADC configuration, as a bus monitor would read them back). */
static inline void s32_iocov_dump(const s32_iocov *c, FILE *f, uint32_t (*val)(void *ctx, uint32_t addr), void *ctx) {
  static const char P[7] = {'A', 'B', 'C', 'D', 'E', 'F', 'G'};
  for (int w = 0; w < 2; w++) {
    const uint8_t *bm = w ? c->wr : c->rd;
    fprintf(f, "%s\"%s\":[", w ? "," : "{", w ? "sfr_write" : "sfr_read");
    bool first = true;
    for (uint32_t i = 0; i < c->nwords; i++)
      if (bm[i >> 3] & (1u << (i & 7u))) { fprintf(f, "%s\"0x%08X\"", first ? "" : ",", c->base + i * c->stride); first = false; }
    fputc(']', f);
  }
  for (int w = 0; w < 2; w++) {
    fprintf(f, ",\"%s\":{", w ? "pin_out" : "pin_in");
    for (int p = 0; p < 7; p++) fprintf(f, "%s\"%c\":\"0x%04X\"", p ? "," : "", P[p], w ? c->pin_out[p] : c->pin_in[p]);
    fputc('}', f);
  }
  if (val) {
    fprintf(f, ",\"sfr_final\":{");
    bool first = true;
    for (uint32_t i = 0; i < c->nwords; i++)
      if (c->wr[i >> 3] & (1u << (i & 7u))) {
        uint32_t a = c->base + i * c->stride;
        fprintf(f, "%s\"0x%08X\":\"0x%08X\"", first ? "" : ",", a, val(ctx, a)); first = false;
      }
    fputc('}', f);
  }
  fprintf(f, ",\"an_conv\":[");
  bool first = true;
  for (int ch = 0; ch < 32; ch++)
    if (c->an_conv & (1u << ch)) { fprintf(f, "%s[%d,%llu]", first ? "" : ",", ch, (unsigned long long)c->an_samples[ch]); first = false; }
  fputs("]}", f);
}
#endif
