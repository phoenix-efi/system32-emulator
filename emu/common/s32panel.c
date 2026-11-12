/* s32panel.c -- the Controller's panel as it shows (see s32panel.h). */
#include "s32panel.h"
#include <stdio.h>
#include <string.h>

void s32_panel_render(s32ct *ct, bool powered, uint8_t rgb[S32_PANEL_W * S32_PANEL_H * 3], s32_panel_state *st) {
  const s32_gfx *g = &ct->gfx;
  s32_panel_state s = {.lit = false, .why = ""};
  s.con1 = g->reg[0x02]; s.con2 = g->reg[0x12]; s.con3 = g->reg[0x13];
  uint16_t pr2 = s32ct_sfr_value(ct, 0x010C), oc1r = s32ct_sfr_value(ct, 0x0196), oc1con = s32ct_sfr_value(ct, 0x0190);
  if (powered && (oc1con & 7u) >= 6u && pr2) {
    unsigned pm = (unsigned)((1000u * (uint32_t)oc1r) / ((uint32_t)pr2 + 1u));
    s.backlight_permille = (uint16_t)(pm > 1000u ? 1000u : pm);
  }
  unsigned w = g->reg[0x0E], h = g->reg[0x0F], dptest = (s.con2 >> 8) & 3u;
  if (!powered) s.why = "unpowered";
  else if (!(s.con1 & 0x8000u)) s.why = "G1EN off";
  else if (!(s.con3 & 0x0200u)) s.why = "pins off";
  else if (dptest) s.why = "test pattern";
  else if (w != S32_PANEL_W || h != S32_PANEL_H) s.why = "size";
  else s.lit = true;
  if (!s.lit) memset(rgb, 0, S32_PANEL_W * S32_PANEL_H * 3);
  else {
    unsigned bpp = 1u << (((s.con2 >> 5) & 7u) & 3u);
    uint32_t base = g->reg[0x0C] | ((uint32_t)(g->reg[0x0D] & 0xFFu) << 16);
    uint8_t lut[256][3];
    for (unsigned i = 0; i < 256; i++) {
      uint16_t c = g->clut[i];
      lut[i][0] = (uint8_t)(((c >> 11) & 31u) * 255u / 31u);
      lut[i][1] = (uint8_t)(((c >> 5) & 63u) * 255u / 63u);
      lut[i][2] = (uint8_t)((c & 31u) * 255u / 31u);
    }
    uint8_t *d = rgb;
    for (uint32_t n = 0; n < S32_PANEL_W * S32_PANEL_H; n++) {
      uint32_t bit = n * bpp;
      unsigned idx = (s32_eds_byte(&ct->cpu, base + (bit >> 3)) >> (bit & 7u)) & ((1u << bpp) - 1u);
      memcpy(d, lut[idx], 3); d += 3;
    }
  }
  if (st) *st = s;
}

static bool px_lit(const uint8_t *rgb, unsigned x, unsigned y) {
  const uint8_t *p = rgb + (y * S32_PANEL_W + x) * 3u;
  return (p[0] | p[1] | p[2]) != 0;
}
static bool px_white(const uint8_t *rgb, unsigned x, unsigned y) {
  const uint8_t *p = rgb + (y * S32_PANEL_W + x) * 3u;
  return p[0] == 255u && p[1] == 255u && p[2] == 255u;
}
static unsigned box_lit(const uint8_t *rgb, unsigned x0, unsigned y0, unsigned x1, unsigned y1) {
  unsigned n = 0;
  for (unsigned y = y0; y <= y1; y++)
    for (unsigned x = x0; x <= x1; x++)
      if (px_lit(rgb, x, y)) n++;
  return n;
}

bool s32_panel_is_home(const uint8_t rgb[S32_PANEL_W * S32_PANEL_H * 3]) {
  static const struct { uint16_t x0, y0, x1, y1; } BOXES[] = {
    {29, 14, 71, 25},    {84, 14, 125, 25},   {149, 14, 191, 25},  {204, 14, 245, 25},  /* ECU1, ECU2, PMP1, PMP2 */
    {60, 50, 110, 65},   {160, 50, 209, 65},  {50, 77, 120, 92},   {155, 77, 215, 92},  /* RPM, MAP labels + values */
    {40, 110, 144, 125}, {165, 110, 205, 125},{37, 134, 88, 149},  {92, 134, 137, 149}, /* Fuel Flow, IGN, value, GPH */
    {160, 134, 210, 149},{252, 75, 312, 136}, {50, 180, 154, 195}, {185, 179, 198, 195},/* IGN val, AFR val, Fuel Trim + val */
    {200, 179, 216, 195},{103, 213, 113, 223},{121, 213, 162, 224},{169, 213, 179, 223},/* %, BATT1 dot, BATT, BATT2 dot */
  };
  unsigned black = 0, wl = 0, wr = 0, wb = 0;
  for (unsigned n = 0; n < S32_PANEL_W * S32_PANEL_H; n++)
    if ((rgb[3u * n] | rgb[3u * n + 1u] | rgb[3u * n + 2u]) == 0) black++;
  if (black < 60000u) return false;
  for (unsigned y = 40; y <= 155; y++) {
    if (px_white(rgb, 25, y)) wl++;
    if (px_white(rgb, 230, y)) wr++;
  }
  for (unsigned x = 30; x <= 225; x++)
    if (px_white(rgb, x, 160)) wb++;
  if (wl < 100u || wr < 100u || wb < 170u) return false;
  for (size_t i = 0; i < sizeof BOXES / sizeof BOXES[0]; i++)
    if (box_lit(rgb, BOXES[i].x0, BOXES[i].y0, BOXES[i].x1, BOXES[i].y1) < 16u) return false;
  return true;
}

int s32_panel_ppm(s32ct *ct, bool powered, const char *path) {
  static uint8_t rgb[S32_PANEL_W * S32_PANEL_H * 3];
  s32_panel_render(ct, powered, rgb, NULL);
  FILE *f = fopen(path, "wb");
  if (!f) return -1;
  fprintf(f, "P6\n%u %u\n255\n", S32_PANEL_W, S32_PANEL_H);
  fwrite(rgb, 1, sizeof rgb, f);
  fclose(f);
  return 0;
}
