/* s32panel.h -- the Controller's 320x240 TFT as the physical panel shows it, one implementation for every front end
 * (emu/pair screen dumps, emu/wasm canvas, the conformance trace), so all of them show the same picture.
 *
 * The panel shows the display buffer the graphics controller scans out (G1DPADR, G1DPW x G1DPH, G1CON2.DPBPP, through
 * the CLUT, RGB565 widened to 8 bits per channel) when the Controller is powered, the graphics module is on
 * (G1CON1.G1EN) and its display pins are driven (G1CON3.DPPINOE); otherwise it is dark. G1CON2.DPTEST (bits 9:8,
 * register map from Microchip's DFP) replaces the picture with the controller's test pattern: 01 = black screen (the
 * stock firmware sets it while it builds a new screen), 10 = bars, 11 = borders (both shown dark here: their
 * pixels are not modelled). A display size other than 320 x 240 does not drive this panel (dark).
 *
 * Reported beside the picture, not applied to it: the backlight PWM duty (OC1 on Timer 2, as the harness report
 * computes it). The panel-enable line RD0 is a pin in the trace; it is not used to gate the picture because the
 * stock firmware keeps RD0 low in every run while the panel shows content (OBSERVED), so RD0 is not this panel's
 * enable (INFERRED). */
#ifndef S32PANEL_H
#define S32PANEL_H
#include "s32ct.h"
#include <stdbool.h>
#include <stdint.h>

#define S32_PANEL_W 320u
#define S32_PANEL_H 240u

typedef struct {
  bool lit;                 /* the picture is the display buffer (else dark) */
  const char *why;          /* "" or why it is dark: "unpowered", "G1EN off", "pins off", "test pattern", "size" */
  uint16_t con1, con2, con3;
  uint16_t backlight_permille;
} s32_panel_state;

/* The panel's pixels, RGB888, row-major 320 x 240. */
void s32_panel_render(s32ct *ct, bool powered, uint8_t rgb[S32_PANEL_W * S32_PANEL_H * 3], s32_panel_state *st);
/* True when the rendered panel picture is the completed home page (gauge page 1: black background, white main gauge
 * border, and every status tab, label, unit, battery indicator and live readout drawn through the final Fuel Trim
 * value). Decided from the RGB888 panel pixels alone. */
bool s32_panel_is_home(const uint8_t rgb[S32_PANEL_W * S32_PANEL_H * 3]);
/* The same picture as a binary PPM file. Returns 0, or -1 if the file cannot be written. */
int s32_panel_ppm(s32ct *ct, bool powered, const char *path);

#endif
