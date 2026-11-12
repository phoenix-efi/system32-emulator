/* s32_spidev.h -- the SPI slave interface shared by both target-system emulators (emu/controller, emu/ecu). A device is
 * called once per byte while its chip-select is asserted (low), and on every chip-select edge. */
#ifndef S32_SPIDEV_H
#define S32_SPIDEV_H
#include <stdbool.h>
#include <stdint.h>

typedef struct s32_spi_dev {
  const char *name;
  void *ctx;
  uint8_t (*exchange)(void *ctx, uint8_t mosi);
  void (*select)(void *ctx, bool asserted);
} s32_spi_dev;

#endif
