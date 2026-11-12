/* s32dev.h -- SPI device models for the Cockpit Controller and ECU boards.
 *
 * FM25V10 FRAM (128 KiB, CS RG7) and PCF2127 RTC (CS RF3) on the Controller's SPI2 (OBSERVED on the chip selects: a
 * READ `03` + 3-byte address on RG7, `20 00 00 00` on RF3), the SD card in SPI mode. Each model plugs into
 * s32ct_spi_attach() / ecu_spi_attach() as an s32_spi_dev.
 */
#ifndef S32DEV_H
#define S32DEV_H
#include "s32_spidev.h"
#include <stdint.h>
#include <time.h>

/* Device faults (emu/common/s32board.c drives them from the fault knobs). Each is a physical failure of the part or its
 * wiring, seen only through the SPI pins. */
enum { S32_DEV_OK = 0,
       S32_DEV_ABSENT,     /* no device answering: MISO pulled high (0xFF), commands have no effect */
       S32_DEV_MISO_LOW,   /* MISO shorted to ground: every byte reads 0x00 */
       S32_DEV_WP,         /* FRAM: array writes have no effect (write-protected / failed part); reads still work */
       S32_DEV_RDERR,      /* SD: every block read answers the data error token (0x08: card ECC failed) */
       S32_DEV_WRERR };    /* SD: every block write answers "data rejected, write error" (0x0D) */

/* ---- FM25V10 ---- */
#define S32_FRAM_BYTES 0x20000u
typedef struct {
  uint8_t mem[S32_FRAM_BYTES];
  uint8_t sr;            /* status register: bit1 WEL, bits 2-3 BP, bit 7 WPEN */
  uint8_t op;            /* opcode of the transaction in progress (0 = expecting opcode) */
  unsigned phase;        /* bytes received after the opcode */
  uint32_t addr;
  bool asleep;
  uint64_t reads, writes;   /* data bytes, for the probe's summary */
  int fault;                /* S32_DEV_OK / ABSENT / MISO_LOW / WP */
} s32_fram;
void s32_fram_init(s32_fram *f, uint8_t fill);
s32_spi_dev s32_fram_dev(s32_fram *f);

/* ---- PCF2127 (SPI) ---- */
typedef struct {
  uint8_t reg[0x20];     /* 0x00-0x1B used; time 0x03-0x09 BCD */
  const uint64_t *cycles;   /* the CPU's Tcy counter: the RTC runs on emulated time */
  double fcy;
  time_t base;           /* epoch at cycles == 0 */
  int phase;             /* -1 = expecting the command byte */
  bool read; uint8_t ptr;
  bool time_written;
  int fault;                /* S32_DEV_OK / ABSENT / MISO_LOW */
  bool invalid;             /* time registers hold invalid BCD (lost backup supply) until the time is written */
} s32_rtc;
void s32_rtc_init(s32_rtc *r, const uint64_t *cycles, double fcy, time_t base);
s32_spi_dev s32_rtc_dev(s32_rtc *r);
time_t s32_rtc_now(const s32_rtc *r);
/* Oscillator-stop flag (seconds register bit 7, PCF2127 data sheet 8.4): set at power-on after the clock stopped; cleared
 * only when the seconds register is written. */
void s32_rtc_set_osf(s32_rtc *r, bool set);

/* ---- SD card, SPI mode (SD Physical Layer Simplified Spec v2, SPI chapter) ----
 * SDHC (block addressed, CCS = 1), backed by an image in memory (512-byte sectors). Commands: CMD0, CMD8, CMD55/ACMD41,
 * CMD58, CMD9/CMD10 (CSD/CID), CMD12, CMD13, CMD16, CMD17/18 (read single/multi), CMD24/25 (write single/multi).
 * Responses come one byte after the command (NCR = 1); data blocks start with token 0xFE (0xFC for multi-write). */
typedef struct {
  uint8_t *img; uint32_t sectors;          /* card contents (owned by the caller) */
  bool dirty;                              /* written since load */
  uint8_t cmd[6]; unsigned ncmd;           /* command being received */
  uint8_t out[600]; unsigned nout, iout;   /* bytes queued to MISO */
  int mode;                                /* 0 cmd, 1 read-multi streaming, 2 write-single data, 3 write-multi data */
  uint32_t addr;                           /* current block */
  uint8_t wbuf[514]; unsigned nw; bool wtok;
  bool app, idle;
  uint64_t reads, writes;
  int fault;                               /* S32_DEV_OK / ABSENT (no response: init fails) / MISO_LOW / RDERR / WRERR */
} s32_sd;
void s32_sd_init(s32_sd *c, uint8_t *img, uint32_t bytes);
s32_spi_dev s32_sd_dev(s32_sd *c);

#endif
