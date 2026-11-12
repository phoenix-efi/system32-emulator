/* Unit tests for the FM25V10 FRAM and PCF2127 RTC models, driven byte by byte through their s32_spi_dev hooks. */
#include "minitest.h"
#include "s32dev.h"
#include <string.h>

static s32_fram F;
static s32_rtc R;
static uint64_t CYC;

static void xfer(s32_spi_dev d, const uint8_t *tx, uint8_t *rx, unsigned n) {
  d.select(d.ctx, true);
  for (unsigned i = 0; i < n; i++) { uint8_t b = d.exchange(d.ctx, tx[i]); if (rx) rx[i] = b; }
  d.select(d.ctx, false);
}

TEST(fram_write_needs_wel_and_wel_clears_after_write) {
  s32_fram_init(&F, 0x00);
  s32_spi_dev d = s32_fram_dev(&F);
  uint8_t w[] = {0x02, 0x01, 0x23, 0x45, 0xAA, 0xBB};
  xfer(d, w, NULL, sizeof w);
  CHECK_EQ(F.mem[0x12345], 0x00);                 /* no WREN: ignored */
  uint8_t wren = 0x06; xfer(d, &wren, NULL, 1);
  uint8_t rdsr[2] = {0x05, 0}, sr[2]; xfer(d, rdsr, sr, 2);
  CHECK_EQ(sr[1] & 0x02, 0x02);
  xfer(d, w, NULL, sizeof w);
  CHECK_EQ(F.mem[0x12345], 0xAA);
  CHECK_EQ(F.mem[0x12346], 0xBB);
  xfer(d, rdsr, sr, 2);
  CHECK_EQ(sr[1] & 0x02, 0);                      /* WEL cleared at CS rise */
}

TEST(fram_read_and_address_wrap) {
  s32_fram_init(&F, 0x00);
  F.mem[0x1FFFF] = 0x11; F.mem[0] = 0x22;
  s32_spi_dev d = s32_fram_dev(&F);
  uint8_t tx[] = {0x03, 0x01, 0xFF, 0xFF, 0, 0}, rx[6];
  xfer(d, tx, rx, sizeof tx);
  CHECK_EQ(rx[4], 0x11);
  CHECK_EQ(rx[5], 0x22);                          /* 17-bit address wraps */
  uint8_t fs[] = {0x0B, 0x01, 0xFF, 0xFF, 0x00, 0}, fr[6];
  xfer(d, fs, fr, sizeof fs);
  CHECK_EQ(fr[5], 0x11);                          /* fast read: one dummy byte first */
}

TEST(fram_rdid) {
  s32_fram_init(&F, 0xFF);
  uint8_t tx[10] = {0x9F}, rx[10];
  xfer(s32_fram_dev(&F), tx, rx, 10);
  CHECK_EQ(rx[1], 0x7F); CHECK_EQ(rx[7], 0xC2); CHECK_EQ(rx[8], 0x24);
}

TEST(rtc_reads_bcd_time_that_advances_with_emulated_time) {
  CYC = 0;
  s32_rtc_init(&R, &CYC, 16e6, 1760000000);       /* 2025-10-09 08:53:20 UTC */
  s32_spi_dev d = s32_rtc_dev(&R);
  uint8_t tx[8] = {0xA3}, rx[8];                  /* read, subaddress 01, from register 3 (seconds) */
  xfer(d, tx, rx, 8);
  CHECK_EQ(rx[1], 0x20); CHECK_EQ(rx[2], 0x53); CHECK_EQ(rx[3], 0x08);
  CHECK_EQ(rx[4], 0x09); CHECK_EQ(rx[6], 0x10); CHECK_EQ(rx[7], 0x25);
  CYC = 16000000ull * 61;                         /* 61 s later */
  xfer(d, tx, rx, 3);
  CHECK_EQ(rx[1], 0x21); CHECK_EQ(rx[2], 0x54);
}

TEST(rtc_write_sets_the_clock_and_control_regs_store) {
  CYC = 0;
  s32_rtc_init(&R, &CYC, 16e6, 0);
  s32_spi_dev d = s32_rtc_dev(&R);
  uint8_t ctl[] = {0x20, 0x00, 0x00, 0x00};       /* write Control_1..3 = 0 (from register 0x00) */
  xfer(d, ctl, NULL, sizeof ctl);
  uint8_t set[] = {0x23, 0x00, 0x30, 0x14, 0x12, 0x01, 0x10, 0x26};   /* seconds .. years: 14:30:00, day 12, month 10, '26 */
  xfer(d, set, NULL, sizeof set);
  CYC = 16000000ull * 5;
  uint8_t tx[4] = {0xA3}, rx[4];
  xfer(d, tx, rx, 4);
  CHECK_EQ(rx[1], 0x05); CHECK_EQ(rx[2], 0x30); CHECK_EQ(rx[3], 0x14);
}

TEST(rtc_address_wraps_after_1b) {
  /* PCF2127 data sheet, register addressing: the address auto-increments "up to register 1Bh; after register 1Bh
   * the auto-incrementing will wrap around to address 00h". */
  CYC = 0;
  s32_rtc_init(&R, &CYC, 16e6, 0);
  s32_spi_dev d = s32_rtc_dev(&R);
  uint8_t tx[] = {0x3A, 0x11, 0x22, 0x33};        /* write 3 bytes from register 0x1A */
  xfer(d, tx, NULL, sizeof tx);
  CHECK_EQ(R.reg[0x1A], 0x11);
  CHECK_EQ(R.reg[0x1B], 0x22);                    /* 0x1B receives the second byte (not skipped) */
  CHECK_EQ(R.reg[0x00], 0x33);                    /* and the next write wraps to 0x00 */
}

TEST(rtc_time_frozen_within_one_read) {
  /* PCF2127 data sheet 8.9.8: during read/write operations the time counting circuits are blocked, so one read
   * access returns a single instant even when a second ticks mid-burst. */
  s32_rtc_init(&R, &CYC, 16e6, 1760000000);       /* 2025-10-09 08:53:20 UTC: a minute carry at +40 s */
  CYC = 39.9999 * 16e6;                           /* 08:53:59.9999: the next tick carries seconds into minutes */
  s32_spi_dev d = s32_rtc_dev(&R);
  d.select(d.ctx, true);
  CHECK_EQ(d.exchange(d.ctx, 0xA3), 0xFF);        /* read from register 3 (seconds) */
  CHECK_EQ(d.exchange(d.ctx, 0xFF), 0x59);        /* seconds of the latched instant */
  CYC += 16e6 / 500;                              /* 2 ms: the second ticks over mid-burst */
  CHECK_EQ(d.exchange(d.ctx, 0xFF), 0x53);        /* minutes of the SAME instant (53), not the new one (54) */
  d.select(d.ctx, false);
}

static s32_sd SD;
static uint8_t IMG[1024 * 1024];
static uint8_t sd_x(uint8_t b) { return s32_sd_dev(&SD).exchange(&SD, b); }
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg, uint8_t *extra, unsigned nextra) {
  uint8_t f[6] = {(uint8_t)(0x40 | cmd), (uint8_t)(arg >> 24), (uint8_t)(arg >> 16), (uint8_t)(arg >> 8), (uint8_t)arg, 0x95};
  for (int i = 0; i < 6; i++) sd_x(f[i]);
  uint8_t r = 0xFF;
  for (int i = 0; i < 8 && r == 0xFF; i++) r = sd_x(0xFF);          /* NCR */
  for (unsigned i = 0; i < nextra; i++) extra[i] = sd_x(0xFF);
  return r;
}

TEST(sd_init_sequence_sdhc) {
  memset(IMG, 0, sizeof IMG);
  s32_sd_init(&SD, IMG, sizeof IMG);
  CHECK_EQ(sd_cmd(0, 0, NULL, 0), 0x01);
  uint8_t r7[4]; CHECK_EQ(sd_cmd(8, 0x1AA, r7, 4), 0x01); CHECK_EQ(r7[3], 0xAA);
  CHECK_EQ(sd_cmd(55, 0, NULL, 0), 0x01);
  CHECK_EQ(sd_cmd(41, 0x40000000, NULL, 0), 0x00);                 /* ready */
  uint8_t ocr[4]; CHECK_EQ(sd_cmd(58, 0, ocr, 4), 0x00); CHECK_EQ(ocr[0] & 0x40, 0x40);   /* CCS: block addressed */
}

TEST(sd_read_and_write_blocks) {
  memset(IMG, 0, sizeof IMG);
  for (int i = 0; i < 512; i++) IMG[3 * 512 + i] = (uint8_t)(i * 3);
  s32_sd_init(&SD, IMG, sizeof IMG);
  CHECK_EQ(sd_cmd(17, 3, NULL, 0), 0x00);
  uint8_t t = 0xFF; for (int i = 0; i < 16 && t != 0xFE; i++) t = sd_x(0xFF);
  CHECK_EQ(t, 0xFE);
  uint8_t blk[512]; for (int i = 0; i < 512; i++) blk[i] = sd_x(0xFF);
  CHECK_EQ(blk[0], 0); CHECK_EQ(blk[100], (uint8_t)300); CHECK_EQ(blk[511], (uint8_t)(511 * 3));
  sd_x(0xFF); sd_x(0xFF);                                           /* CRC */
  CHECK_EQ(sd_cmd(24, 7, NULL, 0), 0x00);
  sd_x(0xFF); sd_x(0xFE);
  for (int i = 0; i < 512; i++) sd_x((uint8_t)(255 - i));
  sd_x(0xFF); sd_x(0xFF);
  uint8_t resp = sd_x(0xFF);
  CHECK_EQ(resp & 0x1F, 0x05);                                      /* data accepted */
  CHECK_EQ(IMG[7 * 512 + 0], 255); CHECK_EQ(IMG[7 * 512 + 511], (uint8_t)(255 - 511));
  CHECK(SD.dirty);
}

TEST(sd_csd_capacity_field) {
  /* SD Physical Layer Simplified Specification (CSD register, v2.0): C_SIZE is the card size in 1024-block units
   * minus one, so a 1 MiB image (2048 sectors) reports C_SIZE = 1. */
  memset(IMG, 0, sizeof IMG);
  s32_sd_init(&SD, IMG, sizeof IMG);
  CHECK_EQ(sd_cmd(9, 0, NULL, 0), 0x00);
  uint8_t tail[20]; for (unsigned i = 0; i < sizeof tail; i++) tail[i] = sd_x(0xFF);
  CHECK_EQ(tail[0], 0xFF); CHECK_EQ(tail[1], 0xFE);                 /* data start token */
  const uint8_t *csd = tail + 2;
  uint32_t csize = ((uint32_t)(csd[7] & 0x3F) << 16) | ((uint32_t)csd[8] << 8) | csd[9];
  CHECK_EQ((long)csize, (long)(sizeof IMG / 512 / 1024 - 1));
}

int main(void) {
  RUN(fram_write_needs_wel_and_wel_clears_after_write);
  RUN(fram_read_and_address_wrap);
  RUN(fram_rdid);
  RUN(rtc_reads_bcd_time_that_advances_with_emulated_time);
  RUN(rtc_write_sets_the_clock_and_control_regs_store);
  RUN(rtc_address_wraps_after_1b);
  RUN(rtc_time_frozen_within_one_read);
  RUN(sd_init_sequence_sdhc);
  RUN(sd_read_and_write_blocks);
  RUN(sd_csd_capacity_field);
  MT_REPORT();
}
