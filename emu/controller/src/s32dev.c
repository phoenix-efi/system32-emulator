/* s32dev.c -- FM25V10 FRAM and PCF2127 RTC SPI slave models (see s32dev.h). Datasheet behaviour only; nothing here is
 * specific to the EFII firmware. */
#include "s32dev.h"
#include <string.h>

/* ---------------- FM25V10 ---------------- */
enum { F_WREN = 0x06, F_WRDI = 0x04, F_RDSR = 0x05, F_WRSR = 0x01, F_READ = 0x03, F_FSTRD = 0x0B, F_WRITE = 0x02,
       F_SLEEP = 0xB9, F_RDID = 0x9F };
static const uint8_t FRAM_ID[9] = {0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0xC2, 0x24, 0x00};   /* Cypress, FM25V10 */

void s32_fram_init(s32_fram *f, uint8_t fill) {
  memset(f, 0, sizeof *f);
  memset(f->mem, fill, sizeof f->mem);
}

static void fram_select(void *ctx, bool asserted) {
  s32_fram *f = ctx;
  if (asserted) { f->asleep = false; f->op = 0; f->phase = 0; return; }
  if (f->op == F_WRITE && f->phase > 3) f->sr &= (uint8_t)~0x02;   /* WEL clears when a write ends */
  if (f->op == F_WRSR && f->phase > 0) f->sr &= (uint8_t)~0x02;
  f->op = 0;
}

static uint8_t fram_exchange(void *ctx, uint8_t mosi) {
  s32_fram *f = ctx;
  if (f->fault == S32_DEV_ABSENT) return 0xFF;
  if (f->fault == S32_DEV_MISO_LOW) return 0x00;
  if (f->asleep) return 0xFF;
  if (f->op == 0) {
    f->op = mosi; f->phase = 0; f->addr = 0;
    if (mosi == F_WREN) f->sr |= 0x02;
    else if (mosi == F_WRDI) f->sr &= (uint8_t)~0x02;
    else if (mosi == F_SLEEP) f->asleep = true;
    return 0xFF;
  }
  unsigned ph = f->phase++;
  switch (f->op) {
  case F_RDSR: return f->sr;
  case F_WRSR:
    if (ph == 0 && (f->sr & 0x02)) f->sr = (uint8_t)((mosi & 0x8C) | (f->sr & 0x02));
    return 0xFF;
  case F_RDID: return ph < sizeof FRAM_ID ? FRAM_ID[ph] : 0x00;
  case F_READ: case F_FSTRD: case F_WRITE: {
    if (ph < 3) { f->addr = ((f->addr << 8) | mosi) & (S32_FRAM_BYTES - 1); return 0xFF; }
    if (f->op == F_FSTRD && ph == 3) return 0xFF;                     /* fast read: one dummy byte */
    uint32_t a = f->addr; f->addr = (f->addr + 1) & (S32_FRAM_BYTES - 1);
    if (f->op == F_WRITE) {
      if ((f->sr & 0x02) && f->fault != S32_DEV_WP) { f->mem[a] = mosi; f->writes++; }   /* no WEL: write ignored */
      return 0xFF;
    }
    f->reads++;
    return f->mem[a];
  }
  default: return 0xFF;
  }
}

s32_spi_dev s32_fram_dev(s32_fram *f) { return (s32_spi_dev){"fram", f, fram_exchange, fram_select}; }

/* ---------------- PCF2127 ---------------- */
static uint8_t bcd(int v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }
static int unbcd(uint8_t b) { return (b >> 4) * 10 + (b & 0x0F); }

time_t s32_rtc_now(const s32_rtc *r) { return r->base + (time_t)((double)*r->cycles / r->fcy); }

static void rtc_latch_time(s32_rtc *r) {
  time_t t = s32_rtc_now(r);
  struct tm tm;
  gmtime_r(&t, &tm);
  if (r->invalid) {                                              /* backup supply lost: registers hold garbage */
    static const uint8_t junk[7] = {0x7F, 0x7F, 0x3F, 0x3F, 0x07, 0x1F, 0xFF};
    for (int i = 0; i < 7; i++) r->reg[3 + i] = (uint8_t)((i == 0 ? (r->reg[3] & 0x80) : 0) | (junk[i] & (i == 0 ? 0x7F : 0xFF)));
    return;
  }
  r->reg[3] = (uint8_t)((r->reg[3] & 0x80) | bcd(tm.tm_sec));   /* keep OSF */
  r->reg[4] = bcd(tm.tm_min);
  r->reg[5] = bcd(tm.tm_hour);                                  /* 24-hour mode */
  r->reg[6] = bcd(tm.tm_mday);
  r->reg[7] = (uint8_t)tm.tm_wday;
  r->reg[8] = bcd(tm.tm_mon + 1);
  r->reg[9] = bcd(tm.tm_year % 100);
}

static void rtc_commit_time(s32_rtc *r) {
  struct tm tm = {0};
  tm.tm_sec = unbcd(r->reg[3] & 0x7F); tm.tm_min = unbcd(r->reg[4] & 0x7F); tm.tm_hour = unbcd(r->reg[5] & 0x3F);
  tm.tm_mday = unbcd(r->reg[6] & 0x3F); tm.tm_mon = unbcd(r->reg[8] & 0x1F) - 1; tm.tm_year = 100 + unbcd(r->reg[9]);
  r->base = timegm(&tm) - (time_t)((double)*r->cycles / r->fcy);
  r->invalid = false;
}

void s32_rtc_set_osf(s32_rtc *r, bool set) { r->reg[3] = (uint8_t)(set ? (r->reg[3] | 0x80) : (r->reg[3] & 0x7F)); }

void s32_rtc_init(s32_rtc *r, const uint64_t *cycles, double fcy, time_t base) {
  memset(r, 0, sizeof *r);
  r->cycles = cycles; r->fcy = fcy; r->base = base; r->phase = -1;
}

static void rtc_select(void *ctx, bool asserted) {
  s32_rtc *r = ctx;
  if (asserted) { r->phase = -1; r->time_written = false; return; }
  if (r->time_written) rtc_commit_time(r);
}

static uint8_t rtc_exchange(void *ctx, uint8_t mosi) {
  s32_rtc *r = ctx;
  if (r->fault == S32_DEV_ABSENT) return 0xFF;
  if (r->fault == S32_DEV_MISO_LOW) return 0x00;
  if (r->phase < 0) {                    /* command byte: bit7 read, bits 6-5 subaddress (01 = registers), 4-0 address */
    r->phase = 0;
    r->read = (mosi & 0x80) != 0;
    r->ptr = mosi & 0x1F;
    if (((mosi >> 5) & 3) != 1) r->ptr = 0xFF;   /* other subaddresses: not modelled */
    if (r->read) rtc_latch_time(r);
    return 0xFF;
  }
  r->phase++;
  if (r->ptr == 0xFF) return 0xFF;
  uint8_t p = r->ptr;
  r->ptr = (uint8_t)((p + 1) % 0x1C);        /* auto-increment, wraps after 0x1B */
  if (r->read) return r->reg[p];
  r->reg[p] = mosi;
  if (p >= 3 && p <= 9) r->time_written = true;
  return 0xFF;
}

s32_spi_dev s32_rtc_dev(s32_rtc *r) { return (s32_spi_dev){"rtc", r, rtc_exchange, rtc_select}; }

/* ---------------- SD card (SPI mode) ---------------- */
void s32_sd_init(s32_sd *c, uint8_t *img, uint32_t bytes) {
  memset(c, 0, sizeof *c);
  c->img = img; c->sectors = bytes / 512u; c->idle = true;
}
static void sd_q(s32_sd *c, const uint8_t *b, unsigned n) {
  if (c->nout + n > sizeof c->out) return;
  memcpy(c->out + c->nout, b, n); c->nout += n;
}
static void sd_q1(s32_sd *c, uint8_t b) { sd_q(c, &b, 1); }
static void sd_queue_block(s32_sd *c, uint32_t blk) {
  sd_q1(c, 0xFF); sd_q1(c, 0xFE);
  if (blk < c->sectors) sd_q(c, c->img + (size_t)blk * 512u, 512); else { uint8_t z[512] = {0}; sd_q(c, z, 512); }
  sd_q1(c, 0xFF); sd_q1(c, 0xFF);                                /* CRC (ignored in SPI mode) */
  c->reads++;
}
static void sd_command(s32_sd *c) {
  unsigned cmd = c->cmd[0] & 0x3Fu;
  uint32_t arg = ((uint32_t)c->cmd[1] << 24) | ((uint32_t)c->cmd[2] << 16) | ((uint32_t)c->cmd[3] << 8) | c->cmd[4];
  uint8_t r1 = c->idle ? 0x01 : 0x00;
  bool app = c->app; c->app = false;
  sd_q1(c, 0xFF);                                                 /* NCR */
  if (app && cmd == 41) { c->idle = false; sd_q1(c, 0x00); return; }
  switch (cmd) {
  case 0: c->idle = true; c->mode = 0; sd_q1(c, 0x01); break;
  case 8: { uint8_t r7[5] = {r1, 0, 0, (uint8_t)(arg >> 8 & 0xF), (uint8_t)arg}; sd_q(c, r7, 5); break; }
  case 55: c->app = true; sd_q1(c, r1); break;
  case 58: { uint8_t r3[5] = {r1, 0xC0, 0xFF, 0x80, 0x00}; sd_q(c, r3, 5); break; }   /* powered up, CCS = 1 (SDHC) */
  case 9: {                                                      /* CSD v2: C_SIZE = sectors / 1024 - 1 */
    uint32_t cs = c->sectors / 1024u - 1u;
    uint8_t csd[16] = {0x40, 0x0E, 0x00, 0x32, 0x5B, 0x59, 0x00, (uint8_t)(cs >> 16 & 0x3F), (uint8_t)(cs >> 8), (uint8_t)cs, 0x7F, 0x80, 0x0A, 0x40, 0x00, 0x01};
    sd_q1(c, 0x00); sd_q1(c, 0xFF); sd_q1(c, 0xFE); sd_q(c, csd, 16); sd_q1(c, 0xFF); sd_q1(c, 0xFF); break;
  }
  case 10: { uint8_t cid[16] = {0x03, 'S', 'D', 'E', 'M', 'U', 'L', '1', 0x10, 0, 0, 0, 1, 0x01, 0x6A, 0x01};
    sd_q1(c, 0x00); sd_q1(c, 0xFF); sd_q1(c, 0xFE); sd_q(c, cid, 16); sd_q1(c, 0xFF); sd_q1(c, 0xFF); break; }
  case 12: c->mode = 0; c->nout = c->iout = 0; sd_q1(c, 0xFF); sd_q1(c, 0x00); break;
  case 13: sd_q1(c, 0x00); sd_q1(c, 0x00); break;
  case 16: sd_q1(c, r1); break;
  case 17: sd_q1(c, 0x00); if (c->fault == S32_DEV_RDERR) { sd_q1(c, 0xFF); sd_q1(c, 0x08); } else sd_queue_block(c, arg); break;
  case 18: sd_q1(c, 0x00); if (c->fault == S32_DEV_RDERR) { sd_q1(c, 0xFF); sd_q1(c, 0x08); break; }
    c->addr = arg; c->mode = 1; sd_queue_block(c, c->addr++); break;
  case 24: sd_q1(c, 0x00); c->addr = arg; c->mode = 2; c->nw = 0; c->wtok = false; break;
  case 25: sd_q1(c, 0x00); c->addr = arg; c->mode = 3; c->nw = 0; c->wtok = false; break;
  default: sd_q1(c, 0x04 | r1); break;                           /* illegal command */
  }
}
static uint8_t sd_exchange(void *ctx, uint8_t mosi) {
  s32_sd *c = ctx;
  if (c->fault == S32_DEV_ABSENT) return 0xFF;                    /* card not answering: every command times out */
  if (c->fault == S32_DEV_MISO_LOW) return 0x00;
  uint8_t miso = 0xFF;
  if (c->iout < c->nout) { miso = c->out[c->iout++]; if (c->iout == c->nout) c->nout = c->iout = 0; }
  else if (c->mode == 1) { sd_queue_block(c, c->addr++); }       /* keep streaming until CMD12 */
  if (c->mode == 2 || c->mode == 3) {                             /* receiving a data block */
    if (!c->wtok) {
      if (mosi == 0xFE || mosi == 0xFC) { c->wtok = true; c->nw = 0; }
      else if (mosi == 0xFD && c->mode == 3) { c->mode = 0; uint8_t busy[3] = {0xFF, 0x00, 0xFF}; sd_q(c, busy, 3); }
      else if ((mosi & 0xC0) == 0x40 && c->mode == 3) { c->mode = 0; c->cmd[0] = mosi; c->ncmd = 1; }
      return miso;
    }
    c->wbuf[c->nw++] = mosi;
    if (c->nw == 514) {                                            /* 512 data + 2 CRC */
      bool ok = c->fault != S32_DEV_WRERR;
      if (ok && c->addr < c->sectors) { memcpy(c->img + (size_t)c->addr * 512u, c->wbuf, 512); c->dirty = true; }
      c->writes++; c->addr++; c->wtok = false;
      uint8_t resp[4] = {ok ? 0x05 : 0x0D, 0x00, 0x00, 0xFF};      /* data accepted (or write error), short busy */
      sd_q(c, resp, 4);
      if (c->mode == 2) c->mode = 0;
    }
    return miso;
  }
  if (c->ncmd == 0 && (mosi & 0xC0) != 0x40) return miso;          /* idle 0xFF between commands */
  c->cmd[c->ncmd++] = mosi;
  if (c->ncmd == 6) { c->ncmd = 0; c->nout = c->iout = 0; sd_command(c); }
  return miso;
}
static void sd_select(void *ctx, bool asserted) {
  s32_sd *c = ctx;
  if (!asserted) { c->ncmd = 0; if (c->mode == 1) { c->mode = 0; c->nout = c->iout = 0; } }
}
s32_spi_dev s32_sd_dev(s32_sd *c) { return (s32_spi_dev){"sd", c, sd_exchange, sd_select}; }
