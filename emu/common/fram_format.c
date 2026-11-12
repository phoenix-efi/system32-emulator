/* fram_format.c -- see fram_format.h. */
#include "fram_format.h"
#include <stdlib.h>
#include <string.h>

static uint16_t u16le(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFFu); p[1] = (uint8_t)(v >> 8); }

void fram_table_decode(const uint8_t *raw, fram_table *t) {
  t->nrows = raw[0];
  t->ncols = raw[1];
  for (unsigned i = 0; i < 16u; i++) {
    t->rpm[i] = u16le(raw + 2 + 2 * i);
    t->map_x100[i] = u16le(raw + 0x22 + 2 * i);
  }
  memcpy(t->cell, raw + 0x42, 256);
  t->checksum = u16le(raw + 0x142);
}

void fram_table_encode(const fram_table *t, uint8_t *raw) {
  raw[0] = t->nrows;
  raw[1] = t->ncols;
  for (unsigned i = 0; i < 16u; i++) {
    put16(raw + 2 + 2 * i, t->rpm[i]);
    put16(raw + 0x22 + 2 * i, t->map_x100[i]);
  }
  memcpy(raw + 0x42, t->cell, 256);
  put16(raw + 0x142, t->checksum);
}

uint16_t fram_table_checksum(const uint8_t *raw) {
  uint16_t s = 0;
  for (unsigned i = 0; i <= 0xA0u; i++) s = (uint16_t)(s + raw[i]);
  return s == 0 ? 0x00FFu : s;
}

uint16_t fram_table_checksum_full(const uint8_t *raw) {
  uint16_t s = 0;
  for (unsigned i = 0; i < FRAM_TABLE_BYTES - 2u; i++) s = (uint16_t)(s + raw[i]);
  return s == 0 ? 0x00FFu : s;
}

void fram_table_seal(uint8_t *raw, bool full) {
  put16(raw + 0x142, full ? fram_table_checksum_full(raw) : fram_table_checksum(raw));
}

bool fram_table_valid(const uint8_t *raw) {
  return raw[0] != 0u && raw[1] != 0u && u16le(raw + 0x142) == fram_table_checksum(raw);
}

bool fram_table_set_cell(uint8_t *raw, unsigned row, unsigned col, uint8_t v) {
  if (row >= 16u || col >= 16u) return false;
  raw[0x42 + row * 16u + col] = v;
  put16(raw + 0x142, fram_table_checksum(raw));
  return true;
}

bool fram_ecu_config(const uint8_t *fram, unsigned n, int32_t *v) {
  if (n < 1u || n > 28u) return false;
  char s[FRAM_SLOT_BYTES + 1];
  memcpy(s, fram + FRAM_ECU_CFG_BASE + (n - 1u) * FRAM_SLOT_BYTES, FRAM_SLOT_BYTES);
  s[FRAM_SLOT_BYTES] = '\0';
  char *end = NULL;
  long x = strtol(s, &end, 10);
  if (end == s) return false;
  *v = (int32_t)x;
  return true;
}

static uint16_t sum8(const uint8_t *p, size_t n) {
  uint16_t s = 0;
  for (size_t i = 0; i < n; i++) s = (uint16_t)(s + p[i]);
  return s;
}

uint16_t fram_ecu_config_sum(const uint8_t *fram) { return sum8(fram + FRAM_ECU_CFG_BASE, FRAM_ECU_CFG_BYTES); }
uint16_t fram_ecu_config_sum_stored(const uint8_t *fram) { return u16le(fram + FRAM_ECU_CFG_SUM); }
uint16_t fram_ct_config_sum(const uint8_t *fram) {
  uint16_t s = sum8(fram, 0x400u);
  return s == 0 ? 1u : s;
}
uint16_t fram_ct_config_sum_stored(const uint8_t *fram) { return u16le(fram + FRAM_CT_CFG_SUM); }
