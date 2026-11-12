/* fram_format.h -- the documented FRAM layout (model/s32_fram.ksy) of the ECU and Controller FM25V10 chips.
 *
 * The FRAM layout is a compatibility contract: a user's configuration image must load on any compatible firmware, so
 * the emulator front ends may read and edit FRAM chip images through it (before boot, or by watching the chip a running
 * ECU writes). They never reach into a firmware's RAM copy of these tables.
 */
#ifndef FRAM_FORMAT_H
#define FRAM_FORMAT_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FRAM_BYTES 131072u
#define FRAM_TABLE_BYTES 0x144u          /* 324: nrows, ncols, 16 RPM, 16 MAP, 16x16 cells, u16 checksum */
#define FRAM_ECU_IGN_TABLE 0x0000u
#define FRAM_ECU_FUEL_TABLE 0x0500u
#define FRAM_ECU_CFG_BASE 0x1480u        /* CONFIG<n> (1..28): ASCII, NUL padded, 16-byte slot n-1 */
#define FRAM_ECU_CFG_BYTES 0x400u
#define FRAM_ECU_CFG_SUM 0x1890u         /* u16le byte sum over the 0x400-byte config region */
#define FRAM_CT_CFG_SUM 0x0400u          /* u16le byte sum over 0x0000..0x03FF, 1 when 0 */
#define FRAM_SLOT_BYTES 16u

typedef struct {
  uint8_t nrows, ncols;
  uint16_t rpm[16];        /* row axis, RPM */
  uint16_t map_x100[16];   /* column axis, MAP in hundredths of inHg (3000 = 30.00 inHg) */
  uint8_t cell[16][16];    /* cell[row][col] */
  uint16_t checksum;       /* as stored at +0x142 */
} fram_table;

void fram_table_decode(const uint8_t *raw, fram_table *t);          /* raw = FRAM_TABLE_BYTES */
void fram_table_encode(const fram_table *t, uint8_t *raw);          /* writes the stored checksum field as given */
/* The table checksum of the stored format (model/s32_fram.ksy): a u16 sum of bytes 0x00..0xA0 (161 bytes), with 0
 * stored as 0x00FF. */
uint16_t fram_table_checksum(const uint8_t *raw);
/* The same sum over all 322 bytes before the checksum word (0x00..0x141), 0 stored as 0x00FF: not the stored format;
 * the bench can put it on a table frame (S32_LINK_TABLE_SEAL=full). */
uint16_t fram_table_checksum_full(const uint8_t *raw);
/* Re-seal a table: store fram_table_checksum (full = false) or fram_table_checksum_full (full = true) at +0x142. */
void fram_table_seal(uint8_t *raw, bool full);
/* A well-formed table: nrows != 0, ncols != 0 and the stored checksum equals fram_table_checksum (which is never 0).
 * No upper bound on nrows / ncols. Reported only (the bench's map view); the emulator never applies it to what a
 * firmware does. */
bool fram_table_valid(const uint8_t *raw);
/* Set one cell and re-seal the checksum. Returns false if row/col is outside 16x16. */
bool fram_table_set_cell(uint8_t *raw, unsigned row, unsigned col, uint8_t v);

/* ECU CONFIG<n> (1..28) from its ASCII slot. Returns false if n is out of range or the slot holds no number. */
bool fram_ecu_config(const uint8_t *fram, unsigned n, int32_t *v);
uint16_t fram_ecu_config_sum(const uint8_t *fram);                  /* the value that belongs at FRAM_ECU_CFG_SUM */
uint16_t fram_ecu_config_sum_stored(const uint8_t *fram);
uint16_t fram_ct_config_sum(const uint8_t *fram);                   /* the value that belongs at FRAM_CT_CFG_SUM */
uint16_t fram_ct_config_sum_stored(const uint8_t *fram);

#endif
