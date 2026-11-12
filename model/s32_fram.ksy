meta:
  id: s32_fram
  title: SPI FRAM Layout (Cypress FM25V10 128 KB — ECU & Cockpit Controller)
  endian: le
  license: Apache-2.0
doc: |
  Declarative binary schema for the Cypress `FM25V10-G` 128 KB (`131,072` B, 17-bit SPI address)
  non-volatile FRAM images used in both the ECU and the Cockpit Controller.

  Evidence: OBSERVED (FRAM images written by the stock v86 firmware on this repository's emulator).
  - ECU FRAM layout:
    - `0x0000..0x0143` (`324` B): ignition table (`16x16` ignition advance map + axes + checksum at `+0x142`).
    - `0x0500..0x0643` (`324` B): fuel table (`16x16` fuel map + axes + checksum at `+0x142`).
    - `0x1480..0x187F` (`1,024` B = `64 x 16` B slots): ASCII NUL-padded slots for ECU settings 1..28 at
      `0x1480 + (n - 1) * 16`.
    - `0x1890..0x1891` (`2` B `u2le`): ECU configuration checksum (16-bit byte sum over `0x1480..0x187F`).
  - Cockpit Controller FRAM layout:
    - `0x0000..0x03FF` (`1,024` B = `64 x 16` B slots): ASCII NUL-padded slots for Controller settings 1..16 at `(n - 1) * 16`
      (when loaded from `CFG.CSV`), OR binary `u2le` words at `(n - 1) * 16`
      (when written from the User Configs screen or the FRAM configuration load).
    - `0x0400..0x0401` (`2` B `u2le`): Controller configuration checksum (byte sum over `0x0000..0x03FF`, or `1` if `0`).

instances:
  ecu_ign_table:
    pos: 0x0000
    type: map_table_324
    doc: ECU ignition timing map (`0x0000..0x0143`, 324 bytes).

  ecu_fuel_table:
    pos: 0x0500
    type: map_table_324
    doc: ECU fuel map (`0x0500..0x0643`, 324 bytes).

  ecu_cfg_slots:
    pos: 0x1480
    type: ascii_slot_16
    repeat: expr
    repeat-expr: 64
    doc: |
      ECU settings 1..28 region (`0x1480..0x187F`, 64 slots of 16 bytes each).
      `CONFIG<n>` (`1 <= n <= 28`) lives at slot `n - 1` (`addr = 0x1480 + (n - 1) * 16`).

  ecu_cfg_checksum:
    pos: 0x1890
    type: u2
    doc: 16-bit byte sum over `0x1480..0x187F`.

  ct_cfg_slots:
    pos: 0x0000
    type: ct_slot_16
    repeat: expr
    repeat-expr: 64
    doc: |
      Cockpit Controller `0x0000..0x03FF` configuration region (64 slots of 16 bytes each).
      Slots `0..15` (settings 1..16, `0x0000..0x00FF`) hold the 16 Controller user settings
      (Fuel Pressure Low/High, Vol/Flow/Temp/Press Units, K Factor, Display Dim Rate,
      Display Min Brightness, Datalog Interval, O2 Scale/Offset, Aux/Fuel Tank params).

  ct_cfg_checksum:
    pos: 0x0400
    type: u2
    doc: 16-bit byte sum over `0x0000..0x03FF`, clamped to 1 if 0.

types:
  map_table_324:
    seq:
      - id: nrows
        type: u1
      - id: ncols
        type: u1
      - id: row_axis
        type: u2
        repeat: expr
        repeat-expr: 16
      - id: col_axis
        type: u2
        repeat: expr
        repeat-expr: 16
      - id: cells
        size: 256
      - id: checksum
        type: u2
        doc: |
          OBSERVED: a 16-bit byte sum over the first 161 bytes of the table (`+0x00..+0xA0`), stored as `0x00FF`
          when the sum is 0. Bytes `+0xA1..+0x141` are not part of the sum.

  ascii_slot_16:
    seq:
      - id: text
        type: strz
        encoding: ASCII
        size: 16

  ct_slot_16:
    seq:
      - id: raw_bytes
        size: 16
    instances:
      u16_le:
        pos: 0
        type: u2
      ascii_str:
        pos: 0
        type: strz
        encoding: ASCII
        size: 16
