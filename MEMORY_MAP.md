# TCA-152A codeplug memory map (driver reference)

Condensed from the OpenTCA `CONFIG_MAP.md`. File offset == radio config
address. Total codeplug: `0x1DB0` (7600) bytes.

## Encodings

- **Channel frequency:** 4-byte little-endian packed BCD, 10 Hz units
  (`freq_Hz = value × 10`). `00 00 05 40` → 400.50000 MHz.
- **FM tuner frequency:** 4-byte LE BCD, **0.1 MHz** units (`MHz = value / 10`).
- **CTCSS/DTCS tone:** 2-byte LE BCD. CTCSS = value × 0.1 Hz. DTCS when
  value ≥ 8000: low 3 digits = code, leading digit = polarity flag
  (8 = normal confirmed; 9 = inverted assumed). `FF FF`/`00 00` = none.

## Layout

| Offset | Size | Contents |
|---|---|---|
| `0x0010` | 128 × 16 B | Channel table (zones A–H × 16) |
| `0x0CA0` | ~17 B | Global settings (VOX, Step, RPT-RL, Channel Group, FM ch#) |
| `0x0CD0` | 25 × 4 B | FM tuner channel table |
| `0x0D40` | 128 × 8 B | Channel names (ASCII, padded) |
| `0x1150` | 128 × 4 B | Per-channel codes (u32) |
| `0x13F0` | 16 B | Scan-list bitmask (1 bit/channel) |
| `0x1510` | 4 B | FM VFO frequency |
| `0x1820` | 256 B | Boot logo (64×32 mono, page-based) |

## Channel record (16 bytes)

| Off | Field | Notes |
|---|---|---|
| +0 | RX freq | BCD LE / 10 Hz |
| +4 | TX freq | `FF FF FF FF` = TX off |
| +8 | RX tone (decode) | CTCSS/DTCS |
| +10 | TX tone (encode) | CTCSS/DTCS |
| +12 | reserved | `00` |
| +13 | flags A | bit2 Busy Lock · bit6 SPEC · bit7 Special QT/DQT |
| +14 | flags B | bit3 = narrow (clear = wide) |
| +15 | TX power | 0 = Low, 1 = Mid, 2 = High |

Scan Add is the `0x13F0` bitmask, not the record: byte `0x13F0 + (ch>>3)`,
bit `ch & 7`, set = in scan list.

## Global settings (`0x0CA0` base, confirmed offsets)

| +off | Field | Encoding |
|---|---|---|
| +2 | FM Work Mode | candidate (bit7) |
| +6 | FM current channel | 1–25 |
| +7 | VOX Gain | low nibble: 0=OFF, 1–5 |
| +8 | Frequency Step | high nibble: 0=2.50K … 6=50.00K, 7=100K |
| +12 | RPT-RL | 0–9 |
| +16 | Channel Group | 0-based (value = group − 1) |
