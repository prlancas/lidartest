# Delta-2 LIDAR Serial Protocol (3irobotix family)

This document describes the binary protocol emitted by the LIDAR over UART
(115200 8N1), as reverse‑engineered from `output.dat`. It is the protocol used
by the **3irobotix Delta‑2** series (Delta‑2A / 2B / 2G, sold under names like
"Droidal" / "DV005").

All multi‑byte numeric fields are **big‑endian** unless noted.

---

## 1. Packet framing

Every measurement packet has this layout. In the captured data every packet is
**78 bytes** long and carries **21 measurement points**, but the format is
self‑describing via the length fields, so a robust parser should not hard‑code
the size.

```
 byte   field                     example   meaning
 ----   -----------------------   -------   --------------------------------------------
  0     sync                      AA        frame start marker
  1..2  frame length (BE)         00 4C     = total_packet_bytes - 2   (0x004C = 76 -> 78 total)
  3     protocol version          01        always 0x01
  4     frame type                61        always 0x61 ('a')
  5     command word              AD        0xAD = measurement / point-cloud data
  6..7  data length (BE)          00 44     length of the data field that follows (0x44 = 68)
  8     rotation speed            7B        revolutions/s = value * 0.05  (0x7B=123 -> ~6.15 rev/s ~ 370 rpm)
  9..10 offset/reserved           FF DC     constant in the capture (0xFFDC, signed = -36 -> -0.36 deg). Treat as fixed.
 11..12 start angle (BE)          57 E4     start angle of this packet in 0.01 deg (0x57E4 = 22500 -> 225.00 deg)
 13..N  sample data               ...       21 samples x 3 bytes (see below)
  N+1   checksum (BE) high        ..        16-bit sum of all preceding bytes, low 16 bits
  N+2   checksum (BE) low         ..
```

### Field notes

- **frame length** (`bytes 1..2`) = total packet length − 2. For a 78‑byte
  packet this is `0x004C = 76`. Use it to find the next packet:
  `next = current + frame_length + 2`.
- **data length** (`bytes 6..7`) = number of bytes in the data field
  (`bytes 8 .. checksum-1`). For 21 samples: `3 (rot/offset) + 2 (start angle) + 21*3 = 68 = 0x44`.
  Number of samples = `(data_length - 5) / 3`.
- **rotation speed** (`byte 8`): multiply by `0.05` to get revolutions per
  second. Captured value ~123 → ~6.15 rev/s (~370 rpm).
- **offset/reserved** (`bytes 9..10`): constant `0xFFDC` throughout the capture.
  Most likely a fixed angular offset/calibration field; not needed to plot points.
- **start angle** (`bytes 11..12`): angle of the *first* sample in this packet,
  in hundredths of a degree. Consecutive packets increase by exactly
  `2250` (22.50°), so there are **16 packets per 360° revolution**.

---

## 2. Sample (measurement point) encoding

Each sample is **3 bytes**:

```
 offset  field            meaning
 ------   --------------   ------------------------------------------
   0      signal/quality   intensity / reflectivity (0..255)
   1      distance high    distance in mm, big-endian high byte
   2      distance low     distance in mm, big-endian low byte
```

- `distance_mm = (byte1 << 8) | byte2`  → **distance in millimetres** (big‑endian).
  Observed range in the capture: 556 mm … 9958 mm (≈0.56 m … 10 m), median ≈2.0 m.
- `distance_mm == 0` means **no return** (out of range / no reflection) — skip it.
- The 21 samples are spread evenly across the packet's 22.5° sector, so the
  angle of sample `i` (i = 0..20) is:

```
angle_deg(i) = start_angle_deg + i * (22.5 / 21)
             = start_angle_deg + i * 1.0714...
```

  i.e. an angular resolution of about **1.07°** (≈336 points per full revolution).

---

## 3. Checksum

The last two bytes are a 16‑bit big‑endian checksum:

```
checksum = sum(every byte from index 0 up to and including the last data byte) & 0xFFFF
```

Validated against the capture: it matches for every well‑formed packet.
Use it (together with the `01 61 AD` header signature) to reject false
`AA 00` sync matches that occur inside sample data or corrupted regions.

---

## 4. Worked example

First packet of `output.dat`:

```
AA 00 4C 01 61 AD 00 44 7B FF DC 57 E4 | 41 05 95 | 3C 05 B8 | 39 05 CE | ...
```

| field            | bytes   | value                                   |
|------------------|---------|-----------------------------------------|
| sync             | `AA`    | start                                   |
| frame length     | `00 4C` | 76 → total packet = 78 bytes            |
| protocol version | `01`    | 1                                       |
| frame type       | `61`    | measurement frame                       |
| command          | `AD`    | point-cloud data                        |
| data length      | `00 44` | 68 → (68−5)/3 = 21 samples              |
| rotation speed   | `7B`    | 123 × 0.05 ≈ 6.15 rev/s                  |
| offset/reserved  | `FF DC` | fixed                                   |
| start angle      | `57 E4` | 0x57E4 = 22500 → **225.00°**            |
| sample 0         | `41 05 95` | quality 0x41=65, dist 0x0595 = **1429 mm** @ 225.00° |
| sample 1         | `3C 05 B8` | quality 0x3C=60, dist 0x05B8 = **1464 mm** @ 226.07° |
| sample 2         | `39 05 CE` | quality 0x39=57, dist 0x05CE = **1486 mm** @ 227.14° |

---

## 5. Bugs in `src/main.cpp`

The decoder in `processLidarByte()` does **not** match the real protocol. Issues:

1. **Hard‑coded packet length is wrong.** It sets `packet_len = 0x52` (82) and
   reads `packet_len + 2 = 84` bytes, but real packets are **78** bytes. Reading
   6 extra bytes eats into the next packet's header and causes the parser to
   permanently de‑sync.

2. **Start angle is read from a single byte.** It uses
   `angle_index = buffer[11]` and `start_angle_deg = angle_index * 2.0`. The
   start angle is a **16‑bit** value in `buffer[11..12]`, in units of 0.01°:
   `start_angle_deg = ((buffer[11] << 8) | buffer[12]) / 100.0`.

3. **Sample data starts at the wrong offset.** It begins at `offset = 12 + i*3`,
   but samples start at byte **13** (after the 2‑byte start angle). Everything is
   shifted by one byte, so quality/distance bytes are mis‑aligned.

4. **Distance endianness is reversed.** It computes
   `(buffer[offset+2] << 8) | buffer[offset+1]` (little‑endian). The distance is
   **big‑endian**: `(buffer[o+1] << 8) | buffer[o+2]` where `o = 13 + i*3`.

5. **Wrong sample count.** It loops `i < 22`, but there are **21** samples; the
   22nd read runs into the checksum.

6. **Wrong per‑sample angle step.** It uses `i * (2.0 / 22.0)` ≈ 0.09°/sample.
   The correct step is `i * (22.5 / 21)` ≈ **1.07°/sample** (≈12× larger).

7. **No checksum validation** and the frame reset relies on the (broken)
   high‑byte angle hitting exactly 0; it should reset on a start‑angle wrap.

### Corrected inner loop (sketch)

```cpp
// packet is 78 bytes; samples start at byte 13, 21 of them, 3 bytes each.
float start_angle_deg = ((buffer[11] << 8) | buffer[12]) / 100.0f;
const float step = 22.5f / 21.0f;            // ~1.0714 deg per sample

for (int i = 0; i < 21; i++) {
  int o = 13 + (i * 3);
  uint8_t  quality = buffer[o];
  uint16_t dist_mm = (buffer[o + 1] << 8) | buffer[o + 2];   // big-endian, mm
  if (dist_mm == 0) continue;                                 // no return

  float ang = fmodf(start_angle_deg + i * step, 360.0f);
  int idx = ((int)lroundf(ang)) % 360;
  if (idx < 0) idx += 360;
  ranges[idx] = dist_mm / 1000.0f;                            // metres
}
```

---

## 6. Capture quality

`output.dat` is a raw UART dump with no flow control, so roughly a third of the
bytes are corrupt or dropped fragments. A length‑ and checksum‑validating parser
recovers **224** clean packets. Because of the loss, only a few complete 16‑packet
revolutions survive intact; most reconstructed "frames" are partial. This is a
property of the capture, not the protocol.
