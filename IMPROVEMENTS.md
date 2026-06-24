# Reducing corrupt LIDAR packets — analysis & proposals

This collects practical ways to cut the corrupt-packet rate on the Delta-2 LIDAR
rig, tailored to the described hardware, plus ideas for extending the live
monitor (`lidar_server.py`). It also covers your plan to vary the motor speed.

> Baseline reference: the capture in `output.dat` lost ~36–47% of its bytes
> (see the `byte loss %` in the monitor). That is *very* high and points at an
> electrical / timing problem, not a protocol problem. The protocol itself
> decodes cleanly once the parser re-syncs (see `LIDAR_PROTOCOL.md`).

---

## 1. Your current setup, as I understand it

```
 36V battery
   ├── buck #1 -> 5V  ──┬── LIDAR logic + laser supply (the "+ / -" pair)
   │                    └── (5V) -> ESP32 (its onboard reg makes 3.3V)
   └── buck #2 -> ~4V ──── LIDAR spin motor
 LIDAR TX (3.3V) ── ~2" wire ──> ESP32 RX   (RX only, lidar is push-only)
```

Good things already in place:
- **Short RX wire (~2").** Short is exactly what you want at 115200.
- **3.3V signalling** matching the ESP32 — no level shifting needed, no
  over-voltage on the RX pin.
- **Receive-only** link — the lidar just streams; there's no command timing to
  get wrong.

The things most likely to cause the corruption are, in rough priority order:
**(A) motor electrical noise**, **(B) shared-rail / ground issues**, and
**(C) the ESP32 dropping bytes while doing WiFi/micro-ROS**.

---

## 2. Most likely causes, ranked

### A. Brushed-motor noise (probably the #1 cause)
A spinning lidar motor is a brushed DC motor. Its brushes arc as they commutate,
producing wideband electrical noise and current spikes. With motor and logic
sharing the same battery/ground, that noise rides on the ground reference and
couples into a 3.3V UART line very easily.

Fixes (cheap, do all of them):
- **Snub the motor at the motor.** Solder a **0.1 µF ceramic across the two
  motor terminals**, and ideally a 0.1 µF from each terminal to the motor case.
  This is the single highest-value fix for brush noise.
- **Bulk capacitance on the motor's 4V buck output** (e.g. 100–470 µF
  electrolytic + 0.1 µF ceramic) right at the buck, to absorb current spikes.
- **Keep motor wires physically away from the RX signal wire**, and don't run
  them parallel. Twist the two motor leads together.
- **Ferrite bead / clip-on ferrite** on the motor leads.

### B. Shared rails and grounding
The 5V rail powers the laser *and* (via the ESP32's regulator) the logic. Lasers
and the ESP32 both draw pulsed current; if the motor or laser causes the 5V (or
the shared ground) to bounce, the UART sampling reference moves and bytes get
misread.

Fixes:
- **Star-ground everything** back to one point at the battery/buck, rather than
  daisy-chaining grounds through the motor. The motor return current should not
  flow through the logic ground path.
- **Decouple the 5V rail heavily** near the ESP32 and the lidar: a big bulk cap
  (470–1000 µF) plus a 0.1 µF ceramic. ESP32 WiFi TX bursts can sag 5V; this is
  a classic cause of brown-out glitches.
- Consider giving the **motor its own ground return wire** straight back to
  buck #2, separate from the signal/logic ground.
- If feasible, **power the motor from a fully separate supply** (separate buck
  with isolated-ish grounding, joined only at the star point). Isolating the
  noisy load from the quiet logic is the textbook fix.

### C. ESP32 dropping bytes (timing, not electrical)
The original firmware reads the UART in `loop()` while also running WiFi +
micro-ROS. At 115200 baud that's ~11.5 KB/s with no flow control; if WiFi
interrupts or the network stack stall the loop, the UART FIFO/ring overflows and
bytes vanish — which looks exactly like the heavy byte loss in `output.dat`.
The old parser also had a fixed-length bug (see `LIDAR_PROTOCOL.md` §5) that
turned a single dropped byte into a long de-sync.

Fixes:
- **Diagnostic first:** bypass the ESP32 entirely. Wire lidar TX straight to a
  USB-UART adapter (3.3V) on the PC and run
  `python3 lidar_server.py --serial /dev/tty.usbserial-XXXX`.
  - If corruption **disappears**, the problem is ESP32 software/timing.
  - If corruption **remains**, it's electrical (motor/power/ground) — focus on
    A and B.
- If keeping the ESP32 in the loop, read the UART in a **dedicated FreeRTOS task
  pinned to core 0**, with a large ring buffer, separate from the WiFi/network
  task on core 1. Use the hardware FIFO and a generous
  `setRxBufferSize()` (you already set 4096; consider 8192+).
- Fix the firmware parser per `LIDAR_PROTOCOL.md` §5 (length-driven framing +
  checksum validation + resync) so a single glitch costs one packet, not a whole
  revolution.

### D. Switching noise from the buck converters
Bucks emit switching ripple and radiated noise at their switching frequency and
harmonics. If a buck's ground/output is noisy it can pollute the 3.3V reference.

Fixes:
- Add **LC or extra output filtering** on the bucks (especially the one feeding
  logic/5V).
- Mount bucks away from the signal wire; keep their input/output loops small.
- Make sure the ESP32's own 3.3V regulator has its input (5V) well decoupled.

---

## 3. About varying the motor speed (your experiment)

A few things worth knowing before you start sweeping the motor voltage:

- **The UART baud rate is fixed at 115200 regardless of motor speed.** Spinning
  faster does **not** change the byte rate; it changes how many measurement
  points land in each 360° revolution. Faster spin -> fewer points per rev;
  slower spin -> more points per rev. The monitor shows live `rev/s` (decoded
  from byte 8) and `frames/s`, so you can watch this directly.
- **Why speed can still affect corruption:** a faster motor commutates more
  often and usually generates *more* brush noise and bigger current spikes, so
  slowing it down often *reduces* electrically-induced corruption. That is most
  likely the effect you'll observe — and it's a strong hint the root cause is
  electrical (cause A/B), which you can then fix properly with snubbing/grounding
  instead of having to run the motor slow.
- **Open-loop voltage drive may make speed unstable.** Many of these lidars
  expect to regulate their own motor (closed-loop via a PWM/“MOTOR” control pin
  with tachometer feedback). Driving the motor from a fixed ~4V buck open-loop
  can let the speed wander; unstable speed can upset the lidar's internal timing
  and its reported rotation value. If your unit has a motor-control/PWM input,
  prefer letting the lidar control speed (or PWM it) rather than a fixed voltage.
- **Don't over-volt the motor.** Stay within the lidar's rated spin range
  (typically ~5–6 rev/s / ~300–360 rpm for Delta-2). Too slow and the scan rate
  drops / the firmware may flag under-speed; too fast risks mechanical wear and
  encoder sync loss.

### Suggested method (use the monitor for this)
1. Start the monitor against the real link (`--serial` or `--tcp`).
2. Click **Reset stats**, set a motor voltage, let it run **~30 s** at steady
   speed, and record `rev/s`, `corrupt %`, `packet err %`, and `byte loss %`.
3. Step the voltage (e.g. 3.5V → 4.0V → 4.5V → 5.0V), resetting stats at each
   step, and build a table of speed vs corruption.
4. Watch `byte loss %` and `packet err %` rather than `corrupt frames %` — the
   frame metric also reacts to speed (fewer packets/rev), whereas byte-loss and
   checksum-error rates are cleaner integrity measures.
5. Then apply the electrical fixes (§2A/B) and repeat — you should be able to run
   at full speed *and* low corruption.

> Note on the "corrupt frames %" metric: a frame is flagged corrupt if any bytes
> were discarded or any checksum failed while it was being assembled. Because
> changing motor speed changes packets-per-revolution, prefer **packet err %**
> and **byte loss %** as your speed-independent integrity numbers.

---

## 4. Quick electrical checklist

- [ ] 0.1 µF ceramic across motor terminals (and to case if possible).
- [ ] Bulk + ceramic cap on the 4V motor buck output.
- [ ] Bulk (470–1000 µF) + 0.1 µF on the 5V logic rail near the ESP32 & lidar.
- [ ] Star-ground; separate motor return; don't share the motor's ground path
      with logic.
- [ ] Motor leads twisted, ferrite fitted, routed away from the RX wire.
- [ ] Confirm lidar TX idles high at ~3.3V and the RX wire has a solid ground
      reference running alongside it.
- [ ] Diagnostic: run direct PC `--serial` (no ESP32) to split electrical vs
      firmware causes.
- [ ] If electrical noise persists, consider an isolated/separate motor supply.

---

## 5. Firmware / software improvements

- Fix `src/main.cpp` per `LIDAR_PROTOCOL.md` §5 (length-driven framing, correct
  16-bit start angle, big-endian mm distance, 21 samples, 22.5/21° step,
  checksum validation). This alone turns one glitch from "lost revolution" into
  "lost single packet".
- Read the UART in a dedicated, core-pinned task with a big ring buffer.
- Have the ESP32 expose its own dropped-byte / checksum-fail counters over the
  network so you can see whether loss happens *before* or *after* the ESP32.
- For pure debugging, add a "raw passthrough" firmware mode that just forwards
  lidar bytes verbatim to TCP — then `lidar_server.py --tcp <port>` sees exactly
  what the lidar sent, and the parser/quality metrics live on the PC.

---

## 6. Ideas to extend the live monitor (`lidar_server.py`)

Nice-to-haves for the debugging UI, roughly in order of usefulness:

1. **Capture-to-disk button** — record the raw byte stream while monitoring so a
   corruption event can be replayed later with `--file`.
2. **Corruption timeline / rolling-window chart** — plot `byte loss %` and
   `packet err %` over the last 60 s so you can *see* the effect of changing
   motor voltage in real time, not just cumulative totals.
3. **Per-angle corruption heat-map** — if errors cluster at particular angles it
   suggests a mechanical/optical or per-position electrical cause rather than
   broadband noise.
4. **CSV / JSON logging of the stats** so the motor-speed sweep (§3) can be
   tabulated and graphed automatically.
5. **Rolling-window stats toggle** (last N seconds) in addition to cumulative.
6. **Motor-speed control from the UI** (if the ESP32 drives the motor PWM):
   switch the SSE stream to a WebSocket so the browser can command speed and the
   server can auto-sweep + correlate speed vs corruption.
7. **Distance/intensity histograms** and a min/typical/max range readout.
8. **Configurable "corrupt frame" definition** (e.g. treat <N packets/rev as
   corrupt, toggle on/off) so the metric can be made speed-independent.
9. **Multiple simultaneous sources** (compare ESP32-forwarded stream vs direct
   serial side-by-side to localise where loss is introduced).
10. **Audible/visual alarm** above a configurable corruption threshold for
    hands-free knob-twiddling during the motor sweep.
