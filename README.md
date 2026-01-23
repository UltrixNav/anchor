# TDOA3 Anchor (ESP32 + DW1000)

This project ports the Bitcraze **TDoA3 anchor** algorithm to an ESP32 (Arduino framework) using the **DW1000Ng** driver. The implementation keeps the same core logic—randomized transmissions, dynamic neighbor selection, clock correction, and distance extraction—while using a compact custom packet wrapper suitable for DW1000Ng.

## What’s included

- `src/tdoa3_anchor.hpp/.cpp`: Core TDoA3 logic (anchor context, clock correction, distance extraction).
- `src/main.cpp`: DW1000Ng radio setup, interrupt handlers, and scheduling loop.
- `platformio.ini`: Pulls the `DW1000Ng` library from PlatformIO.
- `include/tdoa3_config.hpp`: Board/radio/application configuration for `main.cpp` (pins, IDs, radio config, logging, position).
- `include/tdoa3_anchor_config.hpp`: TDoA3 algorithm tuning knobs for `tdoa3_anchor.cpp` (TX rate, filtering, debug).

## Wiring

Update the pin constants in `include/tdoa3_config.hpp` to match your wiring:

- `kPinDwCs`
- `kPinDwIrq`
- `kPinDwRst`

> The defaults are placeholders for the Seeed XIAO ESP32C3. Adjust them to your actual wiring.

## Packet format

The radio uses a small custom wrapper (not full 802.15.4) to keep the payload light:

```
[AppHeader]
  sourceId (1 byte)
  destId   (1 byte)
  type     (1 byte)  // 0x30

[RangeHeader]
  type        (1 byte)  // 0x30
  seq         (1 byte)
  txTimeStamp (4 bytes)
  remoteCount (1 byte)

[RemoteAnchorData] ...
```

This is enough for the TDoA3 algorithm to work while keeping the code and frame sizes simple. If you need full 802.15.4 MAC frames to interoperate with Bitcraze anchors, you can extend the wrapper to match their MAC layer.

## Configuration

All user-tunable settings are kept in headers under `include/`.

### `include/tdoa3_config.hpp` (board + radio + app)

These values affect how the DW1000 is wired/configured and how often/what we log.

- **DW1000 wiring**
  - `kPinDwCs`, `kPinDwIrq`, `kPinDwRst` — SPI chip-select, IRQ, reset pins.
  - `kBuiltInLedFallbackPin` — used only if `LED_BUILTIN` is not defined by the board.

- **Identity / network**
  - `kNetworkId` — PAN/network identifier used for filtering (should match all nodes).
  - `kAnchorId` — unique anchor ID (set a different value on each anchor).

- **Calibration**
  - `kAntennaDelayTicks` — antenna delay calibration (DW1000 TX/RX delay in device ticks).

- **Position block (optional)**
  - `kPositionEnabled` — whether to append a position block to outgoing frames.
  - `kPositionX`, `kPositionY`, `kPositionZ` — anchor position in meters (float). If you don’t have a solver/visualizer using it, keep disabled.

- **RX/TX behavior and logging**
  - `kEnableDebug` — master switch for Serial prints.
  - `kDebugIntervalMs` — how often periodic debug lines are printed.
  - `kUseRxInterrupts` — enable DW1000 IRQ-based RX (currently optional; polling fallback can be used).
  - `kUseRxPollingFallback` — if true, also check RX completion via polling.
  - `kLogEveryRx`, `kLogEveryTx` — verbose per-packet logging.

- **Timing**
  - `kDelayedTxMarginUs` — lead time for delayed TX scheduling (too small => missed delayed TX).
  - `kRxFrameWaitTimeoutUs`, `kSfdTimeoutSymbols` — optional receive timeouts (currently not enabled in code).

- **DW1000 radio config structs**
  - `kRadioConfig` — channel, preamble, data rate, PRF, frame check, etc.
  - `kInterruptConfig` — which DW1000 interrupts are enabled.

> ⚠️ Important: all nodes (anchors + tag) must use compatible PHY settings (channel/preamble/data rate/etc). If you change `kRadioConfig`, update the tag to match.

### `include/tdoa3_anchor_config.hpp` (algorithm tuning)

These values tune the *TDoA3 anchor logic* (how often we transmit, how we filter clock correction, how aggressive we are with debug).

- **Transmit rate control**
  - `kAnchorMinTxFreqHz`, `kAnchorMaxTxFreqHz` — clamps the per-anchor TX frequency.
    The algorithm computes roughly:
    - $f \approx \frac{systemTxFreq}{(anchors+1)}$ then clamps to min/max.
  - `kDelayRandomIntervalMs` — random jitter added to TX schedule to avoid persistent collisions.
  - `kAnchorListUpdateMs` — how often we rebuild the “seen anchors” list and choose neighbors to include.

- **Data validity**
  - `kDistanceValidityMs` — if distance/clock correction data is older than this, it is cleared.

- **Clock correction (drift) filtering**
  - `kMaxClockDeviation` — expected maximum clock deviation (fraction), e.g. 100e-6 = 100 ppm.
  - `kClockCorrectionSpecMin`, `kClockCorrectionSpecMax` — acceptable correction range.
  - `kClockCorrectionAcceptedNoise` — threshold defining “small changes” treated as noise.
  - `kClockCorrectionFilter` — low-pass filter factor (smaller = smoother, slower).
  - `kClockCorrectionBucketMax` — confidence bucket size; good updates fill it, bad drain it.

- **Behavior toggles**
  - `kAnchorDebug` — enable/disable debug prints in `tdoa3_anchor.cpp`.
  - `kAutoCreateContextOnRx` — automatically create context when receiving from an unknown anchor ID.
  - `kAnchorDebugIntervalMs`, `kTxListDebugIntervalMs` — rate limit debug prints.

## Basic setup checklist

1. Set correct DW1000 wiring pins in `include/tdoa3_config.hpp`.
2. Set a unique `kAnchorId` per anchor.
3. Ensure `kRadioConfig` matches the tag and other anchors (channel, data rate, preamble, etc).
4. Calibrate `kAntennaDelayTicks`.
5. (Optional) Fill in `kPositionX/Y/Z` and enable `kPositionEnabled`.

- `kAnchorId` — unique ID for each anchor
- `kNetworkId` — PAN ID for filtering
- `kAntennaDelayTicks` — antenna delay (calibration)
- `kRadioConfig` — channel, data rate, PRF, preamble

## How to run

Build and upload with PlatformIO (VS Code or CLI). If you use the CLI, you can run:

```powershell
pio run
```

```powershell
pio run -t upload
```

## Notes

- The current implementation assumes all anchors use the same simplified packet format.
- For production deployment, add proper calibration, long-term clock drift tracking, and full MAC framing.
- The algorithm is designed for anchors (fixed nodes), not mobile tags.
