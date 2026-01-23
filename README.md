# TDOA3 Anchor (ESP32 + DW1000)

This project ports the Bitcraze **TDoA3 anchor** algorithm to an ESP32 (Arduino framework) using the **DW1000Ng** driver. The implementation keeps the same core logic—randomized transmissions, dynamic neighbor selection, clock correction, and distance extraction—while using a compact custom packet wrapper suitable for DW1000Ng.

## What’s included

- `src/tdoa3_anchor.hpp/.cpp`: Core TDoA3 logic (anchor context, clock correction, distance extraction).
- `src/main.cpp`: DW1000Ng radio setup, interrupt handlers, and scheduling loop.
- `platformio.ini`: Pulls the `DW1000Ng` library from PlatformIO.

## Wiring

Update the pin constants in `src/main.cpp` to match your wiring:

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

Key parameters are in `src/main.cpp`:

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
