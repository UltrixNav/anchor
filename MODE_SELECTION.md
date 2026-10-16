# Anchor Mode Selection

## Overview

The UltrixNav anchor firmware supports two ranging modes:

1. **TDOA3 (Time Difference of Arrival)** - Default mode
2. **TWR (Two-Way Ranging)** - Alternative mode

## TDOA3 Mode (Default)

### Description
- Anchor transmits beacon packets with timestamp information
- Tag receives from multiple anchors and calculates position based on time differences
- Solver/Visualization receives and processes data
- More complex but typically supports multiple tags simultaneously

### Enable
In `platformio.ini`:
```ini
build_flags = 
	-D ANCHOR_MODE_TDOA3=1
```
Or simply comment out all mode flags - TDOA3 is the default.

### Behavior
- Anchor transmits proactively at configured intervals
- Uses delayed transmission for precise timing
- Each anchor broadcasts to all tags

---

## TWR Mode

### Description
- Based on Bitcraze's TWR anchor implementation
- Reactive mode: anchor responds to tag requests
- Tag sequence: POLL → (anchor ANSWER) → FINAL → (anchor REPORT)
- More suitable for point-to-point ranging scenarios

### Enable
In `platformio.ini`:
```ini
build_flags = 
	-D ANCHOR_MODE_TWR=1
```

### Behavior
- Anchor waits for POLL packets from tags
- Responds immediately with ANSWER (contains answer TX timestamp)
- After receiving FINAL, sends REPORT with all three timestamps:
  - Poll reception time
  - Answer transmission time
  - Final reception time

---

## Switching Modes

### Method 1: Edit platformio.ini
Uncomment the desired mode's build flags. Only one mode should be active at a time.

### Method 2: Command Line
```bash
# Build for TDOA3
pio run -e seeed_xiao_esp32c3 -D ANCHOR_MODE_TDOA3=1

# Build for TWR  
pio run -e seeed_xiao_esp32c3 -D ANCHOR_MODE_TWR=1
```

---

## Implementation Details

### Files
- **build_config.hpp**: Contains radio configuration and mode selection logic
- **main.cpp**: Conditional compilation based on selected mode
- **tdoa3_anchor.hpp/cpp**: TDOA3 algorithm implementation
- **twr_anchor.hpp/cpp**: TWR algorithm implementation

### How Mode Switching Works
- The `build_config.hpp` defines `ANCHOR_MODE_TDOA3` macro
- `main.cpp` uses `#if ANCHOR_MODE_TDOA3` preprocessor directives
- At compile time, either TDOA3 or TWR code is included
- This results in minimal binary bloat and clear separation of concerns

### Configuration Parameters
Both modes share common parameters defined in `build_config.hpp`:
- Pin assignments (DW, IRQ, RST)
- Radio configuration (channel, bitrate, preamble)
- Antenna delay calibration
- Debug settings
- Anchor position (if enabled)

---

## Debugging

### TDOA3 Mode Debug Output
```
RADIO cfg: ch=2 rate=6 preamble=3 code=9 frameCheck=1
DBG ms=1000 sysTs=0x1234... rx=10 tx=5 err=0 validAnchors=2
```

### TWR Mode Debug Output
```
RADIO cfg: ch=2 rate=6 preamble=3 code=9 frameCheck=1 mode=TWR
RX src=1 dst=4 len=10 ts=...
TX len=4 seq=1 txTs=...
```

---

## Notes

- Only one mode can be compiled in at a time
- Both modes use the same radio configuration
- The LED indicator behavior differs slightly:
  - TDOA3: LED off if no valid anchors, blinks on activity if anchors present
  - TWR: LED blinks on RX/TX activity
- The modes are not interoperable at runtime (tag must match anchor mode)
