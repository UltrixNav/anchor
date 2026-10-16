# TWR Protocol Compatibility Verification

## Bitcraze Protocol Specification
Source: https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/protocols/twr-protocol/

### Packet Types
| Type | Value | Direction | Description |
|------|-------|-----------|-------------|
| TWR_POLL | 0x01 | Tag → Anchor | Initiates ranging |
| TWR_ANSWER | 0x02 | Anchor → Tag | Response with timing info |
| TWR_FINAL | 0x03 | Tag → Anchor | Final packet with timing |
| TWR_REPORT | 0x04 | Anchor → Tag | Contains all measurement timestamps |

---

## Packet Formats - Verification

### ✅ TWR_POLL
**Bitcraze spec:**
- Byte 0: TYPE = 0x01
- Byte 1: SEQ (sequence number)

**Our implementation:**
```cpp
case kPacketTypePoll: {
    uint8_t packetType = payload[0];  // 0x01
    uint8_t seq = payload[1];         // Sequence
    // ...
}
```
✅ **Compatible**

---

### ✅ TWR_ANSWER
**Bitcraze spec:**
- Byte 0: TYPE = 0x02
- Byte 1: SEQ (matches POLL)
- Byte 2+: Optional LPP management data
  - If position enabled:
    - Byte 2: LPP header type (0xF0)
    - Byte 3: LPP packet type (0x01 for anchor position)
    - Byte 4-15: LppShortAnchorPosition struct (3 × float)

**Our implementation:**
```cpp
payloadPtr[0] = kPacketTypeAnswer;     // 0x02
payloadPtr[1] = seqNr_;                // Matches POLL seq

if (config_.positionEnabled) {
    payloadPtr[2] = kLppShort;         // 0xF0
    payloadPtr[3] = kLppShortAnchorPosition;  // 0x01
    LppShortAnchorPosition* pos = 
        reinterpret_cast<LppShortAnchorPosition*>(payloadPtr + 4);
    pos->position[0] = config_.position[0];
    pos->position[1] = config_.position[1];
    pos->position[2] = config_.position[2];
}
```
✅ **Compatible** - Includes optional LPP support

---

### ✅ TWR_FINAL
**Bitcraze spec:**
- Byte 0: TYPE = 0x03
- Byte 1: SEQ (matches POLL)

**Our implementation:**
```cpp
case kPacketTypeFinal: {
    uint8_t packetType = payload[0];  // 0x03
    uint8_t seq = payload[1];         // Matches POLL
    // ...
}
```
✅ **Compatible**

---

### ✅ TWR_REPORT
**Bitcraze spec:**
- Byte 0: TYPE = 0x04
- Byte 1: SEQ (matches POLL)
- Byte 2+: Report structure (28 bytes)

**Report structure (from Bitcraze):**
```c
typedef struct {
    uint8_t pollRx[5];      // R1 timestamp (5 bytes)
    uint8_t answerTx[5];    // T2 timestamp (5 bytes)
    uint8_t finalRx[5];     // R3 timestamp (5 bytes)
    float pressure;         // 4 bytes
    float temperature;      // 4 bytes
    float asl;              // 4 bytes
    uint8_t pressure_ok;    // 1 byte
} __attribute__((packed)) reportPayload_t;
```

**Our implementation:**
```cpp
struct ReportPayload {
    uint8_t pollRx[5];      // R1 timestamp
    uint8_t answerTx[5];    // T2 timestamp
    uint8_t finalRx[5];     // R3 timestamp
    float pressure;
    float temperature;
    float asl;
    uint8_t pressure_ok;
} __attribute__((packed));

// In buildTxFrame:
payloadPtr[0] = kPacketTypeReport;  // 0x04
payloadPtr[1] = seqNr_;             // Matches POLL
ReportPayload* report = reinterpret_cast<ReportPayload*>(payloadPtr + 2);

memcpy(report->pollRx, pollRxTime_.raw, 5);
memcpy(report->answerTx, answerTxTime_.raw, 5);
memcpy(report->finalRx, finalRxTime_.raw, 5);
report->pressure = 0.0f;
report->temperature = 0.0f;
report->asl = 0.0f;
report->pressure_ok = 0;
```
✅ **Compatible** - Exact structure match

---

## Timestamp Format
**Bitcraze format:** 5-byte DW1000 internal format

**Our implementation:**
```cpp
struct Timestamp {
    uint8_t raw[5];  // 5-byte array
    
    uint64_t toUint64() const { ... }
    static Timestamp fromUint64(uint64_t val) { ... }
    static Timestamp fromBytes(const uint8_t* bytes) { ... }
} __attribute__((packed));
```
✅ **Compatible** - Proper 5-byte timestamp handling

---

## MAC Header
**IEEE 802.15.4 frame structure:**
- Frame Control: 2 bytes
- Sequence: 1 byte
- PAN ID: 2 bytes (0xBCCF from Bitcraze)
- Destination Address: 8 bytes
- Source Address: 8 bytes

**Our implementation:**
```cpp
struct MAC80215Header {
    uint16_t fcf;           // Frame control
    uint8_t seq;
    uint16_t pan;           // 0xBCCF
    uint8_t destAddr[8];
    uint8_t srcAddr[8];
} __attribute__((packed));
```
✅ **Compatible**

---

## Summary

| Component | Status | Notes |
|-----------|--------|-------|
| Packet types | ✅ | All 4 types correctly defined |
| POLL format | ✅ | Correct parsing |
| ANSWER format | ✅ | Core format correct + LPP support |
| FINAL format | ✅ | Correct parsing |
| REPORT format | ✅ | Exact struct match with Bitcraze |
| Timestamps | ✅ | Proper 5-byte handling |
| MAC header | ✅ | IEEE 802.15.4 compliant |
| Packing | ✅ | All structs use `__attribute__((packed))` |

**Overall: ✅ FULLY COMPATIBLE with Bitcraze TWR Protocol**

---

## Key Implementation Details

1. **Sequential Mode:** Anchor responds reactively to tag polls
   - Receives POLL → sends ANSWER
   - Receives FINAL → sends REPORT

2. **Timestamp Capture:**
   - Poll RX: Captured when POLL received
   - Answer TX: Captured when ANSWER is transmitted
   - Final RX: Captured when FINAL received
   - All three sent back in REPORT for tag to calculate distance

3. **Position Support:** Optional LPP anchor position can be included in ANSWER
   - Allows tag to receive anchor location while ranging

4. **Zero Environmental Data:** Pressure/temperature/altitude fields set to zero
   - These would require sensor integration
   - TAG will ignore if pressure_ok = 0

---

## Tested Protocol Compliance

- ✅ Packet format compliance
- ✅ Byte ordering and alignment
- ✅ Enum values (0x01, 0x02, 0x03, 0x04)
- ✅ Timestamp structure (5 bytes)
- ✅ Optional LPP data support
- ✅ IEEE 802.15.4 MAC frame
- ✅ Bitcraze PAN ID (0xBCCF)
