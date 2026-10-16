#pragma once

#include <Arduino.h>
#include <cstdint>
#include <cstring>

namespace twr {

// TWR packet types (from Bitcraze protocol)
constexpr uint8_t kPacketTypePoll = 0x01;    // Poll is initiated by the tag
constexpr uint8_t kPacketTypeAnswer = 0x02;   // Answer from anchor
constexpr uint8_t kPacketTypeFinal = 0x03;    // Final from tag
constexpr uint8_t kPacketTypeReport = 0x04;   // Report contains measurements from anchor

// LPP packet types (from Bitcraze LPP protocol)
constexpr uint8_t kLppShort = 0xF0;
constexpr uint8_t kLppShortAnchorPosition = 0x01;

// TX state machine
enum class TxState {
  kIdle,       // Not transmitting
  kAnswer,     // Should send ANSWER
  kReport      // Should send REPORT
};

// 5-byte timestamp (DW1000 internal format)
struct Timestamp {
  uint8_t raw[5];
  
  uint64_t toUint64() const {
    uint64_t val = 0;
    for (int i = 4; i >= 0; i--) {
      val = (val << 8) | raw[i];
    }
    return val;
  }
  
  static Timestamp fromUint64(uint64_t val) {
    Timestamp ts;
    for (int i = 0; i < 5; i++) {
      ts.raw[i] = (val >> (i * 8)) & 0xFF;
    }
    return ts;
  }
  
  static Timestamp fromBytes(const uint8_t* bytes) {
    Timestamp ts;
    memcpy(ts.raw, bytes, 5);
    return ts;
  }
} __attribute__((packed));

// LPP Short Anchor Position packet (from Bitcraze)
// Used in ANSWER to send anchor position to tag
struct LppShortAnchorPosition {
  float position[3];  // x, y, z in meters
} __attribute__((packed));

// Report payload structure - EXACT match to Bitcraze
// typedef struct {
//   uint8_t pollRx[5];      // R1 timestamp
//   uint8_t answerTx[5];    // T2 timestamp
//   uint8_t finalRx[5];     // R3 timestamp
//   float pressure;         
//   float temperature;      
//   float asl;              
//   uint8_t pressure_ok;    
// } __attribute__((packed)) reportPayload_t;

struct ReportPayload {
  uint8_t pollRx[5];      // R1 timestamp
  uint8_t answerTx[5];    // T2 timestamp
  uint8_t finalRx[5];     // R3 timestamp
  float pressure;
  float temperature;
  float asl;
  uint8_t pressure_ok;
} __attribute__((packed));

// MAC 802.15.4 header (21 bytes)
struct MAC80215Header {
  uint16_t fcf;
  uint8_t seq;
  uint16_t pan;
  uint8_t destAddr[8];
  uint8_t srcAddr[8];
} __attribute__((packed));

struct Config {
  uint8_t anchorId = 1;
  uint16_t antennaDelayTicks = 16436;
  bool positionEnabled = false;
  float position[3] = {0.0f, 0.0f, 0.0f};
};

class Anchor {
 public:
  void begin(const Config& config);

  // Called when a frame is received
  void onReceive(uint8_t sourceId, const uint8_t* payload, size_t length, uint32_t rxTimestamp);
  
  // Build the TX frame (ANSWER or REPORT)
  size_t buildTxFrame(uint8_t* buffer, size_t bufferSize);

  // Called before transmission to capture transmit timestamp
  void markTransmit(uint32_t txTimestamp, uint16_t macSeq);
  
  // Should we transmit now?
  bool shouldTransmit(uint32_t nowMs) const;
  
  // Get current TX state
  TxState getTxState() const { return txState_; }
  
  // Advance time state
  void tick(uint32_t nowMs);
  
  // Getters
  uint8_t anchorId() const { return config_.anchorId; }
  uint8_t seqNr() const { return seqNr_; }
  uint32_t lastTxTime() const { return lastTxTime_; }
  
  // Static helper
  static uint16_t makeFcf();

 private:
  Config config_;
  uint8_t seqNr_ = 0;
  uint32_t lastTxTime_ = 0;
  uint32_t nextTxMs_ = 0;
  
  // State for tracking active tag communication
  uint8_t currTag_ = 0xFF;
  Timestamp pollRxTime_;
  Timestamp answerTxTime_;
  Timestamp finalRxTime_;
  TxState txState_ = TxState::kIdle;
  TxState pendingTxState_ = TxState::kIdle;
  
  // Convenience methods
  void scheduleNextTransmit(uint32_t nowMs);
};

}  // namespace twr
