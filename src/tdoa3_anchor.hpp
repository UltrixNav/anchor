#pragma once

#include <Arduino.h>

namespace tdoa3 {

// Packet type constants
constexpr uint8_t kPacketType = 0x30;          // TDOA3 packet
constexpr uint8_t kLppShortType = 0xF0;        // LPP short packet
constexpr uint8_t kLppAnchorPosition = 0x01;   // LPP anchor position ID

// Network constants
constexpr uint8_t kBroadcastId = 0xFF;
constexpr uint16_t kNetworkId = 0xBCCF;  // Bitcraze default PAN

// Capability constants
constexpr uint8_t kAnchorStorageCount = 16;
constexpr uint8_t kRemoteTxMaxCount = 8;
constexpr size_t kIdCount = 256;
constexpr uint8_t kIdWithoutContext = 0xFF;
constexpr uint8_t kSeqHasDistanceMask = 0x80;

// MAC 802.15.4 header (21 bytes total)
struct MAC80215Header {
  // Frame control: 2 bytes
  // Format: type(3) | security(1) | pending(1) | ack(1) | ipan(1) | rsvd(3) | destMode(2) | version(2) | srcMode(2)
  uint16_t fcf;
  uint8_t seq;
  uint16_t pan;           // 0xBCCF (Bitcraze)
  uint8_t destAddr[8];    // 0x00...0x00 + destId
  uint8_t srcAddr[8];     // 0x00...0x00 + srcId (set at runtime)
} __attribute__((packed));

// TDOA3 range packet header (5 bytes)
struct RangeHeader {
  uint8_t type;           // 0x30 (PACKET_TYPE_TDOA3)
  uint8_t seq;            // 0-127 (bit 7 unused in header)
  uint32_t txTimeStamp;   // Transmission time
  uint8_t remoteCount;    // Number of remote anchor data blocks
} __attribute__((packed));

struct RemoteAnchorDataFull {
  uint8_t id;
  uint8_t seq;
  uint32_t rxTimeStamp;
  uint16_t distance;
} __attribute__((packed));

struct RemoteAnchorDataShort {
  uint8_t id;
  uint8_t seq;
  uint32_t rxTimeStamp;
} __attribute__((packed));

struct AnchorContext {
  uint8_t id = 0;
  bool isUsed = false;
  uint8_t seqNr = 0;
  uint32_t rxTimeStamp = 0;
  uint32_t txTimeStamp = 0;
  uint16_t distance = 0;
  uint32_t distanceUpdateMs = 0;
  bool isDataGoodForTransmission = false;
  double clockCorrection = 0.0;
  uint8_t clockCorrectionBucket = 0;
};

struct Config {
  uint8_t anchorId = 1;
  bool lowBitrate = false;
  uint16_t antennaDelayTicks = 16436;
  float systemTxFreqHigh = 400.0f;
  float systemTxFreqLow = 200.0f;
  bool positionEnabled = false;
  float position[3] = {0.0f, 0.0f, 0.0f};
};

class Anchor {
 public:
  void begin(const Config& config);

  void onReceive(uint8_t sourceId, const uint8_t* macPayload, size_t length, uint32_t rxTimestamp);
  size_t buildTxFrame(uint8_t* buffer, size_t bufferSize);

  void markTransmit(uint32_t txTimestamp, uint16_t macSeq);
  uint32_t nextTxDelayMs();
  void scheduleNextTx(uint32_t nowMs);
  bool shouldTransmit(uint32_t nowMs) const;
  void tick(uint32_t nowMs);

  uint8_t anchorId() const { return anchorId_; }
  uint8_t seqNr() const { return seqNr_; }
  uint32_t lastTxTime() const { return txTime_; }
  uint8_t validAnchorCount() const;
  
  // Static helper for MAC FCF setup
  static uint16_t makeFcf();

 private:
  AnchorContext* getContext(uint8_t anchorId);
  const AnchorContext* getContext(uint8_t anchorId) const;
  void clearAnchorRxCount();
  void removeAnchorContextsNotInList(const uint8_t* id, uint8_t count);
  void createAnchorContext(uint8_t id);
  void createAnchorContextsInList(const uint8_t* id, uint8_t count);
  void purgeStaleData(uint32_t nowMs);
  void updateAnchorLists(uint32_t nowMs);

  double calculateClockCorrection(AnchorContext* anchorCtx, uint32_t remoteTx, uint32_t rx) const;
  bool updateClockCorrection(AnchorContext* anchorCtx, double clockCorrection);
  void fillClockCorrectionBucket(AnchorContext* anchorCtx);
  bool emptyClockCorrectionBucket(AnchorContext* anchorCtx);
  uint16_t calculateDistance(AnchorContext* anchorCtx,
                             uint8_t remoteRxSeqNr,
                             uint32_t remoteTx,
                             uint32_t remoteRx,
                             uint32_t rx) const;
  bool extractFromPacket(const RangeHeader* rangeHeader,
                         const uint8_t* payloadStart,
                         size_t payloadLength,
                         uint32_t* remoteRx,
                         uint8_t* remoteRxSeqNr) const;
  
  // MAC-related helpers
  MAC80215Header* constructMACFrame(uint8_t* buffer, size_t bufferSize, 
                                     uint16_t macSeq, uint16_t payloadSize);

  float systemTxFreq_ = 200.0f;
  uint8_t anchorId_ = 1;
  uint8_t seqNr_ = 0;
  uint32_t txTime_ = 0;
  uint32_t nextTxAtMs_ = 0;
  uint32_t averageTxDelayMs_ = 50;
  uint32_t nextAnchorListUpdateMs_ = 0;
  uint16_t antennaDelayTicks_ = 16436;

  uint8_t remoteTxId_[kRemoteTxMaxCount] = {};
  uint8_t remoteTxIdCount_ = 0;
  uint8_t anchorCtxLookup_[kIdCount] = {};
  AnchorContext anchorCtx_[kAnchorStorageCount] = {};
  uint8_t anchorRxCount_[kIdCount] = {};
  bool positionEnabled_ = false;
  float position_[3] = {0.0f, 0.0f, 0.0f};
  uint16_t macSeqNr_ = 0;  // MAC layer sequence number
};

}  // namespace tdoa3
