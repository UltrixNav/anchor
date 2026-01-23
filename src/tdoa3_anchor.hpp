#pragma once

#include <Arduino.h>

namespace tdoa3 {

constexpr uint8_t kPacketType = 0x30;
constexpr uint8_t kBroadcastId = 0xFF;
constexpr uint8_t kAnchorStorageCount = 16;
constexpr uint8_t kRemoteTxMaxCount = 8;
constexpr size_t kIdCount = 256;
constexpr uint8_t kIdWithoutContext = 0xFF;
constexpr uint8_t kSeqHasDistanceMask = 0x80;
constexpr uint8_t kPositionBlockType = 0xA1;

struct AppHeader {
  uint8_t sourceId;
  uint8_t destId;
  uint8_t type;
} __attribute__((packed));

struct RangeHeader {
  uint8_t type;
  uint8_t seq;
  uint32_t txTimeStamp;
  uint8_t remoteCount;
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

  void onReceive(const uint8_t* data, size_t length, uint32_t rxTimestamp);
  size_t buildTxFrame(uint8_t* buffer, size_t bufferSize) const;

  void markTransmit(uint32_t txTimestamp);
  uint32_t nextTxDelayMs();
  void scheduleNextTx(uint32_t nowMs);
  bool shouldTransmit(uint32_t nowMs) const;
  void tick(uint32_t nowMs);

  uint8_t anchorId() const { return anchorId_; }
  uint8_t seqNr() const { return seqNr_; }
  uint32_t lastTxTime() const { return txTime_; }

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
};

}  // namespace tdoa3
