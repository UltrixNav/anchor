#include "tdoa3_anchor.hpp"

#include "tdoa3_anchor_config.hpp"

#include <DW1000NgConstants.hpp>

namespace tdoa3 {

namespace {
constexpr float kAnchorMaxTxFreq = tdoa3_anchor_config::kAnchorMaxTxFreqHz;
constexpr float kAnchorMinTxFreq = tdoa3_anchor_config::kAnchorMinTxFreqHz;
constexpr uint32_t kDelayRandomIntervalMs = tdoa3_anchor_config::kDelayRandomIntervalMs;
constexpr uint32_t kAnchorListUpdateMs = tdoa3_anchor_config::kAnchorListUpdateMs;
constexpr uint32_t kDistanceValidityMs = tdoa3_anchor_config::kDistanceValidityMs;
constexpr double kClockCorrectionSpecMin = tdoa3_anchor_config::kClockCorrectionSpecMin;
constexpr double kClockCorrectionSpecMax = tdoa3_anchor_config::kClockCorrectionSpecMax;
constexpr double kClockCorrectionAcceptedNoise = tdoa3_anchor_config::kClockCorrectionAcceptedNoise;
constexpr double kClockCorrectionFilter = tdoa3_anchor_config::kClockCorrectionFilter;
constexpr uint8_t kClockCorrectionBucketMax = tdoa3_anchor_config::kClockCorrectionBucketMax;
constexpr bool kAnchorDebug = tdoa3_anchor_config::kAnchorDebug;
constexpr bool kAutoCreateContextOnRx = tdoa3_anchor_config::kAutoCreateContextOnRx;
constexpr uint32_t kAnchorDebugIntervalMs = tdoa3_anchor_config::kAnchorDebugIntervalMs;
constexpr uint32_t kTxListDebugIntervalMs = tdoa3_anchor_config::kTxListDebugIntervalMs;
}  // namespace

void Anchor::begin(const Config& config) {
  anchorId_ = config.anchorId;
  antennaDelayTicks_ = config.antennaDelayTicks;
  systemTxFreq_ = config.lowBitrate ? config.systemTxFreqLow : config.systemTxFreqHigh;
  positionEnabled_ = config.positionEnabled;
  memcpy(position_, config.position, sizeof(position_));

  seqNr_ = 0;
  txTime_ = 0;
  nextTxAtMs_ = 0;
  averageTxDelayMs_ = static_cast<uint32_t>(1000.0f / kAnchorMinTxFreq);
  remoteTxIdCount_ = 0;
  nextAnchorListUpdateMs_ = 0;
  macSeqNr_ = 0;

  memset(anchorCtxLookup_, kIdWithoutContext, sizeof(anchorCtxLookup_));
  for (auto& ctx : anchorCtx_) {
    ctx = AnchorContext{};
    ctx.isUsed = false;
  }
  clearAnchorRxCount();

  randomSeed(static_cast<uint32_t>(anchorId_) * 1103515245UL + 12345UL);
}

AnchorContext* Anchor::getContext(uint8_t anchorId) {
  uint8_t slot = anchorCtxLookup_[anchorId];
  if (slot == kIdWithoutContext) {
    return nullptr;
  }
  return &anchorCtx_[slot];
}

const AnchorContext* Anchor::getContext(uint8_t anchorId) const {
  uint8_t slot = anchorCtxLookup_[anchorId];
  if (slot == kIdWithoutContext) {
    return nullptr;
  }
  return &anchorCtx_[slot];
}

void Anchor::clearAnchorRxCount() {
  memset(anchorRxCount_, 0, sizeof(anchorRxCount_));
}

void Anchor::removeAnchorContextsNotInList(const uint8_t* id, uint8_t count) {
  for (auto& anchorCtx : anchorCtx_) {
    if (!anchorCtx.isUsed) {
      continue;
    }
    bool found = false;
    for (uint8_t j = 0; j < count; j++) {
      if (id[j] == anchorCtx.id) {
        found = true;
        break;
      }
    }
    if (!found) {
      anchorCtxLookup_[anchorCtx.id] = kIdWithoutContext;
      anchorCtx.isUsed = false;
    }
  }
}

void Anchor::createAnchorContext(uint8_t id) {
  if (anchorCtxLookup_[id] != kIdWithoutContext) {
    return;
  }
  for (uint8_t i = 0; i < kAnchorStorageCount; i++) {
    AnchorContext& anchorCtx = anchorCtx_[i];
    if (!anchorCtx.isUsed) {
      anchorCtxLookup_[id] = i;
      anchorCtx = AnchorContext{};
      anchorCtx.id = id;
      anchorCtx.isUsed = true;
      if (kAnchorDebug) {
        Serial.printf("TDOA3 ctx created id=%u\n", id);
      }
      break;
    }
  }
}

void Anchor::createAnchorContextsInList(const uint8_t* id, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    createAnchorContext(id[i]);
  }
}

void Anchor::purgeStaleData(uint32_t nowMs) {
  const uint32_t acceptedCreationTime = nowMs - kDistanceValidityMs;
  for (auto& anchorCtx : anchorCtx_) {
    if (!anchorCtx.isUsed) {
      continue;
    }
    if (anchorCtx.distanceUpdateMs < acceptedCreationTime) {
      anchorCtx.distance = 0;
      anchorCtx.clockCorrection = 0.0;
      anchorCtx.clockCorrectionBucket = 0;
    }
  }
}

void Anchor::updateAnchorLists(uint32_t nowMs) {
  static uint32_t lastDebugMs = 0;
  uint8_t availableId[kIdCount] = {};
  bool availableUsed[kIdCount] = {};
  uint8_t ctxts[kAnchorStorageCount] = {};

  uint8_t availableCount = 0;
  for (size_t i = 0; i < kIdCount; i++) {
    if (anchorRxCount_[i] != 0) {
      availableId[availableCount++] = static_cast<uint8_t>(i);
    }
  }

  if (availableCount == 0) {
    remoteTxIdCount_ = 0;
    averageTxDelayMs_ = static_cast<uint32_t>(1000.0f / kAnchorMinTxFreq);
    clearAnchorRxCount();
    purgeStaleData(nowMs);
    return;
  }

  if (kAnchorDebug && (nowMs - lastDebugMs) >= kAnchorDebugIntervalMs) {
    Serial.printf("TDOA3 update: available=%u\n", availableCount);
    Serial.print("TDOA3 ids: ");
    for (uint8_t i = 0; i < availableCount; i++) {
      Serial.printf("%u ", availableId[i]);
    }
    Serial.println();
    lastDebugMs = nowMs;
  }

  uint8_t remoteTxIdIndex = 0;
  uint8_t contextIndex = 0;
  for (uint8_t i = 0; i < kAnchorStorageCount; i++) {
    uint8_t start = static_cast<uint8_t>(random(0, availableCount));
    for (uint16_t j = 0; j < availableCount; j++) {
      uint8_t index = static_cast<uint8_t>((start + j) % availableCount);
      if (!availableUsed[index]) {
        uint8_t id = availableId[index];
        if (remoteTxIdIndex < kRemoteTxMaxCount) {
          remoteTxId_[remoteTxIdIndex++] = id;
        }
        if (contextIndex < kAnchorStorageCount) {
          ctxts[contextIndex++] = id;
        }
        availableUsed[index] = true;
        break;
      }
    }
  }

  removeAnchorContextsNotInList(ctxts, contextIndex);
  createAnchorContextsInList(ctxts, contextIndex);
  remoteTxIdCount_ = remoteTxIdIndex;
  clearAnchorRxCount();

  if (kAnchorDebug && (nowMs - lastDebugMs) < kAnchorDebugIntervalMs) {
    Serial.printf("TDOA3 selected: ctx=%u txIds=%u\n", contextIndex, remoteTxIdCount_);
  }

  float freq = systemTxFreq_ / (availableCount + 1);
  if (freq > kAnchorMaxTxFreq) {
    freq = kAnchorMaxTxFreq;
  }
  if (freq < kAnchorMinTxFreq) {
    freq = kAnchorMinTxFreq;
  }
  averageTxDelayMs_ = static_cast<uint32_t>(1000.0f / freq);

  purgeStaleData(nowMs);
}

void Anchor::fillClockCorrectionBucket(AnchorContext* anchorCtx) {
  if (anchorCtx->clockCorrectionBucket < kClockCorrectionBucketMax) {
    anchorCtx->clockCorrectionBucket++;
  }
}

bool Anchor::emptyClockCorrectionBucket(AnchorContext* anchorCtx) {
  if (anchorCtx->clockCorrectionBucket > 0) {
    anchorCtx->clockCorrectionBucket--;
    return false;
  }
  return true;
}

bool Anchor::updateClockCorrection(AnchorContext* anchorCtx, double clockCorrection) {
  const double diff = clockCorrection - anchorCtx->clockCorrection;
  bool accepted = false;
  if (-kClockCorrectionAcceptedNoise < diff && diff < kClockCorrectionAcceptedNoise) {
    anchorCtx->clockCorrection = anchorCtx->clockCorrection * (1.0 - kClockCorrectionFilter) +
                                 clockCorrection * kClockCorrectionFilter;
    fillClockCorrectionBucket(anchorCtx);
    accepted = true;
  } else {
    if (emptyClockCorrectionBucket(anchorCtx)) {
      if (kClockCorrectionSpecMin < clockCorrection && clockCorrection < kClockCorrectionSpecMax) {
        anchorCtx->clockCorrection = clockCorrection;
      }
    }
  }
  return accepted;
}

double Anchor::calculateClockCorrection(AnchorContext* anchorCtx,
                                        uint32_t remoteTx,
                                        uint32_t rx) const {
  uint32_t tickCountRemote = remoteTx - anchorCtx->txTimeStamp;
  uint32_t tickCountLocal = rx - anchorCtx->rxTimeStamp;
  if (tickCountRemote == 0) {
    return 0.0;
  }
  return static_cast<double>(tickCountLocal) / static_cast<double>(tickCountRemote);
}

uint16_t Anchor::calculateDistance(AnchorContext* anchorCtx,
                                   uint8_t remoteRxSeqNr,
                                   uint32_t remoteTx,
                                   uint32_t remoteRx,
                                   uint32_t rx) const {
  if (remoteRxSeqNr != seqNr_ || anchorCtx->clockCorrection <= 0.0) {
    return 0;
  }
  uint32_t localTime = rx - txTime_;
  uint32_t remoteTime = static_cast<uint32_t>(
      static_cast<double>(remoteTx - remoteRx) * anchorCtx->clockCorrection);
  if (localTime < remoteTime) {
    return 0;
  }
  uint32_t distance = (localTime - remoteTime) / 2;
  return static_cast<uint16_t>(distance & 0xFFFFu);
}

bool Anchor::extractFromPacket(const RangeHeader* rangeHeader,
                               const uint8_t* payloadStart,
                               size_t payloadLength,
                               uint32_t* remoteRx,
                               uint8_t* remoteRxSeqNr) const {
  const uint8_t* ptr = payloadStart;
  size_t remaining = payloadLength;
  for (uint8_t i = 0; i < rangeHeader->remoteCount && remaining >= sizeof(RemoteAnchorDataShort);
       i++) {
    auto anchorData = reinterpret_cast<const RemoteAnchorDataShort*>(ptr);
    const uint8_t id = anchorData->id;
    if (id == anchorId_) {
      *remoteRxSeqNr = anchorData->seq & ~kSeqHasDistanceMask;
      *remoteRx = anchorData->rxTimeStamp;
      return true;
    }
    const bool hasDistance = (anchorData->seq & kSeqHasDistanceMask) != 0;
    size_t step = hasDistance ? sizeof(RemoteAnchorDataFull) : sizeof(RemoteAnchorDataShort);
    if (remaining < step) {
      break;
    }
    ptr += step;
    remaining -= step;
  }
  return false;
}

void Anchor::onReceive(uint8_t sourceId, const uint8_t* macPayload, size_t length, uint32_t rxTimestamp) {
  // macPayload points to the start of the TDOA3 payload (after MAC header, which is 21 bytes)
  // First byte should be the RangeHeader.type (0x30)
  
  if (length < sizeof(RangeHeader)) {
    return;
  }

  auto rangeHeader = reinterpret_cast<const RangeHeader*>(macPayload);
  if (rangeHeader->type != kPacketType) {
    return;
  }

  // Use the source ID extracted from MAC header
  uint8_t remoteAnchorId = sourceId;
  anchorRxCount_[remoteAnchorId]++;

  AnchorContext* anchorCtx = getContext(remoteAnchorId);
  if (!anchorCtx) {
    if (kAutoCreateContextOnRx) {
      createAnchorContext(remoteAnchorId);
      anchorCtx = getContext(remoteAnchorId);
    }
    if (!anchorCtx) {
      if (kAnchorDebug) {
        Serial.printf("TDOA3 rx without ctx id=%u\n", remoteAnchorId);
      }
      return;
    }
  }

  uint32_t remoteTx = rangeHeader->txTimeStamp;
  uint8_t remoteTxSeqNr = rangeHeader->seq;

  double clockCorrection = calculateClockCorrection(anchorCtx, remoteTx, rxTimestamp);
  bool prevGood = anchorCtx->isDataGoodForTransmission;
  if (updateClockCorrection(anchorCtx, clockCorrection)) {
    anchorCtx->isDataGoodForTransmission = true;
    if (kAnchorDebug && !prevGood) {
      Serial.printf("TDOA3 data good id=%u cc=%.9f bucket=%u\n",
                    remoteAnchorId,
                    anchorCtx->clockCorrection,
                    anchorCtx->clockCorrectionBucket);
    }

    uint32_t remoteRx = 0;
    uint8_t remoteRxSeqNr = 0;
    const uint8_t* payloadStart = macPayload + sizeof(RangeHeader);
    size_t payloadLength = length - sizeof(RangeHeader);
    if (extractFromPacket(rangeHeader, payloadStart, payloadLength, &remoteRx, &remoteRxSeqNr)) {
      uint16_t distance = calculateDistance(anchorCtx, remoteRxSeqNr, remoteTx, remoteRx, rxTimestamp);
      if (distance > antennaDelayTicks_) {
        anchorCtx->distance = distance;
        anchorCtx->distanceUpdateMs = millis();
        if (kAnchorDebug) {
          float meters = static_cast<float>(distance - antennaDelayTicks_) * DISTANCE_OF_RADIO;
          Serial.printf("TDOA3 dist id=%u ticks=%u meters=%.3f\n",
                        remoteAnchorId,
                        distance,
                        meters);
        }
      }
    }
  } else {
    anchorCtx->isDataGoodForTransmission = false;
    if (kAnchorDebug && prevGood) {
      Serial.printf("TDOA3 data bad id=%u cc=%.9f bucket=%u\n",
                    remoteAnchorId,
                    anchorCtx->clockCorrection,
                    anchorCtx->clockCorrectionBucket);
    }
  }

  anchorCtx->rxTimeStamp = rxTimestamp;
  anchorCtx->seqNr = remoteTxSeqNr;
  anchorCtx->txTimeStamp = remoteTx;
}

size_t Anchor::buildTxFrame(uint8_t* buffer, size_t bufferSize) {
  static uint32_t lastTxListDebugMs = 0;
  
  // Layout: MAC header (21) + RangeHeader (5) + RemoteData (6-8 each) + LPP optional (14+)
  const size_t macHeaderSize = sizeof(MAC80215Header);
  const size_t rangeHeaderSize = sizeof(RangeHeader);
  const size_t headerSize = macHeaderSize + rangeHeaderSize;
  
  if (bufferSize < headerSize) {
    return 0;
  }

  // Construct MAC header
  MAC80215Header* macHeader = constructMACFrame(buffer, bufferSize, macSeqNr_, 0);
  if (!macHeader) {
    return 0;
  }

  // Construct range header
  auto rangeHeader = reinterpret_cast<RangeHeader*>(buffer + macHeaderSize);
  rangeHeader->type = kPacketType;
  rangeHeader->seq = seqNr_;
  rangeHeader->txTimeStamp = txTime_;
  rangeHeader->remoteCount = 0;

  uint8_t* ptr = buffer + headerSize;
  size_t remaining = bufferSize - headerSize;

  uint8_t skippedNoData = 0;
  uint8_t includedCount = 0;
  for (uint8_t i = 0; i < remoteTxIdCount_; i++) {
    uint8_t id = remoteTxId_[i];
    const AnchorContext* anchorCtx = getContext(id);
    if (!anchorCtx || !anchorCtx->isDataGoodForTransmission) {
      skippedNoData++;
      continue;
    }

    bool includeDistance = anchorCtx->distance > 0;
    size_t needed = includeDistance ? sizeof(RemoteAnchorDataFull) : sizeof(RemoteAnchorDataShort);
    if (remaining < needed) {
      break;
    }

    if (includeDistance) {
      auto anchorData = reinterpret_cast<RemoteAnchorDataFull*>(ptr);
      anchorData->id = id;
      anchorData->seq = anchorCtx->seqNr | kSeqHasDistanceMask;
      anchorData->rxTimeStamp = anchorCtx->rxTimeStamp;
      anchorData->distance = anchorCtx->distance;
      ptr += sizeof(RemoteAnchorDataFull);
      remaining -= sizeof(RemoteAnchorDataFull);
    } else {
      auto anchorData = reinterpret_cast<RemoteAnchorDataShort*>(ptr);
      anchorData->id = id;
      anchorData->seq = anchorCtx->seqNr;
      anchorData->rxTimeStamp = anchorCtx->rxTimeStamp;
      ptr += sizeof(RemoteAnchorDataShort);
      remaining -= sizeof(RemoteAnchorDataShort);
    }

    rangeHeader->remoteCount++;
    includedCount++;
  }

  // Add LPP position block if enabled
  // Format: [0xF0] [0x01] [x:float] [y:float] [z:float] = 14 bytes
  if (positionEnabled_) {
    const size_t lppNeeded = 1 + 1 + 3 * sizeof(float);  // 14 bytes
    if (remaining >= lppNeeded) {
      ptr[0] = kLppShortType;      // 0xF0
      ptr[1] = kLppAnchorPosition; // 0x01
      memcpy(ptr + 2, position_, 3 * sizeof(float));
      ptr += lppNeeded;
      remaining -= lppNeeded;
    } else if (kAnchorDebug) {
      Serial.println("TDOA3 pos skipped: no space");
    }
  }

  uint32_t nowMs = millis();
  if (kAnchorDebug && (nowMs - lastTxListDebugMs) >= kTxListDebugIntervalMs) {
    Serial.printf("TDOA3 txlist: total=%u included=%u skipped=%u\n",
                  remoteTxIdCount_,
                  includedCount,
                  skippedNoData);
    lastTxListDebugMs = nowMs;
  }

  return static_cast<size_t>(ptr - buffer);
}

void Anchor::markTransmit(uint32_t txTimestamp, uint16_t macSeq) {
  txTime_ = txTimestamp;
  seqNr_ = static_cast<uint8_t>((seqNr_ + 1) & 0x7F);
  macSeqNr_ = macSeq;
}

uint16_t Anchor::makeFcf() {
  // Frame Control Field for MAC 802.15.4
  // bits 0-2:   type = 1 (data frame)
  // bit 3:       security = 0
  // bit 4:       frame pending = 0
  // bit 5:       ack request = 0
  // bit 6:       intra-PAN = 1
  // bits 7-9:    reserved = 0
  // bits 10-11:  dest addr mode = 3 (64-bit)
  // bits 12-13:  version = 1
  // bits 14-15:  src addr mode = 3 (64-bit)
  
  uint16_t fcf = 0;
  fcf |= 0x01;           // type = data frame
  fcf |= (1 << 6);       // intra-PAN
  fcf |= (3 << 10);      // dest addr mode = 64-bit
  fcf |= (1 << 12);      // version = 1
  fcf |= (3 << 14);      // src addr mode = 64-bit
  return fcf;
}

MAC80215Header* Anchor::constructMACFrame(uint8_t* buffer, size_t bufferSize, 
                                           uint16_t macSeq, uint16_t payloadSize) {
  if (bufferSize < sizeof(MAC80215Header)) {
    return nullptr;
  }
  
  auto macHeader = reinterpret_cast<MAC80215Header*>(buffer);
  macHeader->fcf = makeFcf();
  macHeader->seq = macSeq & 0xFF;
  macHeader->pan = kNetworkId;
  
  // Destination address: 0x00...0x00 | 0xFF (broadcast)
  memset(macHeader->destAddr, 0, 8);
  macHeader->destAddr[0] = kBroadcastId;
  
  // Source address: 0x00...0x00 | anchorId
  memset(macHeader->srcAddr, 0, 8);
  macHeader->srcAddr[0] = anchorId_;
  
  return macHeader;
}

uint32_t Anchor::nextTxDelayMs() {
  uint32_t base = averageTxDelayMs_;
  uint32_t offset = static_cast<uint32_t>(random(0, kDelayRandomIntervalMs));
  int32_t delay = static_cast<int32_t>(base) + static_cast<int32_t>(offset) -
                  static_cast<int32_t>(kDelayRandomIntervalMs / 2);
  if (delay < 1) {
    delay = 1;
  }
  return static_cast<uint32_t>(delay);
}

void Anchor::scheduleNextTx(uint32_t nowMs) {
  nextTxAtMs_ = nowMs + nextTxDelayMs();
}

bool Anchor::shouldTransmit(uint32_t nowMs) const {
  return nowMs >= nextTxAtMs_;
}

void Anchor::tick(uint32_t nowMs) {
  if (nowMs >= nextAnchorListUpdateMs_) {
    updateAnchorLists(nowMs);
    nextAnchorListUpdateMs_ = nowMs + kAnchorListUpdateMs;
  }
}

uint8_t Anchor::validAnchorCount() const {
  uint8_t count = 0;
  for (uint8_t i = 0; i < remoteTxIdCount_; i++) {
    const AnchorContext* anchorCtx = getContext(remoteTxId_[i]);
    if (anchorCtx && anchorCtx->isDataGoodForTransmission) {
      count++;
    }
  }
  return count;
}

}  // namespace tdoa3
