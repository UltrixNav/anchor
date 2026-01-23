#include "tdoa3_anchor.hpp"

#include <DW1000NgConstants.hpp>

namespace tdoa3 {

namespace {
constexpr float kAnchorMaxTxFreq = 50.0f;
constexpr float kAnchorMinTxFreq = 20.0f;
constexpr uint32_t kDelayRandomIntervalMs = 10;
constexpr uint32_t kAnchorListUpdateMs = 1000;
constexpr uint32_t kDistanceValidityMs = 2000;
constexpr double kMaxClockDeviation = 100e-6;
constexpr double kClockCorrectionSpecMin = 1.0 - kMaxClockDeviation * 2;
constexpr double kClockCorrectionSpecMax = 1.0 + kMaxClockDeviation * 2;
constexpr double kClockCorrectionAcceptedNoise = 0.3e-6;
constexpr double kClockCorrectionFilter = 0.1;
constexpr uint8_t kClockCorrectionBucketMax = 8;
constexpr bool kAnchorDebug = true;
constexpr bool kAutoCreateContextOnRx = true;
constexpr uint32_t kAnchorDebugIntervalMs = 1000;
constexpr uint32_t kTxListDebugIntervalMs = 1000;
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

void Anchor::onReceive(const uint8_t* data, size_t length, uint32_t rxTimestamp) {
  if (length < sizeof(AppHeader) + sizeof(RangeHeader)) {
    return;
  }
  auto appHeader = reinterpret_cast<const AppHeader*>(data);
  if (appHeader->type != kPacketType) {
    return;
  }
  if (appHeader->destId != kBroadcastId && appHeader->destId != anchorId_) {
    return;
  }
  uint8_t remoteAnchorId = appHeader->sourceId;
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

  auto rangeHeader = reinterpret_cast<const RangeHeader*>(data + sizeof(AppHeader));
  if (rangeHeader->type != kPacketType) {
    return;
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
    const uint8_t* payloadStart = data + sizeof(AppHeader) + sizeof(RangeHeader);
    size_t payloadLength = length - sizeof(AppHeader) - sizeof(RangeHeader);
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
    anchorCtx->isDataGoodForTransmission = (anchorCtx->clockCorrectionBucket > 0);
    if (kAnchorDebug && prevGood) {
      Serial.printf("TDOA3 data bad id=%u cc=%.9f bucket=%u\n",
                    remoteAnchorId,
                    anchorCtx->clockCorrection,
                    anchorCtx->clockCorrectionBucket);
    }
    if (kAnchorDebug && !prevGood) {
      Serial.printf("TDOA3 cc rejected id=%u cc=%.9f bucket=%u\n",
                    remoteAnchorId,
                    clockCorrection,
                    anchorCtx->clockCorrectionBucket);
    }
  }

  anchorCtx->rxTimeStamp = rxTimestamp;
  anchorCtx->seqNr = remoteTxSeqNr;
  anchorCtx->txTimeStamp = remoteTx;
}

size_t Anchor::buildTxFrame(uint8_t* buffer, size_t bufferSize) const {
  static uint32_t lastTxListDebugMs = 0;
  const size_t headerSize = sizeof(AppHeader) + sizeof(RangeHeader);
  if (bufferSize < headerSize) {
    return 0;
  }

  auto appHeader = reinterpret_cast<AppHeader*>(buffer);
  appHeader->sourceId = anchorId_;
  appHeader->destId = kBroadcastId;
  appHeader->type = kPacketType;

  auto rangeHeader = reinterpret_cast<RangeHeader*>(buffer + sizeof(AppHeader));
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

  if (positionEnabled_) {
    const size_t needed = 1 + sizeof(position_);
    if (remaining >= needed) {
      ptr[0] = kPositionBlockType;
      memcpy(ptr + 1, position_, sizeof(position_));
      ptr += needed;
      remaining -= needed;
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

void Anchor::markTransmit(uint32_t txTimestamp) {
  txTime_ = txTimestamp;
  seqNr_ = static_cast<uint8_t>((seqNr_ + 1) & 0x7F);
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

}  // namespace tdoa3
