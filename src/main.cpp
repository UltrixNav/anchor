#include <Arduino.h>
#include <SPI.h>

#include <DW1000Ng.hpp>
#include <DW1000NgConstants.hpp>
#include <DW1000NgUtils.hpp>

#include "tdoa3_anchor.hpp"
#include "tdoa3_config.hpp"

namespace {
#ifndef LED_BUILTIN
constexpr uint8_t kBuiltInLedPin = tdoa3_config::kBuiltInLedFallbackPin;  // XIAO ESP32C3 default user LED
#else
constexpr uint8_t kBuiltInLedPin = LED_BUILTIN;
#endif

const uint8_t kBlueLedPin = 8;  // Blue LED indicator

volatile bool gRxDone = false;
volatile bool gTxDone = false;
volatile bool gRxError = false;
volatile bool gLedActivityNeeded = false;

uint32_t gRxCount = 0;
uint32_t gTxCount = 0;
uint32_t gErrCount = 0;
uint32_t gLastDebugMs = 0;
uint64_t gLastSysTs = 0;

tdoa3::Anchor gAnchor;
uint8_t gRxBuffer[tdoa3_config::kMaxFrameSize] = {};
uint8_t gTxBuffer[tdoa3_config::kMaxFrameSize] = {};

uint64_t alignDelayedTime(uint64_t time) {
  return (time & ~((1ULL << 9) - 1)) + (1ULL << 9);
}

uint64_t delayUsToTicks(uint32_t delayUs) {
  return static_cast<uint64_t>(static_cast<double>(delayUs) / TIME_RES);
}

void handleSent() {
  gTxDone = true;
}

void handleReceived() {
  gRxDone = true;
}

void handleReceiveFailed() {
  gRxError = true;
}

void handleReceiveTimeout() {
  gRxError = true;
}

void startReceive() {
  DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
}

void logRxFrame(const uint8_t* data, uint16_t len, uint64_t rxTimestamp) {
  if (!tdoa3_config::kEnableDebug) {
    return;
  }
  if (!tdoa3_config::kLogEveryRx) {
    return;
  }

  if (len < sizeof(tdoa3::MAC80215Header)) {
    Serial.printf("RX len=%u ts=%llu\n", len, static_cast<unsigned long long>(rxTimestamp));
    return;
  }

  auto macHeader = reinterpret_cast<const tdoa3::MAC80215Header*>(data);
  Serial.printf("RX src=%u dst=%u len=%u ts=%llu\n",
                macHeader->srcAddr[0],
                macHeader->destAddr[0],
                len,
                static_cast<unsigned long long>(rxTimestamp));

  // Payload starts after MAC header (21 bytes)
  const uint8_t* payload = data + sizeof(tdoa3::MAC80215Header);
  size_t payloadLen = len - sizeof(tdoa3::MAC80215Header);
  
  if (payloadLen >= sizeof(tdoa3::RangeHeader)) {
    auto rangeHeader = reinterpret_cast<const tdoa3::RangeHeader*>(payload);
    Serial.printf("  range type=0x%02X seq=%u remoteCount=%u txTs=%lu\n",
                  rangeHeader->type,
                  rangeHeader->seq,
                  rangeHeader->remoteCount,
                  static_cast<unsigned long>(rangeHeader->txTimeStamp));
  }
}

void logTxFrame(size_t len, uint64_t txTimestamp) {
  if (!tdoa3_config::kEnableDebug) {
    return;
  }
  if (!tdoa3_config::kLogEveryTx) {
    return;
  }
  Serial.printf("TX len=%u seq=%u txTs=%llu\n",
                static_cast<unsigned>(len),
                gAnchor.seqNr(),
                static_cast<unsigned long long>(txTimestamp));
}

void logError(const char* label) {
  if (!tdoa3_config::kEnableDebug) {
    return;
  }
  Serial.printf("RADIO %s\n", label);
}

void updateBlueLed() {
  // LED status (inverted: LOW = ON, HIGH = OFF):
  // OFF if no valid anchors
  // BLINKING when RX/TX happening and valid anchors present
  uint8_t validAnchorCount = gAnchor.validAnchorCount();
  
  if (validAnchorCount == 0) {
    // No valid anchors - LED off
    digitalWrite(kBlueLedPin, HIGH);
  } else {
    // Valid anchors present - blink on RX/TX activity
    uint32_t nowMs = millis();
    if (gLedActivityNeeded) {
      // Blink for activity; pulse for ~50ms
      digitalWrite(kBlueLedPin, LOW);
      gLedActivityNeeded = false;
    } else {
      // Fade out over time
      digitalWrite(kBlueLedPin, HIGH);
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(kBuiltInLedPin, OUTPUT);
  digitalWrite(kBuiltInLedPin, HIGH);  // XIAO ESP32C3 LED is active-low
  
  pinMode(kBlueLedPin, OUTPUT);
  digitalWrite(kBlueLedPin, HIGH);  // Blue LED off initially (inverted)

  pinMode(tdoa3_config::kPinDwIrq, INPUT);

  SPI.begin(4, 5, 6, 7);

  DW1000Ng::initialize(tdoa3_config::kPinDwCs, tdoa3_config::kPinDwIrq, tdoa3_config::kPinDwRst);
  DW1000Ng::applyConfiguration(tdoa3_config::kRadioConfig);
  DW1000Ng::applyInterruptConfiguration(tdoa3_config::kInterruptConfig);

  DW1000Ng::setNetworkId(48879U);
  DW1000Ng::setDeviceAddress(tdoa3_config::kAnchorId);

  byte eui[8] = {0xDE, 0xCA, 0xF0, 0x00, 0x00, 0x00, 0x00, tdoa3_config::kAnchorId};
  DW1000Ng::setEUI(eui);
  DW1000Ng::setAntennaDelay(tdoa3_config::kAntennaDelayTicks);
  if (tdoa3_config::forceTxPower) {
    DW1000Ng::setTXPower(0x1F1F1F1Ful);
  }
  // DW1000Ng::setReceiveFrameWaitTimeoutPeriod(kRxFrameWaitTimeoutUs);
  // DW1000Ng::setSfdDetectionTimeout(kSfdTimeoutSymbols);

  if (tdoa3_config::kEnableDebug) {
    Serial.printf("RADIO cfg: ch=%u rate=%u preamble=%u code=%u frameCheck=%u\n",
                  static_cast<unsigned>(tdoa3_config::kRadioConfig.channel),
                  static_cast<unsigned>(tdoa3_config::kRadioConfig.dataRate),
                  static_cast<unsigned>(tdoa3_config::kRadioConfig.preambleLen),
                  static_cast<unsigned>(tdoa3_config::kRadioConfig.preaCode),
                  static_cast<unsigned>(tdoa3_config::kRadioConfig.frameCheck));
    Serial.printf("RADIO rxTimeoutUs=%u sfdTimeout=%u\n",
                  static_cast<unsigned>(tdoa3_config::kRxFrameWaitTimeoutUs),
                  static_cast<unsigned>(tdoa3_config::kSfdTimeoutSymbols));
  }

  DW1000Ng::attachSentHandler(handleSent);
  if (tdoa3_config::kUseRxInterrupts) {
    DW1000Ng::attachReceivedHandler(handleReceived);
    DW1000Ng::attachReceiveFailedHandler(handleReceiveFailed);
    DW1000Ng::attachReceiveTimeoutHandler(handleReceiveTimeout);
  }

  tdoa3::Config config;
  config.anchorId = tdoa3_config::kAnchorId;
  config.antennaDelayTicks = tdoa3_config::kAntennaDelayTicks;
  config.lowBitrate = tdoa3_config::kLowBitrateMode;
  config.positionEnabled = tdoa3_config::kPositionEnabled;
  config.position[0] = tdoa3_config::kPositionX;
  config.position[1] = tdoa3_config::kPositionY;
  config.position[2] = tdoa3_config::kPositionZ;
  gAnchor.begin(config);
  gAnchor.scheduleNextTx(millis());

  startReceive();
}

void loop() {
  const uint32_t nowMs = millis();
  gAnchor.tick(nowMs);

  if ((tdoa3_config::kUseRxPollingFallback || !tdoa3_config::kUseRxInterrupts) && DW1000Ng::isReceiveDone()) {
    gRxDone = true;
  }

  if (gRxDone) {
    gRxDone = false;
    DW1000Ng::clearReceiveStatus();
    gLedActivityNeeded = true;  // Trigger LED blink on RX

    uint16_t len = DW1000Ng::getReceivedDataLength();
    if (len > 0 && len <= tdoa3_config::kMaxFrameSize) {
      DW1000Ng::getReceivedData(gRxBuffer, len);
      uint64_t rxTimestamp = DW1000Ng::getReceiveTimestamp();
      logRxFrame(gRxBuffer, len, rxTimestamp);
      
      // Extract source ID from MAC header and pass to algorithm
      if (len >= sizeof(tdoa3::MAC80215Header)) {
        auto macHeader = reinterpret_cast<const tdoa3::MAC80215Header*>(gRxBuffer);
        uint8_t sourceId = macHeader->srcAddr[0];
        
        // Pass only the payload (after MAC header) to TDOA3 algorithm
        const uint8_t* payload = gRxBuffer + sizeof(tdoa3::MAC80215Header);
        size_t payloadLen = len - sizeof(tdoa3::MAC80215Header);
        gAnchor.onReceive(sourceId, payload, payloadLen, static_cast<uint32_t>(rxTimestamp));
      }
      gRxCount++;
    }
  }

  if (gRxError) {
    gRxError = false;
    DW1000Ng::clearReceiveFailedStatus();
    DW1000Ng::clearReceiveTimeoutStatus();
    logError("RX error/timeout");
    gErrCount++;
  }

  if (gTxDone) {
    gTxDone = false;
    DW1000Ng::clearTransmitStatus();
    startReceive();
  }

  if (gAnchor.shouldTransmit(nowMs)) {
    uint64_t nowUwb = DW1000Ng::getSystemTimestamp();
    uint64_t txTime = nowUwb + delayUsToTicks(tdoa3_config::kDelayedTxMarginUs);
    txTime = alignDelayedTime(txTime);

    gAnchor.markTransmit(static_cast<uint32_t>(txTime), 0);  // MAC seq will be incremented internally
    size_t txLen = gAnchor.buildTxFrame(gTxBuffer, sizeof(gTxBuffer));

    if (txLen > 0) {
      DW1000Ng::forceTRxOff();
      DW1000Ng::clearReceiveStatus();
      DW1000Ng::clearReceiveFailedStatus();
      DW1000Ng::clearReceiveTimeoutStatus();
      byte futureTimeBytes[LENGTH_TIMESTAMP] = {};
      DW1000NgUtils::writeValueToBytes(futureTimeBytes, txTime, LENGTH_TIMESTAMP);
      DW1000Ng::setDelayedTRX(futureTimeBytes);
      DW1000Ng::setTransmitData(gTxBuffer, static_cast<uint16_t>(txLen));
      DW1000Ng::startTransmit(TransmitMode::DELAYED);
      logTxFrame(txLen, txTime);
      gLedActivityNeeded = true;  // Trigger LED blink on TX
      gTxCount++;
    }

    gAnchor.scheduleNextTx(nowMs);
  }

  if (tdoa3_config::kEnableDebug && (nowMs - gLastDebugMs) >= tdoa3_config::kDebugIntervalMs) {
    uint64_t sysTs = DW1000Ng::getSystemTimestamp();
    Serial.printf("DBG ms=%lu sysTs=%llu rx=%lu tx=%lu err=%lu validAnchors=%u\n",
                  static_cast<unsigned long>(nowMs),
                  static_cast<unsigned long long>(sysTs),
                  static_cast<unsigned long>(gRxCount),
                  static_cast<unsigned long>(gTxCount),
                  static_cast<unsigned long>(gErrCount),
                  static_cast<unsigned>(gAnchor.validAnchorCount()));
    Serial.printf("DBG irq=%u rxDone=%u rxFail=%u\n",
                  static_cast<unsigned>(digitalRead(tdoa3_config::kPinDwIrq)),
                  static_cast<unsigned>(DW1000Ng::isReceiveDone()),
                  static_cast<unsigned>(DW1000Ng::isReceiveFailed()));
    if (sysTs == gLastSysTs) {
      Serial.println("WARN: system timestamp not changing (check SPI/clock)!");
    }
    gLastSysTs = sysTs;
    gLastDebugMs = nowMs;
  }

  updateBlueLed();
  delay(1);
}
