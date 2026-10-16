#include <Arduino.h>
#include <SPI.h>

#include <DW1000Ng.hpp>
#include <DW1000NgConstants.hpp>
#include <DW1000NgUtils.hpp>

#include "build_config.hpp"

#if ANCHOR_MODE_TDOA3
  #include "tdoa3_anchor.hpp"
#else
  #include "twr_anchor.hpp"
#endif

namespace {
#ifndef LED_BUILTIN
constexpr uint8_t kBuiltInLedPin = build_config::kBuiltInLedFallbackPin;  // XIAO ESP32C3 default user LED
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

#if ANCHOR_MODE_TDOA3
  tdoa3::Anchor gAnchor;
#else
  twr::Anchor gAnchor;
  bool gTxInFlight = false;
#endif

uint8_t gRxBuffer[build_config::kMaxFrameSize] = {};
uint8_t gTxBuffer[build_config::kMaxFrameSize] = {};

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
  if (!build_config::kEnableDebug) {
    return;
  }
  if (!build_config::kLogEveryRx) {
    return;
  }

  if (len < sizeof(twr::MAC80215Header)) {
    Serial.printf("RX len=%u ts=%llu\n", len, static_cast<unsigned long long>(rxTimestamp));
    return;
  }

#if ANCHOR_MODE_TDOA3
  auto macHeader = reinterpret_cast<const tdoa3::MAC80215Header*>(data);
#else
  auto macHeader = reinterpret_cast<const twr::MAC80215Header*>(data);
#endif

  Serial.printf("RX src=%u dst=%u len=%u ts=%llu\n",
                macHeader->srcAddr[0],
                macHeader->destAddr[0],
                len,
                static_cast<unsigned long long>(rxTimestamp));

#if ANCHOR_MODE_TDOA3
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
#else
  // For TWR, just log the packet type
  if (len > sizeof(twr::MAC80215Header)) {
    uint8_t packetType = data[sizeof(twr::MAC80215Header)];
    Serial.printf("  TWR type=0x%02X\n", packetType);
  }
#endif
}

void logTxFrame(size_t len, uint64_t txTimestamp) {
  if (!build_config::kEnableDebug) {
    return;
  }
  if (!build_config::kLogEveryTx) {
    return;
  }
  Serial.printf("TX len=%u seq=%u txTs=%llu\n",
                static_cast<unsigned>(len),
                gAnchor.seqNr(),
                static_cast<unsigned long long>(txTimestamp));
}

void logError(const char* label) {
  if (!build_config::kEnableDebug) {
    return;
  }
  Serial.printf("RADIO %s\n", label);
}

void updateBlueLed() {
  // LED status (inverted: LOW = ON, HIGH = OFF):
  // OFF if no valid anchors (TDOA3 mode) / always on for TWR
  // BLINKING when RX/TX happening
#if ANCHOR_MODE_TDOA3
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
#else
  // TWR mode: always indicator on activity
  uint32_t nowMs = millis();
  if (gLedActivityNeeded) {
    digitalWrite(kBlueLedPin, LOW);
    gLedActivityNeeded = false;
  } else {
    digitalWrite(kBlueLedPin, HIGH);
  }
#endif
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(kBuiltInLedPin, OUTPUT);
  digitalWrite(kBuiltInLedPin, HIGH);  // XIAO ESP32C3 LED is active-low
  
  pinMode(kBlueLedPin, OUTPUT);
  digitalWrite(kBlueLedPin, HIGH);  // Blue LED off initially (inverted)

  pinMode(build_config::kPinDwIrq, INPUT);

  SPI.begin(4, 5, 6, 7);

  DW1000Ng::initialize(build_config::kPinDwCs, build_config::kPinDwIrq, build_config::kPinDwRst);
  DW1000Ng::applyConfiguration(build_config::kRadioConfig);
  DW1000Ng::applyInterruptConfiguration(build_config::kInterruptConfig);

  DW1000Ng::setNetworkId(48879U);
  DW1000Ng::setDeviceAddress(build_config::kAnchorId);

  byte eui[8] = {0xDE, 0xCA, 0xF0, 0x00, 0x00, 0x00, 0x00, build_config::kAnchorId};
  DW1000Ng::setEUI(eui);
  DW1000Ng::setAntennaDelay(build_config::kAntennaDelayTicks);
  if (build_config::forceTxPower) {
    DW1000Ng::setTXPower(0x1F1F1F1Ful);
  }

  if (build_config::kEnableDebug) {
    Serial.printf("RADIO cfg: ch=%u rate=%u preamble=%u code=%u frameCheck=%u\n",
                  static_cast<unsigned>(build_config::kRadioConfig.channel),
                  static_cast<unsigned>(build_config::kRadioConfig.dataRate),
                  static_cast<unsigned>(build_config::kRadioConfig.preambleLen),
                  static_cast<unsigned>(build_config::kRadioConfig.preaCode),
                  static_cast<unsigned>(build_config::kRadioConfig.frameCheck));
    Serial.printf("RADIO rxTimeoutUs=%u sfdTimeout=%u mode=%s\n",
                  static_cast<unsigned>(build_config::kRxFrameWaitTimeoutUs),
                  static_cast<unsigned>(build_config::kSfdTimeoutSymbols),
#if ANCHOR_MODE_TDOA3
                  "TDOA3");
#else
                  "TWR");
#endif
  }

  DW1000Ng::attachSentHandler(handleSent);
  if (build_config::kUseRxInterrupts) {
    DW1000Ng::attachReceivedHandler(handleReceived);
    DW1000Ng::attachReceiveFailedHandler(handleReceiveFailed);
    DW1000Ng::attachReceiveTimeoutHandler(handleReceiveTimeout);
  }

#if ANCHOR_MODE_TDOA3
  tdoa3::Config config;
  config.anchorId = build_config::kAnchorId;
  config.antennaDelayTicks = build_config::kAntennaDelayTicks;
  config.lowBitrate = build_config::kLowBitrateMode;
  config.positionEnabled = build_config::kPositionEnabled;
  config.position[0] = build_config::kPositionX;
  config.position[1] = build_config::kPositionY;
  config.position[2] = build_config::kPositionZ;
  gAnchor.begin(config);
  gAnchor.scheduleNextTx(millis());
#else
  twr::Config config;
  config.anchorId = build_config::kAnchorId;
  config.antennaDelayTicks = build_config::kAntennaDelayTicks;
  config.positionEnabled = build_config::kPositionEnabled;
  config.position[0] = build_config::kPositionX;
  config.position[1] = build_config::kPositionY;
  config.position[2] = build_config::kPositionZ;
  gAnchor.begin(config);
#endif

  startReceive();
}

void loop() {
  const uint32_t nowMs = millis();
  gAnchor.tick(nowMs);

  if ((build_config::kUseRxPollingFallback || !build_config::kUseRxInterrupts) && DW1000Ng::isReceiveDone()) {
    gRxDone = true;
  }

  if (gRxDone) {
    gRxDone = false;
    DW1000Ng::clearReceiveStatus();
    gLedActivityNeeded = true;  // Trigger LED blink on RX

    uint16_t len = DW1000Ng::getReceivedDataLength();
    if (len > 0 && len <= build_config::kMaxFrameSize) {
      DW1000Ng::getReceivedData(gRxBuffer, len);
      uint64_t rxTimestamp = DW1000Ng::getReceiveTimestamp();
      logRxFrame(gRxBuffer, len, rxTimestamp);
      
      // Extract source ID from MAC header and pass to algorithm
#if ANCHOR_MODE_TDOA3
      if (len >= sizeof(tdoa3::MAC80215Header)) {
        auto macHeader = reinterpret_cast<const tdoa3::MAC80215Header*>(gRxBuffer);
        uint8_t sourceId = macHeader->srcAddr[0];
        
        // Pass only the payload (after MAC header) to TDOA3 algorithm
        const uint8_t* payload = gRxBuffer + sizeof(tdoa3::MAC80215Header);
        size_t payloadLen = len - sizeof(tdoa3::MAC80215Header);
        gAnchor.onReceive(sourceId, payload, payloadLen, static_cast<uint32_t>(rxTimestamp));
      }
#else
      if (len >= sizeof(twr::MAC80215Header)) {
        auto macHeader = reinterpret_cast<const twr::MAC80215Header*>(gRxBuffer);
        uint8_t sourceId = macHeader->srcAddr[0];
        uint8_t destId = macHeader->destAddr[0];
        
        if (build_config::kEnableDebug && build_config::kLogEveryRx) {
          Serial.printf("  destId=0x%02X myId=0x%02X\n", destId, build_config::kAnchorId);
        }
        
        // Only process packets addressed to this anchor
        if (destId == build_config::kAnchorId) {
          // Pass only the payload (after MAC header) to TWR algorithm
          const uint8_t* payload = gRxBuffer + sizeof(twr::MAC80215Header);
          size_t payloadLen = len - sizeof(twr::MAC80215Header);
          gAnchor.onReceive(sourceId, payload, payloadLen, static_cast<uint32_t>(rxTimestamp));
        }
      }
#endif
      gRxCount++;
    }
    
    // Restart receive after processing packet
    startReceive();
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

#if !ANCHOR_MODE_TDOA3
    if (gTxInFlight) {
      uint64_t txTs = DW1000Ng::getTransmitTimestamp();
      gAnchor.markTransmit(static_cast<uint32_t>(txTs), 0);
      gTxInFlight = false;
    }
#endif

    startReceive();
  }

  if (gAnchor.shouldTransmit(nowMs)) {
#if ANCHOR_MODE_TDOA3
    uint64_t nowUwb = DW1000Ng::getSystemTimestamp();
    uint64_t txTime = nowUwb + delayUsToTicks(build_config::kDelayedTxMarginUs);
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
#else
    // TWR mode: immediate transmission (not delayed)
    size_t txLen = gAnchor.buildTxFrame(gTxBuffer, sizeof(gTxBuffer));

    if (txLen > 0) {
      uint64_t nowUwb = DW1000Ng::getSystemTimestamp();
      DW1000Ng::forceTRxOff();
      DW1000Ng::clearReceiveStatus();
      DW1000Ng::clearReceiveFailedStatus();
      DW1000Ng::clearReceiveTimeoutStatus();
      DW1000Ng::setTransmitData(gTxBuffer, static_cast<uint16_t>(txLen));
      DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);
      gTxInFlight = true;
      logTxFrame(txLen, nowUwb);
      gLedActivityNeeded = true;
      gTxCount++;
    }
#endif
  }

  if (build_config::kEnableDebug && (nowMs - gLastDebugMs) >= build_config::kDebugIntervalMs) {
    uint64_t sysTs = DW1000Ng::getSystemTimestamp();
    Serial.printf("DBG ms=%lu sysTs=%llu rx=%lu tx=%lu err=%lu\n",
                  static_cast<unsigned long>(nowMs),
                  static_cast<unsigned long long>(sysTs),
                  static_cast<unsigned long>(gRxCount),
                  static_cast<unsigned long>(gTxCount),
                  static_cast<unsigned long>(gErrCount));
    Serial.printf("DBG irq=%u rxDone=%u rxFail=%u\n",
                  static_cast<unsigned>(digitalRead(build_config::kPinDwIrq)),
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
