#include <Arduino.h>
#include <SPI.h>

#include <DW1000Ng.hpp>
#include <DW1000NgConstants.hpp>
#include <DW1000NgUtils.hpp>

#include "tdoa3_anchor.hpp"

namespace {
constexpr uint8_t kPinDwCs = 7;   // TODO: update to your wiring
constexpr uint8_t kPinDwIrq = 2;  // TODO: update to your wiring
constexpr uint8_t kPinDwRst = 3;  // TODO: update to your wiring

constexpr bool kEnableDebug = true;
constexpr uint32_t kDebugIntervalMs = 1000;

#ifndef LED_BUILTIN
constexpr uint8_t kBuiltInLedPin = 8;  // XIAO ESP32C3 default user LED
#else
constexpr uint8_t kBuiltInLedPin = LED_BUILTIN;
#endif

constexpr uint16_t kNetworkId = 0xBEEF;
constexpr uint8_t kAnchorId = 0x02;
constexpr uint16_t kAntennaDelayTicks = 16436;

constexpr bool kEnableFrameCheck = false;  // set true if all frames use valid CRC
constexpr bool kLowBitrateMode = false;    // match the tag's bitrate settings

constexpr uint16_t kRxFrameWaitTimeoutUs = 0;  // 0 disables timeout (max RX window)
constexpr uint16_t kSfdTimeoutSymbols = 4096 + 64 + 1;  // preamble + SFD + margin

constexpr bool kUseRxPollingFallback = true;

constexpr uint64_t kPreambleLengthTicks = 128ULL * 1017ULL;  // approx 1.017us * 128
constexpr uint64_t kGuardLengthTicks = 64ULL * 128ULL;       // approx 1us
constexpr uint64_t kExtraLengthTicks = 300ULL * 128ULL;      // approx 300us

constexpr size_t kMaxFrameSize = 127;

// Simple TX test mode to validate PHY with BasicReceiver
constexpr bool kTestSimpleTx = true;
constexpr uint32_t kSimpleTxPeriodMs = 500;

device_configuration_t kRadioConfig = {
    .extendedFrameLength = false,
    .receiverAutoReenable = true,
    .smartPower = true,
  .frameCheck = kEnableFrameCheck,
    .nlos = false,
    .sfd = SFDMode::STANDARD_SFD,
    .channel = Channel::CHANNEL_5,
  .dataRate = DataRate::RATE_850KBPS,//kLowBitrateMode ? DataRate::RATE_110KBPS : DataRate::RATE_6800KBPS,
    .pulseFreq = PulseFrequency::FREQ_16MHZ,
    .preambleLen = PreambleLength::LEN_256,
    .preaCode = PreambleCode::CODE_3,
};

interrupt_configuration_t kInterruptConfig = {
    .interruptOnSent = true,
    .interruptOnReceived = true,
    .interruptOnReceiveFailed = true,
    .interruptOnReceiveTimeout = true,
    .interruptOnReceiveTimestampAvailable = false,
    .interruptOnAutomaticAcknowledgeTrigger = false,
};

volatile bool gRxDone = false;
volatile bool gTxDone = false;
volatile bool gRxError = false;

uint32_t gRxCount = 0;
uint32_t gTxCount = 0;
uint32_t gErrCount = 0;
uint32_t gLastDebugMs = 0;
uint64_t gLastSysTs = 0;
uint32_t gLastSimpleTxMs = 0;
uint8_t gSimpleTxSeq = 0;

tdoa3::Anchor gAnchor;
uint8_t gRxBuffer[kMaxFrameSize] = {};
uint8_t gTxBuffer[kMaxFrameSize] = {};

uint64_t alignDelayedTime(uint64_t time) {
  return (time & ~((1ULL << 9) - 1)) + (1ULL << 9);
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
  if (!kEnableDebug) {
    return;
  }

  if (len < sizeof(tdoa3::AppHeader)) {
    Serial.printf("RX len=%u ts=%llu\n", len, static_cast<unsigned long long>(rxTimestamp));
    return;
  }

  auto appHeader = reinterpret_cast<const tdoa3::AppHeader*>(data);
  Serial.printf("RX src=%u dst=%u type=0x%02X len=%u ts=%llu\n",
                appHeader->sourceId,
                appHeader->destId,
                appHeader->type,
                len,
                static_cast<unsigned long long>(rxTimestamp));

  if (len >= sizeof(tdoa3::AppHeader) + sizeof(tdoa3::RangeHeader)) {
    auto rangeHeader = reinterpret_cast<const tdoa3::RangeHeader*>(
        data + sizeof(tdoa3::AppHeader));
    Serial.printf("  range seq=%u remoteCount=%u txTs=%lu\n",
                  rangeHeader->seq,
                  rangeHeader->remoteCount,
                  static_cast<unsigned long>(rangeHeader->txTimeStamp));
  }
}

void logTxFrame(size_t len, uint64_t txTimestamp) {
  if (!kEnableDebug) {
    return;
  }
  Serial.printf("TX len=%u seq=%u txTs=%llu\n",
                static_cast<unsigned>(len),
                gAnchor.seqNr(),
                static_cast<unsigned long long>(txTimestamp));
}

void logError(const char* label) {
  if (!kEnableDebug) {
    return;
  }
  Serial.printf("RADIO %s\n", label);
}

size_t buildSimpleTxFrame(uint8_t* buffer, size_t bufferSize) {
  const char prefix[] = "PING";
  const size_t payloadLen = sizeof(prefix) - 1 + 1;  // prefix + seq
  if (bufferSize < payloadLen) {
    return 0;
  }
  memcpy(buffer, prefix, sizeof(prefix) - 1);
  buffer[sizeof(prefix) - 1] = gSimpleTxSeq++;
  return payloadLen;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(kBuiltInLedPin, OUTPUT);
  digitalWrite(kBuiltInLedPin, HIGH);  // XIAO ESP32C3 LED is active-low

  pinMode(kPinDwIrq, INPUT);

  SPI.begin(4, 5, 6, 7);

  DW1000Ng::initialize(kPinDwCs, kPinDwIrq, kPinDwRst);
  DW1000Ng::applyConfiguration(kRadioConfig);
  DW1000Ng::applyInterruptConfiguration(kInterruptConfig);

  DW1000Ng::setNetworkId(kNetworkId);
  DW1000Ng::setDeviceAddress(kAnchorId);

  byte eui[8] = {0xDE, 0xCA, 0xF0, 0x00, 0x00, 0x00, 0x00, kAnchorId};
  DW1000Ng::setEUI(eui);
  DW1000Ng::setAntennaDelay(kAntennaDelayTicks);
  DW1000Ng::disableFrameFiltering();
  DW1000Ng::setReceiveFrameWaitTimeoutPeriod(kRxFrameWaitTimeoutUs);
  DW1000Ng::setSfdDetectionTimeout(kSfdTimeoutSymbols);

  if (kEnableDebug) {
    Serial.printf("RADIO cfg: ch=%u rate=%u preamble=%u code=%u frameCheck=%u\n",
                  static_cast<unsigned>(kRadioConfig.channel),
                  static_cast<unsigned>(kRadioConfig.dataRate),
                  static_cast<unsigned>(kRadioConfig.preambleLen),
                  static_cast<unsigned>(kRadioConfig.preaCode),
                  static_cast<unsigned>(kRadioConfig.frameCheck));
    Serial.printf("RADIO rxTimeoutUs=%u sfdTimeout=%u\n",
                  static_cast<unsigned>(kRxFrameWaitTimeoutUs),
                  static_cast<unsigned>(kSfdTimeoutSymbols));
    Serial.printf("RADIO irqPin=%u pollingFallback=%u\n",
                  static_cast<unsigned>(kPinDwIrq),
                  static_cast<unsigned>(kUseRxPollingFallback));
  }

  DW1000Ng::attachSentHandler(handleSent);
  DW1000Ng::attachReceivedHandler(handleReceived);
  DW1000Ng::attachReceiveFailedHandler(handleReceiveFailed);
  DW1000Ng::attachReceiveTimeoutHandler(handleReceiveTimeout);

  tdoa3::Config config;
  config.anchorId = kAnchorId;
  config.antennaDelayTicks = kAntennaDelayTicks;
  config.lowBitrate = false;
  gAnchor.begin(config);
  gAnchor.scheduleNextTx(millis());

  startReceive();
}

void loop() {
  const uint32_t nowMs = millis();
  gAnchor.tick(nowMs);

  if (kTestSimpleTx && (nowMs - gLastSimpleTxMs) >= kSimpleTxPeriodMs) {
    size_t txLen = buildSimpleTxFrame(gTxBuffer, sizeof(gTxBuffer));
    if (txLen > 0) {
      DW1000Ng::setTransmitData(gTxBuffer, static_cast<uint16_t>(txLen));
      DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);
      logTxFrame(txLen, DW1000Ng::getSystemTimestamp());
      gTxCount++;
    }
    gLastSimpleTxMs = nowMs;
  }

  if (kUseRxPollingFallback && DW1000Ng::isReceiveDone()) {
    gRxDone = true;
  }

  if (gRxDone) {
    gRxDone = false;
    DW1000Ng::clearReceiveStatus();

    uint16_t len = DW1000Ng::getReceivedDataLength();
    if (len > 0 && len <= kMaxFrameSize) {
      DW1000Ng::getReceivedData(gRxBuffer, len);
      uint64_t rxTimestamp = DW1000Ng::getReceiveTimestamp();
      logRxFrame(gRxBuffer, len, rxTimestamp);
      gAnchor.onReceive(gRxBuffer, len, static_cast<uint32_t>(rxTimestamp));
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

  if (!kTestSimpleTx && gAnchor.shouldTransmit(nowMs)) {
    uint64_t nowUwb = DW1000Ng::getSystemTimestamp();
    uint64_t txTime = nowUwb + kGuardLengthTicks + kPreambleLengthTicks + kExtraLengthTicks;
    txTime = alignDelayedTime(txTime);

    gAnchor.markTransmit(static_cast<uint32_t>(txTime));
    size_t txLen = gAnchor.buildTxFrame(gTxBuffer, sizeof(gTxBuffer));

    if (txLen > 0) {
      byte futureTimeBytes[LENGTH_TIMESTAMP] = {};
      DW1000NgUtils::writeValueToBytes(futureTimeBytes, txTime, LENGTH_TIMESTAMP);
      DW1000Ng::setDelayedTRX(futureTimeBytes);
      DW1000Ng::setTransmitData(gTxBuffer, static_cast<uint16_t>(txLen));
      DW1000Ng::startTransmit(TransmitMode::DELAYED);
      logTxFrame(txLen, txTime);
      gTxCount++;
    }

    gAnchor.scheduleNextTx(nowMs);
  } else if (!kTestSimpleTx) {
    startReceive();
  }

  if (kEnableDebug && (nowMs - gLastDebugMs) >= kDebugIntervalMs) {
    uint64_t sysTs = DW1000Ng::getSystemTimestamp();
    Serial.printf("DBG ms=%lu sysTs=%llu rx=%lu tx=%lu err=%lu\n",
                  static_cast<unsigned long>(nowMs),
                  static_cast<unsigned long long>(sysTs),
                  static_cast<unsigned long>(gRxCount),
                  static_cast<unsigned long>(gTxCount),
                  static_cast<unsigned long>(gErrCount));
    Serial.printf("DBG irq=%u rxDone=%u rxFail=%u\n",
                  static_cast<unsigned>(digitalRead(kPinDwIrq)),
                  static_cast<unsigned>(DW1000Ng::isReceiveDone()),
                  static_cast<unsigned>(DW1000Ng::isReceiveFailed()));
    if (sysTs == gLastSysTs) {
      Serial.println("WARN: system timestamp not changing (check SPI/clock)!");
    }
    gLastSysTs = sysTs;
    gLastDebugMs = nowMs;
  }
}
