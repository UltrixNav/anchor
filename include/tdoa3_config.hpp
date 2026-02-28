#pragma once

#include <cstddef>
#include <cstdint>

#include <DW1000NgConstants.hpp>

namespace tdoa3_config {

// --- DW1000 wiring (update to your hardware) ---
constexpr uint8_t kPinDwCs = 7;
constexpr uint8_t kPinDwIrq = 2;
constexpr uint8_t kPinDwRst = 3;

// --- Debug ---
constexpr bool kEnableDebug = true;
constexpr uint32_t kDebugIntervalMs = 1000;

// --- Built-in LED ---
// Keep main.cpp responsible for selecting LED_BUILTIN vs board-specific fallback
constexpr uint8_t kBuiltInLedFallbackPin = 21;  // XIAO ESP32C3 default user LED

// --- Network / identity ---
constexpr uint16_t kNetworkId = 0xBEEF;
constexpr uint8_t kAnchorId = 0x04;

// --- Anchor position (optional; used by solver/visualization if enabled) ---
constexpr bool kPositionEnabled = false;
constexpr float kPositionX = 0.0f;
constexpr float kPositionY = 0.0f;
constexpr float kPositionZ = 0.0f;

// --- Radio timing/calibration ---
constexpr uint16_t kAntennaDelayTicks = 16430;

// --- RX/TX behavior ---
constexpr bool kEnableFrameCheck = true;  // set true if all frames use valid CRC
constexpr bool kLowBitrateMode = false;   // match the tag's bitrate settings

constexpr uint16_t kRxFrameWaitTimeoutUs = 0;  // 0 disables timeout (max RX window)
constexpr uint16_t kSfdTimeoutSymbols = 4096 + 64 + 1;  // preamble + SFD + margin

constexpr bool kUseRxPollingFallback = true;
constexpr bool kLogEveryTx = false;
constexpr bool kUseRxInterrupts = false;
constexpr bool kLogEveryRx = false;

constexpr uint32_t kDelayedTxMarginUs = 2000;  // minimum lead time for delayed TX

// --- Buffers ---
constexpr size_t kMaxFrameSize = 127;

// --- DW1000-ng configuration structs ---
constexpr device_configuration_t kRadioConfig = {
    .extendedFrameLength = false,
    .receiverAutoReenable = true,
    .smartPower = true,
    .frameCheck = kEnableFrameCheck,
    .nlos = false,
    .sfd = SFDMode::STANDARD_SFD,
    .channel = Channel::CHANNEL_2,
    .dataRate = kLowBitrateMode ? DataRate::RATE_110KBPS : DataRate::RATE_6800KBPS,
    .pulseFreq = PulseFrequency::FREQ_64MHZ,
    .preambleLen = PreambleLength::LEN_128,
    .preaCode = PreambleCode::CODE_9,
};
constexpr bool forceTxPower = false;

constexpr interrupt_configuration_t kInterruptConfig = {
    .interruptOnSent = true,
    .interruptOnReceived = kUseRxInterrupts,
    .interruptOnReceiveFailed = false,
    .interruptOnReceiveTimeout = false,
    .interruptOnReceiveTimestampAvailable = false,
    .interruptOnAutomaticAcknowledgeTrigger = false,
};

}  // namespace tdoa3_config
