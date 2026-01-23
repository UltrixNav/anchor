#pragma once

#include <cstddef>
#include <cstdint>

#include <DW1000NgConstants.hpp>

namespace tdoa3_config {

// --- DW1000 wiring (update to your hardware) ---
inline constexpr uint8_t kPinDwCs = 7;
inline constexpr uint8_t kPinDwIrq = 2;
inline constexpr uint8_t kPinDwRst = 3;

// --- Debug ---
inline constexpr bool kEnableDebug = true;
inline constexpr uint32_t kDebugIntervalMs = 1000;

// --- Built-in LED ---
// Keep main.cpp responsible for selecting LED_BUILTIN vs board-specific fallback
inline constexpr uint8_t kBuiltInLedFallbackPin = 8;  // XIAO ESP32C3 default user LED

// --- Network / identity ---
inline constexpr uint16_t kNetworkId = 0xBEEF;
inline constexpr uint8_t kAnchorId = 0x02;

// --- Anchor position (optional; used by solver/visualization if enabled) ---
inline constexpr bool kPositionEnabled = true;
inline constexpr float kPositionX = 0.0f;
inline constexpr float kPositionY = 0.0f;
inline constexpr float kPositionZ = 0.0f;

// --- Radio timing/calibration ---
inline constexpr uint16_t kAntennaDelayTicks = 16416;

// --- RX/TX behavior ---
inline constexpr bool kEnableFrameCheck = true;  // set true if all frames use valid CRC
inline constexpr bool kLowBitrateMode = false;   // match the tag's bitrate settings

inline constexpr uint16_t kRxFrameWaitTimeoutUs = 0;  // 0 disables timeout (max RX window)
inline constexpr uint16_t kSfdTimeoutSymbols = 4096 + 64 + 1;  // preamble + SFD + margin

inline constexpr bool kUseRxPollingFallback = true;
inline constexpr bool kLogEveryTx = false;
inline constexpr bool kUseRxInterrupts = false;
inline constexpr bool kLogEveryRx = false;

inline constexpr uint32_t kDelayedTxMarginUs = 2000;  // minimum lead time for delayed TX

// --- Buffers ---
inline constexpr size_t kMaxFrameSize = 127;

// --- DW1000-ng configuration structs ---
inline constexpr device_configuration_t kRadioConfig = {
    .extendedFrameLength = false,
    .receiverAutoReenable = true,
    .smartPower = true,
    .frameCheck = kEnableFrameCheck,
    .nlos = false,
    .sfd = SFDMode::STANDARD_SFD,
    .channel = Channel::CHANNEL_5,
    .dataRate = kLowBitrateMode ? DataRate::RATE_110KBPS : DataRate::RATE_850KBPS,
    .pulseFreq = PulseFrequency::FREQ_16MHZ,
    .preambleLen = PreambleLength::LEN_256,
    .preaCode = PreambleCode::CODE_3,
};

inline constexpr interrupt_configuration_t kInterruptConfig = {
    .interruptOnSent = true,
    .interruptOnReceived = kUseRxInterrupts,
    .interruptOnReceiveFailed = false,
    .interruptOnReceiveTimeout = false,
    .interruptOnReceiveTimestampAvailable = false,
    .interruptOnAutomaticAcknowledgeTrigger = false,
};

}  // namespace tdoa3_config
