#pragma once

#include <cstdint>

namespace tdoa3_anchor_config {

// These values tune the *application-level* behavior of the TDoA anchor logic.
// They do NOT change DW1000 radio PHY settings (channel/bitrate/preamble/etc).
//
// If you change these, re-test stability (especially distance output and the
// "data good/bad" messages).

// --- Transmit rate control ---
// Effective per-anchor TX frequency is computed from systemTxFreq/(anchors+1)
// and then clamped to [kAnchorMinTxFreq .. kAnchorMaxTxFreq].
constexpr float kAnchorMaxTxFreqHz = 50.0f;  // upper limit per anchor (Hz)
constexpr float kAnchorMinTxFreqHz = 20.0f;  // lower limit per anchor (Hz)

// Adds small randomness to the TX schedule so anchors don't stay phase-locked.
// The delay is base +/- (kDelayRandomIntervalMs/2).
constexpr uint32_t kDelayRandomIntervalMs = 10;

// How often (ms) we rebuild the list of "seen" anchors and select who to include
// in outgoing packets.
constexpr uint32_t kAnchorListUpdateMs = 1000;

// If distance/clock-correction info wasn't refreshed for this long, drop it.
constexpr uint32_t kDistanceValidityMs = 2000;

// --- Clock correction filtering / acceptance ---
// The DW1000 clock is not perfect. We estimate relative clock drift between anchors
// and filter it to reduce noise.
//
// kMaxClockDeviation is the expected maximum fractional deviation (e.g. 100e-6 = 100 ppm).
constexpr double kMaxClockDeviation = 10e-4;

// Acceptable absolute clockCorrection range around 1.0.
// (Spec range a bit wider because we multiply by 2 in the original logic.)
constexpr double kClockCorrectionSpecMin = 1.0 - kMaxClockDeviation * 2;
constexpr double kClockCorrectionSpecMax = 1.0 + kMaxClockDeviation * 2;

// If the newly measured correction differs by less than this, treat it as noise
// and apply a low-pass filter update.
constexpr double kClockCorrectionAcceptedNoise = 0.3e-6;

// Low-pass filter factor for clock correction updates (0..1).
// Smaller = smoother but slower to adapt.
constexpr double kClockCorrectionFilter = 0.1;

// A small "confidence bucket". Good updates fill it, bad updates drain it.
// While bucket > 0 we still allow using the previous correction.
constexpr uint8_t kClockCorrectionBucketMax = 4;

// --- Behavior toggles ---
constexpr bool kAnchorDebug = true;            // enable Serial debug prints
constexpr bool kAutoCreateContextOnRx = true;  // create context when receiving from a new anchor

// Debug print rate limits.
constexpr uint32_t kAnchorDebugIntervalMs = 1000;
constexpr uint32_t kTxListDebugIntervalMs = 1000;

}  // namespace tdoa3_anchor_config
