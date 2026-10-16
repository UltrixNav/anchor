#include "twr_anchor.hpp"

namespace twr {

uint16_t Anchor::makeFcf() {
  // Frame Control Field for IEEE 802.15.4
  // Bits 0-2: Frame Type = 1 (Data)
  // Bit 3: Security Enabled = 0
  // Bit 4: Frame Pending = 0
  // Bit 5: Ack Request = 0
  // Bit 6: Intra-PAN = 1
  // Bits 7-9: Reserved = 0
  // Bits 10-11: Destination Addressing Mode = 2 (short)
  // Bits 12-13: Frame Version = 0
  // Bits 14-15: Source Addressing Mode = 2 (short)
  
  uint16_t fcf = 0;
  fcf |= 0x01;          // Data frame
  fcf |= (1 << 6);      // Intra-PAN
  fcf |= (2 << 10);     // Dest addr short
  fcf |= (2 << 14);     // Src addr short
  return fcf;
}

void Anchor::begin(const Config& config) {
  config_ = config;
  seqNr_ = 0;
  lastTxTime_ = 0;
  nextTxMs_ = 0;
  currTag_ = 0xFF;
  txState_ = TxState::kIdle;
  pendingTxState_ = TxState::kIdle;
}

void Anchor::tick(uint32_t nowMs) {
  // Nothing special to do in tick for TWR
  // The main loop handles RX and TX based on shouldTransmit()
}

bool Anchor::shouldTransmit(uint32_t nowMs) const {
  return nowMs >= nextTxMs_;
}

void Anchor::scheduleNextTransmit(uint32_t nowMs) {
  // For TWR, we don't transmit proactively - we react to incoming packets
  // Set next transmit far in future so it only happens on explicit trigger
  nextTxMs_ = nowMs + 60000;  // 60 seconds
}

void Anchor::markTransmit(uint32_t txTimestamp, uint16_t macSeq) {
  lastTxTime_ = txTimestamp;
  
  // If we're sending ANSWER, capture the TX timestamp
  if (pendingTxState_ == TxState::kAnswer) {
    answerTxTime_ = Timestamp::fromUint64(static_cast<uint64_t>(txTimestamp));
  }

  pendingTxState_ = TxState::kIdle;
}

void Anchor::onReceive(uint8_t sourceId, const uint8_t* payload, size_t length, uint32_t rxTimestamp) {
  if (length < 2) {
    return;
  }

  uint8_t packetType = payload[0];
  uint8_t seq = payload[1];

  switch (packetType) {
    case kPacketTypePoll: {
      // Received POLL from tag
      // Format (per Bitcraze protocol):
      // Byte 0: TYPE (0x01)
      // Byte 1: SEQ
      
      currTag_ = sourceId;
      pollRxTime_ = Timestamp::fromUint64(static_cast<uint64_t>(rxTimestamp));
      seqNr_ = seq;
      
      // Schedule sending ANSWER immediately
      txState_ = TxState::kAnswer;
      nextTxMs_ = 0;
      
      break;
    }

    case kPacketTypeFinal: {
      // Received FINAL from tag
      // Format (per Bitcraze protocol):
      // Byte 0: TYPE (0x03)
      // Byte 1: SEQ
      
      if (sourceId == currTag_) {
        finalRxTime_ = Timestamp::fromUint64(static_cast<uint64_t>(rxTimestamp));
        
        // Schedule sending REPORT
        txState_ = TxState::kReport;
        nextTxMs_ = 0;
      }
      break;
    }

    default:
      break;
  }
}

size_t Anchor::buildTxFrame(uint8_t* buffer, size_t bufferSize) {
  if (txState_ == TxState::kIdle) {
    return 0;
  }

  pendingTxState_ = txState_;

  size_t frameSize = 0;
  
  if (txState_ == TxState::kAnswer) {
    // Build ANSWER frame
    // Format (per Bitcraze protocol):
    // Byte 0: TYPE (0x02)
    // Byte 1: SEQ
    // Byte 2+: Optional LPP data (position if enabled)
    
    size_t answerPayloadLen = 2;  // TYPE + SEQ
    
    // Check if we can include position data
    if (config_.positionEnabled) {
      answerPayloadLen += 2 + sizeof(LppShortAnchorPosition);  // LPP header + type + position
    }
    
    if (bufferSize < sizeof(MAC80215Header) + answerPayloadLen) {
      return 0;
    }

    MAC80215Header* macHeader = reinterpret_cast<MAC80215Header*>(buffer);
    macHeader->fcf = makeFcf();
    macHeader->seq = seqNr_;
    macHeader->pan = 0xBCCF;
    
    memset(macHeader->destAddr, 0, 8);
    macHeader->destAddr[0] = currTag_;
    memset(macHeader->srcAddr, 0, 8);
    macHeader->srcAddr[0] = config_.anchorId;

    uint8_t* payloadPtr = buffer + sizeof(MAC80215Header);
    payloadPtr[0] = kPacketTypeAnswer;
    payloadPtr[1] = seqNr_;

    // Add optional LPP position data
    if (config_.positionEnabled) {
      payloadPtr[2] = kLppShort;           // LPP header type
      payloadPtr[3] = kLppShortAnchorPosition;  // LPP packet type
      
      LppShortAnchorPosition* pos = 
        reinterpret_cast<LppShortAnchorPosition*>(payloadPtr + 4);
      pos->position[0] = config_.position[0];
      pos->position[1] = config_.position[1];
      pos->position[2] = config_.position[2];
    }

    frameSize = sizeof(MAC80215Header) + answerPayloadLen;
    txState_ = TxState::kIdle;
    
  } else if (txState_ == TxState::kReport) {
    // Build REPORT frame
    // Format (per Bitcraze protocol):
    // Byte 0: TYPE (0x04)
    // Byte 1: SEQ
    // Byte 2+: reportPayload_t structure
    
    size_t reportPayloadLen = 2 + sizeof(ReportPayload);
    
    if (bufferSize < sizeof(MAC80215Header) + reportPayloadLen) {
      return 0;
    }

    MAC80215Header* macHeader = reinterpret_cast<MAC80215Header*>(buffer);
    macHeader->fcf = makeFcf();
    macHeader->seq = seqNr_;
    macHeader->pan = 0xBCCF;
    
    memset(macHeader->destAddr, 0, 8);
    macHeader->destAddr[0] = currTag_;
    memset(macHeader->srcAddr, 0, 8);
    macHeader->srcAddr[0] = config_.anchorId;

    uint8_t* payloadPtr = buffer + sizeof(MAC80215Header);
    payloadPtr[0] = kPacketTypeReport;
    payloadPtr[1] = seqNr_;

    // Build report payload starting at offset 2
    ReportPayload* report = reinterpret_cast<ReportPayload*>(payloadPtr + 2);
    
    // Copy 5-byte timestamps
    memcpy(report->pollRx, pollRxTime_.raw, 5);
    memcpy(report->answerTx, answerTxTime_.raw, 5);
    memcpy(report->finalRx, finalRxTime_.raw, 5);
    
    // Set environment data (all zeros for now)
    report->pressure = 0.0f;
    report->temperature = 0.0f;
    report->asl = 0.0f;
    report->pressure_ok = 0;

    frameSize = sizeof(MAC80215Header) + reportPayloadLen;
    txState_ = TxState::kIdle;
  }
  
  scheduleNextTransmit(millis());
  return frameSize;
}

}  // namespace twr
