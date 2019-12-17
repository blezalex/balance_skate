#include "communicator.h"

#include "crc.h"
#include "proto/protocol.pb.h"

void CRC_ResetDR() {
  hcrc.Instance->CR |= CRC_CR_RESET;
}

uint32_t CRC_CalcCRC(uint32_t val) {
  hcrc.Instance->DR = val;
  return hcrc.Instance->DR;
}

uint32_t CRC_CalcBlockCRC(uint32_t *data, uint32_t blocks_cnt) {
  return HAL_CRC_Accumulate(&hcrc, data, blocks_cnt);
}

bool Communicator::SendMsg(uint8_t msg_id, const uint8_t *data,
                           uint32_t data_len) {
  uint8_t header[kHeaderSize];
  header[0] = msg_id;
  header[1] = data_len + kMetadataSize;
  header[2] = 0;
  header[3] = 0;
  CRC_ResetDR();
  uint32_t crc32 = CRC_CalcCRC(*(uint32_t*) header);

  uint8_t aligned_blocks = data_len / 4;
  if (data_len > 0) {
    crc32 = CRC_CalcBlockCRC((uint32_t*) data, aligned_blocks);
  }
  uint8_t last_block[4] = { 0 };
  int remaining_offset = aligned_blocks * 4;
  for (int i = 0; i < data_len - remaining_offset; i++) {
    last_block[i] = data[remaining_offset + i];
  }
  if (remaining_offset != data_len) {
    crc32 = CRC_CalcCRC(*(uint32_t*) last_block);
  }

  comms_->WaitTx();
  if (!comms_->Send(header, kHeaderSize)) {
    return false;
  }
  comms_->WaitTx();
  if (!comms_->Send(data, data_len)) {
    return false;
  }
  comms_->WaitTx();

  if (!comms_->Send((uint8_t*) &crc32, kSuffixSize)) {
    return false;
  }
  return true;
}

bool Communicator::SendMsg(uint8_t msg_id) {
  uint8_t msg[kHeaderSize + kSuffixSize] = { 0 };
  msg[0] = msg_id;
  msg[1] = kMetadataSize;

  CRC_ResetDR();
  uint32_t crc32 = CRC_CalcCRC(*(uint32_t*) msg);
  msg[kHeaderSize] = ((uint8_t*) &crc32)[0];
  msg[kHeaderSize + 1] = ((uint8_t*) &crc32)[1];

  comms_->WaitTx();
  return comms_->Send(msg, sizeof(msg));
}

int Communicator::update() {
  if (!comms_->HasData())
    return false;

  uint16_t time = millis();
  if ((uint16_t) (time - last_uart_data_time_) > kMsgTimeoutMs) {
    buffer_pos_ = 0;
    move_message_ = false;
//    comms_->Send("Timeout\n", 8);
  }
  last_uart_data_time_ = time;

  if (move_message_) {
    move_message_ = false;
    int bytes_to_move = buffer_pos_ - (int) expected_msg_len();

    if (bytes_to_move > 0) {
      memmove(rx_data, rx_data + expected_msg_len(), bytes_to_move);
      buffer_pos_ = bytes_to_move;
    } else {
      buffer_pos_ = 0;
    }
  }

  int32_t received_bytes = comms_->Read(rx_data + buffer_pos_,
                                        sizeof(rx_data) - buffer_pos_);
   buffer_pos_ += received_bytes;
  if (buffer_pos_ > kHeaderSize) {
    if (buffer_pos_ >= sizeof(rx_data))
      buffer_pos_ = 0;  // too long/invalid

    if (buffer_pos_ >= expected_msg_len()) {
      int32_t msg_len = expected_msg_len() - kSuffixSize;
      CRC_ResetDR();
      int32_t aligned_blocks = msg_len / 4;
      uint32_t crc32;
      if (msg_len > 0) {
        crc32 = CRC_CalcBlockCRC((uint32_t*) rx_data, aligned_blocks);
      }
      uint8_t last_block[4] = { 0 };
      int remaining_offset = aligned_blocks * 4;
      for (int i = 0; i < msg_len - remaining_offset; i++) {
        last_block[i] = rx_data[remaining_offset + i];
      }
      if (remaining_offset != msg_len) {
        crc32 = CRC_CalcCRC(*(uint32_t*) last_block);
      }

      uint8_t *crc_bytes = (uint8_t*) &crc32;
      int msg_id = expected_msg_id();
      if (crc_bytes[0] != expected_msg_crc_lo()
          || crc_bytes[1] != expected_msg_crc_hi()) {
        msg_id = RequestId_MSG_NONE;
        SendMsg(ReplyId_CRC_MISMATCH);
      }

      move_message_ = true;
      return msg_id;
    }
  }

  return 0;
}
