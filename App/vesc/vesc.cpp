#include "vesc.h"
#include "crc.h"
#include <string.h>

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t* buffer, uint16_t number, int32_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index) {
    buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

bool VescComm::sendRequest(const uint8_t* payload, int payload_len) {
  int32_t idx = 0;
  if (payload_len <= 256) {
    tx_data_[idx++] = 0x2;
    tx_data_[idx++] = (uint8_t)payload_len;
  }
  else {
    tx_data_[idx++] = 0x3;
    buffer_append_int16(tx_data_, payload_len, &idx);
  }

  memcpy(tx_data_ + idx, payload, payload_len);
  idx += payload_len;

  uint16_t crc_payload = crc16(payload, payload_len);
  buffer_append_int16(tx_data_, crc_payload, &idx);
  tx_data_[idx++] = 0x3;

  return serial_->Send(tx_data_, idx);
}


bool VescComm::requestStats() {
	uint8_t request[] = { (uint8_t)COMM_PACKET_ID::COMM_GET_VALUES};
	return sendRequest(request , sizeof(request));
}

bool VescComm::setCurrent(float current) {
	uint8_t request[] = { (uint8_t)COMM_PACKET_ID::COMM_SET_CURRENT, 0, 0, 0, 0};
	int32_t send_index = 1;
	buffer_append_float32(request, current, 1000.0, &send_index);
	return sendRequest(request , sizeof(request));
}

bool VescComm::setCurrentBrake(float current) {
	uint8_t request[] = { (uint8_t)COMM_PACKET_ID::COMM_SET_CURRENT_BRAKE, 0, 0, 0, 0};
	int32_t send_index = 1;
	buffer_append_float32(request, current, 1000.0, &send_index);
	return sendRequest(request , sizeof(request));
}

#define kMsgTimeoutMs 100u
#define kHeaderSize 3
#define kFooterSize 3


static int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
	int16_t res =	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

static uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index) {
	uint16_t res = 	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

static int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
	int32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

static uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index) {
	uint32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

static float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index) {
    return (float)buffer_get_int16(buffer, index) / scale;
}

static  float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index) {
    return (float)buffer_get_int32(buffer, index) / scale;
}


int VescComm::update() {
	if (!serial_->HasData())
		return 0;

	uint16_t time = millis();
	if ((uint16_t)(time - last_uart_data_time_) > kMsgTimeoutMs) {
		buffer_pos_ = 0;
	}

	last_uart_data_time_ = time;

	int32_t received_bytes = serial_->Read(rx_data_ + buffer_pos_, sizeof(rx_data_) - buffer_pos_);
	buffer_pos_+= received_bytes;
	if (buffer_pos_ > kHeaderSize) {
		if (buffer_pos_ >= (int32_t)sizeof(rx_data_)) {
			buffer_pos_ = 0; // too long/invalid
		}

		if (buffer_pos_ >= actual_header_size() + expected_msg_len() + kFooterSize) {
			uint16_t crc_payload = crc16(rx_data_ + actual_header_size(), expected_msg_len());

			int footer_start_idx = actual_header_size() + expected_msg_len();

			if (crc_payload >> 8 != rx_data_[footer_start_idx] || (crc_payload & 0xFF) != rx_data_[footer_start_idx + 1]) {
				buffer_pos_ = 0;
				return -1; // Bad CRC
			}

			int msg_id = rx_data_[actual_header_size()];
			int32_t idx = actual_header_size() + 1;

			if (msg_id == (uint8_t)COMM_PACKET_ID::COMM_GET_VALUES) {
				mc_values_.temp_mos_filtered = buffer_get_float16(rx_data_, 1e1, &idx);
				mc_values_.temp_motor_filtered = buffer_get_float16(rx_data_, 1e1, &idx);
				mc_values_.avg_motor_current = buffer_get_float32(rx_data_, 1e2, &idx);
				mc_values_.avg_input_current = buffer_get_float32(rx_data_, 1e2, &idx);
				idx+=8; // i_current, q_curent
				mc_values_.duty_now = buffer_get_float16(rx_data_, 1e3, &idx);
				mc_values_.rpm = buffer_get_float32(rx_data_, 1e0, &idx);
				mc_values_.v_in = buffer_get_float16(rx_data_, 1e1, &idx);
				mc_values_.amp_hours = buffer_get_float32(rx_data_, 1e4, &idx);
				mc_values_.amp_hours_charged = buffer_get_float32(rx_data_, 1e4, &idx);
				idx+=8; // watt hours used, charged
				mc_values_.tachometer =  buffer_get_int32(rx_data_, &idx);
				mc_values_.tachometer_abs =  buffer_get_int32(rx_data_, &idx);
			}

			int bytes_to_move = buffer_pos_ - (footer_start_idx + kFooterSize);
			if (bytes_to_move > 0) {
				memmove(rx_data_, rx_data_ + footer_start_idx + kFooterSize, bytes_to_move);
			}
			buffer_pos_ = bytes_to_move;

			return msg_id;
		}
	}

	return 0;
}
