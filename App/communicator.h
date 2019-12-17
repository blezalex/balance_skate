#pragma once
#include "common.h"
#include "serial.h"

#define kHeaderSize 4 // Id + 1 byte msg len, 2 bytes padding
#define kSuffixSize 2 // CRC
#define kMetadataSize (kHeaderSize + kSuffixSize)
#define kMsgTimeoutMs 1000u // 1 second

class Communicator {
public:
	void Init(Usart* comms_channel) {
	  comms_ = comms_channel;
	}

	bool SendMsg(uint8_t msg_id, const uint8_t* data, uint32_t data_len);

	bool SendMsg(uint8_t msg_id);

	// returns msg_id
	int update();

	const uint8_t* data() {
		return rx_data + kHeaderSize;
	}

	const uint8_t data_len() {
		return expected_msg_len() - kMetadataSize;
	}

private:
	// buffer_pos_ always points  to non-written yet memory. bufferpos = 1 means have only one byte with index 0
	int32_t buffer_pos_ = 0;

	uint8_t expected_msg_id() {
		return rx_data[0];
	}

	uint8_t expected_msg_len() {
		return rx_data[1];
	}

	uint8_t expected_msg_crc_lo() {
		return rx_data[expected_msg_len() - 2];
	}

	uint8_t expected_msg_crc_hi() {
		return rx_data[expected_msg_len() - 1];
	}

	uint8_t rx_data[255];

	uint16_t last_uart_data_time_ = 0;

	// previous update processed a message. buffer_pos_ still poits to its end or some bytes past it. move data if past.
	bool move_message_ = false;

	Usart* comms_;
};
