#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdint.h>

#include "common.h"

class Usart {
public:

	Usart(UART_HandleTypeDef *hw) :
			hw_(hw) {
	}

	bool Send(const uint8_t *data, int32_t size) {
		return !HAL_UART_Transmit(hw_, const_cast<uint8_t *>(data), size, 100);
	}

	bool Send(const char *data, int32_t size) {
		return Send((const uint8_t*) data, size);
	}

	int32_t Read(uint8_t *data, int32_t max_size);

	bool HasData();

private:
	UART_HandleTypeDef *hw_;
};

#endif
