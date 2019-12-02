#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdint.h>
#include <cstring>

#include "common.h"

class Usart {
public:

	void Init(UART_HandleTypeDef *hw) {
	  hw_ = hw;
	}

	bool Send(const uint8_t *data, int32_t size) {
	  if (size > sizeof(buffer_))
	    return false;

	  if (hw_->gState != HAL_UART_STATE_READY) {
	    return false;
	  }

	  memcpy(buffer_, data, size);
		return !HAL_UART_Transmit_IT(hw_, buffer_, size);
	}

	bool Send(const char *data, int32_t size) {
		return Send((const uint8_t*) data, size);
	}

	int32_t Read(uint8_t *data, int32_t max_size);

	bool HasData();

private:
	UART_HandleTypeDef *hw_;
	uint8_t buffer_[256];
};

#endif
