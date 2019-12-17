#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdint.h>
#include <cstring>

#include "common.h"
#include <algorithm>

#include "cmsis_os.h"

class Usart {
public:

	void Init(UART_HandleTypeDef *hw) {
	  hw_ = hw;
	  HAL_DMA_Start(hw_->hdmarx, (uint32_t)&(hw_->Instance->DR), (uint32_t)rx_buffer_, sizeof(rx_buffer_));
	  SET_BIT(hw_->Instance->CR3, USART_CR3_DMAR);
	}

	bool Send(const uint8_t *data, uint32_t size) {
	  if (size > sizeof(tx_buffer_))
	    return false;

	  if (hw_->gState != HAL_UART_STATE_READY) {
	    return false;
	  }

	  memcpy(tx_buffer_, data, size);
		return !HAL_UART_Transmit_IT(hw_, tx_buffer_, size);
	}

	bool Send(const char *data, uint32_t size) {
		return Send((const uint8_t*) data, size);
	}

	uint32_t Read(uint8_t *data, uint32_t max_size) {
	  uint32_t write_pos = sizeof(rx_buffer_) - hw_->hdmarx->Instance->NDTR;
	  uint32_t bytes_available = (sizeof(rx_buffer_) + write_pos - rx_pos_) % sizeof(rx_buffer_);

	  uint32_t bytes_to_copy = std::min(max_size, bytes_available);
	  for (uint32_t i = 0; i < bytes_to_copy; i++) {
	    data[i] = rx_buffer_[rx_pos_++];
	    if (rx_pos_ >= sizeof(rx_buffer_))
	      rx_pos_ = 0;
	  }
	  return bytes_to_copy;
	}

	bool HasData() {
	  uint32_t write_pos = sizeof(rx_buffer_) - hw_->hdmarx->Instance->NDTR;
	  return write_pos != rx_pos_;
	}

	void WaitTx() {
	  while (hw_->gState != HAL_UART_STATE_READY) {
	    osDelay(0);
	  }
	}

private:
	UART_HandleTypeDef *hw_;
	uint8_t tx_buffer_[256];

	// buffer overruns if not read in time
	uint8_t rx_buffer_[128];

	uint32_t rx_pos_ = 0;
};

#endif
