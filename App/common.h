#ifndef COMMON_H_
#define COMMON_H_

#include "../Core/Inc/main.h"

inline uint32_t millis() {
	return HAL_GetTick();
}

//#define min(a,b) ((a)<(b)?(a):(b))
//#define max(a,b) ((a)>(b)?(a):(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

inline uint16_t micros() {
  return TIM11->CNT;
}



#endif
