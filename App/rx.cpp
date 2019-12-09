#include "rx.h"
#include "common.h"

uint16_t rxVals[NUMBER_RX_INPUT];


unsigned long pinTimes[NUMBER_RX_INPUT];


void on_ppm_interrupt()
{
  static uint16_t lastIntTime;
  static int8_t channelIdx;
  static bool wait_blank = true;

  uint16_t currentTime = micros();
  uint16_t pulseDuration = currentTime - lastIntTime;
  if (pulseDuration > 5000) {
    wait_blank = false;
    channelIdx = 0;
  } else if (pulseDuration > MAX_CMD_LEN || pulseDuration < MIN_CMD_LEN) {
    wait_blank = true;
  }
  else if (!wait_blank) {
    if (channelIdx < NUMBER_RX_INPUT) // ignore inputs if more than channels declared
    {
      rxVals[channelIdx] = pulseDuration;
    }

    channelIdx++;
  }

  lastIntTime = currentTime;
}
