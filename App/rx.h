#ifndef rx_h
#define rx_h

#include <inttypes.h>

#define RC_SUM_PPM 1


#define MIN_CMD_LEN 900
#define MAX_CMD_LEN 2100

#define NUMBER_RX_INPUT 6

extern uint16_t rxVals[NUMBER_RX_INPUT];


#define CH_AIL 0
#define CH_ELE 1
#define CH_THR 2
#define CH_RUD 3
#define CH_AUX 4
#define CH_AUX2 5

void initRxInput();

void on_ppm_interrupt();

#endif
