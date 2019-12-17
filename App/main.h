#pragma once


#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

EXTERNC void MainTask();

EXTERNC void CommsTask();

EXTERNC void RcPinInterrupt();
