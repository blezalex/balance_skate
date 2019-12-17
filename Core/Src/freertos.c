/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     

#include "i2c.h"
#include "usart.h"

#include "../../App/main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct MpuUpdates {
	int16_t gyro[3];
	int16_t acc[3];
} MpuUpdate;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
int global_var = 0;
MpuUpdate update;
/* USER CODE END Variables */
osThreadId_t defaultTaskHandle;
osThreadId_t commsHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

#define BOARD_ROTATION_MACRO BOARD_ROTATION_UPSIDE_DOWN_X
#define BOARD_ROTATION_UPSIDE_DOWN_X(XYZ) XYZ[1]*=-1; XYZ[2]*=-1;  // rotated 180 deg around X axis

void i2c_writeReg(uint8_t hwAddr, uint8_t wAddr, uint8_t value) {
	HAL_I2C_Mem_Write(&hi2c1, hwAddr << 1, wAddr, I2C_MEMADD_SIZE_8BIT, &value, 1, 100000);
}

void i2c_read_reg_to_buf(uint8_t hwAddr, uint8_t rAddr, uint8_t* buf, uint8_t size) {
	HAL_I2C_Mem_Read(&hi2c1, hwAddr << 1, rAddr, I2C_MEMADD_SIZE_8BIT, buf, size, 100000);
}

int16_t getAccVal(uint8_t* rawData, uint8_t axis) {
	return ((int16_t)((rawData[axis*2]<<8) | rawData[axis*2 + 1]))>>2;
}

int16_t getGyroVal(uint8_t* rawData, uint8_t axis) {
	return ((int16_t)((rawData[axis * 2] << 8) | rawData[axis * 2 + 1])) >> 2;  // range: +/- 8192; +/- 500 deg/sec
}

void handleGyroData(int16_t* gyro,  uint8_t* rawData) {
	gyro[0] =  getGyroVal(rawData, 1);
	gyro[1] =  getGyroVal(rawData, 0);
	gyro[2] = -getGyroVal(rawData, 2);
	BOARD_ROTATION_MACRO(gyro);
}

void handleAccData(int16_t* acc, uint8_t* rawData) {
	acc[0] = -getAccVal(rawData, 1);
	acc[1] = -getAccVal(rawData, 0);
	acc[2] =  getAccVal(rawData, 2);
	BOARD_ROTATION_MACRO(acc);
}

void handleRawData(MpuUpdate* update, uint8_t* rawData) {
	handleGyroData(update->gyro, rawData+8); // first 6 are ACC data, then 2 temperature and next 6 are gyro data
	handleAccData(update->acc, rawData);
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */
osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .priority = (osPriority_t) osPriorityRealtime,
    .stack_size = 2048
  };
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* definition and creation of comms */
  const osThreadAttr_t comms_attributes = {
    .name = "comms",
    .priority = (osPriority_t) osPriorityLow,
    .stack_size = 2048
  };
  commsHandle = osThreadNew(StartTask02, NULL, &comms_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    
    
    

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

	MainTask();

  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the comms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  CommsTask();
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
