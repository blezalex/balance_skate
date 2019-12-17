#include "../App/main.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "i2c.h"
#include "usart.h"
#include "rx.h"

#include "../App/mpu.h"
#include "imu/MadgwickAHRS.hpp"

#include "pid.h"
#include "vesc/vesc.h"
#include "lpf.h"
#include "MedianFilter.h"
#include "crc.h"

#include "communicator.h"
#include "proto/protocol.pb.h"
#include "settings/settings.h"

#include <cstdio>

#define MPU6050_ADDRESS     0x68u

//#define BOARD_ROTATION_MACRO BOARD_ROTATION_UPSIDE_DOWN_X
//#define BOARD_ROTATION_UPSIDE_DOWN_X(XYZ) XYZ[1]*=-1; XYZ[2]*=-1;  // rotated 180 deg around X axis

typedef struct MpuUpdates {
  int16_t gyro[3];
  int16_t acc[3];
} MpuUpdate;

MpuUpdate update;

void i2c_writeReg(uint8_t hwAddr, uint8_t wAddr, uint8_t value) {
  HAL_I2C_Mem_Write(&hi2c1, hwAddr << 1, wAddr, I2C_MEMADD_SIZE_8BIT, &value, 1,
                    1000);
}

bool i2c_read_reg_to_buf(uint8_t hwAddr, uint8_t rAddr, uint8_t *buf,
                         uint8_t size) {
  return !HAL_I2C_Mem_Read(&hi2c1, hwAddr << 1, rAddr, I2C_MEMADD_SIZE_8BIT, buf, size,
                   1000);
}

int16_t getAccVal(uint8_t *rawData, uint8_t axis) {
  return ((int16_t) ((rawData[axis * 2] << 8) | rawData[axis * 2 + 1])) >> 2;
}

int16_t getGyroVal(uint8_t *rawData, uint8_t axis) {
  return ((int16_t) ((rawData[axis * 2] << 8) | rawData[axis * 2 + 1])) >> 2;  // range: +/- 8192; +/- 500 deg/sec
}

void handleGyroData(int16_t *gyro, uint8_t *rawData) {
  gyro[0] = getGyroVal(rawData, 1);
  gyro[1] = getGyroVal(rawData, 0);
  gyro[2] = -getGyroVal(rawData, 2);
  //BOARD_ROTATION_MACRO(gyro);
}

void handleAccData(int16_t *acc, uint8_t *rawData) {
  acc[0] = -getAccVal(rawData, 1);
  acc[1] = -getAccVal(rawData, 0);
  acc[2] = getAccVal(rawData, 2);
  //BOARD_ROTATION_MACRO(acc);
}

void handleRawData(MpuUpdate *update, uint8_t *rawData) {
  handleGyroData(update->gyro, rawData + 8);  // first 6 are ACC data, then 2 temperature and next 6 are gyro data
  handleAccData(update->acc, rawData);
}

#define ACC_1G 2048
constexpr float MW_GYRO_SCALE = 4 / 65.5;  //MPU6050 and MPU3050   65.5 LSB/(deg/s) and we ignore the last 2 bits

float angles[2];

char buff[100] = { 0 };

Usart motor1_comm;
Usart motor2_comm;

Usart config_comm;

#define MID 1520

float scaleRxInput(int16_t input) {
  return constrain((input - MID) / 400.0, -1, 1);
}


float yaw_rc = 0.1;
LPF yaw_lpf(&yaw_rc);

MedianFilter fwd_med;
MedianFilter yaw_med;

uint8_t scratch[256];

Communicator comms;

Config cfg;

LPF fwd_in_lpf(&cfg.balance_settings.rc_lpf);
LPF yaw_in_lpf(&cfg.balance_settings.rc_lpf);

void CommsTask() {
  config_comm.Init(&huart6);
  comms.Init(&config_comm);
  while (true) {
    uint8_t comms_msg = comms.update();
    switch (comms_msg) {
     case RequestId_READ_CONFIG: {
       int16_t data_len = saveProtoToBuffer(scratch, sizeof(scratch),
                                            Config_fields, &cfg);
       if (data_len > 0) {
         comms.SendMsg(ReplyId_CONFIG, scratch, data_len);
       } else {
         comms.SendMsg(ReplyId_GENERIC_FAIL);
       }
       break;
     }

     case RequestId_GET_DEBUG_BUFFER: {
       break;
     }

     case RequestId_WRITE_CONFIG: {
       bool good =
           readSettingsFromBuffer(&cfg, comms.data(), comms.data_len());
       if (good)
         comms.SendMsg(ReplyId_GENERIC_OK);
       else
         comms.SendMsg(ReplyId_GENERIC_FAIL);
       break;
     }

     case RequestId_GET_STATS: {
       Stats stats = Stats_init_default;

       int16_t data_len =
           saveProtoToBuffer(scratch, sizeof(scratch), Stats_fields, &stats);
       if (data_len != -1) {
         comms.SendMsg(ReplyId_STATS, scratch, data_len);
       } else {
         comms.SendMsg(ReplyId_GENERIC_FAIL);
       }
       break;
     }

     case RequestId_CALLIBRATE_ACC:
       comms.SendMsg(ReplyId_GENERIC_OK);
       break;

     case RequestId_SAVE_CONFIG:
       saveSettingsToFlash(cfg);
       comms.SendMsg(ReplyId_GENERIC_OK);
       break;
    }

    osDelay(2);
  }
}

void MainTask() {
  TIM11->CR1 |= TIM_CR1_CEN;

  cfg = Config_init_default;
  if (!readSettingsFromFlash(&cfg)) {
    cfg = Config_init_default;
  }

  fwd_med.reset();
  yaw_med.reset();

  // Wait for gyro to boot
  osDelay(100);

  i2c_writeReg(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, MPU6050_CLOCK_PLL_ZGYRO);
  i2c_writeReg(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 0);
  i2c_writeReg(MPU6050_ADDRESS, MPU6050_CONFIG, cfg.balance_settings.global_gyro_lpf);
  i2c_writeReg(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, MPU6050_GYRO_FS_500);
  i2c_writeReg(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_FS_4);
  i2c_writeReg(MPU6050_ADDRESS, MPU6050_INT_PIN_CFG, 1 << 4);
  i2c_writeReg(MPU6050_ADDRESS, MPU6050_INT_ENABLE, 1);


  Madgwick mw(&cfg.balance_settings.imu_beta);

//  balance_pid_settings.P = 10;
//  balance_pid_settings.D = 0.11;
//  balance_pid_settings.I = 0.0005;
//  balance_pid_settings.MaxI = 2000;

  PidController balance_pid(&cfg.balance_pid);

//  yaw_pid_settings.P = 0.05;
//  yaw_pid_settings.D = 0.0;
//  yaw_pid_settings.I = 0;
  PidController yaw_pid(&cfg.yaw_pid);

  motor1_comm.Init(&huart1);
  motor2_comm.Init(&huart2);

  VescComm vesc1(&motor1_comm);
  VescComm vesc2(&motor2_comm);

  bool init_complete = false;
  uint8_t raw_data[14] = { 0 };


  for (;;) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    osDelay(1);


    if (!i2c_read_reg_to_buf(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, raw_data,
                        sizeof(raw_data))) {
      while (true) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        osDelay(50);
      }
    }

    //MpuUpdate update;
    handleRawData(&update, raw_data);

    mw.updateIMU(update.gyro[0] * MW_GYRO_SCALE, update.gyro[1] * MW_GYRO_SCALE,
                 update.gyro[2] * MW_GYRO_SCALE, update.acc[0] / (float) ACC_1G,
                 update.acc[1] / (float) ACC_1G, update.acc[2] / (float) ACC_1G,
                 !init_complete);

    angles[0] = mw.getRoll();
    angles[1] = -mw.getPitch();

    const float balance_angle = angles[0];
    const float balance_target = fwd_in_lpf.compute(scaleRxInput(fwd_med.compute(rxVals[1])) * cfg.balance_settings.max_control_angle);
    const float yaw_target = yaw_in_lpf.compute(scaleRxInput(yaw_med.compute(rxVals[0])) * cfg.balance_settings.max_rotation_rate);

    init_complete = init_complete || millis() > 2000;
    if (fabs(balance_angle) < 45 && init_complete) {
      float out = balance_pid.compute(balance_target - balance_angle, -update.gyro[0] * MW_GYRO_SCALE);

      out = constrain(out, -cfg.balance_settings.max_current, cfg.balance_settings.max_current);

      float yaw_out = yaw_pid.compute(yaw_lpf.compute(-update.gyro[2] * MW_GYRO_SCALE) - yaw_target);

      vesc1.setCurrent(out + yaw_out);
      vesc2.setCurrent(out - yaw_out);
    } else {
      balance_pid.reset();
      yaw_pid.reset();

      vesc1.setCurrentBrake(5);
      vesc2.setCurrentBrake(5);
    }
  }
}

EXTERNC void RcPinInterrupt() {
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)) {
    on_ppm_interrupt();
  }

}
