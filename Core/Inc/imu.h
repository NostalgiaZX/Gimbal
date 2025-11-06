
#ifndef SPITEST_IMU_H
#define SPITEST_IMU_H
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "bmi088.h"
#include <cmath>
class imu {
private:
  float dt=0.001;
  float acc_x=0, acc_y=0, acc_z=0;
  uint8_t accrange_raw=0;
  float accrange=0;
  float gyro_x=0, gyro_y=0, gyro_z=0;
  uint8_t gyrorange_raw=0;
  float gyrorange=0;
  uint8_t accdata[6]={0};
  uint8_t gyrodata[6]={0};
  float rollacc=0, pitchacc=0;
  float rollgyro=0, pitchgyro=0,yawgyro=0;
  float roll=0, pitch=0, yaw=0;
  public:
    imu(const float yawori =0,const float deltatime=0.001):yaw(yawori),yawgyro(yawori),dt(deltatime)
    {
        this->init();
    }
  void acc_calculate();
    void acc_read();
  void gyro_calculate();
    void gyro_read();
    void filter(float k=0.4);
    void init();

};

#endif
