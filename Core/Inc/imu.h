#ifndef SPITEST_IMU_H
#define SPITEST_IMU_H
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "bmi088.h"
#include <cmath>
#include "mahony.h"
// 欧拉角
class EulerAngle {
    public:
    float yaw;
    float pitch;
    float roll;
    explicit EulerAngle(float y = 0, float p = 0, float r = 0)
      : yaw(y), pitch(p), roll(r) {}
    void fromQuaternion(const float q[4]);
};

class imuRawData {
    float gyro[3];
    float accel[3];
    float temp[1];
} ;

class imu {
private:
  float dt=0.001;
    //加速度计数据,x,y,z
    float as_[3];
  uint8_t accrange_raw=0;
  float accrange=0;
    //陀螺仪数据,x,y,z
    float ws_[3];
  uint8_t gyrorange_raw=0;
  float gyrorange=0;
  uint8_t accdata[6]={0};
  uint8_t gyrodata[6]={0};
    //四元数
    float q_[4];
  float rollacc=0, pitchacc=0;
  float rollgyro=0, pitchgyro=0,yawgyro=0;
    EulerAngle EulerAngle_degrees;
    EulerAngle Mahony_degrees;
    Mahony mahony;
  public:
    imu(const float yawori =0,const float deltatime=0.001):EulerAngle_degrees(yawori,0,0),Mahony_degrees(yawori,0,0),mahony(deltatime),yawgyro(yawori),dt(deltatime)
    {
        this->init();
    }
    void eulerToQuaternion(float yaw, float pitch, float roll, float q[4]);
  void acc_calculate();
    void acc_read();
  void gyro_calculate();
    void gyro_read();
    void filter(float k=0.4);
    void mahony_update();
    void init();

};

#endif
