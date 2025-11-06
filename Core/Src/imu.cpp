
#define g 9.81f
#define M_PI 3.1415
#include "../Inc/imu.h"

imu bmi088_imu;
void imu::acc_calculate(){
  bmi088_accel_read_reg(0x41, &accrange_raw, 1);
  accrange=(float)(accrange_raw+1)*3.f*g;
  bmi088_accel_read_reg(0x12, accdata, 6);

  acc_x = (float)(int16_t)(accdata[0]+accdata[1]*256)/32768.f*(float)accrange;
  acc_y = (float)(int16_t)(accdata[2]+accdata[3]*256)/32768.f*(float)accrange;
  acc_z = (float)(int16_t)(accdata[4]+accdata[5]*256)/32768.f*(float)accrange;
    acc_read();
}

void imu::gyro_calculate(){

  bmi088_gyro_read_reg(0x0F,&gyrorange_raw,1);
  gyrorange=2000.f/(float)(gyrorange_raw+1);
  bmi088_gyro_read_reg(0x02,gyrodata,6);
  gyro_x=(float)(int16_t)(gyrodata[0]+gyrodata[1]*256)/32768*gyrorange;
  gyro_y=(float)(int16_t)(gyrodata[2]+gyrodata[3]*256)/32768*gyrorange;
  gyro_z=(float)(int16_t)(gyrodata[4]+gyrodata[5]*256)/32768*gyrorange;
    gyro_read();
}
void imu::acc_read()
{
  rollacc=std::atan(acc_y/acc_z)* (180 / M_PI);
    pitchacc=std::atan(-acc_x/std::sqrt(acc_y*acc_y+acc_z*acc_z))* (180 / M_PI);
}
// 计算陀螺仪角度??
void imu::gyro_read()
{
    rollgyro+=gyro_x*dt;
    pitchgyro+=gyro_y*dt;
    yawgyro+=gyro_z*dt;
}

void imu::filter(float k)
{
    roll=rollgyro+(rollacc-rollgyro)*k;
    pitch=pitchgyro+(pitchacc-pitchgyro)*k;
    yaw=yawgyro;
}
void imu::init()
{
    acc_calculate();
    rollgyro=rollacc;
    pitchgyro=pitchacc;

}
