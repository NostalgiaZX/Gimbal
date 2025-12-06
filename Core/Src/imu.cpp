
#define g 9.81f
#define M_PI 3.1415
#include "../Inc/imu.h"
float gyrox;
float gyroy;
float gyroz;
int gyrocount;
imu bmi088_imu;
void imu::acc_calculate(){
  bmi088_accel_read_reg(0x41, &accrange_raw, 1);
  accrange=(float)(accrange_raw+1)*3.f*g;
  bmi088_accel_read_reg(0x12, accdata, 6);
  as_[0]= (float)(int16_t)(accdata[0]+accdata[1]*256)/32768.f*(float)accrange;
  as_[1]= (float)(int16_t)(accdata[2]+accdata[3]*256)/32768.f*(float)accrange;
  as_[2] = (float)(int16_t)(accdata[4]+accdata[5]*256)/32768.f*(float)accrange;
    acc_read();
}

void imu::gyro_calculate(){
/*
  bmi088_gyro_read_reg(0x0F,&gyrorange_raw,1);
  gyrorange=2000.f/(float)(gyrorange_raw+1);
  bmi088_gyro_read_reg(0x02,gyrodata,6);

  ws_[0]=(float)(int16_t)(gyrodata[0]+gyrodata[1]*256)/32768*gyrorange;
  ws_[1]=(float)(int16_t)(gyrodata[2]+gyrodata[3]*256)/32768*gyrorange;
  ws_[2]=(float)(int16_t)(gyrodata[4]+gyrodata[5]*256)/32768*gyrorange;
    gyro_read();
*/
    bmi088_gyro_read_reg(0x0F, &gyrorange_raw, 1);
    gyrorange_raw&=0x07;
    bmi088_gyro_read_reg(0x02, gyrodata, 6);
    int16_t gyro_x_tmp=(gyrodata[1]<<8 )| gyrodata[0];
    int16_t gyro_y_tmp=(gyrodata[3]<<8)| gyrodata[2];
    int16_t gyro_z_tmp=(gyrodata[5]<<8)| gyrodata[4];
    float ratio=0.0f;
    switch (gyrorange_raw) {
    case 0x00:ratio=2000.f/32767.f/180.0f*3.1415926f;break;
    case 0x01:ratio=1000.f/32767.f/180.0f*3.1415926f;break;
    case 0x02:ratio=500.f/32767.f/180.0f*3.1415926f;break;
    case 0x03:ratio=250.f/32767.f/180.0f*3.1415926f;break;
    case 0x04:ratio=125.f/32767.f/180.0f*3.1415926f;break;
    default:ratio=0.0f;break;
    }
    ws_[0]=(float)gyro_x_tmp*ratio-gyrox;
    ws_[1]=(float)gyro_y_tmp*ratio-gyroy;
    ws_[2]=(float)gyro_z_tmp*ratio-gyroz;
    if (gyrocount++<100)
    {
        gyrox+=ws_[0];
        gyroy+=ws_[1];
        gyroz+=ws_[2];
    }
    if (gyrocount==100)
    {
        gyrox=gyrox/gyrocount;
        gyroy=gyroy/gyrocount;
        gyroz=gyroz/gyrocount;
    }
}
void imu::acc_read()
{
  rollacc=std::atan(as_[1]/as_[2])* (180 / M_PI);
    pitchacc=std::atan(as_[0]/std::sqrt(as_[1]*as_[1]+as_[2]*as_[2]))* (180 / M_PI);
}
void imu::gyro_read()
{
    float pitch = EulerAngle_degrees.pitch;
    float roll = EulerAngle_degrees.roll;
    rollgyro+=ws_[0]*dt;
    pitchgyro+=ws_[1]*dt;
    yawgyro+=ws_[2]*dt;
}

void imu::filter(float k)
{
    EulerAngle_degrees.roll=rollgyro+(rollacc-rollgyro)*k;
    EulerAngle_degrees.pitch=pitchgyro+(pitchacc-pitchgyro)*k;
    EulerAngle_degrees.yaw=yawgyro;
}
void imu::init()
{
    acc_calculate();
    rollgyro=rollacc;
    pitchgyro=pitchacc;
    filter();
    //eulerToQuaternion(EulerAngle_degrees.yaw,EulerAngle_degrees.pitch,EulerAngle_degrees.roll,q_);
}
void imu::eulerToQuaternion(float yaw, float pitch, float roll, float q[4])
{
    q[0] = cos(roll * 0.5f) * cos(pitch * 0.5f) * cos(yaw * 0.5f) + sin(roll * 0.5f) * sin(pitch * 0.5f) * sin(yaw * 0.5f);  // q0
    q[1] = sin(roll * 0.5f) * cos(pitch * 0.5f) * cos(yaw * 0.5f) - cos(roll * 0.5f) * sin(pitch * 0.5f) * sin(yaw * 0.5f);  // q1
    q[2] = cos(roll * 0.5f) * sin(pitch * 0.5f) * cos(yaw * 0.5f) + sin(roll * 0.5f) * cos(pitch * 0.5f) * sin(yaw * 0.5f);  // q2
    q[3] = cos(roll * 0.5f) * cos(pitch * 0.5f) * sin(yaw * 0.5f) - sin(roll * 0.5f) * sin(pitch * 0.5f) * cos(yaw * 0.5f);  // q3

}
void imu::mahony_update()
{
    mahony.update(q_,ws_,as_);
    Mahony_degrees.fromQuaternion(q_);
}
void EulerAngle::fromQuaternion(const float q[4])
{

    float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];

    // 计算俯仰角 pitch
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2.0f, sinp)*180.f/M_PI;  // 处理万向锁
    else
        pitch = asin(sinp)*180.f/M_PI;

    // 计算偏航角 yaw
    yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3))*180.f/M_PI;

    // 计算滚转角 roll
    roll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2))*180.f/M_PI;
}
