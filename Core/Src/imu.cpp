
#define g 9.81f
#define M_PI 3.1415
#include "../Inc/imu.h"

imu bmi088_imu;
void imu::acc_calculate(){
  bmi088_accel_read_reg(0x41, &accrange_raw, 1);
  accrange=(float)(accrange_raw+1)*3.f*g;
  bmi088_accel_read_reg(0x12, accdata, 6);

  as[0] = (float)(int16_t)(accdata[0]+accdata[1]*256)/32768.f*(float)accrange;
  as[1] = (float)(int16_t)(accdata[2]+accdata[3]*256)/32768.f*(float)accrange;
  as[2] = (float)(int16_t)(accdata[4]+accdata[5]*256)/32768.f*(float)accrange;
    acc_read();
}

void imu::gyro_calculate(){

  bmi088_gyro_read_reg(0x0F,&gyrorange_raw,1);
  gyrorange=2000.f/(float)(gyrorange_raw+1);
  bmi088_gyro_read_reg(0x02,gyrodata,6);
  ws[0]=(float)(int16_t)(gyrodata[0]+gyrodata[1]*256)/32768*gyrorange;
  ws[1]=(float)(int16_t)(gyrodata[2]+gyrodata[3]*256)/32768*gyrorange;
  ws[2]=(float)(int16_t)(gyrodata[4]+gyrodata[5]*256)/32768*gyrorange;
    gyro_read();
}
void imu::acc_read()
{
  rollacc=std::atan(as[1]/as[2])* (180 / M_PI);
    pitchacc=std::atan(as[0]/std::sqrt(as[1]*as[1]+as[2]*as[2]))* (180 / M_PI);
}
void imu::gyro_read()
{
    float pitch = EulerAngle_degrees.pitch;
    float roll = EulerAngle_degrees.roll;
    rollgyro+=(ws[0]+sin(pitch)*sin(roll)/cos(pitch)*ws[1]+sin(pitch)*cos(roll)/cos(pitch)*ws[2])*dt;
    pitchgyro+=(ws[1]*cos(roll)-sin(roll)*ws[2])*dt;
    yawgyro+=(sin(roll)/cos(pitch)*ws[1]+cos(roll)/cos(pitch)*ws[2])*dt;
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
    eulerToQuaternion(EulerAngle_degrees.yaw,EulerAngle_degrees.pitch,EulerAngle_degrees.roll,q_);
}
void imu::eulerToQuaternion(float yaw, float pitch, float roll, float q[4])
{
    q[0] = cos(roll * 0.5f) * cos(pitch * 0.5f) * cos(yaw * 0.5f) + sin(roll * 0.5f) * sin(pitch * 0.5f) * sin(yaw * 0.5f);  // q0
    q[1] = sin(roll * 0.5f) * cos(pitch * 0.5f) * cos(yaw * 0.5f) - cos(roll * 0.5f) * sin(pitch * 0.5f) * sin(yaw * 0.5f);  // q1
    q[2] = cos(roll * 0.5f) * sin(pitch * 0.5f) * cos(yaw * 0.5f) + sin(roll * 0.5f) * cos(pitch * 0.5f) * sin(yaw * 0.5f);  // q2
    q[3] = cos(roll * 0.5f) * cos(pitch * 0.5f) * sin(yaw * 0.5f) - sin(roll * 0.5f) * sin(pitch * 0.5f) * cos(yaw * 0.5f);  // q3

}
