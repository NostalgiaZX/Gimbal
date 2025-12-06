
#include "motor.h"
float ratio = 1;
extern float ksin;
extern float constant;
#define E 2.71828f
float Motor::getAngle() {
    return angle;
}
float adjust_fintensity(float timesum)
{
    if (timesum<0.f)
    {
        return friction_max*(1-pow(E,timesum));
    }
    return -friction_max*(1-pow(E,-timesum));
}
Motor::Motor(const float ratio_,float skp,float ski,float skd,float skimax,float soutmax,float sdfilterk,float pkp,float pki,float pkd,float pkimax,float poutmax,float pdfilterk) : ratio(ratio_), spid_(skp,ski,skd,skimax,soutmax,sdfilterk), ppid_(pkp,pki,pkd,pkimax,poutmax,pdfilterk) {
    control_method_ = POSITION_SPEED;
    target_angle_ = 0.f;
    feedforward_speed_ = 0.f;
    feedforward_intensity_ = 0.f;
    target_speed_ = 0.f;
    output_intensity_ = 0.f;
    spid_.reset();
    ppid_.reset();
}

void timesumadd(const float rotatespeed,float& timesum,const float dt)
{
    if (rotatespeed < 0.f)
    {
        timesum=timesum>0?-dt:timesum-dt;
    }
    else if (rotatespeed > 0.f)
    {
        timesum=timesum<0?dt:timesum+dt;
    }
    else
    {
        timesum=0.f;
    }
}
float pitchforwardinten(float angle)
{
    const float a = 1.21e-7f;   // x³项系数 (1.21×10^-7)
    const float b = -6.3e-5f;   // x²项系数 (-6.3×10^-5)
    const float c = 0.013f;     // x项系数 (0.013)
    const float d = -0.93f;     // 常数项 (-0.93)

    // 计算三阶多项式结果
    float angle_cubed = powf(angle, 3);  // 用powf（float版pow）避免精度损失
    float angle_squared = powf(angle, 2);
    float I = a * angle_cubed + b * angle_squared + c * angle + d;

    return I;
}
float linermap(float in, float inmin, float inmax, float outmin, float outmax) {
    float out;
    out = (in - inmin) * (outmax - outmin) / (inmax - inmin) + outmin;
    return out;
}
float trans360(float in) {
    while (in > 180.0) {
        in -= 360.0;
    }
    while (in < -180.0) {
        in += 360.0;
    }
    return in;
}
void Motor::canrxmsgcallback(const uint8_t rdata[8]) {
    temp = rdata[6];
    auto I = static_cast<int16_t>((rdata[4] << 8) | rdata[5]);
    current = linermap(I, -16384, 16384, -3, 3);
    auto temprary = static_cast<int16_t>((rdata[2] << 8) | rdata[3]);
    rotate_speed = (float)temprary*6.0f;
    timesumadd(rotate_speed, oneway_timesum, 0.001f);
    //rotate_speed =(float)temp;
    last_ecd_angle = ecd_angle;
    ecd_angle = linermap((rdata[0] << 8) | rdata[1], 0, 8191, 0, 360);
    delta_ecd_angle = trans360(ecd_angle - last_ecd_angle);
    delta_angle = delta_ecd_angle / ratio;
    angle += delta_angle;
    fdb_angle_= angle;
    fdb_speed_= rotate_speed;
}
void Motor::SetPosition(float target_position, float feedforward_speed=0.f, float feedforward_intensity=0.f) {
    control_method_ = POSITION_SPEED;
    target_angle_ = target_position;
    feedforward_speed_ = feedforward_speed;
    feedforward_intensity_ = feedforward_intensity;
}
void Motor::SetSpeed(float target_speed, float feedforward_intensity=0.f) {
    control_method_ = SPEED;
    target_speed_ = target_speed;
    feedforward_intensity_ = feedforward_intensity;
}
void Motor::SetIntensity(float intensity) {
    control_method_ = TORQUE;
    output_intensity_ = intensity;
}
void Motor::Handle() {
    float intensity = 0.0f;
    switch (control_method_) {
        case POSITION_SPEED:{
            target_speed_ = regulate(spid_.calc(target_angle_, fdb_angle_)+feedforward_speed_,spid_.out_max_);
            intensity = regulate(ppid_.calc(target_speed_, fdb_speed_),ppid_.out_max_) + feedforward_intensity_;

				    break;
			}
        case SPEED:{
            fdb_speed_ = rotate_speed;
            intensity = ppid_.calc(target_speed_, fdb_speed_) + feedforward_intensity_;
            break;}
        case TORQUE:{
            intensity = output_intensity_;
            break;}
    }
    output_intensity_ = intensity;
}
Motor yawmotor(ratio,35.0f,20.0f,100.0f,100,2000,0.1,0.005,0.0,0.f,0.4,3,0.1);
Motor pitchmotor(ratio,80.f,50.f,3000,500.0,2000.0,0.1,0.001,0.0,0.f,1.0,3,0.1);