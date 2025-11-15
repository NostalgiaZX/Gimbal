
#include "motor.h"
float ratio = 19.2;
float Motor::getAngle() {
    return angle;
}
Motor::Motor(const float ratio_) : ratio(ratio_), spid_(2.f, 0.6f, 40.f, 15.0f, 35.0f,0.1f), ppid_(0.7f, 0.f, 56.f, 1.0f, 2.5f,0.06f) {
    control_method_ = POSITION_SPEED;
    target_angle_ = 0.f;
    feedforward_speed_ = 0.f;
    feedforward_intensity_ = 0.f;
    target_speed_ = 0.f;
    output_intensity_ = 0.f;
    spid_.reset();
    ppid_.reset();
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
    current = linermap(I, -16384, 16384, -20, 20);
    auto temp = static_cast<int16_t>((rdata[2] << 8) | rdata[3]);
    rotate_speed = (float)temp / 19.2;
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
Motor motor1(ratio);