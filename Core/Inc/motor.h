
#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>
#include <pid.h>
#include <cmath>
#define PI 3.14159f
float linermap(float in,float inmin,float inmax,float outmin,float outmax);
float trans360(float in);
float pitchforwardinten(float angle);
typedef enum ControlMethod {
    TORQUE,
    SPEED,
    POSITION_SPEED,
}CM;
class Motor {
private:
    CM control_method_;
    const float ratio;
    float angle=0.f;
    float delta_angle=0.f;
    float ecd_angle=0.f;
    float last_ecd_angle=0.f;
    float delta_ecd_angle=0.f;
    float rotate_speed=0.f;
    float current=0.f;
    float temp=0.f;
public:

    PID spid_, ppid_;
    float target_angle_, fdb_angle_;
    float target_speed_, fdb_speed_, feedforward_speed_;
    float feedforward_intensity_, output_intensity_;
    explicit Motor(const float ratio_,float skp=2.f,float ski=0.6f,float skd=40.f,float skimax=15.0f,float soutmax=35.0f,float sdfilterk=0.1f,float pkp=0.7f,float pki=0.f,float pkd=56.f,float pkimax=1.f,float poutmax=1.f,float pdfilterk=0.06f);
    void canrxmsgcallback(const uint8_t rdata[8]);

    void SetPosition(float target_position, float feedforward_speed, float feedforward_intensity);
    void SetSpeed(float target_speed, float feedforward_intensity);
    void SetIntensity(float intensity);

    void Handle();

    float getAngle();
};
#endif //MOTOR_H