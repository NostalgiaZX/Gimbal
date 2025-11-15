
#include "../Inc/pid.h"
float regulate(float iout,float imax) {
        if (iout > imax) {
                iout = imax;
        } else if (iout < -imax) {
                iout = -imax;
        }
        return iout;

}
 PID::PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k):kp_(kp),ki_(ki),kd_(kd),i_max_(i_max),out_max_(out_max),d_filter_k_(d_filter_k) {
     output_=0.0f;
 }
void PID::reset(void)
 {
     fdb_ = 0.0f;
     err_ = err_sum_ = last_err_ = 0.0f;
     pout_ = iout_ = dout_ = last_dout_ = 0.0f;
     output_ = 0.0f;
 }
float PID::calc(float ref,float fdb) {
     ref_ = ref;
        fdb_ = fdb;
     last_err_ = err_;
     err_ = ref_-fdb_;
     err_sum_+=err_/10000.0f;
     pout_ =err_*kp_;
    iout_ =regulate(err_sum_*ki_,i_max_);
    last_dout_=dout_;
    dout_=(err_-last_err_)/50.0f*kd_;
    dout_=last_dout_*d_filter_k_+dout_*(1.0f-d_filter_k_);
    output_=pout_+iout_+dout_;
    output_=regulate(output_,out_max_);
    return output_;
 }

