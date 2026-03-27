#include "Pid_controller.h"
#include "Arduino.h"

Pidcontroller::Pidcontroller(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

// PID控制器的更新函数，计算当前速度与目标速度之间的误差，并根据PID算法计算输出值
float Pidcontroller::update(float current_speed)
{
    error_ = target_speed_ - current_speed; // 计算误差
    error_sum_ += error_;                  // 累积误差
    if (error_sum_ > intergral_up_)
        error_sum_ = intergral_up_; // 积分上限
    if (error_sum_ < -intergral_up_)
        error_sum_ = -intergral_up_; // 积分下限
    derror_ = last_error_ - error_;  // 计算误差的变化率
    last_error_ = error_;            // 更新误差

    float output = kp_ * error_ + ki_ * error_sum_ + kd_ * derror_;

    if (output > out_max)
        output = out_max; // 输出上限
    if (output < out_min)
        output = out_min; // 输出下限

    return output; // 返回计算得到的输出值
}

void Pidcontroller::update_pid(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void Pidcontroller::setTargetSpeed(float target_speed)
{
    target_speed_ = target_speed;
}

void Pidcontroller::reset()
{
    error_ = 0;
    error_sum_ = 0;
    last_error_ = 0;
    derror_ = 0;
    kp_ = 0;
    ki_ = 0;
    kd_ = 0;
    intergral_up_ = 2500;
    out_min = 0;
    out_max = 0;
}

void Pidcontroller::out_limit(float min, float max)
{
    out_min = min;
    out_max = max;
}
