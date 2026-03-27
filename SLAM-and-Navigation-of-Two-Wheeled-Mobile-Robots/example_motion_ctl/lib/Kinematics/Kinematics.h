#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include <Arduino.h>

struct RobotParam
{
    int16_t robot_speed;            // 机器人速度，单位为mm/s
    float per_pulse_mm;             // 每个脉冲对应的距离，单位为mm
    int64_t last_encoders_ticks; // 上一次读的编码器数值
};


struct odom_t
{
    float x;
    float y;
    float theta;
    float linear_speed;
    float angular_speed;

};

/*
运动学正解：已知轮子速度，求机器人速度
运动学逆解：已知机器人速度，求轮子速度
*/
class Kinematics
{
public:
    Kinematics() = default;
    ~Kinematics() = default;
    void set_robot_param(uint8_t id, float per_pulse_mm);
    void forward_kinematics(float left_wheel_speed, float right_wheel_speed, float *linear_velocity, float *angular_velocity);
    void inverse_kinematics(float linear_velocity, float angular_velocity, float *left_wheel_speed, float *right_wheel_speed);
    // 更新电机速度和编码器
    void update_motor_speed_and_encoders(int32_t left_encoder_ticks, int32_t right_encoder_ticks, uint64_t current_time_ms);
    // 获取速度
    int16_t get_robot_speed(uint8_t id);
    // 设置轮距
    void set_wheels_distance(float distance_mm);

    void update_odom(uint16_t dt);
    odom_t& get_odom();
    void TransAngleInPI(float theta,  float& out_theta);

private:
    RobotParam robot_param[2];
    uint64_t last_update_time_ms = 0;
    float wheels_distance_mm = 0; // 轮距，单位为mm
    odom_t odom; // 里程计数据
};

#endif // __KINEMATICS_H__