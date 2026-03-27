#include "Kinematics.h"

void Kinematics::set_robot_param(uint8_t id, float per_pulse_mm)
{
    robot_param[id].per_pulse_mm = per_pulse_mm;
}

// 运动学正解：已知轮子速度，求机器人速度
void Kinematics::forward_kinematics(float left_wheel_speed, float right_wheel_speed, float *linear_velocity, float *angular_velocity)
{
    *linear_velocity = (left_wheel_speed + right_wheel_speed) / 2;
    *angular_velocity = (right_wheel_speed - left_wheel_speed) / wheels_distance_mm;
}

// 运动学逆解：已知机器人速度，求轮子速度
void Kinematics::inverse_kinematics(float linear_velocity, float angular_velocity, float *left_wheel_speed, float *right_wheel_speed)
{
    *left_wheel_speed = linear_velocity - (angular_velocity * wheels_distance_mm) / 2;
    *right_wheel_speed = linear_velocity + (angular_velocity * wheels_distance_mm) / 2;
}

// 更新电机速度和编码器
void Kinematics::update_motor_speed_and_encoders(int32_t left_encoder_ticks, int32_t right_encoder_ticks, uint64_t current_time_ms)
{
    int16_t dt = current_time_ms - last_update_time_ms;

    int16_t delta_ticks[2] = {0, 0}; // 存储上一次读的编码器数值

    float current_speed[2] = {0, 0};

    // 计算编码器差值
    delta_ticks[0] = left_encoder_ticks - robot_param[0].last_encoders_ticks;
    delta_ticks[1] = right_encoder_ticks - robot_param[1].last_encoders_ticks;

    robot_param[0].robot_speed = (delta_ticks[0] * 0.098572) / dt * 1000; // 计算当前速度，单位为mm/s
    robot_param[1].robot_speed = (delta_ticks[1] * 0.098572) / dt * 1000;

    robot_param[0].last_encoders_ticks = left_encoder_ticks;
    robot_param[1].last_encoders_ticks = right_encoder_ticks;
    last_update_time_ms = current_time_ms;


    update_odom(dt);
    
}



// 获取速度
int16_t Kinematics::get_robot_speed(uint8_t id)
{
    return robot_param[id].robot_speed;
}

// 设置轮距
void Kinematics::set_wheels_distance(float distance_mm)
{
    wheels_distance_mm = distance_mm;
}


odom_t& Kinematics::get_odom()
{
    return odom;

}


// 更新里程计数据
void Kinematics::update_odom(uint16_t dt)
{
    float dt_s = float(dt) / 1000.0; // 将时间转换为秒
    // 计算线速度和角速度
    
    this->forward_kinematics(robot_param[0].robot_speed, robot_param[1].robot_speed, &odom.linear_speed, &odom.angular_speed);
    odom.linear_speed = odom.linear_speed / 1000; // m/s
    odom.theta += odom.angular_speed * dt_s; 
    TransAngleInPI(odom.theta, odom.theta); // 将角度转换到[-PI, PI]范围内

    

    // 计算xy轴行走距离
    odom.x += odom.linear_speed * cos(odom.theta) * dt_s;
    odom.y += odom.linear_speed * sin(odom.theta) * dt_s;

}

void Kinematics::TransAngleInPI(float theta,  float& out_theta)
{
    if (theta > PI)
    {
        out_theta = theta - 2 * PI;
    }
    else if (theta < -PI)
    {
        out_theta = theta + 2 * PI;
    }
    else
    {
        out_theta = theta;
    }
}
