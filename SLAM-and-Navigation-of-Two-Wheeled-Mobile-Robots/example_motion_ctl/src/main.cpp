#include <Arduino.h>
#include <Esp32PcntEncoder.h>
#include <Esp32McpwmMotor.h>
#include <Pid_controller.h>
#include <Kinematics.h>
#include <Wire.h>
#include <MPU6050_light.h>
// 引入micro-ROS、wifi相关头文件
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
// 执行器
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>

#include <micro_ros_utilities/string_utilities.h>  // 引入字符串工具函数头文件，提供字符串处理功能



// 声明结构体对象
rcl_allocator_t allocator;  // 内存分配器，动态内存分配管理
rclc_support_t support;      //存储时钟、内存分配器和上下文等信息的结构体
rclc_executor_t executor;    // 任务执行器，负责调度和执行微ROS任务 
rcl_node_t node;            // ROS节点，ROS系统中的基本通信单元
rcl_subscription_t cmd_vel_subscriber; // 订阅器对象，用于接收cmd_vel消息
rcl_publisher_t odom_publisher; // 发布器对象，用于发布里程计消息
rcl_publisher_t imu_publisher; // 发布器对象，用于发布IMU消息
geometry_msgs__msg__Twist cmd_vel_msg; // 存储接收到的cmd_vel消息的结构体对象
nav_msgs__msg__Odometry odom_msg; // 存储要发布的里程计消息的结构体对象
sensor_msgs__msg__Imu imu_msg; // 存储要发布的IMU消息的结构体对象
rcl_timer_t odom_timer; // 定时器对象，用于定时发布里程计消息


Esp32PcntEncoder encoders[2]; // 创建一个数组用于存储两个编码器
Esp32McpwmMotor motor;
Pidcontroller pidControllers[2]; // 创建两个PID控制器实例
Kinematics kinematics;
MPU6050 mpu(Wire);

float target_linear_velocity = 0.0; // 目标线速度，单位为mm/s
float target_angular_velocity = 0.0; // 目标角速度，单位为rad/s

float right_wheel_speed = 0.0; // 右轮速度，单位为mm/s
float left_wheel_speed = 0.0;  // 左轮速度，单位为mm/s

constexpr float kDegToRad = PI / 180.0f;
constexpr float kGravity = 9.80665f;

void init_imu_message()
{
   imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "base_link");

   imu_msg.orientation_covariance[0] = -1.0;
   imu_msg.orientation_covariance[1] = 0.0;
   imu_msg.orientation_covariance[2] = 0.0;
   imu_msg.orientation_covariance[3] = 0.0;
   imu_msg.orientation_covariance[4] = 0.0;
   imu_msg.orientation_covariance[5] = 0.0;
   imu_msg.orientation_covariance[6] = 0.0;
   imu_msg.orientation_covariance[7] = 0.0;
   imu_msg.orientation_covariance[8] = 0.0;

   imu_msg.angular_velocity_covariance[0] = 0.05;
   imu_msg.angular_velocity_covariance[4] = 0.05;
   imu_msg.angular_velocity_covariance[8] = 0.02;

   imu_msg.linear_acceleration_covariance[0] = 0.5;
   imu_msg.linear_acceleration_covariance[4] = 0.5;
   imu_msg.linear_acceleration_covariance[8] = 0.5;
}

void fill_imu_message(int64_t stamp)
{
   imu_msg.header.stamp.sec = static_cast<int32_t>(stamp / 1000);
   imu_msg.header.stamp.nanosec = static_cast<uint32_t>((stamp % 1000) * 1000000);

   imu_msg.angular_velocity.x = mpu.getGyroX() * kDegToRad;
   imu_msg.angular_velocity.y = mpu.getGyroY() * kDegToRad;
   imu_msg.angular_velocity.z = mpu.getGyroZ() * kDegToRad;

   imu_msg.linear_acceleration.x = mpu.getAccX() * kGravity;
   imu_msg.linear_acceleration.y = mpu.getAccY() * kGravity;
   imu_msg.linear_acceleration.z = mpu.getAccZ() * kGravity;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
   //获取里程计数据
   odom_t odom_data = kinematics.get_odom();
   int64_t stamp = rmw_uros_epoch_millis(); // 获取当前时间戳，单位为毫秒
   odom_msg.header.stamp.sec = static_cast<int32_t>(stamp / 1000); // 设置里程计消息的时间戳，秒部分
   odom_msg.header.stamp.nanosec = static_cast<uint32_t>((stamp % 1000) * 1000000); // 设置里程计消息的时间戳，纳秒部分
   odom_msg.pose.pose.position.x = odom_data.x; // 设置里程计消息的位置信息，x坐标
   odom_msg.pose.pose.position.y = odom_data.y; // 设置里程计
   odom_msg.pose.pose.position.z = 0.0; // 设置里程计消息的位置信息，z坐标
   // 设置里程计消息的位姿信息，四元数表示
   odom_msg.pose.pose.orientation.w = cos(odom_data.theta / 2); // 设置里程计消息的位姿信息，四元数的w分量
   odom_msg.pose.pose.orientation.x = 0.0; // 设置里程计
   odom_msg.pose.pose.orientation.y = 0.0; // 设置里程计消息的位姿信息，四元数的y分量
   odom_msg.pose.pose.orientation.z = sin(odom_data.theta / 2); // 设置里程计消息的位姿信息，四元数的z分量
   

   odom_msg.twist.twist.linear.x = odom_data.linear_speed; // 设置里程计消息的速度信息，线速度x分量
   odom_msg.twist.twist.linear.y = 0.0; // 设置里程计消息的速度信息，线速度y分量
   odom_msg.twist.twist.linear.z = 0.0; // 设置里程计消息的速度信息，线速度z分量
   // 设置里程计消息的速度信息，角速度分量
   odom_msg.twist.twist.angular.z = odom_data.angular_speed; // 设置里程计消息的速度信息，角速度z分量

   //发布里程计
   if(rcl_publish(&odom_publisher, &odom_msg, NULL) != RCL_RET_OK) // 发布里程计消息，如果发布失败则打印错误信息
   {
      Serial.println("Failed to publish odometry message");
   }

   fill_imu_message(stamp);
   if(rcl_publish(&imu_publisher, &imu_msg, NULL) != RCL_RET_OK)
   {
      Serial.println("Failed to publish imu message");
   }

}

void twist_callback(const void * msg)
{
   // 将接收到的消息转换为geometry_msgs__msg__Twist类型
   const geometry_msgs__msg__Twist * twist_msg = (const geometry_msgs__msg__Twist *)msg;
   // 从消息中提取线速度和角速度
   target_linear_velocity = twist_msg->linear.x * 1000; // 线速度，单位为m/s
   target_angular_velocity = twist_msg->angular.z; // 角速度，单位为rad/s

   
   kinematics.inverse_kinematics(target_linear_velocity, target_angular_velocity, &left_wheel_speed, &right_wheel_speed); // 计算轮子速度
   //Serial.printf("left_wheel_speed=%f,right_wheel_speed=%f\n", left_wheel_speed, right_wheel_speed);

   pidControllers[0].setTargetSpeed(left_wheel_speed); // 设置PID控制器的目标速度
   pidControllers[1].setTargetSpeed(right_wheel_speed);
}



//  单独创建一个任务运行micro-ROS
void micro_ros_task(void *arg)
{
   // 1、设置传输协议并且延迟一段时间等待设置完成
   IPAddress agent_ip;
   agent_ip.fromString("192.168.31.81"); // 替换为实际的agent IP地址
   set_microros_wifi_transports("Xiaomi", "12345678", agent_ip, 8888); // 替换为实际的WiFi SSID、密码、agent IP地址和端口号  
   delay(2000); // 等待

   //2、初始化内存分配器
   allocator = rcl_get_default_allocator(); // 获取默认的内存分配器

   //3、初始化支持结构体
   rclc_support_init(&support, 0, NULL, &allocator); // 初始化支持结构体，传入命令行参数和内存分配器

   //4、创建ROS节点
   rclc_node_init_default(&node, "esp32_node", "", &support); // 创建一个名为"esp32_node"的ROS节点

   //5、初始化执行器
   unsigned int number_of_handles = 2; // 假设有2个任务需要执行
   rclc_executor_init(&executor, &support.context, number_of_handles, &allocator); // 初始化执行器，指定支持结构体、上下文、任务数量和内存分配

   // 6、创建订阅器并将回调函数添加到执行器中
   rclc_subscription_init_best_effort(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"); // 创建一个订阅器，订阅"cmd_vel"话题
   rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, twist_callback, ON_NEW_DATA); // 将订阅器和回调函数添加到执行器中，指定在接收到新数据时调用twist_callback函数   
   // 初始化msg
   odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom"); // 初始化里程计消息的frame_id字段
   odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_footprint"); // 初始化里程计消息的child_frame_id字段
   init_imu_message();
   // 初始化发布器
   rclc_publisher_init_best_effort(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"); // 创建一个发布器，发布"odom"话题
   rclc_publisher_init_best_effort(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data");

   // 初始化定时器
   rclc_timer_init_default(&odom_timer, &support, RCL_MS_TO_NS(50), timer_callback); // 创建一个定时器，周期为100ms，回调函数为timer_callback
   rclc_executor_add_timer(&executor, &odom_timer); // 将定时器添加到执行器中

   // 时间同步
   while(!rmw_uros_epoch_synchronized())
   {
      rmw_uros_sync_session(1000);
      delay(10);

   }
   //循环执行器   
   rclc_executor_spin(&executor); // 启动执行器，进入循环等待和处理任务
}








void setup()
{
   // 1.初始化串口
   Serial.begin(115200); // 初始化串口通信，设置通信速率为115200
   Wire.begin(18, 19);
   // 初始化电机
   motor.attachMotor(0, 22, 23); // 将电机0连接到引脚33和引脚25
   motor.attachMotor(1, 12, 13); // 将电机1连接到引脚26和引脚27
   // 2.设置编码器
   encoders[0].init(0, 32, 33); // 初始化第一个编码器，使用GPIO 32和33连接
   encoders[1].init(1, 26, 25); // 初始化第二个编码器，使用GPIO 26和25连接

   // 初始化pid参数
   // pidcontroller0
   pidControllers[0].update_pid(0.625, 0.125, 0);
   pidControllers[0].out_limit(-100, 100);

   // pidcontroller1
   pidControllers[1].update_pid(0.625, 0.125, 0);
   pidControllers[1].out_limit(-100, 100);

   // 初始化运动学参数
   kinematics.set_wheels_distance(175);     // 设置轮距，单位为mm
   kinematics.set_robot_param(0, 0.098572); // 设置机器人参数
   kinematics.set_robot_param(1, 0.098572);

   byte mpu_status = mpu.begin();
   Serial.print("MPU6050 status: ");
   Serial.println(mpu_status);
   while (mpu_status != 0)
   {
      delay(10);
   }
   Serial.println("Calculating MPU6050 offsets, keep robot still");
   delay(1000);
   mpu.calcOffsets(true, true);
   Serial.println("MPU6050 ready");


   //创建一个任务来运行micro-ROS
   xTaskCreate(micro_ros_task, "micro_ros_task", 10240, NULL, 1, NULL); // 创建一个名为"micro_ros_task"的任务，分配4096字节的堆栈，优先级为1
}

void loop()
{
   delay(10); // 等待10毫秒

   mpu.update();
   kinematics.update_motor_speed_and_encoders(encoders[0].getTicks(), encoders[1].getTicks(), millis()); // 更新电机速度和编码器数据
   motor.updateMotorSpeed(0, pidControllers[0].update(kinematics.get_robot_speed(0)));                   // 更新电机速度，传入当前速度
   motor.updateMotorSpeed(1, pidControllers[1].update(kinematics.get_robot_speed(1)));

   // 读取并打印两个编码器的计数器数值
   // Serial.printf("speed1=%f,speed2=%f\n", kinematics.get_robot_speed(0), kinematics.get_robot_speed(1));
   // Serial.printf("speed1=%f,speed2=%f\n", kinematics.get_robot_speed(0), kinematics.get_robot_speed(1));
   //Serial.printf("x=%f,y=%f,theta=%f\n", kinematics.get_odom().x, kinematics.get_odom().y, kinematics.get_odom().theta);
}

// // 电机速度控制函数
// void motorspeed()
// {
//    int16_t dt = millis() - last_update_time;

//    // motor.updateMotorSpeed(0, 70);
//    // motor.updateMotorSpeed(1, 70);

//    // 计算编码器差值
//    delta_ticks[0] = encoders[0].getTicks() - last_ticks[0];
//    delta_ticks[1] = encoders[1].getTicks() - last_ticks[1];

//    current_speed[0] = (delta_ticks[0] * 0.098572) / dt * 1000; // 计算当前速度，单位为mm/s
//    current_speed[1] = (delta_ticks[1] * 0.098572) / dt * 1000;

//    last_ticks[0] = encoders[0].getTicks();
//    last_ticks[1] = encoders[1].getTicks();
//    last_update_time = millis();

//    // 更新PID控制器并设置电机速度
//    motor.updateMotorSpeed(0, pidControllers[0].update(current_speed[0]));
//    motor.updateMotorSpeed(1, pidControllers[1].update(current_speed[1]));
// }
