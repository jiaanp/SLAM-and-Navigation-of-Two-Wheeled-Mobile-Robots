#ifndef __PID_CONTROLLER__
#define __PID_CONTROLLER__

class Pidcontroller
{
    public:
        Pidcontroller() = default;
        Pidcontroller(float kp, float ki, float kd);

    private:
    //参数
        float target_speed_;
        float kp_;
        float ki_;
        float kd_;
        float dt_;
        float out_min;
        float out_max;
        // pid
        float error_;
        float error_sum_;
        float last_error_;
        float derror_;
        float intergral_up_= 2500; //设置积分上限

    public:
        float update(float current_speed);  //提供当前值
        void setTargetSpeed(float target_speed);
        void update_pid(float kp, float ki, float kd);
        void reset();
        void out_limit(float min, float max);

    


};



#endif  // __PID_CONTROLLER__;
