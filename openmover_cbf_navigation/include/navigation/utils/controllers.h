#ifndef CONTROLLERS
#define CONTROLLERS

#include <string>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <limits.h>

using namespace std;

#define DEBUG 0

class pid_controller
{
public:
    pid_controller();
    ~pid_controller();
    bool set_params(float target_value, float kp, float ki, float kd, float max_value, float min_value);
    float get_control_value(float control_input);
    void reset();

private:
    float target_value_;
    float kp_, ki_, kd_, max_value_, min_value_;
    volatile float int_, last_err_;
};

class acc_controller
{
public:
    acc_controller();
    ~acc_controller();
    bool make_plan(float dist, float max_vel, float min_vel, float max_acc, float dec_ratio);
    float get_current_vel(float x);
    float get_max_vel();
    void reset();

private:
    float dist_, max_vel_, min_vel_, max_acc_, acc_end_x_, rest_trans_, dec_start_x_;
    float trans_rest_, dec_offset_dist_;
    bool forward_;
};

#endif