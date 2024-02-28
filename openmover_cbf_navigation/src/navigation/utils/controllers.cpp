#include <navigation/utils/controllers.h>

pid_controller::pid_controller()
{
    kp_ = 0;
    ki_ = 0;
    kd_ = 0;
    int_ = 0;
    last_err_ = 0;
}

pid_controller::~pid_controller()
{
    std::cout << "pid controller stopped." << std::endl;
}

bool pid_controller::set_params(float target_value, float kp, float ki, float kd, float max_value, float min_value)
{
    bool ret = false;

    target_value_ = target_value;

    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    max_value_ = max_value;
    min_value_ = min_value;

    ret = true;

    return ret;
}

float pid_controller::get_control_value(float control_input)
{
    float ret = 0;

    float current_err = control_input - target_value_;

    int_ += current_err;
    int_ = int_ > LONG_LONG_MAX ? LONG_LONG_MAX : (int_ < LONG_LONG_MIN ? LONG_LONG_MIN : int_);
    current_err = current_err > LONG_LONG_MAX ? LONG_LONG_MAX : (current_err < LONG_LONG_MIN ? LONG_LONG_MIN : current_err);

    ret = kp_ * current_err + ki_ * int_ + kd_ * (current_err - last_err_);
    last_err_ = current_err;

    ret = ret > max_value_ ? max_value_ : (ret < min_value_ ? min_value_ : ret);

    return ret;
}

void pid_controller::reset()
{
    int_ = 0;
    last_err_ = 0;
}

acc_controller::acc_controller()
{
    reset();
}

acc_controller::~acc_controller()
{
    std::cout << "acc controller stopped." << std::endl;
}

bool acc_controller::make_plan(float dist, float max_vel, float min_vel, float max_acc, float dec_offset_time)
{
    bool ret = false;

    forward_ = dist >= 0;
    dist_ = abs(dist);
    max_vel_ = abs(max_vel);
    min_vel_ = abs(min_vel);
    max_acc_ = abs(max_acc);

    if (abs(dist) < 1e-5)
    {
        return true;
    }

    while (true)
    {
        float trans_acc = (max_vel_ * max_vel_) / (2 * max_acc_);
        trans_rest_ = dist_ - 2 * trans_acc;

        if (trans_rest_ >= 0)
        {
            acc_end_x_ = trans_acc;
            dec_start_x_ = trans_rest_ + trans_acc;
            break;
        }

        max_vel_ /= 1.25;
    }

    ret = max_vel_ >= min_vel_;

    if (ret)
    {
        dec_offset_dist_ = dec_offset_time * max_vel_;
        dec_offset_dist_ = dec_offset_dist_ >= trans_rest_ ? 0 : dec_offset_dist_;
    }

    return ret;
}

float acc_controller::get_current_vel(float x)
{
    volatile float ret = 0;

    if (abs(dist_) < 1e-5)
    {
        ret = 0;
    }
    else
    {
        x = abs(x) + dec_offset_dist_;

        if (x == 0)
        {
            ret = min_vel_;
        }
        else if (x > 0 && x <= acc_end_x_)
        {
            ret = max(min_vel_, sqrt(2 * max_acc_ * x));
        }
        else if (x > acc_end_x_ && x <= dec_start_x_)
        {
            ret = max_vel_;
        }
        else if (x > dec_start_x_ && x <= dist_)
        {
            volatile float x_ = acc_end_x_ + dec_start_x_ - x;
            x_ = x_ < 0 ? 0 : x_;
            ret = sqrt(2 * max_acc_ * x_);
        }
        else
        {
            ret = 0;
        }

        ret = forward_ ? ret : -ret;
    }

    return ret;
}

float acc_controller::get_max_vel()
{
    volatile float ret = 0;

    ret = max_vel_;

    return ret;
}

void acc_controller::reset()
{
    max_vel_ = 0;
    min_vel_ = 0;
    max_acc_ = 0;
    acc_end_x_ = 0;
    dec_start_x_ = 0;
    forward_ = true;
}