#include <navigation/utils/local_planner/mover.h>

mover::mover(float max_x_vel, float min_x_vel, float max_x_acc,
             float max_yaw_vel, float min_yaw_vel, float max_yaw_acc,
             float velocity_dec_line_offset_time,
             float velocity_dec_angle_offset_time,
             float recovery_mode_min_dist_timeout,
             float strighting_tolerance, float turning_tolerance,
             float strighting_stop_distance, float turning_stop_distance, float ignore_obstacle_dist,
             float obstacle_acc_ratio,
             float control_freq)
{
    set_kinematic_params(max_x_vel, max_yaw_vel);
    min_x_vel_ = min_x_vel;
    max_x_acc_ = max_x_acc;
    min_yaw_vel_ = min_yaw_vel;
    max_yaw_acc_ = max_yaw_acc;

    velocity_dec_line_offset_time_ = velocity_dec_line_offset_time;
    velocity_dec_angle_offset_time_ = velocity_dec_angle_offset_time;

    recovery_mode_min_dist_timeout_ = recovery_mode_min_dist_timeout;
    recovery_mode_counter_ = 0;

    set_tolerance_params(strighting_tolerance, turning_tolerance);
    turning_tolerance_ = turning_tolerance;
    strighting_stop_distance_ = strighting_stop_distance;
    turning_stop_distance_ = turning_stop_distance;
    ignore_obstacle_dist_ = ignore_obstacle_dist;

    obstacle_acc_ratio_ = obstacle_acc_ratio;

    dist_acc_controller_ = unique_ptr<acc_controller>(new acc_controller());
    angle_acc_controller_ = unique_ptr<acc_controller>(new acc_controller());

    obstacle_state_ = block_state::CLEAR;
    state_ = navigation_state::READY;
    last_state_ = state_;
    strighting_started_ = false;
    turning_started_ = false;
    dist_forward_ = true;
    angle_clockwise_ = true;

    last_dist_diff_ = INFINITY;
    last_angle_diff_ = INFINITY;

    control_freq_ = control_freq;
    vx_delta_ = (1.0f / control_freq_) * max_x_acc_;
    vyaw_delta_ = (1.0f / control_freq_) * max_yaw_acc_;
}

mover::~mover()
{
    if (dist_acc_controller_ != NULL)
    {
        dist_acc_controller_.reset();
    }

    if (angle_acc_controller_ != NULL)
    {
        angle_acc_controller_.reset();
    }
}

void mover::set_kinematic_params(float max_x_vel, float max_yaw_vel)
{
    max_x_vel_ = max_x_vel;
    max_yaw_vel_ = max_yaw_vel;
}

void mover::set_tolerance_params(float strighting_tolerance, float turning_tolerance)
{
    strighting_tolerance_ = strighting_tolerance;
    turning_tolerance_ = turning_tolerance;
}

bool mover::make_plan(geometry_msgs::PoseWithCovarianceStamped pose_init, float dist, float angle, bool ignore_obstacle, bool yaw_servo_mode)
{
    bool ret = false;

    if (state_ == navigation_state::READY)
    {
        pose_init_ = pose_init;
        dist_ = ((abs(dist) <= strighting_tolerance_) ? 0 : dist);

        yaw_servo_mode_ = yaw_servo_mode;

        if (yaw_servo_mode_)
        {
            yaw_target_ = angle;
            angle_ = angles::shortest_angular_distance(
                tf::getYaw(pose_init.pose.pose.orientation), (double)angle);
            angle_ = ((abs(angle_) <= turning_tolerance_) ? 0 : angle_);
        }
        else
        {
            angle_ = ((abs(angle) <= turning_tolerance_) ? 0 : angle);
        }

        current_dist_ = 0;
        current_angle_ = 0;
        recovery_mode_counter_ = 0;
        last_dist_diff_ = INFINITY;
        last_angle_diff_ = INFINITY;
        dist_forward_ = dist_ >= 0;
        angle_clockwise_ = angle_ <= 0;
        cycle_count_ = 0;
        ignore_obstacle_ = ignore_obstacle;

        twist_target_.linear.x = 0;
        twist_target_.angular.z = 0;

        ret |= dist_acc_controller_->make_plan(dist_, max_x_vel_, min_x_vel_, max_x_acc_, velocity_dec_line_offset_time_);

        if (!ret)
        {
            dist_ = 0;
            ret |= dist_acc_controller_->make_plan(dist_, max_x_vel_, min_x_vel_, max_x_acc_, velocity_dec_line_offset_time_);
        }

        ret &= angle_acc_controller_->make_plan(angle_, max_yaw_vel_, min_yaw_vel_, max_yaw_acc_, velocity_dec_angle_offset_time_);

        if (!ret)
        {
            angle_ = 0;
            ret |= dist_acc_controller_->make_plan(angle_, max_yaw_vel_, min_yaw_vel_, max_yaw_acc_, velocity_dec_angle_offset_time_);
        }

        if (ret)
        {
            obstacle_state_ = block_state::CLEAR;
            state_ = navigation_state::PROCESSING;
            strighting_started_ = false;
            turning_started_ = false;
        }
    }

    return ret;
}

bool mover::start_session()
{
    bool ret = false;

    if (state_ == navigation_state::PROCESSING)
    {
        state_ = navigation_state::STRIGHTING;

        ret = true;
    }

    return ret;
}

bool mover::pause_session()
{
    bool ret = false;

    if (state_ != navigation_state::READY &&
        state_ != navigation_state::PROCESSING &&
        state_ != navigation_state::REACHED &&
        state_ != navigation_state::PAUSING &&
        state_ != navigation_state::BLOCKED)
    {
        last_twist_target_ = twist_target_;

        last_state_ = state_;
        state_ = navigation_state::PAUSING;

        ret = true;
    }

    return ret;
}

bool mover::continue_session()
{
    bool ret = false;

    if (state_ == navigation_state::PAUSING)
    {
        state_ = last_state_;

        twist_target_ = last_twist_target_;

        ret = true;
    }

    return ret;
}

navigation_state mover::loop(geometry_msgs::PoseWithCovarianceStamped pose,
                             geometry_msgs::TwistWithCovarianceStamped odom_twist,
                             openmover_msgs::obstacle obstacle)
{
    if (state_ != navigation_state::READY &&
        state_ != navigation_state::REACHED &&
        state_ != navigation_state::PROCESSING &&
        state_ != navigation_state::PAUSING)
    {
        pose_ = pose;
        odom_twist_ = odom_twist;
        obstacle_ = obstacle;

        if ((strighting_started_ || turning_started_) && obstacle_.size > 0)
        {
            obstacle_state_ =
                ((state_ == navigation_state::BLOCKED) ? is_obstacle_detected(
                                                             last_state_before_blocked_)
                                                       : is_obstacle_detected(
                                                             state_));
        }
        else
        {
            obstacle_state_ = block_state::CLEAR;
        }

        if ((state_ == navigation_state::STRIGHTING ||
             state_ == navigation_state::STRIGHTING_RECOVERY) &&
            obstacle_state_ == block_state::CLEAR)
        {
            current_dist_ = (dist_forward_ ? 1.0 : -1.0) * sqrt(pow(pose_.pose.pose.position.x - pose_init_.pose.pose.position.x, 2) +
                                                                pow(pose_.pose.pose.position.y - pose_init_.pose.pose.position.y, 2));

            if (state_ == navigation_state::STRIGHTING &&
                (max(abs(twist_target_.linear.x), abs(odom_twist_.twist.twist.linear.x)) > 1e-3 || !strighting_started_) &&
                abs(dist_) > strighting_tolerance_)
            {
                twist_target_.linear.x = dist_acc_controller_->get_current_vel(current_dist_);
                twist_target_.angular.z = 0;
                strighting_started_ = true;
            }
            else
            {
                if (abs(odom_twist_.twist.twist.linear.x) >= min_x_vel_ / 2.0f &&
                    abs(dist_ - current_dist_) > last_dist_diff_)
                {
                    recovery_mode_counter_++;
                }
                else
                {
                    recovery_mode_counter_ = 0;
                }

                if (abs(dist_ - current_dist_) <= strighting_tolerance_ ||
                    recovery_mode_counter_ >= (int)abs(recovery_mode_min_dist_timeout_ * control_freq_) ||
                    abs(dist_) <= strighting_tolerance_)
                {
                    twist_target_.linear.x = 0;
                    twist_target_.angular.z = 0;

                    init_angle_ = tf::getYaw(pose_.pose.pose.orientation);
                    last_yaw_ = init_angle_;

                    recovery_mode_counter_ = 0;

                    state_ = navigation_state::TURNING;
                }
                else
                {
                    if (abs(dist_) > abs(current_dist_))
                    {
                        twist_target_.linear.x = (dist_forward_ ? 1 : -1) * min_x_vel_;
                        twist_target_.angular.z = 0;
                    }
                    else
                    {
                        twist_target_.linear.x = (dist_forward_ ? -1 : 1) * min_x_vel_;
                        twist_target_.angular.z = 0;
                    }

                    state_ = navigation_state::STRIGHTING_RECOVERY;
                }
            }

            last_dist_diff_ = abs(dist_ - current_dist_);
        }
        else if ((state_ == navigation_state::TURNING ||
                  state_ == navigation_state::TURNING_RECOVERY) &&
                 obstacle_state_ == block_state::CLEAR)
        {
            double current_yaw = tf::getYaw(pose_.pose.pose.orientation);

            if (last_yaw_ > 0.8 * M_PI && current_yaw < -0.8 * M_PI)
            {
                cycle_count_++;
            }
            else if (last_yaw_ < -0.8 * M_PI && current_yaw > 0.8 * M_PI)
            {
                cycle_count_--;
            }

            current_angle_ = cycle_count_ * 2 * M_PI + current_yaw - init_angle_;

            last_yaw_ = current_yaw;

            if (state_ == navigation_state::TURNING &&
                (max(abs(twist_target_.angular.z), abs(odom_twist_.twist.twist.angular.z)) > 1e-3 || !turning_started_) &&
                abs(angle_) > turning_tolerance_)
            {
                twist_target_.angular.z = angle_acc_controller_->get_current_vel(current_angle_);
                twist_target_.linear.x = 0;
                turning_started_ = true;
            }
            else
            {
                if (abs(odom_twist_.twist.twist.angular.z) >= min_yaw_vel_ / 2.0f &&
                    abs(angle_ - current_angle_) > last_angle_diff_)
                {
                    recovery_mode_counter_++;
                }
                else
                {
                    recovery_mode_counter_ = 0;
                }

                if ((!yaw_servo_mode_ && abs(angle_ - current_angle_) <= turning_tolerance_) ||
                    recovery_mode_counter_ >= (int)abs(recovery_mode_min_dist_timeout_ * control_freq_) ||
                    (!yaw_servo_mode_ && abs(angle_) <= turning_tolerance_) ||
                    (yaw_servo_mode_ && abs(angles::shortest_angular_distance(
                                            tf::getYaw(pose_.pose.pose.orientation), yaw_target_)) <= turning_tolerance_))
                {
                    twist_target_.linear.x = 0;
                    twist_target_.angular.z = 0;

                    obstacle_state_ = block_state::CLEAR;
                    state_ = navigation_state::REACHED;
                }
                else
                {
                    if (!yaw_servo_mode_)
                    {
                        if (abs(angle_) > abs(current_angle_))
                        {
                            twist_target_.angular.z = (angle_clockwise_ ? -1 : 1) * min_yaw_vel_;
                            twist_target_.linear.x = 0;
                        }
                        else
                        {
                            twist_target_.angular.z = (angle_clockwise_ ? 1 : -1) * min_yaw_vel_;
                            twist_target_.linear.x = 0;
                        }
                    }
                    else
                    {
                        if (angles::shortest_angular_distance(
                                tf::getYaw(pose_.pose.pose.orientation), yaw_target_) > 0)
                        {
                            twist_target_.angular.z = min_yaw_vel_;
                            twist_target_.linear.x = 0;
                        }
                        else
                        {
                            twist_target_.angular.z = -min_yaw_vel_;
                            twist_target_.linear.x = 0;
                        }
                    }

                    state_ = navigation_state::TURNING_RECOVERY;
                }
            }

            last_angle_diff_ = abs(angle_ - current_angle_);
        }
        else if (state_ == navigation_state::BLOCKED &&
                 obstacle_state_ == block_state::CLEAR)
        {
            twist_target_ = last_twist_target_;
            vx_delta_ /= obstacle_acc_ratio_;
            vyaw_delta_ /= obstacle_acc_ratio_;
            state_ = last_state_before_blocked_;
        }
        else if (state_ != navigation_state::BLOCKED &&
                 obstacle_state_ != block_state::CLEAR)
        {
            last_twist_target_ = twist_target_;
            vx_delta_ *= obstacle_acc_ratio_;
            vyaw_delta_ *= obstacle_acc_ratio_;
            twist_target_.linear.x = 0;
            twist_target_.angular.z = 0;
            last_state_before_blocked_ = state_;
            state_ = navigation_state::BLOCKED;
        }
    }
    else if (state_ == navigation_state::PAUSING)
    {
        twist_target_.linear.x = 0;
        twist_target_.angular.z = 0;
    }

    return state_;
}

geometry_msgs::Twist mover::get_current_twist()
{
    if (twist_target_.linear.x > odom_twist_.twist.twist.linear.x)
    {
        if (!dist_forward_ &&
            (state_ == navigation_state::STRIGHTING || state_ == navigation_state::STRIGHTING_RECOVERY))
        {
            twist_.linear.x = twist_target_.linear.x;
        }
        else
        {
            twist_.linear.x += vx_delta_;

            if (twist_.linear.x > twist_target_.linear.x)
            {
                twist_.linear.x = twist_target_.linear.x;
            }
        }
    }
    else if (twist_target_.linear.x < odom_twist_.twist.twist.linear.x)
    {
        if (dist_forward_ &&
            (state_ == navigation_state::STRIGHTING || state_ == navigation_state::STRIGHTING_RECOVERY))
        {
            twist_.linear.x = twist_target_.linear.x;
        }
        else
        {
            twist_.linear.x -= vx_delta_;

            if (twist_.linear.x < twist_target_.linear.x)
            {
                twist_.linear.x = twist_target_.linear.x;
            }
        }
    }

    if (twist_target_.angular.z > odom_twist_.twist.twist.angular.z)
    {
        if (angle_clockwise_ &&
            (state_ == navigation_state::TURNING || state_ == navigation_state::TURNING_RECOVERY))
        {
            twist_.angular.z = twist_target_.angular.z;
        }
        else
        {
            twist_.angular.z += vyaw_delta_;

            if (twist_.angular.z > twist_target_.angular.z)
            {
                twist_.angular.z = twist_target_.angular.z;
            }
        }
    }
    else if (twist_target_.angular.z < odom_twist_.twist.twist.angular.z)
    {
        if (!angle_clockwise_ &&
            (state_ == navigation_state::TURNING || state_ == navigation_state::TURNING_RECOVERY))
        {
            twist_.angular.z = twist_target_.angular.z;
        }
        else
        {
            twist_.angular.z -= vyaw_delta_;

            if (twist_.angular.z < twist_target_.angular.z)
            {
                twist_.angular.z = twist_target_.angular.z;
            }
        }
    }

    return twist_;
}

float mover::get_current_dist()
{
    return current_dist_;
}

float mover::get_current_angle()
{
    return current_angle_;
}

block_state mover::get_obstacle_state()
{
    return obstacle_state_;
}

void mover::reset_state()
{
    twist_target_.linear.x = 0;
    twist_target_.angular.z = 0;
    strighting_started_ = false;
    turning_started_ = false;
    ignore_obstacle_ = false;
    state_ = navigation_state::READY;
}

block_state mover::is_obstacle_detected(navigation_state state)
{
    block_state ret = block_state::CLEAR;

    if (state == navigation_state::STRIGHTING ||
        state == navigation_state::STRIGHTING_RECOVERY)
    {
        if (dist_forward_)
        {
            for (int i = 0; i < obstacle_.size; i++)
            {
                if (obstacle_.obstacle_forward[i] > 0 &&
                    obstacle_.obstacle_forward[i] < (ignore_obstacle_ ? ignore_obstacle_dist_ : strighting_stop_distance_) &&
                    obstacle_.obstacle_forward[i] < abs(dist_ - current_dist_))
                {
                    ret = block_state::FORWARD;
                    break;
                }
            }
        }
        else
        {
            for (int i = 0; i < obstacle_.size; i++)
            {
                if (obstacle_.obstacle_backward[i] > 0 &&
                    obstacle_.obstacle_backward[i] < (ignore_obstacle_ ? ignore_obstacle_dist_ : strighting_stop_distance_) &&
                    obstacle_.obstacle_backward[i] < abs(dist_ - current_dist_))
                {
                    ret = block_state::BACKWARD;
                    break;
                }
            }
        }
    }
    else if (state == navigation_state::TURNING ||
             state == navigation_state::TURNING_RECOVERY)
    {
        if (!angle_clockwise_)
        {
            for (int i = 0; i < obstacle_.size; i++)
            {
                if (obstacle_.obstacle_left[i] > 0 &&
                    obstacle_.obstacle_left[i] < (ignore_obstacle_ ? ignore_obstacle_dist_ : turning_stop_distance_))
                {
                    ret = block_state::LEFT;
                    break;
                }
            }
        }
        else
        {
            for (int i = 0; i < obstacle_.size; i++)
            {
                if (obstacle_.obstacle_right[i] > 0 &&
                    obstacle_.obstacle_right[i] < (ignore_obstacle_ ? ignore_obstacle_dist_ : turning_stop_distance_))
                {
                    ret = block_state::RIGHT;
                    break;
                }
            }
        }
    }

    return ret;
}