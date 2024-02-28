#ifndef MOVER
#define MOVER

#include <string>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <limits.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <angles/angles.h>
#include <openmover_msgs/obstacle.h>
#include <tf/tf.h>
#include <navigation/utils/controllers.h>
#include <navigation/navigation_state_code.h>

using namespace std;
using namespace navigation_ns;

class mover
{
public:
    mover(float max_x_vel, float min_x_vel, float max_x_acc,
          float max_yaw_vel, float min_yaw_vel, float max_yaw_acc,
          float velocity_dec_line_offset_time,
          float velocity_dec_angle_offset_time,
          float recovery_mode_min_dist_timeout,
          float strighting_tolerance, float turning_tolerance,
          float strighting_stop_distance, float turning_stop_distance, float ignore_obstacle_dist,
          float obstacle_acc_ratio,
          float control_freq);
    ~mover();
    void set_kinematic_params(float max_x_vel, float max_yaw_vel);
    void set_tolerance_params(float strighting_tolerance, float turning_tolerance);
    void set_obstacle_stop_params(float strighting_stop_distance, float turning_stop_distance);
    bool make_plan(geometry_msgs::PoseWithCovarianceStamped pose_init, float dist, float angle, bool ignore_obstacle, bool yaw_servo_mode = false);
    bool start_session();
    bool pause_session();
    bool continue_session();
    navigation_state loop(geometry_msgs::PoseWithCovarianceStamped pose,
                          geometry_msgs::TwistWithCovarianceStamped odom_twist,
                          openmover_msgs::obstacle obstacle);
    geometry_msgs::Twist get_current_twist();
    float get_current_dist();
    float get_current_angle();
    block_state get_obstacle_state();
    void reset_state();

private:
    float max_x_vel_, min_x_vel_, max_x_acc_;
    float max_yaw_vel_, min_yaw_vel_, max_yaw_acc_;
    float velocity_dec_line_offset_time_;
    float velocity_dec_angle_offset_time_;
    float recovery_mode_min_dist_timeout_;
    float strighting_tolerance_, turning_tolerance_;
    float strighting_stop_distance_;
    float turning_stop_distance_;
    float ignore_obstacle_dist_;

    float obstacle_acc_ratio_;

    geometry_msgs::PoseWithCovarianceStamped pose_init_;
    int recovery_mode_counter_;
    float dist_;
    float angle_;
    float last_dist_diff_;
    float last_angle_diff_;
    bool dist_forward_;
    bool angle_clockwise_;
    bool ignore_obstacle_;
    bool yaw_servo_mode_;
    float yaw_target_;

    geometry_msgs::PoseWithCovarianceStamped pose_;
    openmover_msgs::obstacle obstacle_;
    float current_dist_;
    float current_angle_;
    float init_angle_;
    float last_yaw_;
    int cycle_count_;

    geometry_msgs::Twist twist_target_;
    geometry_msgs::Twist last_twist_target_;
    geometry_msgs::Twist twist_;
    geometry_msgs::TwistWithCovarianceStamped odom_twist_;
    float control_freq_;
    float vx_delta_;
    float vyaw_delta_;

    unique_ptr<acc_controller> dist_acc_controller_;
    unique_ptr<acc_controller> angle_acc_controller_;

    block_state obstacle_state_;
    navigation_state state_;
    navigation_state last_state_before_blocked_;
    navigation_state last_state_;

    bool strighting_started_;
    bool turning_started_;

    block_state is_obstacle_detected(navigation_state state);
};

#endif