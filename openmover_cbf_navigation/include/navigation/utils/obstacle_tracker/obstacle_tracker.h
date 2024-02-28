#ifndef OBSTACLE_TRACKER
#define OBSTACLE_TRACKER

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <navigation/utils/math_utils.h>
#include <navigation/utils/obstacle_tracker/obstacle_ekf.h>
#include <map>
#include <navigation/utils/math_utils.h>
#include <localization/localization_sources.h>

using namespace std;

struct obstacle_state_with_id
{
    std_msgs::Header header;
    long long id;
    float x, y;
    float vx, vy;
    float ax, ay;
    float r;
};

class obstacle_tracker
{
public:
    obstacle_tracker(float obstacle_lost_time_threshold,
                     float obstacle_associate_dist_threshold,
                     float obstacle_radius_change_threshold);
    ~obstacle_tracker();
    vector<obstacle_state_with_id> track(const vector<obstacle_description> &obstacles, const ros::Time &stamp);
    tuple<vector<obstacle_state_with_id>, nav_msgs::Path> predict(const long long &id,
                                                                  const ros::Time time_base,
                                                                  const float &period,
                                                                  const float &dt);
    tuple<vector<obstacle_state_with_id>, nav_msgs::Path> predict(const long long &id,
                                                                  const ros::Time time_base,
                                                                  const float &period,
                                                                  const int &step);

private:
    long long associate(const obstacle_description &obstacle);

    long long id_ = 0;

    map<long long, float> obstacle_last_update_times_;
    map<long long, obstacle_ekf> obstacle_ekfs_;
    map<long long, float> obstacle_radius_;

    float obstacle_lost_time_threshold_ = 1.0;
    float obstacle_associate_dist_threshold_ = 1.0;
    float obstacle_radius_change_threshold_ = 0.4;
};

#endif