// 2022-5-18 0.0.0.2A

#ifndef PLANNER_H
#define PLANNER_H

#define POT_HIGH 1.0e10

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <nav_msgs/GetPlan.h>
#include "potential_calculator.h"
#include "expander.h"
#include "traceback.h"
#include "orientation_filter.h"

class Expander;
class GridPath;

class global_planner
{
public:
    global_planner();
    global_planner(std::string name, costmap_2d::Costmap2D *costmap, std::string frame_id);
    ~global_planner();

    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string frame_id);

    bool make_plan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                   std::vector<geometry_msgs::PoseStamped> &plan);

    bool make_plan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, double tolerance,
                   std::vector<geometry_msgs::PoseStamped> &plan);

    bool compute_potential(const geometry_msgs::Point &world_point);

    bool get_plan_from_potential(double start_x, double start_y, double end_x, double end_y,
                                 const geometry_msgs::PoseStamped &goal,
                                 std::vector<geometry_msgs::PoseStamped> &plan);

    double get_point_potential(const geometry_msgs::Point &world_point);

    bool valid_point_potential(const geometry_msgs::Point &world_point);

    bool valid_point_potential(const geometry_msgs::Point &world_point, double tolerance);

    void publish_plan(const std::vector<geometry_msgs::PoseStamped> &path);

protected:
    costmap_2d::Costmap2D *costmap_;
    std::string frame_id_;
    ros::Publisher plan_pub_;
    bool initialized_, allow_unknown_;

private:
    void map_to_world(double mx, double my, double &wx, double &wy);

    bool world_to_map(double wx, double wy, double &mx, double &my);

    void clear_robot_cell(const geometry_msgs::PoseStamped &global_pose, unsigned int mx, unsigned int my);

    void publish_potential(float *potential);

    double planner_window_x_, planner_window_y_, default_tolerance_;
    boost::mutex mutex_;
    ros::ServiceServer make_plan_srv_;

    PotentialCalculator *p_calc_;
    Expander *planner_;
    Traceback *path_maker_;
    OrientationFilter *orientation_filter_;

    bool publish_potential_;
    ros::Publisher potential_pub_;
    int publish_scale_;

    void outline_map(unsigned char *costarr, int nx, int ny, unsigned char value);

    float *potential_array_;
    unsigned int start_x_, start_y_, end_x_, end_y_;

    bool old_navfn_behavior_;
    float convert_offset_;

    bool outline_map_;
};

#endif
