#ifndef OBSTACLE_PARAMETERIZER_H
#define OBSTACLE_PARAMETERIZER_H

#include <ros/ros.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <unistd.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <boost/thread.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <common/common_state_code.h>
#include <navigation/navigation_state_code.h>
#include <localization/localization_sources.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Polygon.h>
#include <navigation/utils/math_utils.h>
#include <navigation/utils/cv_utils.h>
#include <navigation/utils/obstacle_tracker/obstacle_tracker.h>

using namespace std;

class obstacle_parameterizer
{
public:
    obstacle_parameterizer();
    obstacle_parameterizer(const bool &enable_visualization,
                           const int &update_freq,
                           const bool &enable_velocity_compensation,
                           const int &global_map_occupied_thres,
                           const int &global_map_min_cluster_size, const int &global_map_max_cluster_size, const float &global_map_point_nosie,
                           const float &global_map_wall_thres, const int &global_map_houghlines_thres, const float &global_map_houghlines_min_length, const float &global_map_houghlines_max_gap, const float &global_map_houghlines_merge_angle_thres,
                           const float &local_map_radius, const float &local_map_roi_radius, const float &local_map_resolution, const int &local_map_occupied_thres,
                           const int &local_map_min_cluster_size, const int &local_map_max_cluster_size, const float &local_map_point_nosie,
                           const float &local_map_wall_thres, const int &local_map_houghlines_thres, const float &local_map_houghlines_min_length, const float &local_map_houghlines_max_gap, const float &local_map_houghlines_merge_angle_thres,
                           const int &dynamic_obs_hit_global_map_thres, const float &dynamic_obs_lost_time_thres, const float &dynamic_obs_assoc_dist_thres, const float &dynamic_obs_radius_change_thres);
    ~obstacle_parameterizer();
    void initialize();
    ros::Time get_obstacle_time();
    tuple<ros::Time, vector<obstacle_description>> get_global_obstacles();
    tuple<ros::Time, vector<obstacle_description>> get_static_obstacles();
    tuple<ros::Time, vector<obstacle_state_with_id>> get_dynamic_obstacles();
    tuple<ros::Time, vector<vector<obstacle_state_with_id>>> get_dynamic_obstacle_predictions(const double &time_base, const double &period, const double &dt);
    tuple<ros::Time, vector<vector<obstacle_state_with_id>>> get_dynamic_obstacle_predictions(const double &time_base, const double &period, const int &step);

private:
    bool enable_visualization_ = true;

    int update_freq_ = 50;

    bool enable_velocity_compensation_ = false;

    int global_map_occupied_thres_ = 50;

    int global_map_min_cluster_size_ = 10;
    int global_map_max_cluster_size_ = 250000;
    float global_map_point_nosie_ = 0.01;

    float global_map_wall_thres_ = 10;
    int global_map_houghlines_thres_ = 20;
    float global_map_houghlines_min_length_ = 1.0;
    float global_map_houghlines_max_gap_ = 10.0;
    float global_map_houghlines_merge_angle_thres_ = 0.08;

    float local_map_radius_ = 1.5;
    float local_map_roi_radius_ = 1.0;
    float local_map_resolution_ = 0.05;
    int local_map_occupied_thres_ = 50;

    int local_map_min_cluster_size_ = 10;
    int local_map_max_cluster_size_ = 250000;
    float local_map_point_nosie_ = 0.01;

    float local_map_wall_thres_ = 2.5;
    int local_map_houghlines_thres_ = 20;
    float local_map_houghlines_min_length_ = 1.0;
    float local_map_houghlines_max_gap_ = 10.0;
    float local_map_houghlines_merge_angle_thres_ = 0.1;

    int dynamic_obs_hit_global_map_thres_ = 1;
    float dynamic_obs_lost_time_thres_ = 1.0;
    float dynamic_obs_assoc_dist_thres_ = 1.0;
    float dynamic_obs_radius_change_thres_ = 0.5;

    std::mutex mutex_obstacle_;

    std::mutex mutex_twist_;

    unique_ptr<ros::NodeHandle> node_handle_;
    ros::CallbackQueue queue_;

    unique_ptr<tf2_ros::Buffer> tf_buffer_;
    unique_ptr<tf2_ros::TransformListener> tf_listener_;

    ros::Subscriber scan_sub_;

    ros::Subscriber twist_sub_;

    ros::Publisher global_marker_array_pub_;
    ros::Publisher local_marker_array_pub_;
    ros::Publisher local_map_pub_;

    visualization_msgs::MarkerArray global_marker_array_;
    visualization_msgs::MarkerArray local_marker_array_;

    nav_msgs::OccupancyGrid global_map_;

    vector<obstacle_description> global_obstacles_;
    vector<obstacle_description> global_obstacles_focused_;

    vector<obstacle_description> local_static_obstacles_;
    vector<obstacle_state_with_id> local_dynamic_obstacles_;

    octomap::OcTree local_map_;

    sensor_msgs::LaserScan laser_data_;
    geometry_msgs::TransformStamped tf_map_to_lidar_link_;
    geometry_msgs::TransformStamped tf_map_to_base_link_;
    geometry_msgs::TransformStamped last_tf_map_to_lidar_link_;
    geometry_msgs::TransformStamped last_tf_map_to_base_link_;
    geometry_msgs::TransformStamped tf_base_link_to_lidar_link_;
    bool laser_data_received_ = false;

    geometry_msgs::TwistWithCovarianceStamped twist_data_;
    bool twist_data_received_ = false;

    unique_ptr<obstacle_tracker> dynamic_obstacle_tracker_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered;

    void twist_callback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg);
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void thread_loop();
    obstacle_description create_wall_obstacle(const geometry_msgs::Point32 &start,
                                              const geometry_msgs::Point32 &end,
                                              const float &max_height,
                                              const nav_msgs::OccupancyGrid &grid,
                                              const int &threshold);
    vector<obstacle_description> merge_wall_obstacles(
        const vector<obstacle_description> &wall_obstacles,
        const float &angle_diff_thres);
};

#endif