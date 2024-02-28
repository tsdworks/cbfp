#ifndef MATH_UTILS
#define MATH_UTILS

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <angles/angles.h>
#include <openmover_msgs/obstacle.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <octomap/octomap.h>
#include <tuple>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

using namespace std;

enum obstacle_line_type
{
    FRONT = 0,
    REAR = 1,
    LEFT = 2,
    RIGHT = 3
};

struct obstacle_line
{
    obstacle_line_type type;
    geometry_msgs::Point32 p0;
    geometry_msgs::Point32 p1;
};

enum obstacle_type
{
    CONVEX = 0,
    CIRCLE = 1
};

boost::uuids::random_generator generator;

struct obstacle_description
{
    string uuid;
    obstacle_type type;
    float radius;
    geometry_msgs::Point32 center;
    vector<obstacle_line> boundaries;
    obstacle_line center_line;
};

geometry_msgs::Polygon transform_footprint(geometry_msgs::PoseWithCovarianceStamped &pose,
                                           geometry_msgs::Polygon &footprint);

tuple<vector<obstacle_line>, vector<float>> create_obstacle_lines(geometry_msgs::PoseWithCovarianceStamped &pose,
                                                                  geometry_msgs::Polygon &footprint,
                                                                  openmover_msgs::obstacle &obstacle);

int orientation(const geometry_msgs::Point32 &p, const geometry_msgs::Point32 &q, const geometry_msgs::Point32 &r);

bool on_segment(const geometry_msgs::Point32 &p, const geometry_msgs::Point32 &q, const geometry_msgs::Point32 &r);

bool is_intersect(const geometry_msgs::Point32 &p1,
                  const geometry_msgs::Point32 &p2,
                  const geometry_msgs::Point32 &p3,
                  const geometry_msgs::Point32 &p4);

bool is_point_in_rect(const geometry_msgs::Point32 &p, const geometry_msgs::Polygon &rect);

bool is_point_in_obstacle(const geometry_msgs::Point32 &p, const obstacle_description &ob);

bool is_line_intersect_rect(geometry_msgs::Polygon &polygon, vector<obstacle_line> &lines);

bool is_rect_intersect(const obstacle_description &rect1, const obstacle_description &rect2);

bool is_obstacle_intersect(const obstacle_description &ob1, const obstacle_description &ob2);

bool is_obstacle_in_obstacle(const obstacle_description &ob1, const obstacle_description &ob2);

bool is_obstacle_almost_in_obstacle(const obstacle_description &ob1, const obstacle_description &ob2);

float calc_path_point_orientation(geometry_msgs::Pose &pose, vector<geometry_msgs::Pose> &forward_points);

bool is_footprint_has_occupied_point(nav_msgs::OccupancyGridConstPtr grid,
                                     geometry_msgs::Polygon &footprint,
                                     const geometry_msgs::PoseStamped &pose,
                                     float stop_distance,
                                     bool reversed = false,
                                     int sampling_ratio = 1,
                                     float padding_ratio = 1.2);

geometry_msgs::Point32 calculate_polygon_center(const geometry_msgs::Polygon &polygon);

geometry_msgs::Point32 calculate_line_center(const obstacle_line &line);

float calculate_distance(const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2);

float calculate_distance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2);

float calculate_distance(const octomap::point3d &p1, const octomap::point3d &p2);

float calculate_distance(const cv::Point2f &p1, const cv::Point2f &p2);

float calculate_distance(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);

geometry_msgs::Point32 create_point(const float &x, const float &y);

geometry_msgs::Point32 create_point(const float &x, const float &y, const float &z);

geometry_msgs::Point create_pointd(const float &x, const float &y);

geometry_msgs::Point create_pointd(const float &x, const float &y, const float &z);

geometry_msgs::Point point_ros_to_rosd(const geometry_msgs::Point32 &p);

octomap::point3d point_ros_to_octomap(const geometry_msgs::Point32 &p);

geometry_msgs::Point32 point_octomap_to_ros(const octomap::point3d &p);

cv::Point2f point_ros_to_cv(const geometry_msgs::Point32 &p);

geometry_msgs::Point32 point_cv_to_ros(const cv::Point2f &p);

geometry_msgs::Point point_cv_to_rosd(const cv::Point2f &p);

geometry_msgs::Point32 point_pcl_xyz_to_ros(const pcl::PointXYZ &p);

pcl::PointXYZRGB point_ros_to_pcl_xyzrgb(const geometry_msgs::Point32 &p);

geometry_msgs::Point32 point_pcl_xyzrgb_to_ros(const pcl::PointXYZRGB &p);

geometry_msgs::Point point_pcl_xyz_to_rosd(const pcl::PointXYZ &p);

geometry_msgs::Point point_pcl_xyzrgb_to_rosd(const pcl::PointXYZRGB &p);

geometry_msgs::Polygon points_to_polygon(const vector<geometry_msgs::Point32> &points);

geometry_msgs::Polygon points_to_polygon(const vector<octomap::point3d> &points);

geometry_msgs::Polygon points_to_polygon(const vector<cv::Point2f> &points);

geometry_msgs::Polygon points_to_polygon(const geometry_msgs::Point32 &p0,
                                         const geometry_msgs::Point32 &p1,
                                         const geometry_msgs::Point32 &p2,
                                         const geometry_msgs::Point32 &p3);

geometry_msgs::Polygon points_to_polygon(const octomap::point3d &p0,
                                         const octomap::point3d &p1,
                                         const octomap::point3d &p2,
                                         const octomap::point3d &p3);

geometry_msgs::Polygon points_to_polygon(const cv::Point2f &p0,
                                         const cv::Point2f &p1,
                                         const cv::Point2f &p2,
                                         const cv::Point2f &p3);

tuple<geometry_msgs::Point32, geometry_msgs::Point32> get_box_aabb(const geometry_msgs::Polygon &polygon);

tuple<geometry_msgs::Point32, geometry_msgs::Point32> get_box_aabb(const obstacle_description &obstacle);

tuple<int, int, int, int> get_box_aabb_in_grid(const geometry_msgs::Polygon &polygon,
                                               const nav_msgs::OccupancyGrid &grid);

tuple<int, int, int, int> get_box_aabb_in_grid(const obstacle_description &obstacle,
                                               const nav_msgs::OccupancyGrid &grid);

geometry_msgs::Point32 grid_xy_to_position(const int &x, const int &y, const nav_msgs::OccupancyGrid &grid);

geometry_msgs::Point32 grid_index_to_position(const int &index, const nav_msgs::OccupancyGrid &grid);

pcl::PointXYZ grid_xy_to_pointcloud_xyz(const int &x, const int &y, const nav_msgs::OccupancyGrid &grid);

pcl::PointXYZ grid_index_to_pointcloud_xyz(const int &index, const nav_msgs::OccupancyGrid &grid);

pcl::PointXYZRGB grid_xy_to_pointcloud_xyzrgb(const int &x, const int &y, const nav_msgs::OccupancyGrid &grid);

pcl::PointXYZRGB grid_index_to_pointcloud_xyzrgb(const int &index, const nav_msgs::OccupancyGrid &grid);

geometry_msgs::Point32 grid_index_to_position(const int &index, const nav_msgs::OccupancyGrid &grid);

int grid_xy_to_grid_index(const int &x, const int &y, const nav_msgs::OccupancyGrid &grid);

tuple<int, int> grid_index_to_grid_xy(const int &index, const nav_msgs::OccupancyGrid &grid);

tuple<int, int> position_to_grid_xy(const geometry_msgs::Point32 &p, const nav_msgs::OccupancyGrid &grid);

tuple<int, int> position_to_grid_xy(const octomap::point3d &p, const nav_msgs::OccupancyGrid &grid);

tuple<int, int> position_to_grid_xy(const cv::Point2f &p, const nav_msgs::OccupancyGrid &grid);

tuple<int, int> position_to_grid_xy(const pcl::PointXYZ &p, const nav_msgs::OccupancyGrid &grid);

tuple<int, int> position_to_grid_xy(const pcl::PointXYZRGB &p, const nav_msgs::OccupancyGrid &grid);

int position_to_grid_index(const geometry_msgs::Point32 &p, const nav_msgs::OccupancyGrid &grid);

int position_to_grid_index(const octomap::point3d &p, const nav_msgs::OccupancyGrid &grid);

int position_to_grid_index(const cv::Point2f &p, const nav_msgs::OccupancyGrid &grid);

obstacle_line create_line(const geometry_msgs::Point32 &p0, const geometry_msgs::Point32 &p1);

obstacle_line create_line(const float &x0, const float &y0, const float &x1, const float &y1);

obstacle_line create_line(const geometry_msgs::Point &p0, const geometry_msgs::Point &p1);

obstacle_description create_obstacle(const geometry_msgs::Point32 &pos, const float &radius);

obstacle_description create_obstacle(const vector<obstacle_line> &boundaries);

float min_distance_to_line(const geometry_msgs::Point32 &p, const obstacle_line &line);

bool is_line_in_or_cross_circle(const geometry_msgs::Point32 &pos, const float &radius, const obstacle_line &line);

bool is_obstacle_in_roi(const geometry_msgs::Point32 &pos, const float &radius, const obstacle_description &obstacle);

vector<obstacle_description> get_obstacles_in_roi(const geometry_msgs::Point32 &pos, const float &radius, const vector<obstacle_description> &obstacles);

bool is_point_in_circle(const geometry_msgs::Point32 &center, const float &radius, const geometry_msgs::Point32 &p);

tuple<geometry_msgs::Point32, float> circle_from_two_points(const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2);

tuple<geometry_msgs::Point32, float> circle_from_three_points(const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2, const geometry_msgs::Point32 &p3);

tuple<geometry_msgs::Point32, float> welzl(vector<geometry_msgs::Point32> points, vector<geometry_msgs::Point32> points_recurs, size_t n);

tuple<geometry_msgs::Point32, float> find_min_bounding_circle(const vector<geometry_msgs::Point32> &points);

tuple<geometry_msgs::Point32, float> find_min_bounding_circle(const pcl::PointCloud<pcl::PointXYZ> &points);

tuple<geometry_msgs::Point32, float> find_min_bounding_circle(const pcl::PointCloud<pcl::PointXYZRGB> &points);

float min_dist_between_obstacles(const obstacle_description &ob1, const obstacle_description &ob2);

#endif