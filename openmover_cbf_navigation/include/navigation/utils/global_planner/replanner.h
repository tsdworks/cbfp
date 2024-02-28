#ifndef REPLANNER_H
#define REPLANNER_H

#include <vector>
#include <utility>
#include <math.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

struct CollisionAndReplanParams
{
    unsigned char *costs;
    float costsLength;
    float costsWidth;
    // float robotLength;
    // float robotWidth;
    geometry_msgs::Polygon footprint;
    float resolution;
    float stepGradient;
    int numIterMax;

    std::vector<float> shortest_distance_obs;
    std::vector<std::pair<double, double>> shortest_distance_coordinates_obs;
};

struct PathSegmentReplaned
{
    std::vector<std::pair<double, double>> points;
    int start_index;
    int end_index;
};

class CollisionAndReplan
{
public:
    CollisionAndReplan(const CollisionAndReplanParams &params);
    ~CollisionAndReplan();
    bool collision_replan(std::vector<std::pair<double, double>> &path);

private:
    std::vector<int> collisionDetection(const std::vector<std::pair<double, double>> &path_world_optimized);
    std::vector<double> calc_avg_orientations(const std::vector<std::pair<double, double>> &Path);
    double calc_path_point_orientation(const std::pair<double, double> &current, const std::vector<std::pair<double, double>> &forward_points);
    void generateConvexPolygon(float centerX, float centerY, float orientation, geometry_msgs::Polygon footprint, int &minX, int &minY, int &maxX, int &maxY, geometry_msgs::Polygon &obstacleDetectionArea, float size);
    bool is_point_in_rect(geometry_msgs::Point32 &p, geometry_msgs::Polygon &rect);
    bool checkCollisionOBB(unsigned char *costs, int minX, int minY, int maxX, int maxY, geometry_msgs::Polygon &obstacleDetectionArea);

    std::vector<PathSegmentReplaned> generateCollisionSegments(const std::vector<int> &collision_points, const std::vector<std::pair<double, double>> &path, int pathpoint_extended, int max_pathpoint_distance);

    bool PathCollisionReplan(std::vector<PathSegmentReplaned> &collision_segments, std::vector<std::pair<double, double>> &path);
    std::vector<std::pair<double, double>> pathReplan(const std::vector<std::pair<double, double>> &path_collision_segments);
    std::pair<double, double> smoothnessTerm(const std::pair<double, double> &x_im2, const std::pair<double, double> &x_im1,
                                             const std::pair<double, double> &x_i, const std::pair<double, double> &x_ip1, const std::pair<double, double> &x_ip2);
    int optimizationPath(std::vector<std::pair<double, double>> &path, double movement_angle_range);

    unsigned char *costs_;
    float costsLength_;
    float costsWidth_;
    // float robotLength_;
    // float robotWidth_;
    geometry_msgs::Polygon footprint_;
    float resolution_;
    float stepGradient_;
    int numIterMax_;
    std::vector<float> shortest_distance_obs_;
    std::vector<std::pair<double, double>> shortest_distance_coordinates_obs_;
};

#endif