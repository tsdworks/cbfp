// 2022-5-18 0.0.0.2A

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include "navigation/utils/global_planner/planner.h"
#include "navigation/utils/global_planner/dijkstra.h"
#include "navigation/utils/global_planner/grid_path.h"
#include "navigation/utils/global_planner/gradient_path.h"
#include "navigation/utils/global_planner/quadratic_calculator.h"
#include <navigation/utils/global_planner/quadratic_optimization.h>
#include <navigation/utils/global_planner/replanner.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

std::vector<float> shortest_distances;
std::vector<std::pair<double, double>> shortest_distance_coordinates;
geometry_msgs::Polygon footprint_polygon;

void compute_obstacle_shortest_distances(
    unsigned char *costs,
    int width, int height,
    float lethal_cost, float resolution,
    std::vector<float> &shortest_distances,
    std::vector<std::pair<double, double>> &shortest_distance_coordinates)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < width * height; ++i)
    {
        if (costs[i] >= lethal_cost)
        {
            int x = i % width;
            int y = i / width;
            cloud->push_back(pcl::PointXYZ(x * resolution, y * resolution, 0));
        }
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    shortest_distances.resize(width * height, FLT_MAX);
    shortest_distance_coordinates.resize(width * height, std::make_pair(0.0, 0.0));

    pcl::PointXYZ searchPoint;
    for (int i = 0; i < width * height; ++i)
    {
        if (costs[i] < lethal_cost)
        {
            searchPoint.x = (i % width) * resolution;
            searchPoint.y = (i / width) * resolution;
            searchPoint.z = 0;

            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);

            if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
            {
                shortest_distances[i] = std::sqrt(pointNKNSquaredDistance[0]);
                auto &p = cloud->points[pointIdxNKNSearch[0]];
                shortest_distance_coordinates[i] = std::make_pair(p.x / resolution, p.y / resolution);
            }
        }
        else
        {
            shortest_distances[i] = 0.0f;
            shortest_distance_coordinates[i] = std::make_pair(static_cast<double>(i % width), static_cast<double>(i / width));
        }
    }
}

void global_planner::outline_map(unsigned char *costarr, int nx, int ny, unsigned char value)
{
    unsigned char *pc = costarr;

    for (int i = 0; i < nx; i++)
        *pc++ = value;

    pc = costarr + (ny - 1) * nx;

    for (int i = 0; i < nx; i++)
        *pc++ = value;

    pc = costarr;

    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;

    pc = costarr + nx - 1;

    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

global_planner::global_planner() : costmap_(NULL), initialized_(false), allow_unknown_(true),
                                   p_calc_(NULL), planner_(NULL), path_maker_(NULL), orientation_filter_(NULL),
                                   potential_array_(NULL)
{
}

global_planner::global_planner(std::string name, costmap_2d::Costmap2D *costmap, std::string frame_id) : global_planner()
{
    // initialize the planner
    initialize(name, costmap, frame_id);
}

global_planner::~global_planner()
{
    if (p_calc_)
        delete p_calc_;
    if (planner_)
        delete planner_;
    if (path_maker_)
        delete path_maker_;
}

void global_planner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void global_planner::initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string frame_id)
{
    if (!initialized_)
    {
        ROS_INFO("Dijistra Global Planner started.");

        ros::NodeHandle private_nh("~/global_planner");
        ros::NodeHandle global_nh("~");
        costmap_ = costmap;
        frame_id_ = frame_id;

        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();

        private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);

        if (!old_navfn_behavior_)
            convert_offset_ = 0.5;
        else
            convert_offset_ = 0.0;

        bool use_quadratic;
        private_nh.param("use_quadratic", use_quadratic, true);

        if (use_quadratic)
            p_calc_ = new QuadraticCalculator(cx, cy);
        else
            p_calc_ = new PotentialCalculator(cx, cy);

        DijkstraExpansion *de = new DijkstraExpansion(p_calc_, cx, cy);
        if (!old_navfn_behavior_)
            de->setPreciseStart(true);
        planner_ = de;

        ROS_INFO("AAAAAAAA");

        ROS_INFO("AAA %d %d %lf AAA", costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(), costmap_->getResolution());

        compute_obstacle_shortest_distances(costmap_->getCharMap(),
                                            costmap_->getSizeInCellsX(),
                                            costmap_->getSizeInCellsY(),
                                            253,
                                            costmap_->getResolution(),
                                            shortest_distances,
                                            shortest_distance_coordinates);

        ROS_INFO("BBBBBBBB");

        de->shortest_distances_ = shortest_distances;
        de->shortest_distance_coordinates_ = shortest_distance_coordinates;

        ROS_INFO("CCCCCCCC");

        bool use_grid_path;
        private_nh.param("use_grid_path", use_grid_path, false);

        if (use_grid_path)
            path_maker_ = new GridPath(p_calc_);
        else
            path_maker_ = new GradientPath(p_calc_);

        orientation_filter_ = new OrientationFilter();

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

        private_nh.param("allow_unknown", allow_unknown_, true);
        planner_->setHasUnknown(allow_unknown_);
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        private_nh.param("publish_scale", publish_scale_, 100);
        private_nh.param("outline_map", outline_map_, true);

        // 足迹部分
        XmlRpc::XmlRpcValue footprint_param;
        global_nh.getParam("footprint", footprint_param);

        for (int i = 0; i < footprint_param.size(); i++)
        {
            std::vector<float> point_vector;

            for (int j = 0; j < footprint_param[i].size(); j++)
            {
                point_vector.push_back(double(footprint_param[i][j]));
            }

            geometry_msgs::Point32 point;

            point.x = point_vector[0];
            point.y = point_vector[1];

            footprint_polygon.points.push_back(point);

            ROS_WARN("footprint: %f %f", point.x, point.y);
        }

        initialized_ = true;
    }
    else
        ROS_WARN("No action.");
}

void global_planner::clear_robot_cell(const geometry_msgs::PoseStamped &global_pose, unsigned int mx, unsigned int my)
{
    if (!initialized_)
    {
        ROS_ERROR("Global Planner not ready.");
        return;
    }

    // set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

void global_planner::map_to_world(double mx, double my, double &wx, double &wy)
{
    wx = costmap_->getOriginX() + (mx + convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my + convert_offset_) * costmap_->getResolution();
}

bool global_planner::world_to_map(double wx, double wy, double &mx, double &my)
{
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

bool global_planner::make_plan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                               std::vector<geometry_msgs::PoseStamped> &plan)
{
    return make_plan(start, goal, default_tolerance_, plan);
}

bool global_planner::make_plan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                               double tolerance, std::vector<geometry_msgs::PoseStamped> &plan)
{
    boost::mutex::scoped_lock lock(mutex_);

    if (!initialized_)
    {
        ROS_ERROR("Global Planner not ready.");

        return false;
    }

    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    if (goal.header.frame_id != global_frame)
    {
        ROS_WARN("Goal pose error.");

        return false;
    }

    if (start.header.frame_id != global_frame)
    {
        ROS_WARN("Goal pose error.");

        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i))
    {
        ROS_WARN("Goal pose error.");

        return false;
    }
    if (old_navfn_behavior_)
    {
        start_x = start_x_i;
        start_y = start_y_i;
    }
    else
    {
        world_to_map(wx, wy, start_x, start_y);
    }

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i))
    {
        ROS_WARN_THROTTLE(1.0, "Goal pose error.");
        return false;
    }
    if (old_navfn_behavior_)
    {
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    }
    else
    {
        world_to_map(wx, wy, goal_x, goal_y);
    }

    clear_robot_cell(start, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    p_calc_->setSize(nx, ny);
    planner_->setSize(nx, ny);
    path_maker_->setSize(nx, ny);
    potential_array_ = new float[nx * ny];

    if (outline_map_)
        outline_map(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                     nx * ny * 2, potential_array_);

    if (!old_navfn_behavior_)
        planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);
    if (publish_potential_)
        publish_potential(potential_array_);

    if (found_legal)
    {
        if (get_plan_from_potential(start_x, start_y, goal_x, goal_y, goal, plan))
        {
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();

            plan.push_back(goal_copy);
        }
        else
        {
            ROS_ERROR("No path.");
        }
    }
    else
    {
        ROS_ERROR("No path.");
    }

    orientation_filter_->processPath(start, plan);

    publish_plan(plan);

    delete[] potential_array_;

    return !plan.empty();
}

void global_planner::publish_plan(const std::vector<geometry_msgs::PoseStamped> &path)
{
    if (!initialized_)
    {
        ROS_ERROR("Global Planner not ready.");

        return;
    }

    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    for (unsigned int i = 0; i < path.size(); i++)
    {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

bool global_planner::get_plan_from_potential(double start_x, double start_y, double goal_x, double goal_y,
                                             const geometry_msgs::PoseStamped &goal,
                                             std::vector<geometry_msgs::PoseStamped> &plan)
{
    if (!initialized_)
    {
        ROS_ERROR("Global Planner not ready.");

        return false;
    }

    std::string global_frame = frame_id_;

    plan.clear();

    std::vector<std::pair<float, float>> path;

    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path))
    {
        ROS_ERROR("No path.");

        return false;
    }

    // 第一步，平滑
    std::vector<std::pair<double, double>> path_optimized;
    DiscretePointsReferenceLineSmoother Smoother;
    Smoother.Smooth(path, path_optimized);

    // 第二步，重规划
    CollisionAndReplanParams params;
    params.costs = costmap_->getCharMap();
    params.costs = costmap_->getCharMap();
    params.costsLength = costmap_->getSizeInCellsY();
    params.costsWidth = costmap_->getSizeInCellsX();
    params.numIterMax = 5000;
    params.footprint = footprint_polygon;
    params.shortest_distance_obs = shortest_distances;
    params.shortest_distance_coordinates_obs = shortest_distance_coordinates;
    params.stepGradient = 0.01;
    params.resolution = costmap_->getResolution();
    CollisionAndReplan collisionAndReplan(params);
    bool result_collision_replan = collisionAndReplan.collision_replan(path_optimized);

    if (result_collision_replan)
    {
        ros::Time plan_time = ros::Time::now();
        for (int i = path_optimized.size() - 1; i >= 0; i--)
        {
            std::pair<float, float> point = path_optimized[i];

            double world_x, world_y;
            map_to_world(point.first, point.second, world_x, world_y);

            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = global_frame;
            pose.pose.position.x = world_x;
            pose.pose.position.y = world_y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            plan.push_back(pose);
        }
        if (old_navfn_behavior_)
        {
            plan.push_back(goal);
        }
    }
    return !plan.empty();
}

void global_planner::publish_potential(float *potential)
{
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    nav_msgs::OccupancyGrid grid;

    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;

    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
        float potential = potential_array_[i];
        if (potential < POT_HIGH)
        {
            if (potential > max)
            {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
        if (potential_array_[i] >= POT_HIGH)
        {
            grid.data[i] = -1;
        }
        else
        {
            if (fabs(max) < DBL_EPSILON)
            {
                grid.data[i] = -1;
            }
            else
            {
                grid.data[i] = potential_array_[i] * publish_scale_ / max;
            }
        }
    }

    potential_pub_.publish(grid);
}
