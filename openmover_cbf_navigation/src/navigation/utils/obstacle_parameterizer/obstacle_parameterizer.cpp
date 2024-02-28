#include <navigation/utils/obstacle_parameterizer/obstacle_parameterizer.h>

obstacle_parameterizer::obstacle_parameterizer() : local_map_(0.05) {}

obstacle_parameterizer::obstacle_parameterizer(const bool &enable_visualization,
                                               const int &update_freq,
                                               const bool &enable_velocity_compensation,
                                               const int &global_map_occupied_thres,
                                               const int &global_map_min_cluster_size, const int &global_map_max_cluster_size, const float &global_map_point_nosie,
                                               const float &global_map_wall_thres, const int &global_map_houghlines_thres, const float &global_map_houghlines_min_length, const float &global_map_houghlines_max_gap, const float &global_map_houghlines_merge_angle_thres,
                                               const float &local_map_radius, const float &local_map_roi_radius, const float &local_map_resolution, const int &local_map_occupied_thres,
                                               const int &local_map_min_cluster_size, const int &local_map_max_cluster_size, const float &local_map_point_nosie,
                                               const float &local_map_wall_thres, const int &local_map_houghlines_thres, const float &local_map_houghlines_min_length, const float &local_map_houghlines_max_gap, const float &local_map_houghlines_merge_angle_thres,
                                               const int &dynamic_obs_hit_global_map_thres, const float &dynamic_obs_lost_time_thres, const float &dynamic_obs_assoc_dist_thres, const float &dynamic_obs_radius_change_thres) : local_map_(0.05)
{
    enable_visualization_ = enable_visualization;

    update_freq_ = update_freq;

    enable_velocity_compensation_ = enable_velocity_compensation;

    global_map_occupied_thres_ = global_map_occupied_thres;

    global_map_min_cluster_size_ = global_map_min_cluster_size;
    global_map_max_cluster_size_ = global_map_max_cluster_size;
    global_map_point_nosie_ = global_map_point_nosie;

    global_map_wall_thres_ = global_map_wall_thres;
    global_map_houghlines_thres_ = global_map_houghlines_thres;
    global_map_houghlines_min_length_ = global_map_houghlines_min_length;
    global_map_houghlines_max_gap_ = global_map_houghlines_max_gap;
    global_map_houghlines_merge_angle_thres_ = global_map_houghlines_merge_angle_thres;

    local_map_radius_ = local_map_radius;
    local_map_roi_radius_ = local_map_roi_radius;
    local_map_resolution_ = local_map_resolution;
    local_map_occupied_thres_ = local_map_occupied_thres;

    local_map_min_cluster_size_ = local_map_min_cluster_size;
    local_map_max_cluster_size_ = local_map_max_cluster_size;
    local_map_point_nosie_ = local_map_point_nosie;

    local_map_wall_thres_ = local_map_wall_thres;
    local_map_houghlines_thres_ = local_map_houghlines_thres;
    local_map_houghlines_min_length_ = local_map_houghlines_min_length;
    local_map_houghlines_max_gap_ = local_map_houghlines_max_gap;
    local_map_houghlines_merge_angle_thres_ = local_map_houghlines_merge_angle_thres;

    dynamic_obs_hit_global_map_thres_ = dynamic_obs_hit_global_map_thres;
    dynamic_obs_lost_time_thres_ = dynamic_obs_lost_time_thres;
    dynamic_obs_assoc_dist_thres_ = dynamic_obs_assoc_dist_thres;
    dynamic_obs_radius_change_thres_ = dynamic_obs_radius_change_thres;
}

obstacle_parameterizer::~obstacle_parameterizer()
{
    cout << "obstacle parameterizer stopped." << endl;
}

void obstacle_parameterizer::initialize()
{
    node_handle_ = make_unique<ros::NodeHandle>();

    node_handle_->setCallbackQueue(&queue_);

    srand(static_cast<unsigned int>(time(0)));

    tf_buffer_ = make_unique<tf2_ros::Buffer>();
    tf_listener_ = make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    try
    {
        tf_base_link_to_lidar_link_ = tf_buffer_->lookupTransform(
            LOCALIZATION_LIDAR_LINK,
            LOCALIZATION_BASE_LINK,
            ros::Time(0), ros::Duration(60.0));
    }
    catch (...)
    {
        cout << "no tf data from base_link to lidar_link" << endl;
    }

    ros::SubscribeOptions twist_sub_opt =
        ros::SubscribeOptions::create<geometry_msgs::TwistWithCovarianceStamped>(
            "twist",
            2,
            boost::bind(&obstacle_parameterizer::twist_callback, this, _1),
            ros::VoidPtr(),
            &queue_);

    twist_sub_ = node_handle_->subscribe(twist_sub_opt);

    ros::SubscribeOptions scan_sub_opt =
        ros::SubscribeOptions::create<sensor_msgs::LaserScan>(
            "scan",
            2,
            boost::bind(&obstacle_parameterizer::scan_callback, this, _1),
            ros::VoidPtr(),
            &queue_);

    scan_sub_ = node_handle_->subscribe(scan_sub_opt);

    global_marker_array_pub_ = node_handle_->advertise<visualization_msgs::MarkerArray>("global_obstacle_markers", 1, true);
    local_marker_array_pub_ = node_handle_->advertise<visualization_msgs::MarkerArray>("local_obstacle_markers", 1, true);
    local_map_pub_ = node_handle_->advertise<nav_msgs::OccupancyGrid>("local_obstacle_map", 1, true);

    dynamic_obstacle_tracker_ = make_unique<obstacle_tracker>(dynamic_obs_lost_time_thres_,
                                                              dynamic_obs_assoc_dist_thres_,
                                                              dynamic_obs_radius_change_thres_);

    try
    {
        auto grid = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(60.0));

        assert(grid != NULL);

        if (grid != NULL)
        {
            global_map_ = *grid;
        }
    }
    catch (...)
    {
        cout << "could not get global map." << endl;

        return;
    }

    cout << "global obstacle parameterizer started." << endl;

    int marker_id = 0;

    auto width = global_map_.info.width;
    auto height = global_map_.info.height;
    auto resolution = global_map_.info.resolution;

    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (unsigned int y = 0; y < height; y++)
    {
        for (unsigned int x = 0; x < width; x++)
        {
            if (global_map_.data[grid_xy_to_grid_index(x, y, global_map_)] > global_map_occupied_thres_)
            {
                cloud->points.push_back(grid_xy_to_pointcloud_xyzrgb(x, y, global_map_));
            }
        }
    }

    tree = pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);
    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(resolution * 2);
    ec.setMinClusterSize(global_map_min_cluster_size_);
    ec.setMaxClusterSize(global_map_max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    cloud_clustered = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (const auto &indices : cluster_indices)
    {
        for (int i = 0; i < indices.indices.size(); i++)
        {
            (*cloud)[indices.indices[i]].r = rand() % 256;
            (*cloud)[indices.indices[i]].g = rand() % 256;
            (*cloud)[indices.indices[i]].b = rand() % 256;

            (*cloud)[indices.indices[i]].x += (i % 2) * global_map_point_nosie_;
            (*cloud)[indices.indices[i]].y += (i % 2) * global_map_point_nosie_;

            cloud_clustered->push_back((*cloud)[indices.indices[i]]);
        }

        if (cloud_clustered->size() < 3)
        {
            cloud_clustered->clear();

            continue;
        }

        pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;
        convex_hull.setInputCloud(cloud_clustered);
        convex_hull.setComputeAreaVolume(true);
        convex_hull.setDimension(2);
        pcl::PolygonMesh hull;
        convex_hull.reconstruct(hull);

        pcl::PointCloud<pcl::PointXYZRGB> hull_points;
        pcl::fromPCLPointCloud2(hull.cloud, hull_points);

        if (hull_points.points.size() > 2)
        {
            double hull_area = convex_hull.getTotalArea();
            double cloud_area = cloud_clustered->points.size() * resolution * resolution;
            double ratio = hull_area / cloud_area;

            if (ratio > global_map_wall_thres_)
            {
                cv::Mat grid_map = cv::Mat::zeros(height, width, CV_8UC1);

                for (const auto &point : cloud_clustered->points)
                {
                    auto xy = position_to_grid_xy(point, global_map_);

                    if (get<0>(xy) >= 0 && get<0>(xy) < width && get<1>(xy) >= 0 && get<1>(xy) < height)
                    {
                        grid_map.at<uchar>(get<1>(xy), get<0>(xy)) = 255;
                    }
                }

                cv::Mat skeleton = skeletonize(grid_map);

                vector<cv::Vec4i> lines;
                cv::HoughLinesP(skeleton, lines, 1, CV_PI / 180,
                                global_map_houghlines_thres_,
                                global_map_houghlines_min_length_,
                                global_map_houghlines_max_gap_);

                vector<obstacle_description> wall_obstacles;

                for (auto &line : lines)
                {
                    auto obstacle = create_wall_obstacle(
                        grid_xy_to_position(line[0], line[1], global_map_),
                        grid_xy_to_position(line[2], line[3], global_map_),
                        2 * local_map_resolution_, global_map_, global_map_occupied_thres_);

                    wall_obstacles.push_back(obstacle);
                }

                auto merged_wall_obstacles = merge_wall_obstacles(wall_obstacles, global_map_houghlines_merge_angle_thres_);

                for (auto &obstacle : merged_wall_obstacles)
                {
                    global_obstacles_.push_back(obstacle);

                    if (enable_visualization_)
                    {
                        visualization_msgs::Marker marker;
                        marker.header.frame_id = LOCALIZATION_MAP;
                        marker.header.stamp = ros::Time::now();
                        marker.ns = "global_obstacles";
                        marker.id = marker_id++;
                        marker.type = visualization_msgs::Marker::LINE_STRIP;
                        marker.action = visualization_msgs::Marker::ADD;
                        marker.color.r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
                        marker.color.g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
                        marker.color.b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
                        marker.color.a = 1.0;
                        marker.scale.x = 0.02;

                        for (int i = 0; i <= obstacle.boundaries.size(); i++)
                        {
                            marker.points.push_back(point_ros_to_rosd(
                                obstacle.boundaries[i % obstacle.boundaries.size()].p0));
                        }

                        global_marker_array_.markers.push_back(marker);
                    }
                }
            }
            else
            {
                obstacle_description obstacle;
                vector<obstacle_line> obstacle_lines;

                for (int i = 0; i < hull_points.points.size(); i++)
                {
                    obstacle_line line;
                    line.p0 = point_pcl_xyzrgb_to_ros(hull_points.points[i]);
                    line.p1 = point_pcl_xyzrgb_to_ros(hull_points.points[(i + 1) % hull_points.points.size()]);

                    obstacle_lines.push_back(line);
                }

                obstacle = create_obstacle(obstacle_lines);

                global_obstacles_.push_back(obstacle);

                if (enable_visualization_)
                {
                    visualization_msgs::Marker marker;
                    marker.header.frame_id = LOCALIZATION_MAP;
                    marker.header.stamp = ros::Time::now();
                    marker.ns = "global_obstacles";
                    marker.id = marker_id++;
                    marker.type = visualization_msgs::Marker::LINE_STRIP;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 0.02;
                    marker.color.r = 1.0;
                    marker.color.a = 1.0;

                    for (int i = 0; i <= obstacle.boundaries.size(); i++)
                    {
                        marker.points.push_back(point_ros_to_rosd(
                            obstacle.boundaries[i % obstacle.boundaries.size()].p0));
                    }

                    global_marker_array_.markers.push_back(marker);
                }
            }
        }

        // *cloud += *cloud_clustered;
        cloud_clustered->clear();
    }

    if (enable_visualization_)
    {
        global_marker_array_pub_.publish(global_marker_array_);
    }

    cout << "number of global obstacles: " << global_obstacles_.size() << "." << endl;

    local_map_.setResolution(local_map_resolution_);

    boost::thread scan_thread_(boost::bind(&obstacle_parameterizer::thread_loop, this));

    cout << "local obstacle parameterizer started." << endl;

    return;
}

ros::Time obstacle_parameterizer::get_obstacle_time()
{
    lock_guard<mutex> lock(mutex_obstacle_);

    return laser_data_.header.stamp;
}

tuple<ros::Time, vector<obstacle_description>> obstacle_parameterizer::get_global_obstacles()
{
    return {laser_data_.header.stamp, global_obstacles_};
}

tuple<ros::Time, vector<obstacle_description>> obstacle_parameterizer::get_static_obstacles()
{
    vector<obstacle_description> ret;

    lock_guard<mutex> lock(mutex_obstacle_);

    // ret = local_static_obstacles_;
    ret = global_obstacles_focused_;

    // ret.insert(ret.end(), local_static_obstacles_.begin(), local_static_obstacles_.end());

    return {laser_data_.header.stamp, ret};
}

tuple<ros::Time, vector<obstacle_state_with_id>> obstacle_parameterizer::get_dynamic_obstacles()
{
    lock_guard<mutex> lock(mutex_obstacle_);

    return {laser_data_.header.stamp, local_dynamic_obstacles_};
}

tuple<ros::Time, vector<vector<obstacle_state_with_id>>> obstacle_parameterizer::get_dynamic_obstacle_predictions(
    const double &time_base,
    const double &period,
    const double &dt)
{
    vector<vector<obstacle_state_with_id>> ret;

    lock_guard<mutex> lock(mutex_obstacle_);

    for (const auto &obstacle : local_dynamic_obstacles_)
    {
        vector<obstacle_state_with_id> predictions;

        auto tracker_predictions = dynamic_obstacle_tracker_->predict(
            obstacle.id,
            ros::Time().fromSec(time_base),
            (float)period,
            (float)dt);

        predictions = get<0>(tracker_predictions);

        ret.push_back(predictions);
    }

    return {laser_data_.header.stamp, ret};
}

tuple<ros::Time, vector<vector<obstacle_state_with_id>>> obstacle_parameterizer::get_dynamic_obstacle_predictions(
    const double &time_base,
    const double &period,
    const int &step)
{
    vector<vector<obstacle_state_with_id>> ret;

    lock_guard<mutex> lock(mutex_obstacle_);

    for (const auto &obstacle : local_dynamic_obstacles_)
    {
        vector<obstacle_state_with_id> predictions;

        auto tracker_predictions = dynamic_obstacle_tracker_->predict(
            obstacle.id,
            ros::Time().fromSec(time_base),
            (float)period,
            step);

        predictions = get<0>(tracker_predictions);

        ret.push_back(predictions);
    }

    return {laser_data_.header.stamp, ret};
}

obstacle_description obstacle_parameterizer::create_wall_obstacle(const geometry_msgs::Point32 &start,
                                                                  const geometry_msgs::Point32 &end,
                                                                  const float &max_height,
                                                                  const nav_msgs::OccupancyGrid &grid,
                                                                  const int &threshold)
{
    obstacle_description ret;

    ret.type = obstacle_type::CONVEX;

    float dist = calculate_distance(start, end);

    cv::Point2f pt1(start.x, start.y);
    cv::Point2f pt2(end.x, end.y);
    cv::Point2f direction = pt2 - pt1;
    direction = direction / cv::norm(direction);
    cv::Point2f normal(-direction.y, direction.x);

    cv::Point2f p1 = pt1 + max_height * normal;
    cv::Point2f p2 = pt2 + max_height * normal;
    cv::Point2f p3 = pt2 - max_height * normal;
    cv::Point2f p4 = pt1 - max_height * normal;

    geometry_msgs::Polygon polygon = points_to_polygon(p1, p2, p3, p4);

    float area_occupied = 0;

    auto grid_iter = get_box_aabb_in_grid(polygon, grid);

    for (int x = get<0>(grid_iter); x <= get<1>(grid_iter); ++x)
    {
        for (int y = get<2>(grid_iter); y <= get<3>(grid_iter); ++y)
        {
            int index = grid_xy_to_grid_index(x, y, grid);

            geometry_msgs::Point32 point_in_map = grid_index_to_position(index, grid);

            if (grid.data[index] > threshold &&
                is_point_in_rect(point_in_map, polygon))
            {
                area_occupied += grid.info.resolution * grid.info.resolution;
            }
        }
    }

    float min_height = max_height;
    float min_diff = INFINITY;

    for (double height = 0.01; height <= max_height; height += 0.01)
    {
        float area_rect = dist * height;

        p1 = pt1 + height * normal;
        p2 = pt2 + height * normal;
        p3 = pt2 - height * normal;
        p4 = pt1 - height * normal;

        if (area_occupied != 0 && area_rect != 0)
        {
            float diff = abs(area_rect - area_occupied);

            if (diff < min_diff)
            {
                min_diff = diff;
                min_height = height;
            }
        }
    }

    p1 = pt1 + min_height * normal;
    p2 = pt2 + min_height * normal;
    p3 = pt2 - min_height * normal;
    p4 = pt1 - min_height * normal;

    obstacle_line l1, l2, l3, l4;
    l1.p0 = point_cv_to_ros(p1);
    l1.p1 = point_cv_to_ros(p2);
    l2.p0 = point_cv_to_ros(p2);
    l2.p1 = point_cv_to_ros(p3);
    l3.p0 = point_cv_to_ros(p3);
    l3.p1 = point_cv_to_ros(p4);
    l4.p0 = point_cv_to_ros(p4);
    l4.p1 = point_cv_to_ros(p1);

    ret.boundaries = {l1, l2, l3, l4};

    ret.center_line.p0 = start;
    ret.center_line.p1 = end;

    return ret;
}

vector<obstacle_description> obstacle_parameterizer::merge_wall_obstacles(
    const vector<obstacle_description> &wall_obstacles,
    const float &angle_diff_thres)
{
    vector<obstacle_description> ret;

    vector<bool> processed(wall_obstacles.size(), false);
    vector<vector<int>> clusters;

    for (size_t i = 0; i < wall_obstacles.size(); ++i)
    {
        if (processed[i])
        {
            continue;
        }

        vector<int> current_cluster = {static_cast<int>(i)};

        processed[i] = true;

        bool cluster_grown = true;

        while (cluster_grown)
        {
            cluster_grown = false;

            for (size_t j = 0; j < wall_obstacles.size(); ++j)
            {
                if (processed[j] || j == i)
                {
                    continue;
                }

                for (int idx : current_cluster)
                {
                    auto &ob1 = wall_obstacles[idx];
                    auto &ob2 = wall_obstacles[j];

                    if (is_obstacle_intersect(ob1, ob2))
                    {
                        float angle1 = atan2(ob1.center_line.p1.y - ob1.center_line.p0.y, ob1.center_line.p1.x - ob1.center_line.p0.x);
                        float angle2 = atan2(ob2.center_line.p1.y - ob2.center_line.p0.y, ob2.center_line.p1.x - ob2.center_line.p0.x);
                        float angle_diff = abs(angles::shortest_angular_distance(angle1, angle2));

                        if (angle_diff < angle_diff_thres)
                        {
                            current_cluster.push_back(j);
                            processed[j] = true;
                            cluster_grown = true;

                            break;
                        }
                    }
                }
            }
        }

        clusters.push_back(current_cluster);
    }

    bool clusters_merged = true;

    while (clusters_merged)
    {
        clusters_merged = false;

        for (size_t i = 0; i < clusters.size() && !clusters_merged; ++i)
        {
            for (size_t j = i + 1; j < clusters.size(); ++j)
            {
                bool can_merge = false;

                for (int idx_i : clusters[i])
                {
                    for (int idx_j : clusters[j])
                    {
                        auto &ob1 = wall_obstacles[idx_i];
                        auto &ob2 = wall_obstacles[idx_j];

                        if (is_obstacle_intersect(ob1, ob2))
                        {
                            float angle1 = atan2(ob1.center_line.p1.y - ob1.center_line.p0.y, ob1.center_line.p1.x - ob1.center_line.p0.x);
                            float angle2 = atan2(ob2.center_line.p1.y - ob2.center_line.p0.y, ob2.center_line.p1.x - ob2.center_line.p0.x);
                            float angle_diff = abs(angles::shortest_angular_distance(angle1, angle2));

                            if (angle_diff < angle_diff_thres)
                            {
                                can_merge = true;

                                break;
                            }
                        }
                    }

                    if (can_merge)
                    {
                        break;
                    }
                }

                if (can_merge)
                {
                    clusters[i].insert(clusters[i].end(), clusters[j].begin(), clusters[j].end());
                    clusters.erase(clusters.begin() + j);
                    clusters_merged = true;

                    break;
                }
            }
        }
    }

    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> merged_obstacles_clouds;

    for (const auto &cluster : clusters)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int idx : cluster)
        {
            for (const auto &line : wall_obstacles[idx].boundaries)
            {
                current_cloud->push_back(point_ros_to_pcl_xyzrgb(line.p0));
            }
        }

        merged_obstacles_clouds.push_back(current_cloud);
    }

    vector<obstacle_description> obstacles_temp;

    for (const auto &cloud : merged_obstacles_clouds)
    {
        if (cloud->size() > 2)
        {
            pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;
            convex_hull.setInputCloud(cloud);
            convex_hull.setComputeAreaVolume(true);
            convex_hull.setDimension(2);
            pcl::PolygonMesh hull;
            convex_hull.reconstruct(hull);

            pcl::PointCloud<pcl::PointXYZRGB> hull_points;
            pcl::fromPCLPointCloud2(hull.cloud, hull_points);

            if (hull_points.size() > 2)
            {
                obstacle_description obstacle;
                vector<obstacle_line> obstacle_lines;

                for (int i = 0; i < hull_points.points.size(); i++)
                {
                    obstacle_line line;
                    line.p0 = point_pcl_xyzrgb_to_ros(hull_points.points[i]);
                    line.p1 = point_pcl_xyzrgb_to_ros(hull_points.points[(i + 1) % hull_points.points.size()]);

                    obstacle_lines.push_back(line);
                }

                obstacle = create_obstacle(obstacle_lines);

                obstacles_temp.push_back(obstacle);
            }
        }
    }

    vector<bool> deleted(obstacles_temp.size(), false);

    for (int i = 0; i < obstacles_temp.size(); i++)
    {
        if (deleted[i])
        {
            continue;
        }

        for (int j = 0; j < obstacles_temp.size(); j++)
        {
            if (deleted[j] || i == j)
            {
                continue;
            }

            if (is_obstacle_in_obstacle(obstacles_temp[j], obstacles_temp[i]) ||
                is_obstacle_almost_in_obstacle(obstacles_temp[j], obstacles_temp[i]))
            {
                deleted[j] = true;
            }
        }
    }

    for (int i = 0; i < obstacles_temp.size(); i++)
    {
        if (!deleted[i])
        {
            ret.push_back(obstacles_temp[i]);
        }
    }

    return ret;
}

void obstacle_parameterizer::thread_loop()
{
    double duration = 1.0 / update_freq_;

    while (ros::ok())
    {
        queue_.callAvailable(ros::WallDuration(duration));
    }

    return;
}

void obstacle_parameterizer::twist_callback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg)
{
    lock_guard<mutex> lock(mutex_twist_);

    twist_data_ = *msg;

    twist_data_received_ = true;
}

void obstacle_parameterizer::scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    lock_guard<mutex> lock(mutex_obstacle_);

    laser_data_ = *msg;

    try
    {
        tf_map_to_lidar_link_ = tf_buffer_->lookupTransform(
            LOCALIZATION_MAP,
            LOCALIZATION_LIDAR_LINK,
            ros::Time(0));

        tf_map_to_base_link_ = tf_buffer_->lookupTransform(
            LOCALIZATION_MAP,
            LOCALIZATION_BASE_LINK,
            ros::Time(0));
    }
    catch (...)
    {
        cout << "no tf data from map to lidar, skipped." << endl;

        return;
    }

    if (enable_velocity_compensation_ && twist_data_received_)
    {
        lock_guard<mutex> lock(mutex_twist_);

        double time_diff = laser_data_.header.stamp.toSec() - tf_map_to_base_link_.header.stamp.toSec();

        double yaw = tf::getYaw(tf_map_to_base_link_.transform.rotation);

        tf_map_to_base_link_.transform.translation.x += twist_data_.twist.twist.linear.x * time_diff * cos(yaw);
        tf_map_to_base_link_.transform.translation.y += twist_data_.twist.twist.linear.x * time_diff * sin(yaw);
        tf_map_to_base_link_.transform.rotation = tf::createQuaternionMsgFromYaw(yaw + twist_data_.twist.twist.angular.z * time_diff);

        tf2::Transform tf_base_link_to_lidar_link;
        tf2::fromMsg(tf_base_link_to_lidar_link_.transform, tf_base_link_to_lidar_link);
        tf2::Vector3 linear_velocity_base(twist_data_.twist.twist.linear.x,
                                          twist_data_.twist.twist.linear.y,
                                          twist_data_.twist.twist.linear.z);
        tf2::Vector3 linear_velocity_lidar = tf_base_link_to_lidar_link.getBasis() * linear_velocity_base;

        yaw = tf::getYaw(tf_map_to_lidar_link_.transform.rotation);
        tf_map_to_lidar_link_.transform.translation.x += linear_velocity_lidar.x() * time_diff * cos(yaw);
        tf_map_to_lidar_link_.transform.translation.y += linear_velocity_lidar.x() * time_diff * sin(yaw);
        tf_map_to_lidar_link_.transform.rotation = tf::createQuaternionMsgFromYaw(yaw + twist_data_.twist.twist.angular.z * time_diff);
    }

    octomap::point3d sensor_origin(tf_map_to_base_link_.transform.translation.x,
                                   tf_map_to_base_link_.transform.translation.y,
                                   0.0);

    for (unsigned int i = 0; i < laser_data_.ranges.size(); ++i)
    {
        float angle = laser_data_.angle_min + laser_data_.angle_increment * i;
        float range = laser_data_.ranges[i];

        if (range < laser_data_.range_min || range > laser_data_.range_max)
        {
            continue;
        }

        tf2::Vector3 point_lidar_link(range * cos(angle), range * sin(angle), 0.0);

        tf2::Transform transform;
        tf2::fromMsg(tf_map_to_lidar_link_.transform, transform);
        tf2::Vector3 point_map = transform * point_lidar_link;
        octomap::point3d end_point = octomap::point3d(point_map.x(), point_map.y(), 0.0);

        local_map_.insertRay(sensor_origin, end_point, local_map_radius_);
    }

    for (octomap::OcTree::leaf_iterator it = local_map_.begin_leafs(), end = local_map_.end_leafs(); it != end; ++it)
    {
        if (local_map_.isNodeOccupied(*it))
        {
            octomap::point3d point(it.getX(), it.getY(), it.getZ());

            if ((point - sensor_origin).norm() > local_map_radius_)
            {
                local_map_.deleteNode(it.getKey());
            }
        }
    }

    last_tf_map_to_lidar_link_ = tf_map_to_lidar_link_;
    last_tf_map_to_base_link_ = tf_map_to_base_link_;

    nav_msgs::OccupancyGrid local_map_grid;
    local_map_grid.header.frame_id = LOCALIZATION_MAP;
    local_map_grid.header.stamp = ros::Time::now();

    double grid_size = 2 * local_map_radius_;
    double resolution = local_map_.getResolution();
    local_map_grid.info.resolution = resolution;
    local_map_grid.info.width = grid_size / resolution;
    local_map_grid.info.height = grid_size / resolution;
    auto width = local_map_grid.info.width;
    auto height = local_map_grid.info.height;

    local_map_grid.info.origin.position.x = tf_map_to_base_link_.transform.translation.x - grid_size / 2;
    local_map_grid.info.origin.position.y = tf_map_to_base_link_.transform.translation.y - grid_size / 2;
    local_map_grid.info.origin.position.z = 0;
    local_map_grid.info.origin.orientation.w = 1.0;

    local_map_grid.data.resize(width * height, 0);

    for (octomap::OcTree::leaf_iterator it = local_map_.begin_leafs(), end = local_map_.end_leafs(); it != end; ++it)
    {
        if (it != NULL)
        {
            octomap::point3d point(it.getX(), it.getY(), it.getZ());

            if ((point - sensor_origin).norm() <= local_map_radius_ && local_map_.isNodeOccupied(*it))
            {
                local_map_grid.data[position_to_grid_index(create_point(it.getX(), it.getY()), local_map_grid)] = 100;
            }
        }
    }

    if (enable_visualization_)
    {
        local_map_pub_.publish(local_map_grid);
    }

    cloud->clear();

    for (unsigned int y = 0; y < height; y++)
    {
        for (unsigned int x = 0; x < width; x++)
        {
            if (local_map_grid.data[grid_xy_to_grid_index(x, y, local_map_grid)] > local_map_occupied_thres_)
            {
                cloud->points.push_back(grid_xy_to_pointcloud_xyzrgb(x, y, local_map_grid));
            }
        }
    }

    if (cloud->empty())
    {
        return;
    }

    global_obstacles_focused_.clear();
    local_static_obstacles_.clear();
    local_dynamic_obstacles_.clear();

    int marker_id = 0;

    tree->setInputCloud(cloud);
    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(resolution * 2);
    ec.setMinClusterSize(local_map_min_cluster_size_);
    ec.setMaxClusterSize(local_map_max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    cloud_clustered->clear();

    vector<obstacle_description> local_dynamic_obstacles;

    if (enable_visualization_)
    {
        for (int i = 0; i < local_marker_array_.markers.size(); i++)
        {
            local_marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;
        }

        local_marker_array_pub_.publish(local_marker_array_);

        local_marker_array_.markers.clear();
    }

    for (const auto &indices : cluster_indices)
    {
        for (int i = 0; i < indices.indices.size(); i++)
        {
            (*cloud)[indices.indices[i]].r = rand() % 256;
            (*cloud)[indices.indices[i]].g = rand() % 256;
            (*cloud)[indices.indices[i]].b = rand() % 256;

            (*cloud)[indices.indices[i]].x += (i % 2) * local_map_point_nosie_;
            (*cloud)[indices.indices[i]].y += (i % 2) * local_map_point_nosie_;

            cloud_clustered->push_back((*cloud)[indices.indices[i]]);
        }

        if (cloud_clustered->size() < 2)
        {
            cloud_clustered->clear();

            continue;
        }

        pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;
        convex_hull.setInputCloud(cloud_clustered);
        convex_hull.setComputeAreaVolume(true);
        convex_hull.setDimension(2);
        pcl::PolygonMesh hull;
        convex_hull.reconstruct(hull);

        pcl::PointCloud<pcl::PointXYZRGB> hull_points;
        pcl::fromPCLPointCloud2(hull.cloud, hull_points);

        if (hull_points.points.size() > 2)
        {
            bool is_dynamic_obstacle = false;

            obstacle_description obstacle_temp;
            vector<obstacle_line> obstacle_lines_temp;

            for (int i = 0; i < hull_points.points.size(); i++)
            {
                obstacle_line line;
                line.p0 = point_pcl_xyzrgb_to_ros(hull_points.points[i]);
                line.p1 = point_pcl_xyzrgb_to_ros(hull_points.points[(i + 1) % hull_points.points.size()]);

                obstacle_lines_temp.push_back(line);
            }

            obstacle_temp = create_obstacle(obstacle_lines_temp);

            if (obstacle_temp.boundaries.size() > 0)
            {
                auto grid_iter = get_box_aabb_in_grid(obstacle_temp, global_map_);
                int point_counter = 0;
                int hit_counter = 0;

                for (int x = get<0>(grid_iter); x <= get<1>(grid_iter); ++x)
                {
                    for (int y = get<2>(grid_iter); y <= get<3>(grid_iter); ++y)
                    {
                        int index = grid_xy_to_grid_index(x, y, global_map_);

                        geometry_msgs::Point32 point_in_map = grid_index_to_position(index, global_map_);

                        bool point_in_obstacle = false;

                        if (is_point_in_obstacle(point_in_map, obstacle_temp))
                        {
                            point_in_obstacle = true;

                            point_counter++;
                        }

                        if (global_map_.data[index] > local_map_occupied_thres_ &&
                            point_in_obstacle)
                        {
                            hit_counter++;
                        }
                    }
                }

                if (point_counter > 0 && hit_counter < dynamic_obs_hit_global_map_thres_)
                {
                    // ROS_WARN("DYNAMIC: %d %d", point_counter, hit_counter);

                    auto circle = find_min_bounding_circle(*cloud_clustered);

                    local_dynamic_obstacles.push_back(create_obstacle(get<0>(circle), get<1>(circle)));

                    is_dynamic_obstacle = true;

                    if (enable_visualization_)
                    {
                        visualization_msgs::Marker marker;
                        marker.header.frame_id = LOCALIZATION_MAP;
                        marker.header.stamp = ros::Time::now();
                        marker.ns = "local_obstacles";
                        marker.id = marker_id++;
                        marker.type = visualization_msgs::Marker::CYLINDER;
                        marker.action = visualization_msgs::Marker::ADD;
                        marker.pose.position.x = get<0>(circle).x;
                        marker.pose.position.y = get<0>(circle).y;
                        marker.pose.position.z = 0;
                        marker.pose.orientation.x = 0.0;
                        marker.pose.orientation.y = 0.0;
                        marker.pose.orientation.z = 0.0;
                        marker.pose.orientation.w = 1.0;
                        marker.scale.x = 2 * get<1>(circle);
                        marker.scale.y = 2 * get<1>(circle);
                        marker.scale.z = 0.01;
                        marker.color.a = 1.0;
                        marker.color.r = 1.0;
                        marker.color.g = 1.0;
                        marker.color.b = 0.0;

                        local_marker_array_.markers.push_back(marker);
                    }
                }
            }

            if (!is_dynamic_obstacle)
            {
                double hull_area = convex_hull.getTotalArea();

                double cloud_area = cloud_clustered->points.size() * resolution * resolution;
                double ratio = hull_area / cloud_area;

                if (ratio > local_map_wall_thres_)
                {
                    cv::Mat grid_map = cv::Mat::zeros(height, width, CV_8UC1);

                    for (const auto &point : cloud_clustered->points)
                    {
                        auto xy = position_to_grid_xy(point, local_map_grid);

                        if (get<0>(xy) >= 0 && get<0>(xy) < width && get<1>(xy) >= 0 && get<1>(xy) < height)
                        {
                            grid_map.at<uchar>(get<1>(xy), get<0>(xy)) = 255;
                        }
                    }

                    cv::Mat skeleton = skeletonize(grid_map);

                    vector<cv::Vec4i> lines;
                    cv::HoughLinesP(skeleton, lines, 1, CV_PI / 180,
                                    local_map_houghlines_thres_,
                                    local_map_houghlines_min_length_,
                                    local_map_houghlines_max_gap_);

                    vector<obstacle_description> wall_obstacles;

                    for (auto &line : lines)
                    {
                        auto obstacle = create_wall_obstacle(
                            grid_xy_to_position(line[0], line[1], local_map_grid),
                            grid_xy_to_position(line[2], line[3], local_map_grid),
                            2 * local_map_resolution_, local_map_grid, local_map_occupied_thres_);

                        wall_obstacles.push_back(obstacle);
                    }

                    auto merged_wall_obstacles = merge_wall_obstacles(wall_obstacles, local_map_houghlines_merge_angle_thres_);

                    for (auto &obstacle : merged_wall_obstacles)
                    {
                        local_static_obstacles_.push_back(obstacle);

                        if (enable_visualization_)
                        {
                            visualization_msgs::Marker marker;
                            marker.header.frame_id = LOCALIZATION_MAP;
                            marker.header.stamp = ros::Time::now();
                            marker.ns = "local_obstacles";
                            marker.id = marker_id++;
                            marker.type = visualization_msgs::Marker::LINE_STRIP;
                            marker.action = visualization_msgs::Marker::ADD;
                            marker.color.r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
                            marker.color.g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
                            marker.color.b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
                            marker.color.a = 1.0;
                            marker.scale.x = 0.02;

                            for (int i = 0; i <= obstacle.boundaries.size(); i++)
                            {
                                marker.points.push_back(point_ros_to_rosd(
                                    obstacle.boundaries[i % obstacle.boundaries.size()].p0));
                            }

                            // local_marker_array_.markers.push_back(marker);
                        }
                    }
                }
                else
                {
                    obstacle_description obstacle;
                    vector<obstacle_line> obstacle_lines;

                    for (int i = 0; i < hull_points.points.size(); i++)
                    {
                        obstacle_line line;
                        line.p0 = point_pcl_xyzrgb_to_ros(hull_points.points[i]);
                        line.p1 = point_pcl_xyzrgb_to_ros(hull_points.points[(i + 1) % hull_points.points.size()]);

                        obstacle_lines.push_back(line);
                    }

                    obstacle = create_obstacle(obstacle_lines);

                    local_static_obstacles_.push_back(obstacle);

                    if (enable_visualization_)
                    {
                        visualization_msgs::Marker marker;
                        marker.header.frame_id = LOCALIZATION_MAP;
                        marker.header.stamp = ros::Time::now();
                        marker.ns = "local_obstacles";
                        marker.id = marker_id++;
                        marker.type = visualization_msgs::Marker::LINE_STRIP;
                        marker.action = visualization_msgs::Marker::ADD;
                        marker.pose.orientation.w = 1.0;
                        marker.scale.x = 0.02;
                        marker.color.r = 1.0;
                        marker.color.a = 1.0;

                        for (int i = 0; i <= obstacle.boundaries.size(); i++)
                        {
                            marker.points.push_back(point_ros_to_rosd(
                                obstacle.boundaries[i % obstacle.boundaries.size()].p0));
                        }

                        // local_marker_array_.markers.push_back(marker);
                    }
                }
            }
        }

        // *cloud += *cloud_clustered;
        cloud_clustered->clear();
    }

    local_dynamic_obstacles_ = dynamic_obstacle_tracker_->track(local_dynamic_obstacles, ros::Time::now());

    // cout << local_dynamic_obstacles_.size() << " dynamic obstacles tracked." << endl;

    global_obstacles_focused_ = get_obstacles_in_roi(
        point_octomap_to_ros(sensor_origin),
        local_map_roi_radius_,
        global_obstacles_);

    // cout << global_obstacles_focused_.size() << " global obstacles focused." << endl;

    if (enable_visualization_)
    {
        for (const auto &obstacle : global_obstacles_focused_)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = LOCALIZATION_MAP;
            marker.header.stamp = ros::Time::now();
            marker.ns = "local_obstacles";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.r = 1;
            marker.color.g = 0;
            marker.color.b = 0;
            marker.color.a = 1.0;
            marker.scale.x = 0.02;

            for (int i = 0; i <= obstacle.boundaries.size(); i++)
            {
                marker.points.push_back(point_ros_to_rosd(
                    obstacle.boundaries[i % obstacle.boundaries.size()].p0));
            }

            // local_marker_array_.markers.push_back(marker);
        }

        local_marker_array_pub_.publish(local_marker_array_);
    }
}