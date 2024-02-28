#include <navigation/utils/math_utils.h>

geometry_msgs::Polygon transform_footprint(geometry_msgs::PoseWithCovarianceStamped &pose,
                                           geometry_msgs::Polygon &footprint)
{
    geometry_msgs::Polygon ret;

    ret.points.clear();

    size_t size = footprint.points.size();

    for (int i = 0; i < size; i++)
    {
        geometry_msgs::Point32 point;

        float x = footprint.points[i].x;
        float y = footprint.points[i].y;
        float yaw = tf::getYaw(pose.pose.pose.orientation);

        point.x = pose.pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
        point.y = pose.pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));

        ret.points.push_back(point);
    }

    return ret;
}

tuple<vector<obstacle_line>, vector<float>> create_obstacle_lines(geometry_msgs::PoseWithCovarianceStamped &pose,
                                                                  geometry_msgs::Polygon &footprint,
                                                                  openmover_msgs::obstacle &obstacle)
{
    tuple<vector<obstacle_line>, vector<float>> ret;

    get<0>(ret).clear();
    get<1>(ret).clear();

    size_t size = obstacle.obstacle_forward.size();

    float width = abs(footprint.points[1].y - footprint.points[0].y);

    float width_per_grid = width / (float)size;

    float min_dist = INFINITY;

    for (int i = 0; i < size; i++)
    {
        if (obstacle.obstacle_forward[i] != INFINITY)
        {
            obstacle_line line;

            line.type == obstacle_line_type::FRONT;

            line.p0.x = obstacle.obstacle_forward[i] + min(footprint.points[0].x, footprint.points[1].x);
            line.p1.x = line.p0.x;
            min_dist = min(min_dist, obstacle.obstacle_forward[i]);

            line.p0.y = i * width_per_grid - width / 2.0;
            line.p1.y = (i + 1) * width_per_grid - width / 2.0;

            float x = line.p0.x;
            float y = line.p0.y;
            float yaw = tf::getYaw(pose.pose.pose.orientation);

            line.p0.x = pose.pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
            line.p0.y = pose.pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));

            x = line.p1.x;
            y = line.p1.y;

            line.p1.x = pose.pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
            line.p1.y = pose.pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));

            get<0>(ret).push_back(line);
        }
    }

    get<1>(ret).push_back(min_dist);

    size = obstacle.obstacle_backward.size();

    width = abs(footprint.points[2].y - footprint.points[3].y);

    width_per_grid = width / (float)size;

    min_dist = INFINITY;

    for (int i = 0; i < size; i++)
    {
        if (obstacle.obstacle_backward[i] != INFINITY)
        {
            obstacle_line line;

            line.type == obstacle_line_type::REAR;

            line.p0.x = -obstacle.obstacle_backward[i] + max(footprint.points[2].x, footprint.points[3].x);
            line.p1.x = line.p0.x;
            min_dist = min(min_dist, obstacle.obstacle_backward[i]);

            line.p0.y = (width / 2.0) - (i * width_per_grid);
            line.p1.y = (width / 2.0) - ((i + 1) * width_per_grid);

            float x = line.p0.x;
            float y = line.p0.y;
            float yaw = tf::getYaw(pose.pose.pose.orientation);

            line.p0.x = pose.pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
            line.p0.y = pose.pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));

            x = line.p1.x;
            y = line.p1.y;

            line.p1.x = pose.pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
            line.p1.y = pose.pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));

            get<0>(ret).push_back(line);
        }
    }

    get<1>(ret).push_back(min_dist);

    size = obstacle.obstacle_left.size();

    float length = abs(footprint.points[0].x - footprint.points[3].x);

    float length_per_grid = length / (float)size;

    min_dist = INFINITY;

    for (int i = 0; i < size; i++)
    {
        if (obstacle.obstacle_left[i] != INFINITY)
        {
            obstacle_line line;

            line.type == obstacle_line_type::LEFT;

            line.p0.y = obstacle.obstacle_left[i] + min(footprint.points[0].y, footprint.points[3].y);
            line.p1.y = line.p0.y;

            line.p0.x = (length / 2.0) - (i * length_per_grid);
            line.p1.x = (length / 2.0) - ((i + 1) * length_per_grid);
            min_dist = min(min_dist, obstacle.obstacle_left[i]);

            float x = line.p0.x;
            float y = line.p0.y;
            float yaw = tf::getYaw(pose.pose.pose.orientation);

            line.p0.x = pose.pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
            line.p0.y = pose.pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));

            x = line.p1.x;
            y = line.p1.y;

            line.p1.x = pose.pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
            line.p1.y = pose.pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));

            get<0>(ret).push_back(line);
        }
    }

    get<1>(ret).push_back(min_dist);

    size = obstacle.obstacle_right.size();

    length = abs(footprint.points[1].x - footprint.points[2].x);

    length_per_grid = length / (float)size;

    min_dist = INFINITY;

    for (int i = 0; i < size; i++)
    {
        if (obstacle.obstacle_right[i] != INFINITY)
        {
            obstacle_line line;

            line.type == obstacle_line_type::RIGHT;

            line.p0.y = -obstacle.obstacle_right[i] + max(footprint.points[1].y, footprint.points[2].y);
            line.p1.y = line.p0.y;
            min_dist = min(min_dist, obstacle.obstacle_right[i]);

            line.p0.x = (i * length_per_grid) - (length / 2.0);
            line.p1.x = ((i + 1) * length_per_grid) - (length / 2.0);

            float x = line.p0.x;
            float y = line.p0.y;
            float yaw = tf::getYaw(pose.pose.pose.orientation);

            line.p0.x = pose.pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
            line.p0.y = pose.pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));

            x = line.p1.x;
            y = line.p1.y;

            line.p1.x = pose.pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
            line.p1.y = pose.pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));

            get<0>(ret).push_back(line);
        }
    }

    get<1>(ret).push_back(min_dist);

    return ret;
}

int orientation(const geometry_msgs::Point32 &p, const geometry_msgs::Point32 &q, const geometry_msgs::Point32 &r)
{
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

    if (fabs(val) < 1e-10)
    {
        return 0;
    }

    return (val > 0) ? 1 : 2;
}

bool on_segment(const geometry_msgs::Point32 &p, const geometry_msgs::Point32 &q, const geometry_msgs::Point32 &r)
{
    return (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y));
}

bool is_intersect(const geometry_msgs::Point32 &p1,
                  const geometry_msgs::Point32 &p2,
                  const geometry_msgs::Point32 &p3,
                  const geometry_msgs::Point32 &p4)
{
    int o1 = orientation(p3, p4, p1);
    int o2 = orientation(p3, p4, p2);
    int o3 = orientation(p1, p2, p3);
    int o4 = orientation(p1, p2, p4);

    if (o1 != o2 && o3 != o4)
    {
        return true;
    }

    if (o1 == 0 && on_segment(p3, p1, p4))
    {
        return true;
    }
    if (o2 == 0 && on_segment(p3, p2, p4))
    {
        return true;
    }
    if (o3 == 0 && on_segment(p1, p3, p2))
    {
        return true;
    }
    if (o4 == 0 && on_segment(p1, p4, p2))
    {
        return true;
    }

    return false;
}

bool is_point_in_rect(const geometry_msgs::Point32 &p, const geometry_msgs::Polygon &rect)
{
    int count = 0;

    size_t size = rect.points.size();

    for (unsigned int i = 0; i < size; i++)
    {
        geometry_msgs::Point32 a = rect.points[i];
        geometry_msgs::Point32 b = rect.points[(i + 1) % size];

        if ((a.y <= p.y && b.y > p.y) || (b.y <= p.y && a.y > p.y))
        {
            double t = (p.y - a.y) / (double)(b.y - a.y);

            if (a.x + t * (b.x - a.x) > p.x)
            {
                count++;
            }
        }
    }

    return count % 2 != 0;
}

bool is_point_in_obstacle(const geometry_msgs::Point32 &p, const obstacle_description &ob)
{
    if (ob.type == CONVEX)
    {
        geometry_msgs::Polygon rect;

        for (const auto &line : ob.boundaries)
        {
            rect.points.push_back(line.p0);
        }

        return is_point_in_rect(p, rect);
    }
    else if (ob.type == CIRCLE)
    {
        float dist = calculate_distance(p, ob.center);

        return dist <= ob.radius;
    }

    return false;
}

bool is_line_intersect_rect(geometry_msgs::Polygon &polygon, vector<obstacle_line> &lines)
{
    geometry_msgs::Point32 top_right = polygon.points[0];
    geometry_msgs::Point32 top_left = polygon.points[1];
    geometry_msgs::Point32 bottom_left = polygon.points[2];
    geometry_msgs::Point32 bottom_right = polygon.points[3];

    for (auto line : lines)
    {
        if (is_intersect(top_left, top_right, line.p0, line.p1) ||
            is_intersect(top_right, bottom_right, line.p0, line.p1) ||
            is_intersect(bottom_right, bottom_left, line.p0, line.p1) ||
            is_intersect(bottom_left, top_left, line.p0, line.p1) ||
            (is_point_in_rect(line.p0, polygon) && is_point_in_rect(line.p1, polygon)))
        {
            return true;
        }
    }

    return false;
}

bool is_rect_intersect(const obstacle_description &rect1, const obstacle_description &rect2)
{
    for (auto line1 : rect1.boundaries)
    {
        for (auto line2 : rect2.boundaries)
        {
            if (is_intersect(line1.p0, line1.p1, line2.p0, line2.p1))
            {
                return true;
            }
        }
    }

    return false;
}

bool is_obstacle_intersect(const obstacle_description &ob1, const obstacle_description &ob2)
{
    if (ob1.type == CIRCLE && ob2.type == CIRCLE)
    {
        if (calculate_distance(ob1.center, ob2.center) < (ob1.radius + ob2.radius))
        {
            return true;
        }
    }
    else if (ob1.type == CONVEX && ob2.type == CONVEX)
    {
        for (auto line : ob1.boundaries)
        {
            if (is_point_in_obstacle(line.p0, ob2))
            {
                return true;
            }
        }

        for (auto line : ob2.boundaries)
        {
            if (is_point_in_obstacle(line.p0, ob1))
            {
                return true;
            }
        }

        if (is_rect_intersect(ob1, ob2))
        {
            return true;
        }
    }
    else
    {
        obstacle_description circle = (ob1.type == CIRCLE) ? ob1 : ob2;
        obstacle_description rect = (ob1.type == CONVEX) ? ob1 : ob2;

        if (is_point_in_obstacle(circle.center, rect))
        {
            return true;
        }

        for (const auto &line : rect.boundaries)
        {
            if (is_line_in_or_cross_circle(circle.center, circle.radius, line))
            {
                return true;
            }
        }
    }

    return false;
}

bool is_obstacle_in_obstacle(const obstacle_description &ob1, const obstacle_description &ob2)
{
    for (const auto &line : ob1.boundaries)
    {
        if (!is_point_in_obstacle(line.p0, ob2))
        {
            return false;
        }
    }

    return true;
}

bool is_obstacle_almost_in_obstacle(const obstacle_description &ob1, const obstacle_description &ob2)
{
    int counter = 0;

    for (const auto &line : ob1.boundaries)
    {
        if (!is_point_in_obstacle(line.p0, ob2))
        {
            counter++;
        }
    }

    return (counter * 1.0 / ob1.boundaries.size()) <= 0.2;
}

float calc_path_point_orientation(geometry_msgs::Pose &pose, vector<geometry_msgs::Pose> &forward_points)
{
    float avg_dx = 0.0;
    float avg_dy = 0.0;
    int num_points = 0;

    for (const auto &point : forward_points)
    {
        float dx = point.position.x - pose.position.x;
        float dy = point.position.y - pose.position.y;

        float distance = sqrt(dx * dx + dy * dy);

        if (distance <= 0.05)
        {
            avg_dx += dx;
            avg_dy += dy;
            num_points++;
        }
        else
        {
            break;
        }
    }

    if (num_points > 0)
    {
        avg_dx /= (float)num_points;
        avg_dy /= (float)num_points;
    }

    float avg_orientation = atan2(avg_dy, avg_dx);

    return avg_orientation;
}

bool is_footprint_has_occupied_point(nav_msgs::OccupancyGridConstPtr grid,
                                     geometry_msgs::Polygon &footprint,
                                     const geometry_msgs::PoseStamped &pose,
                                     float stop_distance,
                                     bool reversed,
                                     int sampling_ratio,
                                     float padding_ratio)
{
    geometry_msgs::Point32 min_corner, max_corner;
    min_corner.x = pose.pose.position.x - stop_distance * padding_ratio;
    min_corner.y = pose.pose.position.y - stop_distance * padding_ratio;
    max_corner.x = pose.pose.position.x + stop_distance * padding_ratio;
    max_corner.y = pose.pose.position.y + stop_distance * padding_ratio;

    int min_x_index = max(0, (int)((min_corner.x - grid->info.origin.position.x) / grid->info.resolution));
    int min_y_index = max(0, (int)((min_corner.y - grid->info.origin.position.y) / grid->info.resolution));
    int max_x_index = min((int)grid->info.width - 1, (int)((max_corner.x - grid->info.origin.position.x) / grid->info.resolution));
    int max_y_index = min((int)grid->info.height - 1, (int)((max_corner.y - grid->info.origin.position.y) / grid->info.resolution));

    geometry_msgs::Polygon obstacle_detection_area;

    if (!reversed)
    {
        float yaw = tf::getYaw(pose.pose.orientation);

        geometry_msgs::Point32 point;
        float x = footprint.points[0].x;
        float y = footprint.points[0].y;
        point.x = pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
        point.y = pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));
        obstacle_detection_area.points.push_back(point);

        x = footprint.points[1].x;
        y = footprint.points[1].y;
        point.x = pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
        point.y = pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));
        obstacle_detection_area.points.push_back(point);

        x = footprint.points[1].x + stop_distance * padding_ratio;
        y = footprint.points[1].y;
        point.x = pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
        point.y = pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));
        obstacle_detection_area.points.push_back(point);

        x = footprint.points[0].x + stop_distance * padding_ratio;
        y = footprint.points[0].y;
        point.x = pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
        point.y = pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));
        obstacle_detection_area.points.push_back(point);
    }
    else
    {
        float yaw = angles::normalize_angle(
            tf::getYaw(pose.pose.orientation) + M_PI);

        geometry_msgs::Point32 point;
        float x = footprint.points[2].x;
        float y = footprint.points[2].y;
        point.x = pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
        point.y = pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));
        obstacle_detection_area.points.push_back(point);

        x = footprint.points[3].x;
        y = footprint.points[3].y;
        point.x = pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
        point.y = pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));
        obstacle_detection_area.points.push_back(point);

        x = footprint.points[3].x - stop_distance * padding_ratio;
        y = footprint.points[3].y;
        point.x = pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
        point.y = pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));
        obstacle_detection_area.points.push_back(point);

        x = footprint.points[2].x - stop_distance * padding_ratio;
        y = footprint.points[2].y;
        point.x = pose.pose.position.x + (x * cos(yaw) - y * sin(yaw));
        point.y = pose.pose.position.y + (y * cos(yaw) + x * sin(yaw));
        obstacle_detection_area.points.push_back(point);
    }

    for (int i = min_x_index; i <= max_x_index; i += sampling_ratio)
    {
        for (int j = min_y_index; j <= max_y_index; j += sampling_ratio)
        {
            geometry_msgs::Point32 point;
            point.x = grid->info.origin.position.x + i * grid->info.resolution;
            point.y = grid->info.origin.position.y + j * grid->info.resolution;

            if (is_point_in_rect(point, obstacle_detection_area))
            {
                int index = j * grid->info.width + i;

                if (grid->data[index] >= 50)
                {
                    return true;
                }
            }
        }
    }

    return false;
}

geometry_msgs::Point32 calculate_polygon_center(const geometry_msgs::Polygon &polygon)
{
    geometry_msgs::Point32 center;

    for (const auto &point : polygon.points)
    {
        center.x += point.x;
        center.y += point.y;
    }

    center.x /= polygon.points.size();
    center.y /= polygon.points.size();

    return center;
}

geometry_msgs::Point32 calculate_line_center(const obstacle_line &line)
{
    geometry_msgs::Point32 center;

    center.x = (line.p0.x + line.p1.x) / 2.0;
    center.y = (line.p0.y + line.p1.y) / 2.0;

    return center;
}

float calculate_distance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2)
{
    return hypot(hypot(p1.position.x - p2.position.x, p1.position.y - p2.position.y), p1.position.z - p2.position.z);
}

float calculate_distance(const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2)
{
    return hypot(hypot(p1.x - p2.x, p1.y - p2.y), p1.z - p2.z);
}

float calculate_distance(const octomap::point3d &p1, const octomap::point3d &p2)
{
    return hypot(hypot(p1.x() - p2.x(), p1.y() - p2.y()), p1.z() - p2.z());
}

float calculate_distance(const cv::Point2f &p1, const cv::Point2f &p2)
{
    return hypot(p1.x - p2.x, p1.y - p2.y);
}

float calculate_distance(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2)
{
    return hypot(hypot(p1.x - p2.x, p1.y - p2.y), p1.z - p2.z);
}

geometry_msgs::Point32 create_point(const float &x, const float &y)
{
    geometry_msgs::Point32 ret;

    ret.x = x;
    ret.y = y;

    return ret;
}

geometry_msgs::Point32 create_point(const float &x, const float &y, const float &z)
{
    geometry_msgs::Point32 ret = create_point(x, y);

    ret.z = z;

    return ret;
}

geometry_msgs::Point create_pointd(const float &x, const float &y)
{
    geometry_msgs::Point ret;

    ret.x = x;
    ret.y = y;

    return ret;
}

geometry_msgs::Point create_pointd(const float &x, const float &y, const float &z)
{
    geometry_msgs::Point ret = create_pointd(x, y);

    ret.z = z;

    return ret;
}

geometry_msgs::Point point_ros_to_rosd(const geometry_msgs::Point32 &p)
{
    geometry_msgs::Point ret;

    ret.x = p.x;
    ret.y = p.y;
    ret.z = p.z;

    return ret;
}

octomap::point3d point_ros_to_octomap(const geometry_msgs::Point32 &p)
{
    return octomap::point3d(p.x, p.y, p.z);
}

geometry_msgs::Point32 point_octomap_to_ros(const octomap::point3d &p)
{
    geometry_msgs::Point32 ret;

    ret.x = p.x();
    ret.y = p.y();
    ret.z = p.z();

    return ret;
}

cv::Point2f point_ros_to_cv(const geometry_msgs::Point32 &p)
{
    return cv::Point2f(p.x, p.y);
}

geometry_msgs::Point32 point_cv_to_ros(const cv::Point2f &p)
{
    geometry_msgs::Point32 ret;

    ret.x = p.x;
    ret.y = p.y;
    ret.z = 0.0;

    return ret;
}

geometry_msgs::Point point_cv_to_rosd(const cv::Point2f &p)
{
    geometry_msgs::Point ret;

    ret.x = p.x;
    ret.y = p.y;
    ret.z = 0.0;

    return ret;
}

geometry_msgs::Point32 point_pcl_xyz_to_ros(const pcl::PointXYZ &p)
{
    geometry_msgs::Point32 ret;

    ret.x = p.x;
    ret.y = p.y;
    ret.z = p.z;

    return ret;
}

geometry_msgs::Point32 point_pcl_xyzrgb_to_ros(const pcl::PointXYZRGB &p)
{
    geometry_msgs::Point32 ret;

    ret.x = p.x;
    ret.y = p.y;
    ret.z = p.z;

    return ret;
}

pcl::PointXYZRGB point_ros_to_pcl_xyzrgb(const geometry_msgs::Point32 &p)
{
    pcl::PointXYZRGB ret;

    ret.x = p.x;
    ret.y = p.y;
    ret.z = p.z;

    return ret;
}

geometry_msgs::Point point_pcl_xyz_to_rosd(const pcl::PointXYZ &p)
{
    geometry_msgs::Point ret;

    ret.x = p.x;
    ret.y = p.y;
    ret.z = p.z;

    return ret;
}

geometry_msgs::Point point_pcl_xyzrgb_to_rosd(const pcl::PointXYZRGB &p)
{
    geometry_msgs::Point ret;

    ret.x = p.x;
    ret.y = p.y;
    ret.z = p.z;

    return ret;
}

geometry_msgs::Polygon points_to_polygon(const vector<geometry_msgs::Point32> &points)
{
    geometry_msgs::Polygon polygon;

    for (const auto &p : points)
    {
        polygon.points.push_back(p);
    }

    return polygon;
}

geometry_msgs::Polygon points_to_polygon(const vector<octomap::point3d> &points)
{
    geometry_msgs::Polygon polygon;

    for (const auto &p : points)
    {
        polygon.points.push_back(point_octomap_to_ros(p));
    }

    return polygon;
}

geometry_msgs::Polygon points_to_polygon(const vector<cv::Point2f> &points)
{
    geometry_msgs::Polygon polygon;

    for (const auto &p : points)
    {
        polygon.points.push_back(point_cv_to_ros(p));
    }

    return polygon;
}

geometry_msgs::Polygon points_to_polygon(const geometry_msgs::Point32 &p0,
                                         const geometry_msgs::Point32 &p1,
                                         const geometry_msgs::Point32 &p2,
                                         const geometry_msgs::Point32 &p3)
{
    return points_to_polygon(vector<geometry_msgs::Point32>{p0, p1, p2, p3});
}

geometry_msgs::Polygon points_to_polygon(const octomap::point3d &p0,
                                         const octomap::point3d &p1,
                                         const octomap::point3d &p2,
                                         const octomap::point3d &p3)
{
    return points_to_polygon(vector<octomap::point3d>{p0, p1, p2, p3});
}

geometry_msgs::Polygon points_to_polygon(const cv::Point2f &p0,
                                         const cv::Point2f &p1,
                                         const cv::Point2f &p2,
                                         const cv::Point2f &p3)
{
    return points_to_polygon(vector<cv::Point2f>{p0, p1, p2, p3});
}

tuple<geometry_msgs::Point32, geometry_msgs::Point32> get_box_aabb(const geometry_msgs::Polygon &polygon)
{
    float min_x = numeric_limits<float>::max();
    float min_y = numeric_limits<float>::max();
    float max_x = -numeric_limits<float>::max();
    float max_y = -numeric_limits<float>::max();

    for (const auto &point : polygon.points)
    {
        min_x = min(min_x, point.x);
        min_y = min(min_y, point.y);
        max_x = max(max_x, point.x);
        max_y = max(max_y, point.y);
    }

    geometry_msgs::Point32 min_point, max_point;

    min_point.x = min_x;
    min_point.y = min_y;
    min_point.z = 0;

    max_point.x = max_x;
    max_point.y = max_y;
    max_point.z = 0;

    return {min_point, max_point};
}

tuple<geometry_msgs::Point32, geometry_msgs::Point32> get_box_aabb(const obstacle_description &obstacle)
{
    geometry_msgs::Point32 min_point, max_point;

    if (obstacle.type == obstacle_type::CONVEX)
    {
        min_point.x = min_point.y = numeric_limits<float>::max();
        max_point.x = max_point.y = -numeric_limits<float>::max();

        for (const auto &boundary : obstacle.boundaries)
        {
            min_point.x = min({min_point.x, boundary.p0.x, boundary.p1.x});
            min_point.y = min({min_point.y, boundary.p0.y, boundary.p1.y});
            max_point.x = max({max_point.x, boundary.p0.x, boundary.p1.x});
            max_point.y = max({max_point.y, boundary.p0.y, boundary.p1.y});
        }
    }
    else if (obstacle.type == obstacle_type::CIRCLE)
    {
        min_point.x = obstacle.center.x - obstacle.radius;
        min_point.y = obstacle.center.y - obstacle.radius;
        max_point.x = obstacle.center.x + obstacle.radius;
        max_point.y = obstacle.center.y + obstacle.radius;
    }

    return {min_point, max_point};
}

tuple<int, int, int, int> get_box_aabb_in_grid(const geometry_msgs::Polygon &polygon, const nav_msgs::OccupancyGrid &grid)
{
    auto [min_point, max_point] = get_box_aabb(polygon);

    int min_x = max(0, static_cast<int>((min_point.x - grid.info.origin.position.x) / grid.info.resolution));
    int min_y = max(0, static_cast<int>((min_point.y - grid.info.origin.position.y) / grid.info.resolution));
    int max_x = min(static_cast<int>(grid.info.width), static_cast<int>((max_point.x - grid.info.origin.position.x) / grid.info.resolution));
    int max_y = min(static_cast<int>(grid.info.height), static_cast<int>((max_point.y - grid.info.origin.position.y) / grid.info.resolution));

    return {min_x, max_x, min_y, max_y};
}

tuple<int, int, int, int> get_box_aabb_in_grid(const obstacle_description &obstacle, const nav_msgs::OccupancyGrid &grid)
{
    auto [min_point, max_point] = get_box_aabb(obstacle);

    int min_x = max(0, static_cast<int>((min_point.x - grid.info.origin.position.x) / grid.info.resolution));
    int min_y = max(0, static_cast<int>((min_point.y - grid.info.origin.position.y) / grid.info.resolution));
    int max_x = min(static_cast<int>(grid.info.width), static_cast<int>((max_point.x - grid.info.origin.position.x) / grid.info.resolution));
    int max_y = min(static_cast<int>(grid.info.height), static_cast<int>((max_point.y - grid.info.origin.position.y) / grid.info.resolution));

    return {min_x, max_x, min_y, max_y};
}

geometry_msgs::Point32 grid_xy_to_position(const int &x, const int &y, const nav_msgs::OccupancyGrid &grid)
{
    geometry_msgs::Point32 position;

    position.x = grid.info.origin.position.x + (x + 0.5) * grid.info.resolution;
    position.y = grid.info.origin.position.y + (y + 0.5) * grid.info.resolution;
    position.z = 0;

    return position;
}

geometry_msgs::Point32 grid_index_to_position(const int &index, const nav_msgs::OccupancyGrid &grid)
{
    int x = index % grid.info.width;
    int y = index / grid.info.width;

    return grid_xy_to_position(x, y, grid);
}

pcl::PointXYZ grid_xy_to_pointcloud_xyz(const int &x, const int &y, const nav_msgs::OccupancyGrid &grid)
{
    float real_x = grid.info.origin.position.x + (x + 0.5) * grid.info.resolution;
    float real_y = grid.info.origin.position.y + (y + 0.5) * grid.info.resolution;

    return pcl::PointXYZ(real_x, real_y, 0.0f);
}

pcl::PointXYZ grid_index_to_pointcloud_xyz(const int &index, const nav_msgs::OccupancyGrid &grid)
{
    int x = index % grid.info.width;
    int y = index / grid.info.width;

    return grid_xy_to_pointcloud_xyz(x, y, grid);
}

pcl::PointXYZRGB grid_xy_to_pointcloud_xyzrgb(const int &x, const int &y, const nav_msgs::OccupancyGrid &grid)
{
    pcl::PointXYZRGB ret;

    float real_x = grid.info.origin.position.x + (x + 0.5) * grid.info.resolution;
    float real_y = grid.info.origin.position.y + (y + 0.5) * grid.info.resolution;

    ret.x = real_x;
    ret.y = real_y;

    return ret;
}

pcl::PointXYZRGB grid_index_to_pointcloud_xyzrgb(const int &index, const nav_msgs::OccupancyGrid &grid)
{
    int x = index % grid.info.width;
    int y = index / grid.info.width;

    return grid_xy_to_pointcloud_xyzrgb(x, y, grid);
}

int grid_xy_to_grid_index(const int &x, const int &y, const nav_msgs::OccupancyGrid &grid)
{
    return y * grid.info.width + x;
}

tuple<int, int> grid_index_to_grid_xy(const int &index, const nav_msgs::OccupancyGrid &grid)
{
    int x = index % grid.info.width;
    int y = index / grid.info.width;

    return {x, y};
}

tuple<int, int> position_to_grid_xy(const geometry_msgs::Point32 &p, const nav_msgs::OccupancyGrid &grid)
{
    int x = static_cast<int>((p.x - grid.info.origin.position.x) / grid.info.resolution);
    int y = static_cast<int>((p.y - grid.info.origin.position.y) / grid.info.resolution);

    return {x, y};
}

tuple<int, int> position_to_grid_xy(const octomap::point3d &p, const nav_msgs::OccupancyGrid &grid)
{
    return position_to_grid_xy(point_octomap_to_ros(p), grid);
}

tuple<int, int> position_to_grid_xy(const cv::Point2f &p, const nav_msgs::OccupancyGrid &grid)
{
    return position_to_grid_xy(point_cv_to_ros(p), grid);
}

tuple<int, int> position_to_grid_xy(const pcl::PointXYZ &p, const nav_msgs::OccupancyGrid &grid)
{
    return position_to_grid_xy(point_pcl_xyz_to_ros(p), grid);
}

tuple<int, int> position_to_grid_xy(const pcl::PointXYZRGB &p, const nav_msgs::OccupancyGrid &grid)
{
    return position_to_grid_xy(point_pcl_xyzrgb_to_ros(p), grid);
}

int position_to_grid_index(const geometry_msgs::Point32 &p, const nav_msgs::OccupancyGrid &grid)
{
    auto [x, y] = position_to_grid_xy(p, grid);

    auto index = y * grid.info.width + x;

    return max(0, static_cast<int>(min(static_cast<int>(y * grid.info.width + x), static_cast<int>(grid.data.size() - 1))));
}

int position_to_grid_index(const octomap::point3d &p, const nav_msgs::OccupancyGrid &grid)
{
    return position_to_grid_index(point_octomap_to_ros(p), grid);
}

int position_to_grid_index(const cv::Point2f &p, const nav_msgs::OccupancyGrid &grid)
{
    return position_to_grid_index(point_cv_to_ros(p), grid);
}

obstacle_line create_line(const geometry_msgs::Point32 &p0, const geometry_msgs::Point32 &p1)
{
    obstacle_line ret;

    ret.p0 = p0;
    ret.p1 = p1;

    return ret;
}

obstacle_line create_line(const float &x0, const float &y0, const float &x1, const float &y1)
{
    return create_line(create_point(x0, y0), create_point(x1, y1));
}

obstacle_line create_line(const geometry_msgs::Point &p0, const geometry_msgs::Point &p1)
{
    return create_line(p0.x, p0.y, p1.x, p1.y);
}

obstacle_description create_obstacle(const geometry_msgs::Point32 &pos, const float &radius)
{
    obstacle_description ret;

    boost::uuids::uuid uuid = generator();
    ret.uuid = boost::uuids::to_string(uuid);
    ret.type = obstacle_type::CIRCLE;
    ret.center = pos;
    ret.radius = radius;

    return ret;
}

obstacle_description create_obstacle(const vector<obstacle_line> &boundaries)
{
    obstacle_description ret;

    boost::uuids::uuid uuid = generator();
    ret.uuid = boost::uuids::to_string(uuid);
    ret.type = obstacle_type::CONVEX;
    ret.boundaries = boundaries;

    // 计算重心
    float area = 0;
    float Cx = 0;
    float Cy = 0;
    size_t N = ret.boundaries.size();

    for (size_t i = 0; i < N; ++i)
    {
        auto p0 = ret.boundaries[i].p0;
        auto p1 = ret.boundaries[(i + 1) % N].p0;

        float a = p0.x * p1.y - p1.x * p0.y;
        area += a;
        Cx += (p0.x + p1.x) * a;
        Cy += (p0.y + p1.y) * a;
    }

    if (area != 0)
    {
        area /= 2.0;
        Cx /= (6.0 * area);
        Cy /= (6.0 * area);
        ret.center = create_point(Cx, Cy);
    }

    return ret;
}

float min_distance_to_line(const geometry_msgs::Point32 &p, const obstacle_line &line)
{
    float line_length = sqrt(pow(line.p1.x - line.p0.x, 2) + pow(line.p1.y - line.p0.y, 2));

    if (line_length == 0)
    {
        return sqrt(pow(p.x - line.p0.x, 2) + pow(p.y - line.p0.y, 2));
    }

    float t = ((p.x - line.p0.x) * (line.p1.x - line.p0.x) + (p.y - line.p0.y) * (line.p1.y - line.p0.y)) / pow(line_length, 2);

    t = max(0.0f, min(1.0f, t));

    geometry_msgs::Point32 projection = create_point(line.p0.x + t * (line.p1.x - line.p0.x),
                                                     line.p0.y + t * (line.p1.y - line.p0.y));

    return sqrt(pow(p.x - projection.x, 2) + pow(p.y - projection.y, 2));
}

bool is_line_in_or_cross_circle(const geometry_msgs::Point32 &pos, const float &radius, const obstacle_line &line)
{
    return min_distance_to_line(pos, line) < radius;
}

bool is_obstacle_in_roi(const geometry_msgs::Point32 &pos, const float &radius, const obstacle_description &obstacle)
{
    if (obstacle.type == obstacle_type::CONVEX)
    {
        for (const auto &line : obstacle.boundaries)
        {
            if (is_line_in_or_cross_circle(pos, radius, line))
            {
                return true;
            }
        }
    }
    else if (obstacle.type == obstacle_type::CIRCLE)
    {
        float dist = sqrt(pow(pos.x - obstacle.center.x, 2) + pow(pos.y - obstacle.center.y, 2));

        return dist < (radius + obstacle.radius);
    }

    return false;
}

vector<obstacle_description> get_obstacles_in_roi(const geometry_msgs::Point32 &pos, const float &radius, const vector<obstacle_description> &obstacles)
{
    vector<obstacle_description> obstacles_in_roi;

    for (const auto &obstacle : obstacles)
    {
        if (is_obstacle_in_roi(pos, radius, obstacle))
        {
            obstacles_in_roi.push_back(obstacle);
        }
    }

    return obstacles_in_roi;
}

bool is_point_in_circle(const geometry_msgs::Point32 &center, const float &radius, const geometry_msgs::Point32 &p)
{
    return calculate_distance(p, center) <= radius;
}

tuple<geometry_msgs::Point32, float> circle_from_two_points(const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2)
{
    geometry_msgs::Point32 center;

    center.x = (p1.x + p2.x) / 2;
    center.y = (p1.y + p2.y) / 2;

    float radius = calculate_distance(p1, center);

    return {center, radius};
}

tuple<geometry_msgs::Point32, float> circle_from_three_points(const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2, const geometry_msgs::Point32 &p3)
{
    float d = 2 * (p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y));

    geometry_msgs::Point32 center;

    center.x = ((p1.x * p1.x + p1.y * p1.y) * (p2.y - p3.y) + (p2.x * p2.x + p2.y * p2.y) * (p3.y - p1.y) + (p3.x * p3.x + p3.y * p3.y) * (p1.y - p2.y)) / d;
    center.y = ((p1.x * p1.x + p1.y * p1.y) * (p3.x - p2.x) + (p2.x * p2.x + p2.y * p2.y) * (p1.x - p3.x) + (p3.x * p3.x + p3.y * p3.y) * (p2.x - p1.x)) / d;

    float radius = calculate_distance(p1, center);

    return {center, radius};
}

tuple<geometry_msgs::Point32, float> welzl(vector<geometry_msgs::Point32> points, vector<geometry_msgs::Point32> points_recurs, size_t n)
{
    if (n == 0 || points_recurs.size() == 3)
    {
        switch (points_recurs.size())
        {
        case 0:
            return {geometry_msgs::Point32(), 0};
        case 1:
            return {points_recurs[0], 0};
        case 2:
            return circle_from_two_points(points_recurs[0], points_recurs[1]);
        case 3:
            return circle_from_three_points(points_recurs[0], points_recurs[1], points_recurs[2]);
        }
    }

    size_t idx = rand() % n;
    geometry_msgs::Point32 p = points[idx];
    swap(points[idx], points[n - 1]);

    auto [circle_center, circle_radius] = welzl(points, points_recurs, n - 1);

    if (is_point_in_circle(circle_center, circle_radius, p))
    {
        return {circle_center, circle_radius};
    }

    points_recurs.push_back(p);

    return welzl(points, points_recurs, n - 1);
}

tuple<geometry_msgs::Point32, float> find_min_bounding_circle(const vector<geometry_msgs::Point32> &points)
{
    vector<geometry_msgs::Point32> points_modifiable = points;
    vector<geometry_msgs::Point32> points_recurs;

    return welzl(points_modifiable, points_recurs, points.size());
}

tuple<geometry_msgs::Point32, float> find_min_bounding_circle(const pcl::PointCloud<pcl::PointXYZ> &points)
{
    vector<geometry_msgs::Point32> ros_points;

    for (const auto &point : points)
    {
        ros_points.push_back(point_pcl_xyz_to_ros(point));
    }

    return find_min_bounding_circle(ros_points);
}

tuple<geometry_msgs::Point32, float> find_min_bounding_circle(const pcl::PointCloud<pcl::PointXYZRGB> &points)
{
    vector<geometry_msgs::Point32> ros_points;

    for (const auto &point : points)
    {
        ros_points.push_back(point_pcl_xyzrgb_to_ros(point));
    }

    return find_min_bounding_circle(ros_points);
}

float min_dist_between_obstacles(const obstacle_description &ob1, const obstacle_description &ob2)
{
    if (ob1.type == obstacle_type::CONVEX && ob2.type == obstacle_type::CONVEX)
    {
        if (is_obstacle_intersect(ob1, ob2) ||
            is_obstacle_in_obstacle(ob1, ob2) ||
            is_obstacle_in_obstacle(ob2, ob1))
        {
            return 0;
        }

        float min_dist = numeric_limits<float>::max();

        for (const auto &line2 : ob2.boundaries)
        {
            for (const auto &line1 : ob1.boundaries)
            {
                float dist_to_line = min_distance_to_line(line1.p0, line2);
                min_dist = min(min_dist, dist_to_line);
                dist_to_line = min_distance_to_line(line1.p1, line2);
                min_dist = min(min_dist, dist_to_line);
            }
        }

        for (const auto &line1 : ob1.boundaries)
        {
            for (const auto &line2 : ob2.boundaries)
            {
                float dist_to_line = min_distance_to_line(line2.p0, line1);
                min_dist = min(min_dist, dist_to_line);
                dist_to_line = min_distance_to_line(line2.p1, line1);
                min_dist = min(min_dist, dist_to_line);
            }
        }

        return min_dist;
    }
    else if (ob1.type == obstacle_type::CIRCLE && ob2.type == obstacle_type::CIRCLE)
    {
        float min_dist = calculate_distance(ob1.center, ob2.center) - ob1.radius - ob2.radius;

        min_dist = min_dist < 0 ? 0 : min_dist;

        return min_dist;
    }
    else
    {
        return -1;
    }
}