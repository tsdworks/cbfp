#include <navigation/utils/global_planner/replanner.h>

CollisionAndReplan::CollisionAndReplan(const CollisionAndReplanParams &params)
    : costs_(params.costs), costsLength_(params.costsLength),
      costsWidth_(params.costsWidth),
      //   robotLength_(params.robotLength),
      //   robotWidth_(params.robotWidth),
      footprint_(params.footprint),
      resolution_(params.resolution),
      stepGradient_(params.stepGradient),
      numIterMax_(params.numIterMax), shortest_distance_obs_(params.shortest_distance_obs),
      shortest_distance_coordinates_obs_(params.shortest_distance_coordinates_obs)
{
}
CollisionAndReplan::~CollisionAndReplan()
{
}

bool CollisionAndReplan::collision_replan(std::vector<std::pair<double, double>> &path)
{
    return true;

    if (path.size() <= 0)
    {
        printf("...\n");
    }

    std::vector<int> collision_points = collisionDetection(path);

    if (collision_points.size() <= 0)
    {
        printf("Pass the test.\n");

        return true;
    }
    else
    {
        printf("Need replanning.\n");
    }

    int pathpoint_extended = 60;
    int max_pathpoint_distance = pathpoint_extended / 2;
    std::vector<PathSegmentReplaned> collision_segments = generateCollisionSegments(collision_points, path, pathpoint_extended, max_pathpoint_distance);

    // 重规划
    bool temp = PathCollisionReplan(collision_segments, path);
    if (temp == false)
    {
        printf("Replan fail.\n");
    }
    else
    {
        printf("Replanned.\n");
    }
    return temp;
}

std::vector<int> CollisionAndReplan::collisionDetection(const std::vector<std::pair<double, double>> &path)
{
    int size = path.size();
    int minX, minY, maxX, maxY;
    std::vector<int> collision_points;

    std::vector<double> avg_orientation;
    avg_orientation = calc_avg_orientations(path);

    for (int i = 0; i < size; i++)
    {
        float robotX = path[i].first;
        float robotY = path[i].second;
        float robotOrientation = avg_orientation[i];
        geometry_msgs::Polygon obstacle_detection_area;

        generateConvexPolygon(robotX, robotY, robotOrientation, footprint_, minX, minY, maxX, maxY, obstacle_detection_area, resolution_);

        bool collision = checkCollisionOBB(costs_, minX, minY, maxX, maxY, obstacle_detection_area);
        if (collision)
        {
            collision_points.push_back(i);
        }
        else
        {
        }
    }

    return collision_points;
}

std::vector<PathSegmentReplaned> CollisionAndReplan::generateCollisionSegments(const std::vector<int> &collision_points, const std::vector<std::pair<double, double>> &path, int pathpoint_extended, int max_pathpoint_distance)
{
    int size = path.size();
    std::vector<PathSegmentReplaned> collision_segments;

    for (size_t i = 0; i < collision_points.size();)
    {
        int collision_point = collision_points[i];
        int start_index = std::max(0, collision_point - pathpoint_extended / 2);
        int end_index = std::min((int)(size - 1), collision_point + pathpoint_extended / 2);

        ++i;
        if (i <= collision_points.size() - 1)
        {
            int next_collision_point = collision_points[i];
            int distance = next_collision_point - collision_point;

            while (distance < max_pathpoint_distance && i <= collision_points.size() - 1)
            {
                collision_point = next_collision_point;
                ++i;
                if (i == collision_points.size())
                {
                    break;
                }
                next_collision_point = collision_points[i];
                distance = next_collision_point - collision_point;
            }

            end_index = std::min((int)(size - 1), collision_point + pathpoint_extended / 2);
        }

        PathSegmentReplaned segment;
        for (int j = start_index; j <= end_index; ++j)
        {
            segment.points.push_back(path[j]);
        }
        printf("start_index = %d,end_index=%d.\n", start_index, end_index);
        segment.start_index = start_index;
        segment.end_index = end_index;
        collision_segments.push_back(segment);

        /*
        int last_point = collision_points.back();
        if (end_index >= last_point - pathpoint_extended/2 ) {
            break;
        }
        */
    }

    return collision_segments;
}

bool CollisionAndReplan::PathCollisionReplan(std::vector<PathSegmentReplaned> &collision_segments, std::vector<std::pair<double, double>> &path)
{
    int num_success = 0;

    for (size_t i = 0; i < collision_segments.size(); i++)
    {
        std::vector<std::pair<double, double>> collision_path = collision_segments[i].points;

        int start_index = collision_segments[i].start_index;
        int end_index = collision_segments[i].end_index;

        int count = 0;

        // int num_max = numIterMax_/50;
        int num_max = numIterMax_;
        while (count < num_max)
        {
            std::vector<std::pair<double, double>> path_replaned = pathReplan(collision_path);
            // optimizationPath(path_replaned, M_PI / 17);
            count++;

            std::vector<double> avg_orientation;
            avg_orientation = calc_avg_orientations(path_replaned);

            size_t size = path_replaned.size();
            int minX, minY, maxX, maxY;
            bool result_replan = false;

            for (int i = 0; i < size; i++)
            {
                float robotX = path_replaned[i].first;
                float robotY = path_replaned[i].second;
                float robotOrientation = avg_orientation[i];

                geometry_msgs::Polygon obstacle_detection_area;
                generateConvexPolygon(robotX, robotY, robotOrientation, footprint_, minX, minY, maxX, maxY, obstacle_detection_area, resolution_);
                result_replan = checkCollisionOBB(costs_, minX, minY, maxX, maxY, obstacle_detection_area);

                if (result_replan)
                {
                    collision_path = path_replaned;
                    break;
                }
            }

            if (result_replan == false)
            {
                num_success++;
                int temp = 0;
                for (int j = start_index; j <= end_index; ++j)
                {
                    path[j] = path_replaned[temp];
                    temp++;
                }
                break;
            }

            if (count = num_max)
            {
                printf("Still has collision.");
            }
        }
    }

    if (num_success == collision_segments.size())
    {
        return true;
    }
    else
    {
        return false;
    }
}

std::vector<std::pair<double, double>> CollisionAndReplan::pathReplan(const std::vector<std::pair<double, double>> &path_collision_segments)
{
    std::vector<std::pair<double, double>> path_replaned = path_collision_segments;
    float wSmoothness = 0.3;
    float wObstacle = 0.7;
    float step = stepGradient_;
    float obsDMax_ = 0.6;

    for (int iteration = 0; iteration < 50; ++iteration)
    {
        for (int i = 2; i < path_replaned.size() - 2; ++i)
        {
            std::pair<double, double> gradient_smoothness = smoothnessTerm(path_replaned[i - 2], path_replaned[i - 1], path_replaned[i], path_replaned[i + 1], path_replaned[i + 2]);

            std::pair<double, double> gradient_obstacle(0.0, 0.0);
            int index = (int)path_replaned[i].first + costsWidth_ * (int)path_replaned[i].second;
            float obsDst = shortest_distance_obs_[index];
            double x_obs = shortest_distance_coordinates_obs_[index].first;
            double y_obs = shortest_distance_coordinates_obs_[index].second;

            if (obsDst < obsDMax_ && obsDst > 1e-6)
            {
                gradient_obstacle.first = 2 * (obsDst - obsDMax_) * (path_replaned[i].first - x_obs) / obsDst;
                gradient_obstacle.second = 2 * (obsDst - obsDMax_) * (path_replaned[i].second - y_obs) / obsDst;
            }

            float totalWeight = wSmoothness + wObstacle;

            double dx_correction = step * (wObstacle * gradient_obstacle.first + wSmoothness * gradient_smoothness.first) / totalWeight;
            double dy_correction = step * (wObstacle * gradient_obstacle.second + wSmoothness * gradient_smoothness.second) / totalWeight;

            path_replaned[i].first -= dx_correction;
            path_replaned[i].second -= dy_correction;
        }
    }

    return path_replaned;
}

double CollisionAndReplan::calc_path_point_orientation(const std::pair<double, double> &current, const std::vector<std::pair<double, double>> &forward_points)
{
    double avg_dx = 0.0;
    double avg_dy = 0.0;
    int num_points = 0;

    for (const auto &point : forward_points)
    {
        double dx = point.first - current.first;
        double dy = point.second - current.second;

        double distance = sqrt(dx * dx + dy * dy);

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
        avg_dx /= static_cast<double>(num_points);
        avg_dy /= static_cast<double>(num_points);
    }

    double avg_orientation = atan2(avg_dy, avg_dx);

    return avg_orientation;
}

std::vector<double> CollisionAndReplan::calc_avg_orientations(const std::vector<std::pair<double, double>> &Path)
{
    std::vector<double> avg_orientation(Path.size(), 0.0);
    size_t size = Path.size();

    for (int i = 0; i < size; i++)
    {
        std::pair<double, double> current;
        current.first = Path[i].first;
        current.second = Path[i].second;

        if (i < size - 2)
        {
            std::vector<std::pair<double, double>> forward_points;

            forward_points.clear();

            for (int j = 1; j <= std::min(5, static_cast<int>(size - i - 2)); j += 1)
            {
                forward_points.push_back(std::make_pair(Path[i + j].first, Path[i + j].second));
            }

            avg_orientation[i] = calc_path_point_orientation(current, forward_points);
        }
        // else if (i != size - 1)
        else
        {
            avg_orientation[i] = avg_orientation[i - 1];
        }
    }

    return avg_orientation;
}

void CollisionAndReplan::generateConvexPolygon(float centerX, float centerY, float orientation, geometry_msgs::Polygon footprint, int &minX, int &minY, int &maxX, int &maxY, geometry_msgs::Polygon &obstacleDetectionArea, float size)
{
    geometry_msgs::Point32 point;

    float cosYaw = cos(orientation);
    float sinYaw = sin(orientation);

    float x = footprint.points[0].x;
    float y = footprint.points[0].y;
    float x1 = centerX + (x * cosYaw - y * sinYaw) / size;
    float y1 = centerY + (y * cosYaw + x * cosYaw) / size;
    point.x = x1;
    point.y = y1;
    obstacleDetectionArea.points.push_back(point);

    x = footprint.points[1].x;
    y = footprint.points[1].y;
    float x2 = centerX + (x * cosYaw - y * sinYaw) / size;
    float y2 = centerY + (y * cosYaw + x * sinYaw) / size;
    point.x = x2;
    point.y = y2;
    obstacleDetectionArea.points.push_back(point);

    x = footprint.points[1].x;
    y = footprint.points[1].y;
    float x3 = centerX + (x * cosYaw - y * sinYaw) / size;
    float y3 = centerY + (y * cosYaw + x * sinYaw) / size;
    point.x = x3;
    point.y = y3;
    obstacleDetectionArea.points.push_back(point);

    x = footprint.points[0].x;
    y = footprint.points[0].y;
    float x4 = centerX - (x * cosYaw - y * sinYaw) / size;
    float y4 = centerY - (y * cosYaw + x * sinYaw) / size;
    point.x = x4;
    point.y = y4;
    obstacleDetectionArea.points.push_back(point);

    minX = std::min({x1, x2, x3, x4});
    minY = std::min({y1, y2, y3, y4});
    maxX = std::max({x1, x2, x3, x4});
    maxY = std::max({y1, y2, y3, y4});
}

bool CollisionAndReplan::is_point_in_rect(geometry_msgs::Point32 &p, geometry_msgs::Polygon &rect)
{
    int count = 0;

    for (unsigned int i = 0; i < rect.points.size(); i++)
    {
        geometry_msgs::Point32 a = rect.points[i];
        geometry_msgs::Point32 b = rect.points[(i + 1) % rect.points.size()];

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

bool CollisionAndReplan::checkCollisionOBB(unsigned char *costs, int minX, int minY, int maxX, int maxY, geometry_msgs::Polygon &obstacleDetectionArea)
{
    for (int x = minX; x <= maxX; ++x)
    {
        for (int y = minY; y <= maxY; ++y)
        {
            geometry_msgs::Point32 point;
            point.x = x;
            point.y = y;

            if (is_point_in_rect(point, obstacleDetectionArea))
            {
                int index = y * costsWidth_ + x;

                if (costs[index] >= 253)
                {
                    return true;
                }
            }
        }
    }

    return false;
}

std::pair<double, double> CollisionAndReplan::smoothnessTerm(
    const std::pair<double, double> &x_im2,
    const std::pair<double, double> &x_im1,
    const std::pair<double, double> &x_i,
    const std::pair<double, double> &x_ip1,
    const std::pair<double, double> &x_ip2)
{

    std::pair<double, double> gradient(0.0, 0.0);
    // calculate the gradient
    gradient.first = (x_im2.first - 4 * x_im1.first + 6 * x_i.first - 4 * x_ip1.first + x_ip2.first);
    gradient.second = (x_im2.second - 4 * x_im1.second + 6 * x_i.second - 4 * x_ip1.second + x_ip2.second);

    return gradient;
}

double inline normalizeAngle(
    double val,
    double min = -M_PI,
    double max = M_PI)
{
    float norm = 0.0;
    if (val >= min)
        norm = min + fmod((val - min), (max - min));
    else
        norm = max - fmod((min - val), (max - min));

    return norm;
}

int CollisionAndReplan::optimizationPath(std::vector<std::pair<double, double>> &path, double movement_angle_range)
{
    if (path.empty())
        return 0;
    size_t pose_size = path.size() - 1;
    double px, py, cx, cy, nx, ny, a_p, a_n;
    bool is_run = false;
    int ci = 0;
    for (ci = 0; ci < 1500; ci++)
    {
        is_run = false;
        for (size_t i = 1; i < pose_size; i++)
        {
            px = path[i - 1].first;
            py = path[i - 1].second;

            cx = path[i].first;
            cy = path[i].second;

            nx = path[i + 1].first;
            ny = path[i + 1].second;

            a_p = normalizeAngle(atan2(cy - py, cx - px), 0, 2 * M_PI);
            a_n = normalizeAngle(atan2(ny - cy, nx - cx), 0, 2 * M_PI);

            if (std::max(a_p, a_n) - std::min(a_p, a_n) > movement_angle_range)
            {
                path[i].first = (px + nx) / 2;
                path[i].second = (py + ny) / 2;
                is_run = true;
            }
        }
        if (!is_run)
            return ci;
    }
    return ci;
}
