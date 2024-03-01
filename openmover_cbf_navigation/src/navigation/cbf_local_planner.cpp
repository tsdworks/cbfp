#include <navigation/utils/obstacle_parameterizer/obstacle_parameterizer.h>
#include <osqp/osqp.h>
#include <navfn/MakeNavPlan.h>
#include <casadi/casadi.hpp>
#include <vector>
#include <cmath>
#include <navigation/utils/local_planner/mover.h>
#include <openmover_msgs/obstacle.h>

#define NODE_NAME "cbf_local_planner"
#define TAG "MPC-SGA-CBF Switchable Planner"

using namespace std;

// Obstacle parameterizer
obstacle_parameterizer obstacle_parameterizer_instance;

geometry_msgs::TwistWithCovarianceStamped odom_twist_data;
geometry_msgs::PoseWithCovarianceStamped odom_data, odom_data_end;
bool odom_twist_received = false;
bool odom_received = false;
geometry_msgs::Twist cmd_vel_data;
geometry_msgs::PoseWithCovarianceStamped pose_data, pose_data_end;
geometry_msgs::PoseWithCovarianceStamped pose_data_circle;
tf2::Transform base_link_to_circle_center;
bool pose_received = false;
geometry_msgs::Pose goal;

ros::Subscriber odom_twist_sub;
ros::Subscriber odom_sub;
ros::Subscriber simple_goal_sub;

ros::Publisher cmd_vel_pub;

ros::ServiceClient global_planner_client;

geometry_msgs::Polygon footprint_polygon;
obstacle_description footprint_rect_dynamic;
obstacle_description footprint_rect_static;
obstacle_description footprint_circle;

bool reached = false;
bool stand = false;
bool go = false;
bool use_cbf = false;
bool use_pf = true;

int current_point_in_path = 0;

// NMPC-DCBF
int horizon = 10;
int horizon_cbf = 5;
float dt = 0.1;
float sim_time = (float)horizon * dt;
float max_x_vel = 0.3;
float max_yaw_vel = 0.5;
float max_x_acc = 0.8;
float max_yaw_acc = 0.8;
float pomega = 10.0;
float gamma_k = 0.2;

float gamma_st = 0.8;
float pomega_st = 10.0;
float margin_dist_st = 0.0;

// SGA-DCBF
float max_x_vel_sga = 0.3;
float max_x_acc_sga = 0.8;
float min_x_vel_sga = 0.03;
float max_yaw_vel_sga = 0.5;
float max_yaw_acc_sga = 0.4;
float sim_time_sga = 0.5;
int sim_vx_space_size_sga = 20;
int sim_vyaw_space_size_sga = 60;
int trajectory_sim_step_sga = 50;
bool use_kinematic_limits_sga = true;
bool use_advanced_evaluation_sga = true;
float follow_point_weight_sga = 1.0;
float follow_path_weight_sga = 18.0;
float gamma_sga = 0.8;
int max_sim_point_index_sga = 0;
float chassis_radius_sga = 0.4;

int rotation_state_sga = 0;
unique_ptr<mover> mover_sga;

vector<tuple<double, string, int, int, double>> time_list;

casadi::MX x;
casadi::MX u;

std::vector<double> diag_elements = {100.0, 100.0};
casadi::MX mat_Q = casadi::MX::diag(diag_elements);
double terminal_weight = 10.0;
std::vector<double> diag_elements_zero = {0.0, 0.0};
casadi::MX mat_R = casadi::MX::diag(diag_elements_zero);
casadi::MX mat_Rold = casadi::MX::diag(diag_elements_zero);
casadi::MX mat_dR = casadi::MX::diag(diag_elements_zero);

vector<geometry_msgs::PoseStamped> path;

double error = 0;

tuple<casadi::DM, casadi::DM> convert_obstacle_to_matrix(const obstacle_description &obs)
{
    static map<string, tuple<casadi::DM, casadi::DM>> obstacle_records;

    if (obstacle_records.find(obs.uuid) != obstacle_records.end())
    {
        return obstacle_records[obs.uuid];
    }

    std::vector<double> A_values;
    std::vector<double> b_values;

    auto center = obs.center;

    for (const auto &line : obs.boundaries)
    {
        double dx = line.p1.x - line.p0.x;
        double dy = line.p1.y - line.p0.y;

        double length = std::sqrt(dx * dx + dy * dy);

        if (length < std::numeric_limits<double>::epsilon())
        {
            continue;
        }

        double nx = -dy / length;
        double ny = dx / length;

        double cx = center.x - line.p0.x;
        double cy = center.y - line.p0.y;

        if (nx * cx + ny * cy > 0)
        {
            nx = -nx;
            ny = -ny;
        }

        double b = nx * line.p0.x + ny * line.p0.y;

        A_values.push_back(nx);
        A_values.push_back(ny);
        b_values.push_back(b);
    }

    casadi::DM mat_A = casadi::DM::zeros(b_values.size(), 2);
    casadi::DM vec_b = casadi::DM::zeros(b_values.size(), 1);

    size_t size = b_values.size();

    for (size_t i = 0; i < size; ++i)
    {
        mat_A(i, 0) = A_values[i * 2];
        mat_A(i, 1) = A_values[i * 2 + 1];
        vec_b(i) = b_values[i];

        // ROS_WARN("b: %lf", b_values[i]);
    }

    obstacle_records[obs.uuid] = {mat_A, vec_b};

    return {mat_A, vec_b};
}

std::tuple<casadi::DM, casadi::DM, casadi::DM>
get_dist_region_to_region(const casadi::MX &mat_A1, const casadi::MX &vec_b1,
                          const casadi::MX &mat_A2, const casadi::MX &vec_b2)
{
    casadi::Opti opti;

    auto point1 = opti.variable(mat_A1.size2(), 1);
    auto point2 = opti.variable(mat_A2.size2(), 1);

    auto constraint1 = casadi::MX::mtimes(mat_A1, point1) <= vec_b1;
    auto constraint2 = casadi::MX::mtimes(mat_A2, point2) <= vec_b2;
    opti.subject_to(constraint1);
    opti.subject_to(constraint2);

    // dist_vec: 2x1
    auto dist_vec = point1 - point2;
    // 1x2 * 2x1 -> 1x1
    auto cost = casadi::MX::mtimes(dist_vec.T(), dist_vec);
    opti.minimize(cost);

    casadi::Dict opts;
    opts["expand"] = true;
    opts["ipopt.max_iter"] = 100;
    opts["verbose"] = false;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;
    opts["ipopt.linear_solver"] = "ma97";
    opts["ipopt.mu_strategy"] = "adaptive";
    opti.solver("ipopt", opts);

    casadi::OptiSol sol = opti.solve();

    casadi::DM dist = sol.value(casadi::MX::norm_2(dist_vec));

    // lamb: px1, mu: px1
    casadi::DM lamb, mu;

    lamb = casadi::DM::zeros(mat_A1.size1(), 1);
    mu = casadi::DM::zeros(mat_A2.size1(), 1);

    if (static_cast<double>(dist) > 0)
    {
        lamb = sol.value(opti.dual(constraint1)) / (2 * dist);
        mu = sol.value(opti.dual(constraint2)) / (2 * dist);
    }

    return std::make_tuple(dist, lamb, mu);
}

void simple_goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal = msg->pose;

    navfn::MakeNavPlan make_plan;

    make_plan.request.start.header.frame_id = LOCALIZATION_MAP;
    make_plan.request.start.header.stamp = ros::Time::now();
    make_plan.request.start.pose.orientation = pose_data.pose.pose.orientation;
    make_plan.request.start.pose.position = pose_data.pose.pose.position;

    make_plan.request.goal.header.frame_id = LOCALIZATION_MAP;
    make_plan.request.goal.header.stamp = ros::Time::now();
    make_plan.request.goal.pose.orientation = goal.orientation;
    make_plan.request.goal.pose.position = goal.position;

    make_plan.response.plan_found = 0;
    bool plan_result = false;
    int try_count = 0;

    while ((!plan_result || make_plan.response.plan_found != 1 ||
            !ros::service::waitForService("openmover/navigation/global_planner/make_plan", 60.0)) &&
           try_count < 5)
    {
        if (try_count >= 1)
        {
            ROS_WARN("%s: Global planner error, retrying for %d time(s).", TAG, try_count);
        }

        plan_result = global_planner_client.call(make_plan);

        try_count++;

        ros::Duration(1).sleep();
    }

    if (make_plan.response.plan_found == 1)
    {
        path = make_plan.response.path;

        // std::ofstream out_file_path("/home/tsdworks/openmover_robot/path.txt");

        // float path_dist = 0;

        // for (int i = 0; i < path.size(); i++)
        // {
        //     out_file_path << path[i].pose.position.x << " " << path[i].pose.position.y << endl;

        //     if (i > 0)
        //     {
        //         path_dist += calculate_distance(path[i].pose, path[i - 1].pose);
        //     }
        // }

        // out_file_path << path_dist << endl;

        // out_file_path.close();

        current_point_in_path = 0;

        go = true;

        stand = false;

        rotation_state_sga = 0;
    }
}

void odom_twist_callback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg)
{
    odom_twist_data = *msg;

    odom_twist_received = true;
}

void odom_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    odom_data = *msg;

    odom_received = true;
}

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    pose_data = *msg;

    tf2::Transform pose_transform;
    tf2::fromMsg(pose_data.pose.pose, pose_transform);
    tf2::Transform transformed_pose = base_link_to_circle_center * pose_transform;

    auto pose_circle_transform = tf2::toMsg(transformed_pose);
    pose_data_circle.pose.pose.position.x = pose_circle_transform.translation.x;
    pose_data_circle.pose.pose.position.y = pose_circle_transform.translation.y;
    pose_data_circle.pose.pose.position.z = pose_circle_transform.translation.z;
    pose_data_circle.pose.pose.orientation = pose_circle_transform.rotation;
    pose_data_circle.header = pose_data.header;

    pose_received = true;
}

float update_current_point_in_path()
{
    float min_dist_to_base = INFINITY;

    size_t size = path.size();

    for (int i = current_point_in_path; i < size; i++)
    {
        float dist = calculate_distance(path[i].pose, pose_data.pose.pose);

        if (dist < min_dist_to_base)
        {
            current_point_in_path = i;
            min_dist_to_base = dist;
        }

        if (dist >= sim_time * max_x_vel)
        {
            break;
        }
    }

    for (int i = current_point_in_path; i >= 0; i--)
    {
        float dist = calculate_distance(path[i].pose, pose_data.pose.pose);

        if (dist < min_dist_to_base)
        {
            current_point_in_path = i;
            min_dist_to_base = dist;
        }

        if (dist >= sim_time * max_x_vel)
        {
            break;
        }
    }

    return min_dist_to_base;

    // current_point_in_path = 0;

    return 0;
}

tuple<float, float> calc_goal_angle()
{
    tuple<float, float> ret(0, 0);

    float current_yaw = angles::normalize_angle(
        tf::getYaw(pose_data.pose.pose.orientation));

    float angle_to_goal = atan2(goal.position.y - pose_data.pose.pose.position.y,
                                goal.position.x - pose_data.pose.pose.position.x);

    get<0>(ret) = (float)angles::shortest_angular_distance(current_yaw, angle_to_goal);
    get<1>(ret) = (float)angles::normalize_angle(current_yaw + angle_to_goal);

    return ret;
}

tuple<float, float> calc_global_plan_angle(int point_index)
{
    tuple<float, float> ret(0, 0);

    if (path.size() <= 1)
    {
        return calc_goal_angle();
    }

    float current_yaw = angles::normalize_angle(
        tf::getYaw(pose_data.pose.pose.orientation));

    float sum_sin = 0;
    float sum_cos = 0;
    int point_num = 0;

    for (int i = current_point_in_path; i <= point_index; i++)
    {
        float angle_to_point =
            (float)angles::shortest_angular_distance(current_yaw, atan2(path[i].pose.position.y - pose_data.pose.pose.position.y,
                                                                        path[i].pose.position.x - pose_data.pose.pose.position.x));

        sum_sin += sin(angle_to_point);
        sum_cos += cos(angle_to_point);

        point_num++;
    }

    if (point_num > 0)
    {
        get<0>(ret) = atan2(sum_sin / point_num, sum_cos / point_num);
        get<1>(ret) = (float)angles::normalize_angle(current_yaw + get<0>(ret));
    }
    else
    {
        get<0>(ret) = 0;
        get<1>(ret) = 0;
    }

    return ret;
}

int get_max_sim_distance_point_index()
{
    int ret = -1;

    for (int i = current_point_in_path; i < path.size(); i++)
    {
        float dist = calculate_distance(pose_data.pose.pose, path[i].pose);

        if (dist > max_x_vel_sga * sim_time_sga)
        {
            ret = (i <= 0 ? 0 : (i - 1));

            break;
        }
    }

    if (ret == -1)
    {
        ret = path.size() - 1;
        ret = ret < 0 ? 0 : ret;
    }

    return ret;
}

int get_max_sim_angle_point_index(int point_index)
{
    int ret = point_index;

    for (int i = point_index; i >= current_point_in_path; i--)
    {
        auto angle = calc_global_plan_angle(i);

        if (abs(get<0>(angle)) <= max_yaw_vel_sga * sim_time_sga)
        {
            ret = i;
            break;
        }
    }

    // point_to_track_pub_.publish(path[ret]);

    return ret;
}
nav_msgs::Path trajectory_simulation(float &vx, float &vyaw,
                                     int max_sim_point_index,
                                     float max_x_vel, float max_yaw_vel,
                                     float sim_time,
                                     const vector<obstacle_description> &obs)
{
    nav_msgs::Path best_trajectory;
    nav_msgs::Path current_trajectory;

    float dist = calculate_distance(path[max_sim_point_index].pose, pose_data.pose.pose);

    float current_yaw = tf::getYaw(pose_data.pose.pose.orientation);

    float current_vx = odom_twist_data.twist.twist.linear.x;
    float start_search_vx = max(min_x_vel_sga, current_vx - sim_time * max_x_acc_sga);
    float end_search_vx = max(start_search_vx, min(current_vx + sim_time * max_x_acc_sga, dist / sim_time));

    float delta_search_vx = (end_search_vx - start_search_vx) / (sim_vx_space_size_sga <= 0 ? 1 : sim_vx_space_size_sga);
    float target_search_vx = start_search_vx;

    float current_vyaw = odom_twist_data.twist.twist.angular.z;
    float start_search_vyaw = current_vyaw - sim_time * max_yaw_acc_sga;
    float end_search_vyaw = current_vyaw + sim_time * max_yaw_acc_sga;

    float delta_search_vyaw = (end_search_vyaw - start_search_vyaw) / (sim_vyaw_space_size_sga <= 0 ? 1 : sim_vyaw_space_size_sga);
    float target_search_vyaw = start_search_vyaw;

    float delta_sim_time = sim_time / (trajectory_sim_step_sga <= 0 ? 1 : trajectory_sim_step_sga);

    float delta_current_search_vx = max_x_acc_sga * delta_sim_time;
    float delta_current_search_vyaw = max_yaw_acc_sga * delta_sim_time;

    map<string, float> dist_current;

    for (const auto &ob : obs)
    {
        dist_current[ob.uuid] = min_dist_between_obstacles(footprint_rect_dynamic, ob);
    }

    float max_score = -INFINITY;

    for (int i = -1; i < sim_vx_space_size_sga; i++)
    {
        if (i == -1)
        {
            target_search_vx = 0;
        }
        else if (i == 0)
        {
            target_search_vx = start_search_vx;
        }

        for (int j = -1; j < sim_vyaw_space_size_sga; j++)
        {
            if (j == -1)
            {
                target_search_vyaw = 0;
            }
            else if (j == 0)
            {
                target_search_vyaw = start_search_vyaw;
            }

            float current_search_vx = use_kinematic_limits_sga ? odom_twist_data.twist.twist.linear.x : target_search_vx;
            float current_search_vyaw = use_kinematic_limits_sga ? odom_twist_data.twist.twist.angular.z : target_search_vyaw;
            float last_search_vx = current_search_vx;
            float last_search_vyaw = current_search_vyaw;

            float rotated_sim_yaw = 0;

            current_trajectory.poses.clear();
            geometry_msgs::PoseStamped current_pose;
            current_pose.pose = pose_data.pose.pose;
            current_trajectory.poses.push_back(current_pose);

            for (int k = 0; k < trajectory_sim_step_sga; k++)
            {
                if (use_kinematic_limits_sga)
                {
                    if (current_search_vx < target_search_vx)
                    {
                        current_search_vx += delta_current_search_vx;

                        if (current_search_vx > target_search_vx)
                        {
                            current_search_vx = target_search_vx;
                        }
                    }
                    else if (current_search_vx > target_search_vx)
                    {
                        current_search_vx -= delta_current_search_vx;

                        if (current_search_vx < target_search_vx)
                        {
                            current_search_vx = target_search_vx;
                        }
                    }

                    if (current_search_vyaw < target_search_vyaw)
                    {
                        current_search_vyaw += delta_current_search_vyaw;

                        if (current_search_vyaw > target_search_vyaw)
                        {
                            current_search_vyaw = target_search_vyaw;
                        }
                    }
                    else if (current_search_vyaw > target_search_vyaw)
                    {
                        current_search_vyaw -= delta_current_search_vyaw;

                        if (current_search_vyaw < target_search_vyaw)
                        {
                            current_search_vyaw = target_search_vyaw;
                        }
                    }
                }

                geometry_msgs::PoseStamped current_sim_pose;
                current_sim_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

                current_sim_pose.pose.position.x =
                    current_trajectory.poses[k].pose.position.x +
                    (((last_search_vx + current_search_vx) / 2.0f) * delta_sim_time) * cos(current_yaw + rotated_sim_yaw);
                current_sim_pose.pose.position.y =
                    current_trajectory.poses[k].pose.position.y +
                    (((last_search_vx + current_search_vx) / 2.0f) * delta_sim_time) * sin(current_yaw + rotated_sim_yaw);

                rotated_sim_yaw += ((last_search_vyaw + current_search_vyaw) / 2.0f) * delta_sim_time;

                current_sim_pose.pose.orientation = tf::createQuaternionMsgFromYaw(current_yaw + rotated_sim_yaw);

                last_search_vx = current_search_vx;
                last_search_vyaw = current_search_vyaw;

                current_trajectory.poses.push_back(current_sim_pose);
            }

            vector<obstacle_line> lines_dynamic;

            geometry_msgs::PoseWithCovarianceStamped pose_next;
            pose_next.pose.pose = current_trajectory.poses.back().pose;
            auto footprint_tranform_sga = transform_footprint(pose_next, footprint_polygon);

            for (int i = 0; i < footprint_tranform_sga.points.size(); i++)
            {
                obstacle_line line;
                line.p0 = footprint_tranform_sga.points[i];
                line.p1 = footprint_tranform_sga.points[(i + 1) % footprint_tranform_sga.points.size()];

                lines_dynamic.push_back(line);
            }

            auto footprint_rect_dynamic_sga = create_obstacle(lines_dynamic);

            bool cbf_ok = true;

            for (const auto &ob : obs)
            {
                float dist_next = min_dist_between_obstacles(footprint_rect_dynamic_sga, ob);

                if (dist_next < (1 - gamma_sga) * dist_current[ob.uuid])
                {
                    cbf_ok = false;

                    break;
                }
            }

            if (!cbf_ok)
            {
                continue;
            }

            float dist_to_point = calculate_distance(current_trajectory.poses[current_trajectory.poses.size() - 1].pose,
                                                     path[max_sim_point_index].pose);

            float current_score = 0;

            if (max_sim_point_index == path.size() - 1)
            {
                current_score =
                    1.0f / (follow_point_weight_sga * dist_to_point);
            }
            else
            {
                float mean_dist_to_path = use_advanced_evaluation_sga ? 0 : INFINITY;

                if (!use_advanced_evaluation_sga)
                {
                    for (int k = current_point_in_path; k < path.size(); k++)
                    {
                        float dist = calculate_distance(current_trajectory.poses[current_trajectory.poses.size() - 1].pose,
                                                        path[k].pose);

                        mean_dist_to_path = min(mean_dist_to_path, dist);

                        if (dist > max_x_vel * sim_time)
                        {
                            break;
                        }
                    }
                }
                else
                {
                    float min_dist_to_path = INFINITY;

                    int end_point_in_path = current_point_in_path;

                    for (int k = current_point_in_path; k < path.size(); k++)
                    {
                        float dist = calculate_distance(current_trajectory.poses[current_trajectory.poses.size() - 1].pose,
                                                        path[k].pose);

                        if (dist <= min_dist_to_path)
                        {
                            end_point_in_path = k;
                            min_dist_to_path = dist;
                        }

                        if (dist > max_x_vel * sim_time)
                        {
                            break;
                        }
                    }

                    float index_increment_ratio =
                        1.0f * (end_point_in_path - current_point_in_path + 1) / (current_trajectory.poses.size() + 1);
                    float path_point_index_raw = current_point_in_path;
                    int path_point_index = current_point_in_path;
                    int trajectory_point_index = 0;
                    int dist_counter = 0;

                    mean_dist_to_path = 0;

                    while (path_point_index <= end_point_in_path && trajectory_point_index < current_trajectory.poses.size())
                    {
                        mean_dist_to_path += calculate_distance(path[path_point_index].pose,
                                                                current_trajectory.poses[trajectory_point_index].pose);
                        dist_counter++;

                        path_point_index_raw += index_increment_ratio;
                        path_point_index = (int)path_point_index_raw;
                        trajectory_point_index++;
                    }

                    mean_dist_to_path /= (dist_counter == 0 ? 1.0f : (1.0f * dist_counter));
                }

                current_score =
                    1.0f / ((follow_point_weight_sga * dist_to_point + follow_path_weight_sga * mean_dist_to_path) /
                            (follow_point_weight_sga + follow_path_weight_sga + 1e-5));
            }

            if (current_score >= max_score)
            {
                max_score = current_score;

                best_trajectory.poses = current_trajectory.poses;

                vx = target_search_vx;
                vyaw = target_search_vyaw;
            }

            target_search_vyaw += delta_search_vyaw;
        }

        target_search_vx += delta_search_vx;

        target_search_vyaw = start_search_vyaw;
    }

    return best_trajectory;
}

bool rotate(float angle = 0.0f, float tolerance = 0.0f, float max_yaw_vel = 0.0f, bool yaw_servo_mode = false)
{
    bool ret = false;

    if (rotation_state_sga == 0)
    {
        cmd_vel_data.linear.x = 0;
        cmd_vel_data.angular.z = 0;

        if (abs(odom_twist_data.twist.twist.linear.x) <= 0.01 &&
            abs(odom_twist_data.twist.twist.angular.z) <= 0.01)
        {
            mover_sga->reset_state();
            mover_sga->set_kinematic_params(max_x_vel_sga, max_yaw_vel_sga);
            mover_sga->set_tolerance_params(0.01, tolerance);

            mover_sga->make_plan(pose_data, 0, angle, true, yaw_servo_mode);

            mover_sga->start_session();

            rotation_state_sga = 1;
        }
    }
    else if (rotation_state_sga == 1)
    {
        openmover_msgs::obstacle obs;
        obs.size = 0;

        auto state = mover_sga->loop(pose_data, odom_twist_data, obs);

        if (state == navigation_state::REACHED)
        {
            mover_sga->reset_state();

            cmd_vel_data.linear.x = 0;
            cmd_vel_data.angular.z = 0;

            rotation_state_sga = 2;
        }
        else
        {
            cmd_vel_data = mover_sga->get_current_twist();
        }
    }
    else if (rotation_state_sga == 2)
    {
        rotation_state_sga = 0;

        ret = true;
    }

    return ret;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    // setlocale(LC_CTYPE, "zh_CN.utf8");

    ROS_INFO("%s: Started.", TAG);

    ros::NodeHandle node_handle;
    ros::NodeHandle node_handle_param_global("~");

    obstacle_parameterizer_instance.initialize();

    ros::Publisher cmd_vel_pub = node_handle.advertise<geometry_msgs::Twist>("openmover/control/cmd_vel", 1);
    ros::Publisher cbf_footprint_pub = node_handle.advertise<geometry_msgs::PolygonStamped>("cbf_footprint", 1);

    global_planner_client = node_handle.serviceClient<navfn::MakeNavPlan>("openmover/navigation/global_planner/make_plan");

    ros::Subscriber odom_twist_sub = node_handle.subscribe("openmover/localization/twist", 2, odom_twist_callback);
    ros::Subscriber odom_sub = node_handle.subscribe("openmover/hardware/odom", 2, odom_callback);
    ros::Subscriber pose_sub = node_handle.subscribe("openmover/localization/pose", 2, pose_callback);
    ros::Subscriber simple_goal_sub = node_handle.subscribe("openmover/navigation/simple_goal", 2, simple_goal_callback);

    XmlRpc::XmlRpcValue footprint_param;
    node_handle_param_global.getParam("footprint", footprint_param);

    for (int i = 0; i < footprint_param.size(); i++)
    {
        vector<float> point_vector;

        for (int j = 0; j < footprint_param[i].size(); j++)
        {
            point_vector.push_back(double(footprint_param[i][j]));
        }

        geometry_msgs::Point32 point;

        point.x = point_vector[0];
        point.y = point_vector[1];

        footprint_polygon.points.push_back(point);
    }

    auto circle = find_min_bounding_circle(footprint_polygon.points);
    footprint_circle = create_obstacle(get<0>(circle), get<1>(circle));
    base_link_to_circle_center.setOrigin(tf2::Vector3(get<0>(circle).x, get<0>(circle).y, 0));
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    base_link_to_circle_center.setRotation(q);

    vector<obstacle_line> lines_static;

    for (int i = 0; i < footprint_polygon.points.size(); i++)
    {
        obstacle_line line;
        line.p0 = footprint_polygon.points[i];
        line.p1 = footprint_polygon.points[(i + 1) % footprint_polygon.points.size()];

        lines_static.push_back(line);
    }

    footprint_rect_static = create_obstacle(lines_static);

    auto robot_rep_static = convert_obstacle_to_matrix(footprint_rect_static);
    auto robot_G_static = get<0>(robot_rep_static);
    auto robot_g_static = get<1>(robot_rep_static);

    auto static_obs_full = get<1>(obstacle_parameterizer_instance.get_global_obstacles());

    for (const auto obstacle : static_obs_full)
    {
        convert_obstacle_to_matrix(obstacle);
    }

    mover_sga = unique_ptr<mover>(new mover(max_x_vel_sga, min_x_vel_sga, max_x_acc_sga, max_yaw_vel_sga, 0.1, max_yaw_acc_sga,
                                            0.05,
                                            0.05,
                                            1.0,
                                            0.01, 0.01, 0.01, 0, 0,
                                            1.2,
                                            50));

    ros::Rate rate(50);

    while (ros::ok())
    {
        ros::spinOnce();

        double dx = base_link_to_circle_center.getOrigin().x();
        double dy = base_link_to_circle_center.getOrigin().y();
        double fyaw = tf::getYaw(pose_data.pose.pose.orientation);
        double fx = pose_data.pose.pose.position.x + dx * cos(fyaw) - dy * sin(fyaw);
        double fy = pose_data.pose.pose.position.y + dx * sin(fyaw) + dy * sin(fyaw);

        geometry_msgs::PolygonStamped poly;

        for (int i = 0; i < 36; ++i)
        {
            double angle = i * 2 * M_PI / 36;
            geometry_msgs::Point32 point;
            point.x = fx + footprint_circle.radius * cos(angle);
            point.y = fy + footprint_circle.radius * sin(angle);
            poly.polygon.points.push_back(point);
        }

        poly.header.frame_id = LOCALIZATION_MAP;
        poly.header.stamp = ros::Time::now();

        cbf_footprint_pub.publish(poly);

        vector<obstacle_line> lines_dynamic;

        auto footprint_polygon_tranform = transform_footprint(pose_data, footprint_polygon);

        for (int i = 0; i < footprint_polygon_tranform.points.size(); i++)
        {
            obstacle_line line;
            line.p0 = footprint_polygon_tranform.points[i];
            line.p1 = footprint_polygon_tranform.points[(i + 1) % footprint_polygon_tranform.points.size()];

            lines_dynamic.push_back(line);
        }

        footprint_rect_dynamic = create_obstacle(lines_dynamic);

        auto robot_rep_dynamic = convert_obstacle_to_matrix(footprint_rect_dynamic);
        auto robot_G_dynamic = get<0>(robot_rep_dynamic);
        auto robot_g_dynamic = get<1>(robot_rep_dynamic);

        if (go)
        {
            string run_type;

            auto stt = ros::Time::now();

            tuple<ros::Time, vector<vector<obstacle_state_with_id>>> predictions = obstacle_parameterizer_instance.get_dynamic_obstacle_predictions(
                ros::Time::now().toSec(),
                horizon * dt,
                horizon);

            auto dynamic_obs = get<1>(predictions);

            if (dynamic_obs.size() > 0)
            {
                horizon = 20;
                dt = 0.2;
                sim_time = horizon * dt;
            }
            else
            {
                horizon = 10;
                dt = 0.1;
                sim_time = horizon * dt;
            }

            vector<obstacle_description> static_obs = get<1>(obstacle_parameterizer_instance.get_static_obstacles());

            if (calculate_distance(pose_data.pose.pose, goal) <= 0.01 || stand)
            {
                run_type = "turn";

                if (!stand)
                {
                    ROS_WARN("------------------");
                    ROS_WARN("%s: Reached in the XY tolerance。", TAG);
                    ROS_WARN("Current: (%lf %lf)", pose_data.pose.pose.position.x, pose_data.pose.pose.position.y);
                    ROS_WARN("Goal: (%lf %lf)", goal.position.x, goal.position.y);
                    ROS_WARN("Control Error: %f m", calculate_distance(pose_data.pose.pose, goal));
                    ROS_WARN("------------------");

                    pose_data_end = pose_data;

                    odom_data_end = odom_data;

                    error = calculate_distance(pose_data.pose.pose, goal);

                    stand = true;
                }

                if (rotation_state_sga == 0)
                {
                    float angle_to_goal = (float)angles::shortest_angular_distance(
                        tf::getYaw(pose_data.pose.pose.orientation), tf::getYaw(goal.orientation));
                    float goal_yaw = tf::getYaw(goal.orientation);

                    rotate(goal_yaw, 0.01, max_yaw_vel_sga, true);
                }
                else if (rotate())
                {
                    cmd_vel_data.linear.x = 0;
                    cmd_vel_data.angular.z = 0;

                    ros::Duration(2.0).sleep();

                    go = false;

                    reached = true;

                    stand = false;

                    rotation_state_sga = 0;
                }

                cmd_vel_pub.publish(cmd_vel_data);

                rate.sleep();

                continue;
            }

            float min_dist_to_path = update_current_point_in_path();

            if (calculate_distance(pose_data.pose.pose, goal) > 0.1 || !use_pf)
            {
                run_type = "mpc";

                casadi::Opti opti;

                x = opti.variable(3, horizon + 1);
                u = opti.variable(2, horizon);

                opti.subject_to(x(0, 0) == pose_data.pose.pose.position.x);
                opti.subject_to(x(1, 0) == pose_data.pose.pose.position.y);
                opti.subject_to(x(2, 0) == tf::getYaw(pose_data.pose.pose.orientation));

                for (int i = 0; i < horizon; ++i)
                {
                    opti.subject_to(u(0, i) <= max_x_vel);
                    opti.subject_to(u(0, i) >= -max_x_vel);
                    opti.subject_to(u(1, i) <= max_yaw_vel);
                    opti.subject_to(u(1, i) >= -max_yaw_vel);
                }

                for (int i = 0; i < horizon - 1; ++i)
                {
                    opti.subject_to(u(0, i + 1) - u(0, i) <= max_x_acc * dt);
                    opti.subject_to(u(0, i + 1) - u(0, i) >= -max_x_acc * dt);

                    opti.subject_to(u(1, i + 1) - u(1, i) <= max_yaw_acc * dt);
                    opti.subject_to(u(1, i + 1) - u(1, i) >= -max_yaw_acc * dt);
                }

                opti.subject_to(u(0, 0) - cmd_vel_data.linear.x <= max_x_acc * dt);
                opti.subject_to(u(0, 0) - cmd_vel_data.linear.x >= -max_x_acc * dt);
                opti.subject_to(u(1, 0) - cmd_vel_data.angular.z <= max_yaw_acc * dt);
                opti.subject_to(u(1, 0) - cmd_vel_data.angular.z >= -max_yaw_acc * dt);

                for (int i = 0; i < horizon; ++i)
                {
                    opti.subject_to(x(0, i + 1) == x(0, i) + u(0, i) * cos(x(2, i)) * dt);
                    opti.subject_to(x(1, i + 1) == x(1, i) + u(0, i) * sin(x(2, i)) * dt);
                    opti.subject_to(x(2, i + 1) == x(2, i) + u(1, i) * dt);
                }

                if (static_obs.size() < 3)
                {
                    size_t dynamic_obs_size = dynamic_obs.size();

                    for (int i = 0; i < dynamic_obs_size; i++)
                    {
                        // horizonx1
                        casadi::MX omega = opti.variable(horizon, 1);

                        for (int j = 0; j < horizon - 1; j++)
                        {
                            casadi::DM obs_x_current = dynamic_obs[i][0].x;
                            casadi::DM obs_y_current = dynamic_obs[i][0].y;
                            casadi::DM obs_x_next = dynamic_obs[i][0].x;
                            casadi::DM obs_y_next = dynamic_obs[i][0].y;
                            casadi::MX dist_current = footprint_circle.radius + dynamic_obs[i][0].r + 0.1;
                            casadi::MX dist_next = footprint_circle.radius + dynamic_obs[i][0].r + 0.1;

                            casadi::MX theta = x(2, j);
                            casadi::MX theta_next = x(2, j + 1);

                            casadi::MX robot_x_current = x(0, j) + dx * casadi::MX::cos(theta) - dy * casadi::MX::sin(theta);
                            casadi::MX robot_y_current = x(1, j) + dx * casadi::MX::sin(theta) + dy * casadi::MX::cos(theta);
                            casadi::MX robot_x_next = x(0, j + 1) + dx * casadi::MX::cos(theta_next) - dy * casadi::MX::sin(theta_next);
                            casadi::MX robot_y_next = x(1, j + 1) + dx * casadi::MX::sin(theta_next) + dy * casadi::MX::cos(theta_next);

                            casadi::MX h_current = casadi::MX::sqrt(
                                                       casadi::MX::pow(robot_x_current - obs_x_current, 2) +
                                                       casadi::MX::pow(robot_y_current - obs_y_current, 2)) -
                                                   dist_current;
                            casadi::MX h_next = casadi::MX::sqrt(
                                                    casadi::MX::pow(robot_x_next - obs_x_next, 2) +
                                                    casadi::MX::pow(robot_y_next - obs_y_next, 2)) -
                                                dist_next;

                            opti.subject_to(omega(i) > 0);
                            opti.subject_to(omega(i) <= 1);

                            if (use_cbf)
                            {
                                opti.subject_to(h_next >= (1 - omega(i) * gamma_k) * h_current);
                            }
                            else
                            {
                                opti.subject_to(h_next >= (1 - omega(i) * gamma_k * 2) * h_current);
                            }

                            opti.set_initial(omega(i), 0.6);
                        }
                    }
                }

                casadi::MX decay_rate_relaxing_cost = 0;

                auto cbf_start_time = ros::Time::now();

                for (const auto &obstacle : static_obs)
                {
                    auto obs_rep = convert_obstacle_to_matrix(obstacle);
                    // p0x2
                    casadi::DM obs_A = std::get<0>(obs_rep);
                    // p0x1
                    casadi::DM obs_b = std::get<1>(obs_rep);

                    auto dist_res = get_dist_region_to_region(obs_A, obs_b, robot_G_dynamic, robot_g_dynamic);
                    // 1x1
                    casadi::DM cbf_curr = std::get<0>(dist_res);
                    // p0x1
                    casadi::DM lamb_curr = std::get<1>(dist_res);
                    // p1x1
                    casadi::DM mu_curr = std::get<2>(dist_res);

                    // p0xhorizon_cbf
                    casadi::MX lamb = opti.variable(obs_A.size1(), horizon_cbf);
                    // p1xhorizon_cbf
                    casadi::MX mu = opti.variable(robot_G_static.size1(), horizon_cbf);
                    // horizonx1
                    casadi::MX omega = opti.variable(horizon_cbf, 1);

                    for (int i = 0; i < horizon_cbf; i++)
                    {
                        // // 2x2
                        // casadi::MX robot_R = casadi::MX::horzcat(
                        //     {casadi::MX::vertcat({casadi::MX::cos(x(2, i + 1)), casadi::MX::sin(x(2, i + 1))}),
                        //      casadi::MX::vertcat({-casadi::MX::sin(x(2, i + 1)), casadi::MX::cos(x(2, i + 1))})});
                        // 2x1
                        casadi::MX robot_T = x(casadi::Slice(0, 2), i + 1);

                        opti.subject_to(lamb(casadi::Slice(), i) >= 0);
                        opti.subject_to(mu(casadi::Slice(), i) >= 0);

                        if (use_cbf)
                        {
                            opti.subject_to(
                                // 1xp1 * p1*1 -> 1x1
                                -casadi::MX::mtimes(robot_g_static.T(), mu(casadi::Slice(), i)) +
                                    // casadi::MX::mtimes(obs_A, robot_T) : p0x2 * 2x1 -> p0x1
                                    // obs_b : p0x1
                                    // (casadi::MX::mtimes(obs_A, robot_T) - obs_b).T() : 1xp0
                                    // lamb(casadi::Slice(), i) : p0x1
                                    // casadi::MX::mtimes((casadi::MX::mtimes(obs_A, robot_T) - obs_b).T(), lamb(casadi::Slice(), i))) : 1x1
                                    casadi::MX::mtimes((casadi::MX::mtimes(obs_A, robot_T) - obs_b).T(), lamb(casadi::Slice(), i)) >=
                                (1 - omega(i) * casadi::MX::pow(gamma_st, i + 1)) * (cbf_curr - margin_dist_st) + margin_dist_st);
                        }
                        else
                        {
                            opti.subject_to(
                                // 1xp1 * p1*1 -> 1x1
                                -casadi::MX::mtimes(robot_g_static.T(), mu(casadi::Slice(), i)) +
                                    // casadi::MX::mtimes(obs_A, robot_T) : p0x2 * 2x1 -> p0x1
                                    // obs_b : p0x1
                                    // (casadi::MX::mtimes(obs_A, robot_T) - obs_b).T() : 1xp0
                                    // lamb(casadi::Slice(), i) : p0x1
                                    // casadi::MX::mtimes((casadi::MX::mtimes(obs_A, robot_T) - obs_b).T(), lamb(casadi::Slice(), i))) : 1x1
                                    casadi::MX::mtimes((casadi::MX::mtimes(obs_A, robot_T) - obs_b).T(), lamb(casadi::Slice(), i)) >=
                                0.03);
                        }

                        opti.subject_to(
                            // 2xp1 * p1x1 -> 2x1
                            casadi::MX::mtimes(robot_G_dynamic.T(), mu(casadi::Slice(), i)) +
                                // casadi::MX::mtimes(robot_R.T(), obs_A.T()) 2x2 * 2xp0 -> 2xp0
                                // casadi::MX::mtimes(casadi::MX::mtimes(robot_R.T(), obs_A.T()), lamb(casadi::Slice(), i)): 2xp0 * p0x1 -> 2x1
                                casadi::MX::mtimes(obs_A.T(), lamb(casadi::Slice(), i)) ==
                            0);

                        // 2xp0 * p0x1 -> 2x1
                        casadi::MX temp = casadi::MX::mtimes(obs_A.T(), lamb(casadi::Slice(), i));
                        opti.subject_to(casadi::MX::mtimes(temp.T(), temp) <= 1);
                        opti.subject_to(omega(i) > 0);
                        decay_rate_relaxing_cost += pomega_st * casadi::MX::pow(omega(i) - 1, 2);

                        opti.set_initial(lamb(casadi::Slice(), i), lamb_curr);
                        opti.set_initial(mu(casadi::Slice(), i), mu_curr);
                        opti.set_initial(omega(i), 0.5);
                    }
                }

                auto cbf_end_time = ros::Time::now();

                vector<geometry_msgs::Pose> reference_trajectory;
                float point_dist = 0;

                for (int i = current_point_in_path + 1; i < path.size(); i++)
                {
                    point_dist += calculate_distance(path[i - 1].pose, path[i].pose);

                    bool point_in_obstalce = false;

                    size_t dynamic_obs_size_1 = dynamic_obs.size();

                    for (int kk = 0; kk < dynamic_obs_size_1; kk++)
                    {
                        geometry_msgs::Point32 pp;
                        pp.x = path[i].pose.position.x;
                        pp.y = path[i].pose.position.y;
                        geometry_msgs::Point32 ppc;
                        ppc.x = dynamic_obs[kk][0].x;
                        ppc.y = dynamic_obs[kk][0].y;

                        if (calculate_distance(pp, ppc) <= dynamic_obs[kk][0].r + 0.3)
                        {
                            point_in_obstalce = true;

                            break;
                        }
                    }

                    if (point_dist >= dt * max_x_vel && !point_in_obstalce)
                    {
                        point_dist = 0;

                        reference_trajectory.push_back(path[i].pose);
                    }

                    if (reference_trajectory.size() == horizon)
                    {
                        break;
                    }
                }

                while (reference_trajectory.size() < horizon)
                {
                    reference_trajectory.push_back(path.back().pose);
                }

                casadi::MX reference_trajectory_tracking_cost = 0;

                for (int i = 0; i < horizon - 1; ++i)
                {
                    double ref_x = reference_trajectory[i].position.x;
                    double ref_y = reference_trajectory[i].position.y;

                    casadi::MX x_diff = casadi::MX::vertcat({x(0, i) - ref_x, x(1, i) - ref_y});

                    diag_elements = {100.0 + i * 5, 100.0 + i * 5};
                    mat_Q = casadi::MX::diag(diag_elements);

                    reference_trajectory_tracking_cost += casadi::MX::mtimes(x_diff.T(), casadi::MX::mtimes(mat_Q, x_diff));
                }

                double ref_x_last = reference_trajectory.back().position.x;
                double ref_y_last = reference_trajectory.back().position.y;
                casadi::MX x_diff_last = casadi::MX::vertcat({x(0, horizon - 1) - ref_x_last, x(1, horizon - 1) - ref_y_last});
                reference_trajectory_tracking_cost += terminal_weight * casadi::MX::mtimes(
                                                                            x_diff_last.T(), casadi::MX::mtimes(mat_Q, x_diff_last));

                casadi::MX input_stage_cost = 0;

                for (int i = 0; i < horizon; ++i)
                {
                    input_stage_cost += casadi::MX::mtimes(
                        u(casadi::Slice(), i).T(), casadi::MX::mtimes(mat_R, u(casadi::Slice(), i)));
                }

                casadi::MX prev_u = casadi::MX::vertcat({cmd_vel_data.linear.x, cmd_vel_data.angular.z});

                casadi::MX prev_input_cost = 0;

                prev_input_cost += casadi::MX::mtimes(
                    (u(casadi::Slice(), 0) - prev_u).T(),
                    casadi::MX::mtimes(mat_Rold, u(casadi::Slice(), 0) - prev_u));

                casadi::MX input_smoothness_cost = 0;

                for (int i = 0; i < horizon - 1; ++i)
                {
                    casadi::MX u_diff = u(casadi::Slice(), i + 1) - u(casadi::Slice(), i);
                    input_smoothness_cost += casadi::MX::mtimes(
                        u_diff.T(),
                        casadi::MX::mtimes(mat_dR, u_diff));
                }

                casadi::DM x_init = casadi::DM::vertcat({pose_data.pose.pose.position.x, pose_data.pose.pose.position.y, tf::getYaw(pose_data.pose.pose.orientation)});

                casadi::DM u_init = casadi::DM::vertcat({cmd_vel_data.linear.x, cmd_vel_data.angular.z});

                for (int i = 0; i < horizon; ++i)
                {
                    opti.set_initial(x(casadi::Slice(), i + 1), x_init);
                    opti.set_initial(u(casadi::Slice(), i), u_init);
                }

                casadi::MX final_cost = reference_trajectory_tracking_cost + input_stage_cost + prev_input_cost + input_smoothness_cost + decay_rate_relaxing_cost;
                // casadi::MX final_cost = reference_trajectory_tracking_cost + input_stage_cost + prev_input_cost + input_smoothness_cost;
                opti.minimize(final_cost);

                casadi::Dict ipopt_opts;
                ipopt_opts["expand"] = true;
                ipopt_opts["ipopt.max_iter"] = 100;
                ipopt_opts["verbose"] = false;
                ipopt_opts["ipopt.acceptable_tol"] = 1e-3;
                ipopt_opts["ipopt.acceptable_obj_change_tol"] = 1e-3;
                ipopt_opts["ipopt.print_level"] = 0;
                ipopt_opts["ipopt.linear_solver"] = "ma57";
                ipopt_opts["ipopt.mu_strategy"] = "adaptive";
                ipopt_opts["print_time"] = 0;

                auto start_time = ros::Time::now();

                opti.solver("ipopt", ipopt_opts);

                try
                {
                    casadi::OptiSol opt_sol = opti.solve();

                    auto end_time = ros::Time::now();

                    ROS_INFO("Calc time: %lf", end_time.toSec() - start_time.toSec());

                    casadi::DM u_opt = opt_sol.value(u);

                    cmd_vel_data.linear.x = static_cast<double>(u_opt(0, 0));
                    cmd_vel_data.angular.z = static_cast<double>(u_opt(1, 0));
                }
                catch (...)
                {
                    cmd_vel_data.linear.x = 0;
                    cmd_vel_data.angular.z = 0;
                }

                cmd_vel_pub.publish(cmd_vel_data);
            }
            else
            {
                run_type = "sga";

                max_sim_point_index_sga = get_max_sim_angle_point_index(get_max_sim_distance_point_index());

                float vx = 0, vyaw = 0;

                auto current_traj = trajectory_simulation(vx, vyaw,
                                                          max_sim_point_index_sga,
                                                          max_x_vel_sga, max_yaw_vel_sga,
                                                          sim_time_sga,
                                                          static_obs);

                if (use_kinematic_limits_sga)
                {
                    volatile float combined_vel = abs(vx) + abs(vyaw * chassis_radius_sga);

                    if (combined_vel > max_x_vel)
                    {
                        volatile float vel_factor = max_x_vel / combined_vel;
                        vx *= vel_factor;
                        vyaw *= vel_factor;
                    }
                }

                if (abs(vx) < min_x_vel_sga)
                {
                    vx = copysign(min_x_vel_sga, vx);
                }

                cmd_vel_data.linear.x = vx;
                cmd_vel_data.angular.z = vyaw;

                cmd_vel_pub.publish(cmd_vel_data);
            }

            auto edt = ros::Time::now();

            tuple<double, string, int, int, double> state = {edt.toSec(), run_type, static_obs.size(), dynamic_obs.size(), edt.toSec() - stt.toSec()};

            time_list.push_back(state);
        }

        rate.sleep();
    }

    // std::ofstream out_file("/cbf_time.txt");

    // for (const auto &state : time_list)
    // {
    //     out_file << std::get<0>(state) << " "
    //              << std::get<1>(state) << " "
    //              << std::get<2>(state) << " "
    //              << std::get<3>(state) << " "
    //              << std::get<4>(state) << std::endl;
    // }

    // out_file << goal.position.x << " "
    //          << goal.position.y << " "
    //          << tf::getYaw(goal.orientation) << " "
    //          << pose_data_end.pose.pose.position.x << " "
    //          << pose_data_end.pose.pose.position.y << " "
    //          << tf::getYaw(pose_data_end.pose.pose.orientation) << " "
    //          << odom_data_end.pose.pose.position.x << " "
    //          << odom_data_end.pose.pose.position.y << " "
    //          << tf::getYaw(odom_data_end.pose.pose.orientation) << " "
    //          << error << endl;

    // out_file.close();

    return 0;
}