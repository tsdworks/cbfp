#include <navigation/utils/obstacle_tracker/obstacle_tracker.h>

obstacle_tracker::obstacle_tracker(float obstacle_lost_time_threshold,
                                   float obstacle_associate_dist_threshold,
                                   float obstacle_radius_change_threshold)
{
    obstacle_lost_time_threshold_ = obstacle_lost_time_threshold;
    obstacle_associate_dist_threshold_ = obstacle_associate_dist_threshold;
    obstacle_radius_change_threshold_ = obstacle_radius_change_threshold;
}

obstacle_tracker::~obstacle_tracker() {}

vector<obstacle_state_with_id> obstacle_tracker::track(const vector<obstacle_description> &obstacles, const ros::Time &stamp)
{
    vector<obstacle_state_with_id> tracked_obstacles;

    map<long long, bool> object_measured;

    for (const auto &obs : obstacles)
    {
        long long id = associate(obs);

        if (id == -1)
        {
            id = id_++;
            obstacle_ekfs_[id] = obstacle_ekf();

            Eigen::VectorXd initial_state(6);
            initial_state << obs.center.x, obs.center.y, 0, 0, 0, 0;

            Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(6, 6);
            initial_covariance(0, 0) = 0.01;
            initial_covariance(1, 1) = 0.01;
            initial_covariance(2, 2) = 1;
            initial_covariance(3, 3) = 1;
            initial_covariance(4, 4) = 1;
            initial_covariance(5, 5) = 1;

            Eigen::MatrixXd process_noise = Eigen::MatrixXd::Identity(6, 6);
            process_noise(0, 0) = 0.1;
            process_noise(1, 1) = 0.1;
            process_noise(2, 2) = 0.1;
            process_noise(3, 3) = 0.1;
            process_noise(4, 4) = 0.1;
            process_noise(5, 5) = 0.1;

            Eigen::MatrixXd measurement_noise = Eigen::MatrixXd::Identity(2, 2);
            measurement_noise(0, 0) = 0.01;
            measurement_noise(1, 1) = 0.01;

            obstacle_ekfs_[id].initialize(initial_state,
                                          initial_covariance,
                                          process_noise,
                                          measurement_noise,
                                          stamp.toSec());

            obstacle_radius_[id] = obs.radius;

            object_measured[id] = true;
        }
        else
        {
            Eigen::VectorXd measurement(2);
            measurement << obs.center.x, obs.center.y;
            obstacle_ekfs_[id].update(measurement, stamp.toSec());

            object_measured[id] = true;
        }

        obstacle_last_update_times_[id] = stamp.toSec();
    }

    auto it = obstacle_last_update_times_.begin();

    while (it != obstacle_last_update_times_.end())
    {
        if (stamp.toSec() - it->second > obstacle_lost_time_threshold_)
        {
            obstacle_ekfs_.erase(it->first);
            obstacle_radius_.erase(it->first);

            it = obstacle_last_update_times_.erase(it);
        }
        else
        {
            if (object_measured.find(it->first) == object_measured.end())
            {
                obstacle_ekfs_[it->first].update(stamp.toSec());
            }

            ++it;
        }
    }

    for (const auto &kv : obstacle_ekfs_)
    {
        auto ekf_state = kv.second.get_state();

        obstacle_state_with_id state;
        state.header.frame_id = LOCALIZATION_MAP;
        state.header.stamp = stamp;
        state.id = kv.first;
        state.x = ekf_state(0);
        state.y = ekf_state(1);
        state.vx = ekf_state(2);
        state.vy = ekf_state(3);
        state.ax = ekf_state(4);
        state.ay = ekf_state(5);
        state.r = obstacle_radius_[kv.first];

        tracked_obstacles.push_back(state);
    }

    return tracked_obstacles;
}

tuple<vector<obstacle_state_with_id>, nav_msgs::Path> obstacle_tracker::predict(const long long &id,
                                                                                const ros::Time time_base,
                                                                                const float &period,
                                                                                const float &dt)
{
    vector<obstacle_state_with_id> predicted_states;
    nav_msgs::Path predicted_path;

    predicted_path.header.frame_id = LOCALIZATION_MAP;
    predicted_path.header.stamp = time_base;

    if (obstacle_ekfs_.find(id) != obstacle_ekfs_.end())
    {
        vector<Eigen::VectorXd> predicted = obstacle_ekfs_[id].predict(time_base.toSec(), period, dt);

        for (int i = 0; i < predicted.size(); i++)
        {
            obstacle_state_with_id obs_state;
            obs_state.header.stamp = time_base + ros::Duration(i * dt);
            obs_state.id = id;
            obs_state.x = predicted[i](0);
            obs_state.y = predicted[i](1);
            obs_state.vx = predicted[i](2);
            obs_state.vy = predicted[i](3);
            obs_state.ax = predicted[i](4);
            obs_state.ay = predicted[i](5);
            obs_state.r = obstacle_radius_[id];

            geometry_msgs::PoseStamped pose;
            pose.header = obs_state.header;
            pose.pose.position.x = obs_state.x;
            pose.pose.position.y = obs_state.y;
            pose.pose.orientation.w = 1;

            predicted_states.push_back(obs_state);
            predicted_path.poses.push_back(pose);
        }
    }

    return make_tuple(predicted_states, predicted_path);
}

tuple<vector<obstacle_state_with_id>, nav_msgs::Path> obstacle_tracker::predict(const long long &id,
                                                                                const ros::Time time_base,
                                                                                const float &period,
                                                                                const int &step)
{
    vector<obstacle_state_with_id> predicted_states;
    nav_msgs::Path predicted_path;

    predicted_path.header.frame_id = LOCALIZATION_MAP;
    predicted_path.header.stamp = time_base;

    double dt = period / (double)step;

    if (obstacle_ekfs_.find(id) != obstacle_ekfs_.end())
    {
        vector<Eigen::VectorXd> predicted = obstacle_ekfs_[id].predict(time_base.toSec(), period, step);

        for (int i = 0; i < predicted.size(); i++)
        {
            obstacle_state_with_id obs_state;
            obs_state.header.stamp = time_base + ros::Duration(i * dt);
            obs_state.id = id;
            obs_state.x = predicted[i](0);
            obs_state.y = predicted[i](1);
            obs_state.vx = predicted[i](2);
            obs_state.vy = predicted[i](3);
            obs_state.ax = predicted[i](4);
            obs_state.ay = predicted[i](5);
            obs_state.r = obstacle_radius_[id];

            geometry_msgs::PoseStamped pose;
            pose.header = obs_state.header;
            pose.pose.position.x = obs_state.x;
            pose.pose.position.y = obs_state.y;
            pose.pose.orientation.w = 1;

            predicted_states.push_back(obs_state);
            predicted_path.poses.push_back(pose);
        }
    }

    return make_tuple(predicted_states, predicted_path);
}

long long obstacle_tracker::associate(const obstacle_description &obstacle)
{
    long long best_id = -1;

    double min_distance = std::numeric_limits<double>::max();

    for (const auto &kv : obstacle_ekfs_)
    {
        Eigen::VectorXd state = kv.second.get_state();

        double distance = sqrt(pow(state(0) - obstacle.center.x, 2) + pow(state(1) - obstacle.center.y, 2));

        if (distance < min_distance &&
            (distance < obstacle_associate_dist_threshold_ &&
             abs(obstacle.radius - obstacle_radius_[kv.first]) / (obstacle_radius_[kv.first] + 1e-5) < obstacle_radius_change_threshold_))
        {
            // cout << abs(obstacle.radius - obstacle_radius_[kv.first]) / (obstacle_radius_[kv.first] + 1e-5) << endl;

            min_distance = distance;
            best_id = kv.first;
        }
    }

    return best_id;
}