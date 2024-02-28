#include <navigation/utils/obstacle_tracker/obstacle_ekf.h>

obstacle_ekf::obstacle_ekf() {}

obstacle_ekf::~obstacle_ekf() {}

void obstacle_ekf::initialize(const Eigen::VectorXd &initial_state,
                              const Eigen::MatrixXd &initial_covariance,
                              const Eigen::MatrixXd &process_noise,
                              const Eigen::MatrixXd &measurement_noise,
                              const double &time)
{
    state_ = initial_state;
    covariance_ = initial_covariance;
    process_noise_ = process_noise;
    measurement_noise_ = measurement_noise;
    identity_ = Eigen::MatrixXd::Identity(state_.size(), state_.size());

    last_update_time_ = time;
}

vector<Eigen::VectorXd> obstacle_ekf::predict(const double &time_base, const double &period, const double &dt)
{
    vector<Eigen::VectorXd> ret;

    if (dt > 0)
    {
        double time_diff = time_base - last_update_time_;
        double time = 0;
        Eigen::VectorXd state = state_;

        if (time_diff > 0)
        {
            Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
            F(0, 2) = time_diff;
            F(1, 3) = time_diff;
            F(0, 4) = 0.5 * time_diff * time_diff;
            F(1, 5) = 0.5 * time_diff * time_diff;
            F(2, 4) = time_diff;
            F(3, 5) = time_diff;

            state = F * state;
        }

        ret.push_back(state);

        while (time < period)
        {
            time += dt;

            Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
            F(0, 2) = dt;
            F(1, 3) = dt;
            F(0, 4) = 0.5 * dt * dt;
            F(1, 5) = 0.5 * dt * dt;
            F(2, 4) = dt;
            F(3, 5) = dt;

            state = F * state;

            ret.push_back(state);
        }
    }

    return ret;
}

vector<Eigen::VectorXd> obstacle_ekf::predict(const double &time_base, const double &period, const int &step)
{
    double dt = period / (double)step;

    auto ret = predict(time_base, period, dt);

    if (ret.size() > step + 1)
    {
        while (ret.size() != step + 1)
        {
            ret.pop_back();
        }
    }
    else if (ret.size() > 0 && ret.size() < step + 1)
    {
        while (ret.size() != step + 1)
        {
            ret.push_back(ret.back());
        }
    }

    return ret;
}

void obstacle_ekf::update(const Eigen::VectorXd &measurement, const double &time)
{
    double period = time - last_update_time_;

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 2) = period;
    F(1, 3) = period;
    F(0, 4) = 0.5 * period * period;
    F(1, 5) = 0.5 * period * period;
    F(2, 4) = period;
    F(3, 5) = period;

    state_ = F * state_;
    covariance_ = F * covariance_ * F.transpose() + process_noise_;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 6);
    H(0, 0) = 1;
    H(1, 1) = 1;

    Eigen::VectorXd y = measurement - H * state_;
    Eigen::MatrixXd S = H * covariance_ * H.transpose() + measurement_noise_;
    Eigen::MatrixXd K = covariance_ * H.transpose() * S.inverse();

    state_ += K * y;
    covariance_ = (identity_ - K * H) * covariance_;

    last_update_time_ = time;
}

void obstacle_ekf::update(const double &time)
{
    double period = time - last_update_time_;

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 2) = period;
    F(1, 3) = period;
    F(0, 4) = 0.5 * period * period;
    F(1, 5) = 0.5 * period * period;
    F(2, 4) = period;
    F(3, 5) = period;

    state_ = F * state_;
    covariance_ = F * covariance_ * F.transpose() + process_noise_;

    last_update_time_ = time;
}

Eigen::VectorXd obstacle_ekf::get_state() const
{
    return state_;
}

Eigen::MatrixXd obstacle_ekf::get_covariance() const
{
    return covariance_;
}
