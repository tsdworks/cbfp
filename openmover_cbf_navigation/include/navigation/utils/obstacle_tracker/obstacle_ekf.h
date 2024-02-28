#ifndef OBSTACLE_EKF_H
#define OBSTACLE_EKF_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>

using namespace std;

class obstacle_ekf
{
public:
    obstacle_ekf();
    ~obstacle_ekf();
    void initialize(const Eigen::VectorXd &initial_state,
                    const Eigen::MatrixXd &initial_covariance,
                    const Eigen::MatrixXd &process_noise,
                    const Eigen::MatrixXd &measurement_noise,
                    const double &time);
    vector<Eigen::VectorXd> predict(const double &time_base, const double &period, const double &dt);
    vector<Eigen::VectorXd> predict(const double &time_base, const double &period, const int &step);
    void update(const Eigen::VectorXd &measurement, const double &time);
    void update(const double &time);

    Eigen::VectorXd get_state() const;
    Eigen::MatrixXd get_covariance() const;

private:
    Eigen::VectorXd state_;
    Eigen::MatrixXd covariance_;
    Eigen::MatrixXd process_noise_;
    Eigen::MatrixXd measurement_noise_;
    Eigen::MatrixXd identity_;

    double last_update_time_ = -1.0;
};

#endif