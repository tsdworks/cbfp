#include <iostream>
#include <algorithm>
#include <cmath>
#include <navigation/utils/global_planner/quadratic_optimization.h>
#include <navigation/utils/global_planner/fem_pos_deviation_smoother.h>

DiscretePointsReferenceLineSmoother::DiscretePointsReferenceLineSmoother()
{
}

DiscretePointsReferenceLineSmoother::~DiscretePointsReferenceLineSmoother()
{
}

void DiscretePointsReferenceLineSmoother::Smooth(
    const std::vector<std::pair<float, float>> &path_original,
    std::vector<std::pair<double, double>> &path_optimized)
{

    std::vector<std::pair<double, double>> raw_point2d;
    std::vector<double> anchorpoints_lateralbound;

    for (const auto &point : path_original)
    {
        raw_point2d.emplace_back(point);
        anchorpoints_lateralbound.emplace_back(3);
    }

    anchorpoints_lateralbound.front() = 0.0;
    anchorpoints_lateralbound.back() = 0.0;

    bool status = false;
    status = FemPosSmooth(raw_point2d, anchorpoints_lateralbound,
                          &path_optimized);
    if (!status)
    {
        std::cout << "discrete_points reference line smoother fails" << std::endl;
    }
    if (path_optimized.size() < 2)
    {
        std::cout << "Fail to generate smoothed reference line." << std::endl;
    }
}

bool DiscretePointsReferenceLineSmoother::FemPosSmooth(
    std::vector<std::pair<double, double>> &raw_point2d,
    const std::vector<double> &bounds,
    std::vector<std::pair<double, double>> *ptr_smoothed_point2d)
{
    FemPosDeviationSmoother smoother;

    //  box contraints on pos are used in fem pos smoother, thus shrink the
    //  bounds by 1.0 / sqrt(2.0)
    std::vector<double> box_bounds = bounds;
    const double box_ratio = 1.0 / std::sqrt(2.0);
    for (auto &bound : box_bounds)
    {
        bound *= box_ratio;
    }

    std::vector<double> opt_x;
    std::vector<double> opt_y;
    bool status = smoother.Solve(raw_point2d, box_bounds, &opt_x, &opt_y);

    if (!status)
    {
        std::cout << "Fem Pos reference line smoothing failed" << std::endl;
        return false;
    }

    if (opt_x.size() < 2 || opt_y.size() < 2)
    {
        std::cout << "Return by fem pos smoother is wrong. Size smaller than 2 " << std::endl;
        return false;
    }

    size_t point_size = opt_x.size();

    for (size_t i = 0; i < point_size; ++i)
    {
        ptr_smoothed_point2d->emplace_back(opt_x[i], opt_y[i]);
    }

    return true;
}
