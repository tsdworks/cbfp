#ifndef QUADRATIC_OPTIMIZATION_H
#define QUADRATIC_OPTIMIZATION_H

#include <utility>
#include <vector>
#include <Eigen/Eigen>

class DiscretePointsReferenceLineSmoother
{
public:
    DiscretePointsReferenceLineSmoother();
    ~DiscretePointsReferenceLineSmoother();

    void Smooth(
        const std::vector<std::pair<float, float>> &path_original, std::vector<std::pair<double, double>> &path_optimized);

private:
    bool FemPosSmooth(
        std::vector<std::pair<double, double>> &raw_point2d,
        const std::vector<double> &bounds,
        std::vector<std::pair<double, double>> *ptr_smoothed_point2d);
};

#endif