#include <navigation/utils/global_planner/fem_pos_deviation_smoother.h>
#include <navigation/utils/global_planner/fem_pos_deviation_osqp_interface.h>
#include <iostream>

FemPosDeviationSmoother::FemPosDeviationSmoother() {}

bool FemPosDeviationSmoother::Solve(
	const std::vector<std::pair<double, double>> &raw_point2d,
	const std::vector<double> &bounds, std::vector<double> *opt_x,
	std::vector<double> *opt_y)
{
	return QpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
	return true;
}

bool FemPosDeviationSmoother::QpWithOsqp(
	const std::vector<std::pair<double, double>> &raw_point2d,
	const std::vector<double> &bounds, std::vector<double> *opt_x,
	std::vector<double> *opt_y)
{

	if (opt_x == nullptr || opt_y == nullptr)
	{
		std::cout << "opt_x or opt_y is nullptr" << std::endl;
		return false;
	}

	FemPosDeviationOsqpInterface solver;

	solver.set_ref_points(raw_point2d);
	solver.set_bounds_around_refs(bounds);

	if (!solver.Solve())
	{
		return false;
	}

	*opt_x = solver.opt_x();
	*opt_y = solver.opt_y();
	return true;
}