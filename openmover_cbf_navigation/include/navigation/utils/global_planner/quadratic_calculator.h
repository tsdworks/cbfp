#ifndef QUADRATIC_CALCULATOR_H
#define QUADRATIC_CALCULATOR_H

#include <vector>
#include "potential_calculator.h"

class QuadraticCalculator : public PotentialCalculator
{
public:
    QuadraticCalculator(int nx, int ny) : PotentialCalculator(nx, ny) {}

    float calculatePotential(float *potential, unsigned char cost, int n, float prev_potential);
};

#endif
