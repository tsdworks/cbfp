#ifndef GRID_PATH_H
#define GRID_PATH_H

#include <vector>
#include "traceback.h"

class GridPath : public Traceback
{
public:
    GridPath(PotentialCalculator *p_calc) : Traceback(p_calc) {}
    virtual ~GridPath() {}
    bool getPath(float *potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float>> &path);
};

#endif
