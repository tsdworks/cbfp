#ifndef GLOBAL_PLANNER_ORIENTATION_FILTER_H
#define GLOBAL_PLANNER_ORIENTATION_FILTER_H

#include <nav_msgs/Path.h>

enum OrientationMode
{
    NONE,
    FORWARD,
    INTERPOLATE,
    FORWARDTHENINTERPOLATE,
    BACKWARD,
    LEFTWARD,
    RIGHTWARD
};

class OrientationFilter
{
public:
    OrientationFilter() : omode_(NONE) {}

    virtual void processPath(const geometry_msgs::PoseStamped &start,
                             std::vector<geometry_msgs::PoseStamped> &path);

    void setAngleBasedOnPositionDerivative(std::vector<geometry_msgs::PoseStamped> &path, int index);
    void interpolate(std::vector<geometry_msgs::PoseStamped> &path,
                     int start_index, int end_index);

    void setMode(OrientationMode new_mode) { omode_ = new_mode; }
    void setMode(int new_mode) { setMode((OrientationMode)new_mode); }

    void setWindowSize(size_t window_size) { window_size_ = window_size; }

protected:
    OrientationMode omode_;
    int window_size_;
};

#endif
