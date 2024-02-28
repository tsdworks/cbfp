#include <navfn/MakeNavPlan.h>
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include "navigation/utils/global_planner/planner.h"
#include "localization/localization_sources.h"

#define NODE_NAME "global_planner_handler"
#define TAG "Global Planner Handler"

#define COST_MAP_WAIT_UPDATE_DURATION 2.0
#define COST_MAP_WAIT_DURATION 5.0

namespace cm = costmap_2d;
namespace rm = geometry_msgs;

using cm::Costmap2D;
using cm::Costmap2DROS;
using rm::PoseStamped;
using std::string;
using std::vector;

class planner_with_costmap : public global_planner
{
public:
    planner_with_costmap(string name, Costmap2DROS *cmap, ros::Time time) : global_planner(name, cmap->getCostmap(), cmap->getGlobalFrameID())
    {
        ros::NodeHandle private_nh("~");
        cmap_ = cmap;

        time_ = time;

        make_plan_service_ = private_nh.advertiseService("make_plan", &planner_with_costmap::plan_request_callback, this);
    }

    bool plan_request_callback(navfn::MakeNavPlan::Request &req, navfn::MakeNavPlan::Response &resp)
    {
        ROS_INFO("%s: Planning Global Path...", TAG);

        vector<PoseStamped> path;

        req.start.header.frame_id = LOCALIZATION_MAP;
        req.goal.header.frame_id = LOCALIZATION_MAP;

        while (!cmap_->isCurrent())
        {
            ros::Duration(COST_MAP_WAIT_UPDATE_DURATION).sleep();
        }

        if (ros::Time::now() - time_ < ros::Duration(COST_MAP_WAIT_DURATION))
        {
            ros::Duration(COST_MAP_WAIT_DURATION).sleep();
        }

        bool success = make_plan(req.start, req.goal, path);

        resp.plan_found = success;

        if (success)
        {
            ROS_INFO("%s: Plan OK!", TAG);

            resp.path = path;
        }
        else
        {
            ROS_WARN("%s: Plan Fail!", TAG);
        }

        return true;
    }

private:
    Costmap2DROS *cmap_;
    ros::ServiceServer make_plan_service_;
    ros::Subscriber pose_sub_;
    ros::Time time_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    setlocale(LC_CTYPE, "zh_CN.utf8");

    ROS_INFO("%s: Started", TAG);

    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);

    costmap_2d::Costmap2DROS lcr("costmap", buffer);

    planner_with_costmap pppp("planner", &lcr, ros::Time::now());

    ros::spin();

    return 0;
}
