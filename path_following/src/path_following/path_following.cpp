#include "path_following.h"
#include "pluginlib/class_list_macros.h"

using namespace helm;

void PathFollowing::initialize() {

    m_nh.reset(
        new ros::NodeHandle(ros::this_node::getNamespace() + "/" + m_name)
    );

}

PathFollowing::PathFollowing() {
    std::cout << "a message from path following" << std::endl;
}

bool PathFollowing::request_set_point(mvp_control::ControlProcess *set_point)
{

    *set_point = mvp_control::ControlProcess();

    return true;
}



PLUGINLIB_EXPORT_CLASS(helm::PathFollowing, helm::BehaviorBase)