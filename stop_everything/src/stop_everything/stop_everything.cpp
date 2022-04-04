#include "stop_everything.h"
#include "pluginlib/class_list_macros.h"

using namespace helm;

void StopEverything::initialize() {

    m_nh.reset(
        new ros::NodeHandle(ros::this_node::getNamespace() + "/" + m_name)
    );

}

StopEverything::StopEverything() {
    std::cout << "a message from stop everything" << std::endl;
}

bool StopEverything::request_set_point(mvp_control::ControlProcess *set_point)
{

    *set_point = mvp_control::ControlProcess();

    ROS_INFO_STREAM("Requested set point from behavior " << m_name <<
             ", type stop everything");

    return true;
}



PLUGINLIB_EXPORT_CLASS(helm::StopEverything, helm::BehaviorBase)