#include "periodic_surface.h"
#include "pluginlib/class_list_macros.h"

using namespace helm;

void PeriodicSurface::initialize() {

    m_nh.reset(
        new ros::NodeHandle(ros::this_node::getNamespace() + "/" + m_name)
    );

}

PeriodicSurface::PeriodicSurface() {
    std::cout << "a message from periodic surface" << std::endl;
}

bool PeriodicSurface::request_set_point(mvp_control::ControlProcess *set_point)
{

    *set_point = mvp_control::ControlProcess();

    ROS_INFO_STREAM("Requested set point from behavior " << m_name <<
             ", type periodic surface");

    return true;
}



PLUGINLIB_EXPORT_CLASS(helm::PeriodicSurface, helm::BehaviorBase)