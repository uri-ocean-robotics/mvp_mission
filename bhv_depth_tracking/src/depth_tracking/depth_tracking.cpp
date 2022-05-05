#include "depth_tracking.h"
#include "pluginlib/class_list_macros.h"

using namespace helm;

void DepthTracking::initialize() {

    m_nh.reset(
        new ros::NodeHandle(ros::this_node::getName() + "/" + m_name)
    );

    m_dofs = decltype(m_dofs){
        ctrl::DOF::PITCH,
    };

    m_sub = m_nh->subscribe(
        "desired_depth", 100, &DepthTracking::f_cb_sub, this);

    m_nh->param("initialize_depth",m_requested_depth, 0.0);

    m_nh->param("p_gain", m_p_gain, 1.0);

    m_nh->param("d_gain", m_d_gain, 0.0);

    m_nh->param("max_pitch", m_max_pitch, M_PI_2);
}

void DepthTracking::f_cb_sub(const std_msgs::Float64::ConstPtr &msg) {

    m_requested_depth = msg->data;

}

DepthTracking::DepthTracking() {
    std::cout << "a message from depth tracking" << std::endl;
}

DepthTracking::~DepthTracking() {
    m_sub.shutdown();
}

bool DepthTracking::request_set_point(mvp_control::ControlProcess *set_point) {

    auto error = m_requested_depth - m_process_values.position.z;

    auto pitch = - (m_p_gain * error + m_d_gain * (-m_process_values.velocity.z));

    if(fabs(pitch) > m_max_pitch) {
        set_point->orientation.y = pitch >= 0 ? m_max_pitch : -m_max_pitch;
    } else {
        set_point->orientation.y = pitch;
    }

    return true;
}


PLUGINLIB_EXPORT_CLASS(helm::DepthTracking, helm::BehaviorBase)