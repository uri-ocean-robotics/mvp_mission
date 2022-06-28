#include "depth_tracking.h"
#include "pluginlib/class_list_macros.h"

using namespace helm;

void DepthTracking::initialize() {

    m_nh.reset(
        new ros::NodeHandle(ros::this_node::getName() + "/" + m_name)
    );

    //! @par Declare the dofs to be controlled
    m_dofs = decltype(m_dofs){
        ctrl::DOF::PITCH,
        ctrl::DOF::Z
    };

    m_sub = m_nh->subscribe(
        "desired_depth", 100, &DepthTracking::f_cb_sub, this);

    m_nh->param("desired_depth",m_requested_depth, 0.0);

    m_nh->param("max_pitch", m_max_pitch, M_PI_2);

    m_nh->param("forward_distance", m_fwd_distance, 3.0);

    m_nh->param("use_heave_velocity", m_use_heave_velocity, false);

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

    //! @note Set Pitch angle.

    //! @note I didn't want to change the sign afterwards.
    auto error = m_process_values.position.z - m_requested_depth;

    double pitch;

    pitch = atan(error / m_fwd_distance);

    if(m_use_heave_velocity) {
        if(m_process_values.velocity.x != 0)  {
            pitch += atan(
                m_process_values.velocity.z / m_process_values.velocity.x);
        }
    }

     if(fabs(pitch) > m_max_pitch) {
        if(pitch >= 0) {
            pitch = m_max_pitch;
        } else {
            pitch = -m_max_pitch;
        }
    }
    set_point->orientation.y = pitch;

    //! @note Set depth directly so low-level controller deals with it.

    set_point->position.z = m_requested_depth;

    return true;
}


PLUGINLIB_EXPORT_CLASS(helm::DepthTracking, helm::BehaviorBase)