/*
    This file is part of MVP-Mission program.

    MVP-Mission is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MVP-Mission is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MVP-Mission.  If not, see <https://www.gnu.org/licenses/>.

    Author: Emir Cem Gezer
    Email: emircem@uri.edu;emircem.gezer@gmail.com
    Year: 2022

    Copyright (C) 2022 Smart Ocean Systems Laboratory
*/


#include "depth_tracking.h"
#include "pluginlib/class_list_macros.h"

using namespace helm;

void DepthTracking::initialize() {

    m_nh.reset(
        new ros::NodeHandle(ros::this_node::getName() + "/" + get_name())
    );

    //! @par Declare the dofs to be controlled
    BehaviorBase::m_dofs = decltype(m_dofs){
        mvp_msgs::ControlMode::DOF_Z,
        mvp_msgs::ControlMode::DOF_PITCH
    };

    m_sub = m_nh->subscribe(
        "desired_depth", 100, &DepthTracking::f_cb_sub, this);

    m_nh->param("desired_depth",m_requested_depth, 0.0);

    m_nh->param("max_pitch", m_max_pitch, M_PI_2);

    m_nh->param("forward_distance", m_fwd_distance, 3.0);

    m_nh->param("use_heave_velocity", m_use_heave_velocity, false);
    m_nh->param("pitch_enabled", m_pitch_enabled, false);

}

auto DepthTracking::configure_dofs() -> decltype(m_dofs) {

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

bool DepthTracking::request_set_point(mvp_msgs::ControlProcess *set_point) {

    //! @note Set Pitch angle.

    //! @note I didn't want to change the sign afterwards.
    auto error = BehaviorBase::m_process_values.position.z - m_requested_depth;

    double pitch;

    pitch = atan(error / m_fwd_distance);

    if(m_use_heave_velocity) {
        if(BehaviorBase::m_process_values.velocity.x != 0)  {
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
    set_point->orientation.y = 0;
    if(m_pitch_enabled){
    set_point->orientation.y = pitch;
    }

    //! @note Set depth directly so low-level controller deals with it.
    set_point->position.z = m_requested_depth;

    return true;
}


PLUGINLIB_EXPORT_CLASS(helm::DepthTracking, helm::BehaviorBase)