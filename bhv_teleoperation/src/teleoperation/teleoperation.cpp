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

    Author: Lin Zhao
    Email: linzhao@uri.edu
    Year: 2022

    Copyright (C) 2022 Smart Ocean Systems Laboratory
*/


#include "teleoperation.h"
#include "pluginlib/class_list_macros.h"

using namespace helm;

Teleoperation::Teleoperation()
  : m_use_joy(false), m_last_yaw (false), m_last_pitch(false) {
    std::cout << "A message from the teleoperation" << std::endl;
}


Teleoperation::~Teleoperation() {

    m_joy_sub.shutdown();

}

void Teleoperation::initialize() {

    /**
     * @brief Initialize node handler with the behavior namespace
     *
     * @details Parameters for the behavior is loaded under
     * /helm/<behavior_name> namespace. Therefore, nodehandler must use that
     * name as well or should take that namespace into account when reading the
     * parameters.
     *
     * @note variables with redundant class names are used for emphesizing the
     * base class member variables.
     */
    m_pnh.reset(
        new ros::NodeHandle(ros::this_node::getName() + "/" + get_name())
    );

    m_nh.reset(new ros::NodeHandle(""));

    // ROS related: load parameters, setup sub/pub

    m_pnh->param<double>("max_surge", m_max_surge, 1.0);
    m_pnh->param<double>("max_pitch_rate", m_max_pitch_rate, 3.15);
    m_pnh->param<double>("max_yaw_rate", m_max_yaw_rate, 3.15);
    m_pnh->param<std::string>("joy_topic", m_joy_topic_name, "joy");
    m_pnh->param<int>("joy_map/axes_surge", m_axes_surge, 1);
    m_pnh->param<int>("joy_map/axes_pitch", m_axes_pitch, 4);
    m_pnh->param<int>("joy_map/axes_yaw", m_axes_yaw, 3);
    m_pnh->param<int>("joy_map/button_enable", m_button_enable, 5);


    m_joy_sub = m_nh->subscribe(m_joy_topic_name, 100, &Teleoperation::f_joy_cb, this);

    /**
     * @brief Declare the degree of freedoms to be controlled by the behavior
     *
     * @details This member variable dictates the DOFs that can be controllable
     * by the behavior. If this member is not initialized, behavior can only
     * trigger state changes. This vector gets values from enum type of
     * mvp_msgs/ControlMode enums.
     *
     */
    BehaviorBase::m_dofs = decltype(m_dofs){
        // for flight mode
        mvp_msgs::ControlMode::DOF_SURGE,
        mvp_msgs::ControlMode::DOF_PITCH,
        mvp_msgs::ControlMode::DOF_YAW,

        // for another control mode
        mvp_msgs::ControlMode::DOF_SURGE,
        mvp_msgs::ControlMode::DOF_PITCH_RATE,
        mvp_msgs::ControlMode::DOF_YAW_RATE,
    };
}

void Teleoperation::f_joy_cb(const sensor_msgs::Joy::ConstPtr &m) {
    // grab control value from joystick
    m_joy_surge = m->axes[m_axes_surge];
    m_joy_yaw_rate = m->axes[m_axes_yaw];
    m_joy_pitch_rate = m->axes[m_axes_pitch];

    // open or close joystick: RB
    if (m->buttons[m_button_enable] == 1 ) {
        // first time enable joystick and record vehicle pose
        if(!m_use_joy) {
            // record global information
            m_recorded_picth = BehaviorBase::m_process_values.orientation.y;
            m_recorded_yaw = BehaviorBase::m_process_values.orientation.z;
        }

        m_use_joy = true;

        // only record pose if pitch input decrease to 0
        if(m_joy_pitch_rate == 0) {
            if(m_last_pitch) {
                // record global information
                m_recorded_picth = BehaviorBase::m_process_values.orientation.y;
                m_last_pitch = false;
            }
        }
        else {
            m_last_pitch = true;
        }

        // only record pose if yaw input decrease to 0
        if(m_joy_yaw_rate == 0) {
            if(m_last_yaw) {
                // record global information
                m_recorded_yaw = BehaviorBase::m_process_values.orientation.z;
                m_last_yaw = false;
            }
        }
        else {
            m_last_yaw = true;
        }
    }
    else {
        m_use_joy = false;
    }
}


void Teleoperation::activated() {
    /**
     * @brief This function is called when the behavior gets activated.
     *
     * @note This function is an inherited virtual function. It may left
     * unimplemented.
     *
     * @details This function is called when the behavior internal state
     * defined by #BehaviorBase::m_actived changes to true.
     */
    std::cout << "teleoperation behavior is activated!" << std::endl;
}

void Teleoperation::disabled() {
    /**
     * @brief This function is called when the behavior gets disabled.
     *
     * @note This function is an inherited virtual function. It may left
     * unimplemented.
     *
     * @details This function is called when the behavior internal state
     * defined by #BehaviorBase::m_actived changes to false.
     */
    std::cout << "teleoperation behavior is disabled!" << std::endl;
}

//! NOTE: for the pitch and yaw, we can't direct assign the joystick value as desired_value,
//!       because pitch and yaw are in the global frame, but surge is ok. it's in the body frame.
bool Teleoperation::request_set_point(
    mvp_msgs::ControlProcess *set_point) {

    if( !m_use_joy ) {
        return false;
    }

    // get surge input
    double surge_rate = m_max_surge * m_joy_surge.load(std::memory_order_relaxed);
    // get pitch input
    double pitch_rate = m_max_pitch_rate * m_joy_pitch_rate.load(std::memory_order_relaxed);
    double pitch_angle = pitch_rate * (1.0 / get_helm_frequency());
    // get yaw input
    double yaw_rate = m_max_yaw_rate * m_joy_yaw_rate.load(std::memory_order_relaxed);
    double yaw_angle = yaw_rate * (1.0 / get_helm_frequency());

    // Set body frame velocity
    set_point->velocity.x = surge_rate;
    set_point->angular_rate.y = pitch_rate;
    set_point->angular_rate.z =  yaw_rate;

    // Set global frame orientation angles
    set_point->orientation.y =  m_recorded_picth + pitch_angle;
    set_point->orientation.z =  m_recorded_yaw - yaw_angle;

    return true;
}

/**
 * @brief Behavior must export the class to the Plugin library.
 */
PLUGINLIB_EXPORT_CLASS(helm::Teleoperation, helm::BehaviorBase)