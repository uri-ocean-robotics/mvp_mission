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
    Year: 2023

    Copyright (C) 2023 Smart Ocean Systems Laboratory
*/

#include "direct_control.h"
#include "pluginlib/class_list_macros.h"

using namespace helm;

DirectControl::DirectControl(){
    std::cout << "A message from the DirectControl" << std::endl;
}

DirectControl::~DirectControl() {
}

void DirectControl::initialize() {

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
    m_pnh->param<double>("max_x", m_max_x, 5.0);
    m_pnh->param<double>("max_y", m_max_y, 5.0);
    m_pnh->param<double>("max_z", m_max_z, 5.0);

    m_pnh->param<double>("max_roll", m_max_roll, M_PI_2);
    m_pnh->param<double>("max_pitch", m_max_pitch, M_PI_2);
    m_pnh->param<double>("max_yaw", m_max_yaw, M_PI);

    m_pnh->param<double>("max_surge", m_max_surge, 1.0);
    m_pnh->param<double>("max_sway", m_max_sway, 1.0);
    m_pnh->param<double>("max_heave", m_max_heave, 1.0);

    m_pnh->param<double>("max_roll_rate", m_max_roll_rate, M_PI);
    m_pnh->param<double>("max_pitch_rate", m_max_pitch_rate, M_PI);
    m_pnh->param<double>("max_yaw_rate", m_max_yaw_rate, M_PI);

    // Load desired values for control
    m_pnh->param<double>("desired_x", m_desired_x, 0.0);
    m_pnh->param<double>("desired_y", m_desired_y, 0.0);
    m_pnh->param<double>("desired_z", m_desired_z, 0.0);

    m_pnh->param<double>("desired_roll", m_desired_roll, 0.0);
    m_pnh->param<double>("desired_pitch", m_desired_pitch, 0.0);
    m_pnh->param<double>("desired_yaw", m_desired_yaw, 0.0);

    m_pnh->param<double>("desired_surge", m_desired_surge, 0.0);
    m_pnh->param<double>("desired_sway", m_desired_sway, 0.0);
    m_pnh->param<double>("desired_heave", m_desired_heave, 0.0);

    m_pnh->param<double>("desired_roll_rate", m_desired_roll_rate, 0.0);
    m_pnh->param<double>("desired_pitch_rate", m_desired_pitch_rate, 0.0);
    m_pnh->param<double>("desired_yaw_rate", m_desired_yaw_rate, 0.0);

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
        // for poistion
        mvp_msgs::ControlMode::DOF_X,
        mvp_msgs::ControlMode::DOF_Y,
        mvp_msgs::ControlMode::DOF_Z,
        // for orientation 
        mvp_msgs::ControlMode::DOF_ROLL,
        mvp_msgs::ControlMode::DOF_PITCH,
        mvp_msgs::ControlMode::DOF_YAW,
        // for velocity
        mvp_msgs::ControlMode::DOF_SURGE,
        mvp_msgs::ControlMode::DOF_SWAY,
        mvp_msgs::ControlMode::DOF_HEAVE,
        // for angular velocity
        mvp_msgs::ControlMode::DOF_ROLL_RATE,
        mvp_msgs::ControlMode::DOF_PITCH_RATE,
        mvp_msgs::ControlMode::DOF_YAW_RATE,
    };
}


void DirectControl::activated() {
    /**
     * @brief This function is called when the behavior gets activated.
     *
     * @note This function is an inherited virtual function. It may left
     * unimplemented.
     *
     * @details This function is called when the behavior internal state
     * defined by #BehaviorBase::m_actived changes to true.
     */
    std::cout << "DirectControl behavior is activated!" << std::endl;
}

void DirectControl::disabled() {
    /**
     * @brief This function is called when the behavior gets disabled.
     *
     * @note This function is an inherited virtual function. It may left
     * unimplemented.
     *
     * @details This function is called when the behavior internal state
     * defined by #BehaviorBase::m_actived changes to false.
     */
    std::cout << "DirectControl behavior is disabled!" << std::endl;
}

bool DirectControl::request_set_point(
    mvp_msgs::ControlProcess *set_point) {

    // Set position
    if(m_desired_x != 0.0) {
        // check the limitation
        if(m_desired_x > m_max_x) {
            set_point->position.x = m_max_x;
        }
        else if (m_desired_x < -m_max_x) {
            set_point->position.x = -m_max_x;
        }
        else {
            set_point->position.x = m_desired_x;
        }
    }
    if(m_desired_y != 0.0) {
        // check the limitation
        if(m_desired_y > m_max_y) {
            set_point->position.y = m_max_y;
        }
        else if (m_desired_y < -m_max_y) {
            set_point->position.y = -m_max_y;
        }
        else {
            set_point->position.y = m_desired_y;
        }

    }
    if(m_desired_z != 0.0) {
        // check the limitation
        if(m_desired_z > m_max_z) {
            set_point->position.z = m_max_z;
        }
        else if (m_desired_z < -m_max_z) {
            set_point->position.z = -m_max_z;
        }
        else {
            set_point->position.z = m_desired_z;
        }
    }

    // Set orientation
    if(m_desired_roll != 0.0) {
        // check the limitation
        if(m_desired_roll > m_max_roll) {
            set_point->orientation.x = m_max_roll;
        }
        else if(m_desired_roll < -m_max_roll) {
            set_point->orientation.x = -m_max_roll;
        }       
        else {
            set_point->orientation.x = m_desired_roll;
        } 
    }
    if(m_desired_pitch != 0.0) {
        // check the limitation
        if(m_desired_pitch > m_max_pitch) {
            set_point->orientation.y = m_max_pitch;
        }
        else if(m_desired_pitch < -m_max_pitch) {
            set_point->orientation.y = -m_max_pitch;
        }       
        else {
            set_point->orientation.y = m_desired_pitch;
        } 
    }
    if(m_desired_yaw != 0.0) {
        // check the limitation
        if(m_desired_yaw > m_max_yaw) {
            set_point->orientation.z = m_max_yaw;
        }
        else if(m_desired_yaw < -m_max_yaw) {
            set_point->orientation.z = -m_max_yaw;
        }       
        else {
            set_point->orientation.z = m_desired_yaw;
        } 
    }

    // Set velocity
    if(m_desired_surge != 0.0) {
        // check the limitation
        if(m_desired_surge > m_max_surge) {
            set_point->velocity.x = m_max_surge;
        }
        else if(m_desired_surge < -m_max_surge) {
            set_point->velocity.x = -m_max_surge;
        }
        else {
            set_point->velocity.x = m_desired_surge;
        }
    }
    if(m_desired_sway != 0.0) {
        // check the limitation
        if(m_desired_sway > m_max_sway) {
            set_point->velocity.y = m_max_sway;
        }
        else if(m_desired_sway < -m_max_sway) {
            set_point->velocity.y = -m_max_sway;
        }
        else {
            set_point->velocity.y = m_desired_sway;
        }
    }
    if(m_desired_heave != 0.0) {
        // check the limitation
        if(m_desired_heave > m_max_heave) {
            set_point->velocity.z = m_max_heave;
        }
        else if(m_desired_heave < -m_max_heave) {
            set_point->velocity.z = -m_max_heave;
        }
        else {
            set_point->velocity.z = m_desired_heave;
        }      
    }

    // Set angular velocity
    if(m_desired_roll_rate != 0.0) {
        // check the limitation
        if(m_desired_roll_rate > m_max_roll_rate) {
            set_point->angular_rate.x = m_max_roll_rate;
        }
        else if(m_desired_roll_rate < -m_max_roll_rate) {
            set_point->angular_rate.x = -m_max_roll_rate;
        }
        else {
            set_point->angular_rate.x = m_desired_roll_rate;
        }
    }
    if(m_desired_pitch_rate != 0.0) {
        // check the limitation
        if(m_desired_pitch_rate > m_max_pitch_rate) {
            set_point->angular_rate.y = m_max_pitch_rate;
        }
        else if(m_desired_pitch_rate < -m_max_pitch_rate) {
            set_point->angular_rate.y = -m_max_pitch_rate;
        }
        else {
            set_point->angular_rate.y = m_desired_pitch_rate;
        }
    }
    if(m_desired_yaw_rate != 0.0) {
        // check the limitation
        if(m_desired_yaw_rate > m_max_yaw_rate) {
            set_point->angular_rate.z = m_max_yaw_rate;
        }
        else if(m_desired_yaw_rate < -m_max_yaw_rate) {
            set_point->angular_rate.z = -m_max_yaw_rate;
        }
        else {
            set_point->angular_rate.z = m_desired_yaw_rate;
        }
    }

    return true;
}

/**
 * @brief Behavior must export the class to the Plugin library.
 */
PLUGINLIB_EXPORT_CLASS(helm::DirectControl, helm::BehaviorBase)