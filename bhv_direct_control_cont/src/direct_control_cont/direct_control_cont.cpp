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

#include "direct_control_cont.h"
#include "pluginlib/class_list_macros.h"

using namespace helm;

DirectControlCont::DirectControlCont(){
    std::cout << "A message from the DirectControlCont" << std::endl;
}

DirectControlCont::~DirectControlCont() {
}

void DirectControlCont::initialize() {

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

    m_transform_listener.reset(new
        tf2_ros::TransformListener(m_transform_buffer)
    );
    
    // Set frame id
    m_pnh->param<std::string>("global_link", global_link, "world_ned");
    m_pnh->param<std::string>("local_link", local_link, "cg_link");

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

    // Subscriber for new command
    continuous_command_sub = m_nh->subscribe("continuous_command_topic", 100, &DirectControlCont::continuous_update, this);

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


void DirectControlCont::continuous_update(const mvp_msgs::ControlProcess::ConstPtr& new_values){

    // Saturation
    // Set Position
    m_desired_x = std::min(std::max(new_values->position.x, -m_max_x), m_max_x);
    m_desired_y = std::min(std::max(new_values->position.y, -m_max_y), m_max_y);
    m_desired_z = std::min(std::max(new_values->position.z, -m_max_z), m_max_z);

    // Set orientation
    m_desired_roll = std::min(std::max(new_values->orientation.x, -m_max_roll), m_max_roll);
    m_desired_pitch = std::min(std::max(new_values->orientation.y, -m_max_pitch), m_max_pitch);
    m_desired_yaw = std::min(std::max(new_values->orientation.z, -m_max_yaw), m_max_yaw);

    // Set velocity
    m_desired_surge = std::min(std::max(new_values->velocity.x, -m_max_surge), m_max_surge);
    m_desired_sway = std::min(std::max(new_values->velocity.y, -m_max_sway), m_max_sway);
    m_desired_heave = std::min(std::max(new_values->velocity.z, -m_max_heave), m_max_heave);

    // Set angular velocity
    m_desired_roll_rate = std::min(std::max(new_values->angular_rate.x, -m_max_roll_rate), m_max_roll_rate);
    m_desired_pitch_rate = std::min(std::max(new_values->angular_rate.y, -m_max_pitch_rate), m_max_pitch_rate);
    m_desired_yaw_rate = std::min(std::max(new_values->angular_rate.z, -m_max_yaw_rate), m_max_yaw_rate);


    Eigen::Vector3d p_world, rpy_world;
    try {
        // Transform the position from user defined global link to mvp_control global link
        auto tf_world_setpoint = m_transform_buffer.lookupTransform(
            get_helm_global_link(),
            global_link,
            ros::Time::now(),
            ros::Duration(10.0)
        );

        auto tf_eigen = tf2::transformToEigen(tf_world_setpoint);

        p_world = tf_eigen.rotation() * 
                                  Eigen::Vector3d(m_desired_x, m_desired_y, m_desired_z)
                                  + tf_eigen.translation();
        rpy_world = tf_eigen.rotation() * 
                                    Eigen::Vector3d(m_desired_roll, m_desired_pitch, m_desired_yaw);

        //assume the set point uvw and pqr are in the m_cg_link_id

    } catch(tf2::TransformException &e) {
        ROS_WARN_STREAM_THROTTLE(10, std::string("Can't ???: ") + e.what());
        return;
    }

    m_desired_x = p_world.x();    
    m_desired_y = p_world.y();    
    m_desired_z = p_world.z();    

    m_desired_roll = rpy_world.x();
    m_desired_pitch = rpy_world.y();
    m_desired_yaw = rpy_world.z();

    // Assume uvw, pqr in cg_link
    local_link = "cg_link";
    m_desired_surge = m_desired_surge;
    m_desired_sway = m_desired_sway;
    m_desired_heave = m_desired_heave; 

    m_desired_roll_rate = m_desired_roll_rate;
    m_desired_pitch_rate = m_desired_pitch_rate;
    m_desired_yaw_rate = m_desired_yaw_rate;

}


void DirectControlCont::activated() {
    /**
     * @brief This function is called when the behavior gets activated.
     *
     * @note This function is an inherited virtual function. It may left
     * unimplemented.
     *
     * @details This function is called when the behavior internal state
     * defined by #BehaviorBase::m_actived changes to true.
     */
    std::cout << "DirectControlCont behavior is activated!" << std::endl;
}

void DirectControlCont::disabled() {
    /**
     * @brief This function is called when the behavior gets disabled.
     *
     * @note This function is an inherited virtual function. It may left
     * unimplemented.
     *
     * @details This function is called when the behavior internal state
     * defined by #BehaviorBase::m_actived changes to false.
     */
    std::cout << "DirectControlCont behavior is disabled!" << std::endl;
}

bool DirectControlCont::request_set_point(
    mvp_msgs::ControlProcess *set_point) {
    
    // Set Position
    set_point->position.x = m_desired_x;
    set_point->position.y = m_desired_y;
    set_point->position.z = m_desired_z;

    // Set orientation
    set_point->orientation.x = m_desired_roll;
    set_point->orientation.y = m_desired_pitch;
    set_point->orientation.z = m_desired_yaw;

    // Set velocity
    set_point->velocity.x = m_desired_surge;
    set_point->velocity.y = m_desired_sway;
    set_point->velocity.z = m_desired_heave;
   
    // Set angular velocity
    set_point->angular_rate.x = m_desired_roll_rate;
    set_point->angular_rate.y = m_desired_pitch_rate;
    set_point->angular_rate.z = m_desired_yaw_rate;

    return true;
}

/**
 * @brief Behavior must export the class to the Plugin library.
 */
PLUGINLIB_EXPORT_CLASS(helm::DirectControlCont, helm::BehaviorBase)