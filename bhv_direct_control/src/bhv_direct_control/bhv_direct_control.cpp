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

    Author: Mingxi Zhou
    Email: mzhou@uri.edu
    Year: 2024

    Copyright (C) 2024 Smart Ocean Systems Laboratory
*/
#include "bhv_direct_control/bhv_direct_control.h"
#include "tf2/time.h"
#include <chrono>
#include <functional>
#include <memory>

using namespace helm;
using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

DirectControl::DirectControl() {
    std::cout << "A message from the DirectControl" << std::endl;
}

DirectControl::~DirectControl() {
}

void DirectControl::initialize(const rclcpp::Node::WeakPtr &parent) 
{
    /*************************************************************************/
    /* get the node */
    m_node = parent;
    auto node = m_node.lock();
    m_logger = node->get_logger();

    /*************************************************************************/
    /* Load Parameters for ROS2 */
    
    //setup tf buffer
    m_transform_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    m_transform_listener = std::make_unique<tf2_ros::TransformListener>(*m_transform_buffer);

    //use array instead of 12 variables
    std::string prefix = get_name() + "/";

    //load initial desired values
    node->declare_parameter(prefix + "desired_x", 0.0);
    node->get_parameter(prefix + "desired_x", m_bhv_setpoint.position.x);
    node->declare_parameter(prefix + "desired_y", 0.0);
    node->get_parameter(prefix + "desired_y", m_bhv_setpoint.position.y);
    node->declare_parameter(prefix + "desired_z", 0.0);
    node->get_parameter(prefix + "desired_z", m_bhv_setpoint.position.z);
    
    node->declare_parameter(prefix + "desired_roll", 0.0);
    node->get_parameter(prefix + "desired_roll", m_bhv_setpoint.orientation.x);
    node->declare_parameter(prefix + "desired_pitch", 0.0);
    node->get_parameter(prefix + "desired_pitch", m_bhv_setpoint.orientation.y);
    node->declare_parameter(prefix + "desired_yaw", 0.0);
    node->get_parameter(prefix + "desired_yaw", m_bhv_setpoint.orientation.z);
    
    node->declare_parameter(prefix + "desired_u", 0.0);
    node->get_parameter(prefix + "desired_u", m_bhv_setpoint.velocity.x);
    node->declare_parameter(prefix + "desired_v", 0.0);
    node->get_parameter(prefix + "desired_v", m_bhv_setpoint.velocity.y);
    node->declare_parameter(prefix + "desired_w", 0.0);
    node->get_parameter(prefix + "desired_w", m_bhv_setpoint.velocity.z);

    node->declare_parameter(prefix + "desired_p", 0.0);
    node->get_parameter(prefix + "desired_p", m_bhv_setpoint.angular_rate.x);
    node->declare_parameter(prefix + "desired_q", 0.0);
    node->get_parameter(prefix + "desired_q", m_bhv_setpoint.angular_rate.y);
    node->declare_parameter(prefix + "desired_r", 0.0);
    node->get_parameter(prefix + "desired_r", m_bhv_setpoint.angular_rate.z);

    std::string node_name = node->get_name();
    std::string ns = node->get_namespace();
    if (!ns.empty() && ns[0] == '/') {
        ns = ns.substr(1);
    }

    std::string global_link, child_link;
    //define the frames that all calculation will be based on 
    node->declare_parameter(prefix + "default_bhv_world_link", "world_ned");
    node->get_parameter(prefix + "default_bhv_world_link", global_link);

    node->declare_parameter(prefix + "default_bhv_child_link", "cg_link");
    node->get_parameter(prefix + "default_bhv_child_link", child_link);
    
    bhv_global_link = ns + "/" + global_link;
    bhv_child_link = ns + "/" + child_link;

    m_bhv_setpoint.header.frame_id = bhv_global_link;
    m_bhv_setpoint.child_frame_id = bhv_child_link;


    ///topics
    m_setpoint_sub = node->create_subscription<mvp_msgs::msg::ControlProcess>("~/"+ prefix + "desired_setpoints", 10, 
                                                                std::bind(&DirectControl::m_setpoint_callback, 
                                                                this, _1));



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
        mvp_msgs::msg::ControlMode::DOF_X,
        mvp_msgs::msg::ControlMode::DOF_Y,
        mvp_msgs::msg::ControlMode::DOF_Z,
        // for orientation 
        mvp_msgs::msg::ControlMode::DOF_ROLL,
        mvp_msgs::msg::ControlMode::DOF_PITCH,
        mvp_msgs::msg::ControlMode::DOF_YAW,
        // for velocity
        mvp_msgs::msg::ControlMode::DOF_U,
        mvp_msgs::msg::ControlMode::DOF_V,
        mvp_msgs::msg::ControlMode::DOF_W,
    };

}


void DirectControl::m_setpoint_callback(const mvp_msgs::msg::ControlProcess::SharedPtr msg)
{
    
    mvp_msgs::msg::ControlProcess::SharedPtr temp_setpoint = std::make_shared<mvp_msgs::msg::ControlProcess>();
    // auto temp_setpoint = std::make_shared<mvp_msgs::msg::ControlProcess>();
    // mvp_msgs::msg::ControlProcess data =*msg;
    transform_control_process_msg(*msg, temp_setpoint, get_helm_world_link(), get_helm_child_link());
    m_bhv_setpoint = *temp_setpoint;
    

}


void DirectControl::activated() 
{

    std::cout << "DirectControl behavior is activated!" << std::endl;
}

void DirectControl::disabled() 
{
    std::cout << "DirectControl behavior is disabled!" << std::endl;
}


bool DirectControl::request_set_point(
    mvp_msgs::msg::ControlProcess *set_point) 
{
    *set_point = m_bhv_setpoint;

    return true;
}


/**
 * @brief Behavior must export the class to the Plugin library.
 */
 #include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(helm::DirectControl, helm::BehaviorBase)