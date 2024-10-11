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

#include "bhv_altitude_tracking/bhv_altitude_tracking.h"
#include <chrono>
#include <functional>
#include <memory>

using namespace helm;
using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


AltitudeTracking::AltitudeTracking() {
    std::cout << "A message from the AltitudeTracking" << std::endl;
}

AltitudeTracking::~AltitudeTracking() {
}

void AltitudeTracking::initialize(const rclcpp::Node::WeakPtr &parent) 
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
    std::string prefix = get_name() + "/";

    std::string node_name = node->get_name();
    std::string ns = node->get_namespace();
    if (!ns.empty() && ns[0] == '/') {
        ns = ns.substr(1);
    }
    //get vector params

    node->declare_parameter(prefix + "desired_altitude", m_desired_altitude);
    node->get_parameter(prefix + "desired_altitude", m_desired_altitude);

    node->declare_parameter(prefix + "altitude_tracking_mode", m_altitude_mode);
    node->get_parameter(prefix + "altitude_tracking_mode", m_altitude_mode);
    
    m_altitude_sub = node->create_subscription<geometry_msgs::msg::PointStamped>("~/"+ prefix + "altitude_data", 100, 
                                                                std::bind(&AltitudeTracking::f_m_altitude_cb, 
                                                                this, _1));

    m_desired_altitude_sub = node->create_subscription<std_msgs::msg::Float64>("~/"+ prefix + "desired_altitude", 100, 
                                                                std::bind(&AltitudeTracking::f_c_altitude_cb, 
                                                                this, _1));
    BehaviorBase::m_dofs = decltype(m_dofs){
        mvp_msgs::msg::ControlMode::DOF_Z,
    };
}


void AltitudeTracking::activated() 
{
    std::cout << "AltitudeTracking behavior is activated!" << std::endl;
}

void AltitudeTracking::disabled() 
{
    std::cout << "AltitudeTracking behavior is disabled!" << std::endl;
}

void AltitudeTracking::f_m_altitude_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    auto steady_clock = rclcpp::Clock();
    //transform point stamp into world_ned
    geometry_msgs::msg::PointStamped point_in_helm_global;

    try{
        point_in_helm_global = m_transform_buffer->transform(*msg, get_helm_global_link());   
        m_altitude =  point_in_helm_global.point.z;
    }

    catch (tf2::TransformException &ex) {
        RCLCPP_WARN_STREAM_THROTTLE(m_logger, steady_clock, 10, std::string("Could NOT transform"));
    }
}

void AltitudeTracking::f_c_altitude_cb(const std_msgs::msg::Float64::SharedPtr msg)
{
    m_desired_altitude = msg->data;
}


bool AltitudeTracking::request_set_point(mvp_msgs::msg::ControlProcess *set_point) 
{
    switch(m_altitude_mode)
    {
        case 0:
            if(m_altitude < m_desired_altitude)
            {
                set_point->position.z = BehaviorBase::m_process_values.position.z + m_altitude - m_desired_altitude;
                return true;
            }
            break;

        case 1:
            set_point->position.z = BehaviorBase::m_process_values.position.z + m_altitude - m_desired_altitude;
            return true;

        default:
            break;
    }

    return true;
}

/**
 * @brief Behavior must export the class to the Plugin library.
 */
 #include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(helm::AltitudeTracking, helm::BehaviorBase)