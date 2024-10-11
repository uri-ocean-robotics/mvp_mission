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

    node->declare_parameter(prefix + "pitch_gain", 0.0);
    node->get_parameter(prefix + "pitch_gain", m_pitch_gain);

    node->declare_parameter(prefix + "pitch_assist_band", 0.0);
    node->get_parameter(prefix + "pitch_assist_band", m_pitch_assist_band);

    node->declare_parameter(prefix + "max_pitch", 0.0);
    node->get_parameter(prefix + "max_pitch", m_max_pitch);

    std::string m_altitude_measurement_topic;
    std::string m_desired_altitude_topic;

    node->declare_parameter(prefix + "altitude_measurement_topic", m_altitude_measurement_topic);
    node->get_parameter(prefix + "altitude_measurement_topic", m_altitude_measurement_topic);

    node->declare_parameter(prefix + "desired_altitude_topic", m_desired_altitude_topic);
    node->get_parameter(prefix + "desired_altitude_topic", m_desired_altitude_topic);
    
    m_altitude_sub = node->create_subscription<geometry_msgs::msg::PointStamped>(m_altitude_measurement_topic, 100, 
                                                                std::bind(&AltitudeTracking::f_m_altitude_cb, 
                                                                this, _1));

    m_desired_altitude_sub = node->create_subscription<std_msgs::msg::Float64>("~/"+ prefix + m_desired_altitude_topic, 100, 
                                                                std::bind(&AltitudeTracking::f_c_altitude_cb, 
                                                                this, _1));
    BehaviorBase::m_dofs = decltype(m_dofs){
        mvp_msgs::msg::ControlMode::DOF_Z,
        mvp_msgs::msg::ControlMode::DOF_PITCH
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
    geometry_msgs::msg::PointStamped ps;
    ps.header = msg->header;
    ps.point.x = msg->point.x;
    ps.point.y = msg->point.y;
    ps.point.z = msg->point.z;
    // printf("target frame = %s\r\n", get_helm_global_link().c_str());
    // printf("point frame = %s\r\n", ps.header.frame_id.c_str());
    try{
        point_in_helm_global = m_transform_buffer->transform(ps, get_helm_global_link().c_str(), tf2::durationFromSec(1.0));   
        m_bottom_depth =  point_in_helm_global.point.z;
        // printf("altitude transformed =%lf\n\r", m_bottom_depth);
    }

    catch (tf2::TransformException &ex) {
        RCLCPP_WARN_STREAM_THROTTLE(m_logger, steady_clock, 10, std::string("Could NOT transform dvl altitude into bottom depth"));
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
            if(m_bottom_depth - m_desired_altitude < BehaviorBase::m_process_values.position.z)
            {
                set_point->position.z = m_bottom_depth - m_desired_altitude;
                // printf("altitude safety depth =%lf\n\r", set_point->position.z);
                auto steady_clock = rclcpp::Clock();
                RCLCPP_WARN_STREAM_THROTTLE(m_logger, steady_clock, 10, std::string("minimum altitude exceeded"));

                //compute the desired pitch
                if(fabs(set_point->position.z - BehaviorBase::m_process_values.position.z) > m_pitch_assist_band)
                {
                m_pitch = -m_pitch_gain * (set_point->position.z - BehaviorBase::m_process_values.position.z); //positive error needs a negative pitch in cg_link
                m_pitch = std::min(std::max(m_pitch, -m_max_pitch), m_max_pitch);
                }
                else{
                    m_pitch = 0;
                }
                set_point->orientation.y = m_pitch;
                return true;
            }
            break;

        case 1:
            set_point->position.z = m_bottom_depth - m_desired_altitude;
            // printf("altitude tracking depth =%lf\n\r", set_point->position.z);
            // printf("altitude =%lf\n\r", m_bottom_depth);
            //compute the desired pitch
            if(fabs(set_point->position.z - BehaviorBase::m_process_values.position.z) > m_pitch_assist_band)
            {
            m_pitch = -m_pitch_gain * (set_point->position.z - BehaviorBase::m_process_values.position.z); //positive error needs a negative pitch in cg_link
            m_pitch = std::min(std::max(m_pitch, -m_max_pitch), m_max_pitch);

            }
            else{
                m_pitch = 0;
            }
            set_point->orientation.y = m_pitch;
            

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