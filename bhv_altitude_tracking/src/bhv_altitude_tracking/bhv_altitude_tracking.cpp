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
#include "tf2/time.h"

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

    //get vector params

    node->declare_parameter(prefix + "desired_altitude", m_desired_altitude);
    node->get_parameter(prefix + "desired_altitude", m_desired_altitude);

    node->declare_parameter(prefix + "altitude_tracking_mode", m_altitude_mode);
    node->get_parameter(prefix + "altitude_tracking_mode", m_altitude_mode);

    node->declare_parameter(prefix + "no_altitude_state", "start");
    node->get_parameter(prefix + "no_altitude_state", m_state_done);

    node->declare_parameter(prefix + "pitch_gain", 0.0);
    node->get_parameter(prefix + "pitch_gain", m_pitch_gain);

    node->declare_parameter(prefix + "pitch_assist_band", 0.0);
    node->get_parameter(prefix + "pitch_assist_band", m_pitch_assist_band);

    node->declare_parameter(prefix + "max_pitch", 0.0);
    node->get_parameter(prefix + "max_pitch", m_max_pitch);

    node->declare_parameter(prefix + "no_altitude_timeout", 5.0);
    node->get_parameter(prefix + "no_altitude_timeout", m_no_altitude_timeout);


    std::string m_altitude_measurement_topic;
    std::string m_desired_altitude_topic;

    node->declare_parameter(prefix + "altitude_measurement_topic", m_altitude_measurement_topic);
    node->get_parameter(prefix + "altitude_measurement_topic", m_altitude_measurement_topic);

    node->declare_parameter(prefix + "desired_altitude_topic", m_desired_altitude_topic);
    node->get_parameter(prefix + "desired_altitude_topic", m_desired_altitude_topic);
    
    m_altitude_sub = node->create_subscription<geometry_msgs::msg::PointStamped>(m_altitude_measurement_topic, 1, 
                                                                std::bind(&AltitudeTracking::f_m_altitude_cb, 
                                                                this, _1));

    m_desired_altitude_sub = node->create_subscription<std_msgs::msg::Float64>("~/"+ prefix + m_desired_altitude_topic, 1, 
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
    //msg is a point in the sensor frame?
    auto steady_clock = rclcpp::Clock();
    //transform point stamp into world_ned
    geometry_msgs::msg::PointStamped point_in_bhv_global;
   
    try{
        // printf("transforming frame from %s to %s\r\n", msg->header.frame_id.c_str(), m_bhv_setpoint.header.frame_id.c_str());
        // RCLCPP_INFO(
        //     m_logger,
        //     "Header time: sec = %d, nanosec = %u",
        //     msg->header.stamp.sec,
        //     msg->header.stamp.nanosec
        // );
        point_in_bhv_global = m_transform_buffer->transform(*msg, m_bhv_setpoint.header.frame_id.c_str(), 1000ms);   
        m_bottom_depth =  point_in_bhv_global.point.z;
        // printf("altitude transformed =%lf\n\r", m_bottom_depth);
        m_last_altitude_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    }

    catch (tf2::TransformException &ex) {
        RCLCPP_WARN_STREAM_THROTTLE(m_logger, steady_clock, 1000, std::string("altitude_tracking warning") + ex.what());
        // RCLCPP_ERROR(m_logger, "Transform failed: %s", ex.what());
    }
}

void AltitudeTracking::f_c_altitude_cb(const std_msgs::msg::Float64::SharedPtr msg)
{
    //this will be positive 
    m_desired_altitude = msg->data;

    m_desired_altitude = std::max( 1.0, m_desired_altitude); //limit the minimum altitude to 1 meter.
}


bool AltitudeTracking::request_set_point(mvp_msgs::msg::ControlProcess *set_point) 
{
    //get the current pose and transform into the bhv frame
    auto temp_pose = std::make_shared<mvp_msgs::msg::ControlProcess>();
    transform_control_process_msg(BehaviorBase::m_process_values, temp_pose, 
                                  m_bhv_setpoint.header.frame_id, m_bhv_setpoint.child_frame_id);

    double m_depth = temp_pose->position.z;
    double m_pitch = temp_pose->orientation.y;

    double c_depth;
    double m_d_pitch;
    auto steady_clock = rclcpp::Clock();

    if(rclcpp::Clock(RCL_ROS_TIME).now().seconds()-m_last_altitude_time > m_no_altitude_timeout)
    {
        RCLCPP_WARN_STREAM_THROTTLE(m_logger, steady_clock, 1000, std::string("no altitude timeout trigged"));

        //reserved for change state//
        change_state(m_state_done);
        return false;
    }

    switch(m_altitude_mode)
    {
        case 0:
            if(m_depth > m_bottom_depth - m_desired_altitude){
                c_depth = m_bottom_depth - m_desired_altitude;
                // printf("altitude safety depth =%lf\n\r", set_point->position.z);
                
                RCLCPP_WARN_STREAM_THROTTLE(m_logger, steady_clock, 1000, std::string("minimum altitude exceeded"));
            }
            else{
                return false;
            }
            break;
        //continuous following
        case 1: 
            c_depth = m_bottom_depth - m_desired_altitude;
            break;
        
        default:
            return false;
            break;
    }

    //update the bhv set point
    //compute the desired pitch
    if(fabs(c_depth - m_depth) > m_pitch_assist_band)
    {
    
    m_d_pitch = -m_pitch_gain * (c_depth - m_depth); //positive error needs a negative pitch in cg_link
    m_d_pitch = std::min(std::max(m_d_pitch, -m_max_pitch), m_max_pitch);

    }
    else{
        m_d_pitch = 0;
    }


    m_bhv_setpoint.orientation.y = m_pitch + m_d_pitch;
    m_bhv_setpoint.position.z = c_depth;
    auto temp_setpoint = std::make_shared<mvp_msgs::msg::ControlProcess>();
    transform_control_process_msg(m_bhv_setpoint, temp_setpoint, get_helm_world_link(), get_helm_child_link());

    *set_point = *temp_setpoint;

    return true;
}

/**
 * @brief Behavior must export the class to the Plugin library.
 */
 #include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(helm::AltitudeTracking, helm::BehaviorBase)