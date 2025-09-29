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

#include "bhv_depth_tracking/bhv_depth_tracking.h"
#include "tf2/time.h"
#include <chrono>
#include <functional>
#include <memory>

using namespace helm;
using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


DepthTracking::DepthTracking() {
    std::cout << "A message from the DirectControl" << std::endl;
}

DepthTracking::~DepthTracking() {
}

void DepthTracking::initialize(const rclcpp::Node::WeakPtr &parent) 
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
    //get vector params


    node->declare_parameter(prefix + "depth_list", c_depth_list);
    node->get_parameter(prefix + "depth_list", c_depth_list);

    node->declare_parameter(prefix + "pitch_list", c_pitch_list);
    node->get_parameter(prefix + "pitch_list", c_pitch_list);

    node->declare_parameter(prefix + "depth_time_list", c_depth_time_list);
    node->get_parameter(prefix + "depth_time_list", c_depth_time_list);

    node->declare_parameter(prefix + "acceptable_depth_band", m_depth_band);
    node->get_parameter(prefix + "acceptable_depth_band", m_depth_band);

    if(c_depth_list.size()!= c_pitch_list.size() || c_depth_time_list.size() !=c_depth_time_list.size())
    {
        printf("Warning!! depth tracking param size mismatch\r\n");
    }

    node->declare_parameter(prefix + "initial_wait_time", m_initial_wait_time);
    node->get_parameter(prefix + "initial_wait_time", m_initial_wait_time);


    node->declare_parameter(prefix + "depth_failed_timeout", m_depth_failed_timeout);
    node->get_parameter(prefix + "depth_failed_timeout", m_depth_failed_timeout);

    node->declare_parameter(prefix + "state_done", m_state_done);
    node->get_parameter(prefix + "state_done", m_state_done);

    node->declare_parameter(prefix + "state_fail", m_state_fail);
    node->get_parameter(prefix + "state_fail", m_state_fail);

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

    m_depth_tracking_sub = node->create_subscription<std_msgs::msg::Float32MultiArray>("~/"+ prefix + "update_depth_tracking_array", 10, 
        std::bind(&DepthTracking::m_depth_tracking_callback, 
        this, _1));
    
    m_depth_tracking_enabled = false;
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
        mvp_msgs::msg::ControlMode::DOF_Z,
        mvp_msgs::msg::ControlMode::DOF_PITCH,
    };
}


void DepthTracking::activated() 
{
    rclcpp::Clock clock(RCL_SYSTEM_TIME);

    bhv_timer =  clock.now();
    m_depth_index = 0;
    m_depth_tracking_enabled = true;
    m_depth_initial_timeout = false;
    m_depth_holding_flag = false;
    std::cout << "Depth tracking behavior is activated!" << std::endl;
    printf("Initial wait time is %lf\r\n", m_initial_wait_time);
}

void DepthTracking::disabled() 
{
    std::cout << "Depth tracking  behavior is disabled!" << std::endl;
}

void DepthTracking::m_depth_tracking_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    //checking the size
    int num_rows = msg->layout.dim[0].size;
    int num_cols = msg->layout.dim[1].size;
    // Expected total number of elements
    int expected_size = num_rows * num_cols;
    if(expected_size != msg->data.size() || num_rows >3)
    {
        printf("depth tracking multi array size incorrect \r\n");
        return;
    }
    std::vector<std::vector<double>> matrix(num_rows, std::vector<double>(num_cols));
    // Populate the matrix row-wise
    for (int i = 0; i < num_rows; ++i) {
        for (int j = 0; j < num_cols; ++j) {
            matrix[i][j] = msg->data[i * num_cols + j];  // Row-major indexing
        }
    }

    c_depth_list = matrix[0];
    c_pitch_list = matrix[1];
    c_depth_time_list = matrix[2];

    //start depth tracking right away
    m_depth_index = 0;
    m_depth_tracking_enabled = true;
    m_depth_initial_timeout = true;
    m_depth_holding_flag = false;

    printf("depth tracking param: %lf, %lf,%lf\r\n", 
        c_depth_list[m_depth_index], c_depth_time_list[m_depth_index], c_pitch_list[m_depth_index]);    
}


bool DepthTracking::request_set_point(
    mvp_msgs::msg::ControlProcess *set_point) 
{
    auto steady_clock = rclcpp::Clock();
    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    rclcpp::Time now = clock.now();

    if(m_activated == false)
    {
        return false;
    }

    if(m_depth_tracking_enabled)
    {
        if(now.seconds() - bhv_timer.seconds() > m_initial_wait_time && !m_depth_initial_timeout)
        {
            m_depth_index = 0;
            m_depth_initial_timeout = true;
            m_depth_holding_flag = false;
            printf("Initial depth tracking wait time reached \r\n");
            bhv_timer =  clock.now();
            printf("depth tracking param: %lf, %lf,%lf\r\n", 
                c_depth_list[m_depth_index], c_depth_time_list[m_depth_index], c_pitch_list[m_depth_index]);
            
        }

        if(m_depth_initial_timeout)
        {
            //get the set point
            m_bhv_setpoint.orientation.y = c_pitch_list[m_depth_index];
            m_bhv_setpoint.position.z = c_depth_list[m_depth_index];

            //transform into the helm frame
            auto temp_setpoint = std::make_shared<mvp_msgs::msg::ControlProcess>();
            transform_control_process_msg(m_bhv_setpoint, temp_setpoint, get_helm_world_link(), get_helm_child_link());

            *set_point = *temp_setpoint;
            
            //check if i have reached the depth to start the depth hold timer
            auto temp_pose = std::make_shared<mvp_msgs::msg::ControlProcess>();
            transform_control_process_msg(BehaviorBase::m_process_values, temp_pose, 
                                        m_bhv_setpoint.header.frame_id, m_bhv_setpoint.child_frame_id);

            double d_z = temp_pose->position.z - c_depth_list[m_depth_index];
            // printf("dz = %lf, dt=%lf\r\n", d_z, now.seconds() - bhv_timer.seconds());

            if( std::fabs(d_z)<m_depth_band && !m_depth_holding_flag)
            {
                m_depth_holding_flag = true;
                bhv_timer =  clock.now();
                printf("Depth has reaced, timer started \r\n");

            }

            //checking for timer only when the depth hold started
            if(now.seconds() - bhv_timer.seconds() >c_depth_time_list[m_depth_index] && m_depth_holding_flag)
            {
                m_depth_index ++;
                bhv_timer =  clock.now();
                m_depth_holding_flag = false; //reset the depth hold flag.
                printf("depth tracking param: %lf, %lf,%lf\r\n", 
                    c_depth_list[m_depth_index], c_depth_time_list[m_depth_index], c_pitch_list[m_depth_index]);
                //if i have reached the last one change state
                if(m_depth_index == c_depth_time_list.size())
                {
                    change_state(m_state_done);
                    m_depth_tracking_enabled = false;
                    m_depth_initial_timeout = false;
                }
            }
            
            //check if the depth failed because it cannot get close to the depth.
            if(now.seconds() - bhv_timer.seconds() > m_depth_failed_timeout && !m_depth_holding_flag)
            {
                change_state(m_state_fail); //change state
                m_depth_index = 0;
                m_depth_tracking_enabled = false;
                m_depth_initial_timeout = false;
                printf("Reaching depth failed\r\n");

            }
        }
    }

    
    return true;
}


/**
 * @brief Behavior must export the class to the Plugin library.
 */
 #include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(helm::DepthTracking, helm::BehaviorBase)