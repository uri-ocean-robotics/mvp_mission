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
    Year: 2024

    Copyright (C) 2024 Smart Ocean Systems Laboratory
*/


#include "bhv_surfacing/bhv_surfacing.h"
#include "tf2/time.h"
#include <cmath>

using namespace helm;

Surfacing::Surfacing() {
    std::cout << "A message from the surfacing" << std::endl;
}

Surfacing::~Surfacing() {}

void Surfacing::initialize(const rclcpp::Node::WeakPtr &parent) {

    /*************************************************************************/
    /* get the node */

    m_node = parent;
    auto node = m_node.lock();
    m_logger = node->get_logger();

     //setup tf buffer
    m_transform_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    m_transform_listener = std::make_unique<tf2_ros::TransformListener>(*m_transform_buffer);

    /*************************************************************************/
    /* Load Parameters for ROS2 */

    // Load max parameters
    std::string prefix = get_name() + "/";
    std::string m_ns;
    m_ns = node->get_namespace();
    if (!m_ns.empty() && m_ns[0] == '/') {
        m_ns = m_ns.substr(1);
    }

    std::string global_link, child_link;
    node->declare_parameter(prefix + "default_bhv_world_link", "world_ned");
    node->get_parameter(prefix + "default_bhv_world_link", global_link);

    node->declare_parameter(prefix + "default_bhv_child_link", "cg_link");
    node->get_parameter(prefix + "default_bhv_child_link", child_link);
    
    //always the same as the helm
    m_bhv_setpoint.header.frame_id = m_ns + "/" + global_link;
    m_bhv_setpoint.child_frame_id = m_ns + "/" + child_link;

    // Load increments in control

    node->declare_parameter(prefix + "no_gps_timeout", 3600.0);
    node->get_parameter(prefix + "no_gps_timeout", u_submerged_period_with_no_gps);

    node->declare_parameter(prefix + "min_gps_count_at_surface", 20);
    node->get_parameter(prefix + "min_gps_count_at_surface", u_min_surface_gps_count);

    node->declare_parameter(prefix + "surfacing_duration", 3600.0);
    node->get_parameter(prefix + "surfacing_duration", u_surfacing_duration);

    node->declare_parameter(prefix + "surfacing_depth", 0.0);
    node->get_parameter(prefix + "surfacing_depth", c_surfacing_depth);

    node->declare_parameter(prefix + "surfacing_at_start", false);
    node->get_parameter(prefix + "surfacing_at_start", m_set_point_pub);

    node->declare_parameter(prefix + "no_imu_timeout", 3600.0);
    node->get_parameter(prefix + "no_imu_timeout", u_no_imu_timeout);

    node->declare_parameter(prefix + "no_dvl_timeout", 3600.0);
    node->get_parameter(prefix + "no_dvl_timeout", u_no_dvl_timeout);

    node->declare_parameter(prefix + "dvl_acceptable_var", 10.0);
    node->get_parameter(prefix + "dvl_acceptable_var", u_dvl_acceptable_var);

    node->declare_parameter(prefix + "navigation_fail_state", "");
    node->get_parameter(prefix + "navigation_fail_state", u_navigation_fail_state);

    //reserved params
    node->declare_parameter(prefix + "no_comm_timeout", INFINITY);
    node->get_parameter(prefix + "no_comm_timeout", u_submerged_period_with_no_comm);
    //reserved
    node->declare_parameter(prefix + "float_to_surface", false);
    node->get_parameter(prefix + "float_to_surface", u_floating_to_surface_flag);

    node->declare_parameter(prefix + "ctrl_set_srv", "controller/set");

    node->get_parameter(prefix + "ctrl_set_srv", m_ctrl_set_srv);
    
    m_ctrl_set_srv = "/" + m_ns + "/" + m_ctrl_set_srv;

    std::string m_gps_fix_topic;
    node->declare_parameter(prefix + "gps_fix_topic", "gps/fix");
    node->get_parameter(prefix + "gps_fix_topic", m_gps_fix_topic);

    std::string m_dvl_topic;
    node->declare_parameter(prefix + "dvl_topic", "dvl/data");
    node->get_parameter(prefix + "dvl_topic", m_dvl_topic);

    std::string m_imu_topic;
    node->declare_parameter(prefix + "imu_topic", "imu/data");
    node->get_parameter(prefix + "imu_topic", m_imu_topic);

    m_gps_fix_subscriber = node->create_subscription<sensor_msgs::msg::NavSatFix>(m_gps_fix_topic, 10, 
                                                            std::bind(&Surfacing::f_cb_gps_fix, 
                                                            this, std::placeholders::_1));


    m_imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(m_imu_topic,10,
                                                            std::bind(&Surfacing::f_cb_imu, 
                                                            this, std::placeholders::_1));

    m_dvl_sub = node->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(m_dvl_topic, 10,
                                                            std::bind(&Surfacing::f_cb_dvl, 
                                                            this, std::placeholders::_1));

    m_surfacing_flag_pub = node->create_publisher<std_msgs::msg::Int8MultiArray>(prefix + "surfacing_flags", 0);

    /*************************************************************************/
    /* Setup ROS2 sub/pub/srv/... */
    m_dive_trigger_srv = node->create_service<std_srvs::srv::Trigger>(
            "~/" + prefix + "stop_surfacing",
            std::bind(&Surfacing::f_dive_trigger, this, std::placeholders::_1, std::placeholders::_2));
    // controller srv
    m_ctrl_set_client = node->create_client<std_srvs::srv::SetBool>(m_ctrl_set_srv);

    while (!m_ctrl_set_client->wait_for_service(2s)) {
        RCLCPP_WARN(m_logger, 
            "service(%s) not available, waiting again...", m_ctrl_set_srv.c_str());
    }


    /*************************************************************************/
    /* Declare the degree of freedoms to be controlled by the behavior */

    BehaviorBase::m_dofs = decltype(m_dofs){
        // for poistion
        mvp_msgs::msg::ControlMode::DOF_Z,
    };


    /////initialize the desired pose first
    m_bhv_setpoint.position.z = c_surfacing_depth;

    //no_gps_timeout_flag, no_dvl_timeout, no_imu_timeout, no_coomm_timeout
    m_surfacing_flag.data = {0, 0, 0, 0};  // Fill with your int8_t values

}

void Surfacing::activated() {
    /**
     * @brief This function is called when the behavior gets activated.
     *
     * @note This function is an inherited virtual function. It may left
     * unimplemented.
     *
     * @details This function is called when the behavior internal state
     * defined by #BehaviorBase::m_actived changes to true.
     */
    std::cout << "surfacing behavior: is activated!" << std::endl;
    std::cout << "No GPS surfacing condition:"<< u_submerged_period_with_no_gps << std::endl;
    std::cout << "No Comm surfacing condition:"<< u_submerged_period_with_no_comm << std::endl;
    std::cout << "No IMU time condition:"<< u_no_imu_timeout << std::endl;
    std::cout << "No DVL time condition:"<< u_no_dvl_timeout << std::endl;
    std::cout << "Surfacing condition: "<< u_surfacing_duration << std::endl;


    m_last_dvl_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();

    m_last_imu_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();

}

void Surfacing::disabled() {
    /**
     * @brief This function is called when the behavior gets disabled.
     *
     * @note This function is an inherited virtual function. It may left
     * unimplemented.
     *
     * @details This function is called when the behavior internal state
     * defined by #BehaviorBase::m_actived changes to false.
     */
    std::cout << "surfacing behavior is disabled!" << std::endl;
    m_set_point_pub = false;
}

void Surfacing::f_dive_trigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    m_gps_flag = true;
    m_set_point_pub = false;
    m_last_gps_time =  rclcpp::Clock(RCL_ROS_TIME).now().seconds(); //use current time as the last gps time for timing the surfacing
    response->success = true;
    response->message = "dive triggered";
    RCLCPP_WARN(m_logger, "Surfacing_bhv: Stop surfacing service called");

}

void Surfacing::f_cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    m_last_imu_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    m_surfacing_flag.data[2] = 0;  
}

void Surfacing::f_cb_dvl(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
    if(msg->twist.covariance[0]<u_dvl_acceptable_var && 
      msg->twist.covariance[7]<u_dvl_acceptable_var && 
      msg->twist.covariance[14]<u_dvl_acceptable_var)
    {
        m_last_dvl_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
        m_surfacing_flag.data[1] = 0;  
    }
}

void Surfacing::f_cb_gps_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{

    //if GPS has fix
    if (msg->status.status != sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
    {
        if (m_gps_flag == false)
        {
            m_first_gps_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
            m_bhv_setpoint.position.z = c_surfacing_depth;  //record the current depth for the next time
            
            // printf("initial GPS obtained \r\n");
            RCLCPP_INFO(m_logger, "Surfacing_bhv: initial GPS obtained");

            m_surface_gps_count = 0; 
        }

        m_gps_flag = true;     
        // c_surfacing_depth = BehaviorBase::m_process_values.position.z;
        m_last_gps_time =  rclcpp::Clock(RCL_ROS_TIME).now().seconds();
        m_surface_gps_count ++;
        // printf("total valid gps fix count = %d\r\n", m_surface_gps_count);
    }
}


bool Surfacing::request_set_point(
    mvp_msgs::msg::ControlProcess *set_point) {
    auto steady_clock = rclcpp::Clock();
    
    if(m_activated == false)
    {
        return false;
    }

    m_surfacing_flag_pub->publish(m_surfacing_flag);

    if( (m_last_gps_time - m_first_gps_time > u_surfacing_duration)  && m_gps_flag && m_set_point_pub)
    {
        if(m_surface_gps_count>u_min_surface_gps_count)
        {
        m_set_point_pub = false;  //duration has exceeded and i will not set depth
        m_surfacing_flag.data[0] = 0;  
        RCLCPP_WARN(m_logger, "Surfacing_bhv: valid gps fix count [%d] has reached", u_min_surface_gps_count);
        }
        else{
        RCLCPP_WARN_STREAM_THROTTLE(m_logger, steady_clock, 1000, std::string("Surfacing_bhv: valid gps fix count [" + std::to_string(u_min_surface_gps_count) + "] has not reached"));

        }
        // RCLCPP_WARN(m_logger, "Surfacing_bhv: surfacing duration has exceeded");
        RCLCPP_WARN_STREAM_THROTTLE(m_logger, steady_clock, 1000, std::string("Surfacing_bhv: surfacing duration [" + std::to_string(u_surfacing_duration) + "] has exceeded"));

    }

    double m_current_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();

    if(m_current_time - m_last_gps_time > u_submerged_period_with_no_gps && !m_set_point_pub)
    {
        m_surfacing_flag.data[0] = 1;  
        m_set_point_pub = true;
        m_gps_flag = false; //set to false so we can get the first gps time.
        m_surface_gps_count = 0; 
        RCLCPP_WARN(m_logger, "Surfacing_bhv: surfacing request triggered");


    }

    if(m_activated)
    {
        if(m_current_time - m_last_imu_time > u_no_imu_timeout)
        {
            m_surfacing_flag.data[2] = 1;  
            // printf("bhv_surfacing: No IMU triggered state change \r\n");
            // RCLCPP_WARN(m_logger, "No IMU triggered state change");
            RCLCPP_WARN_STREAM_THROTTLE(m_logger, steady_clock, 1000, std::string("Surfacing_bhv: No IMU timeout triggered state change"));


            change_state(u_navigation_fail_state);
            return false;

        }

        if(m_current_time - m_last_dvl_time > u_no_dvl_timeout)
        {
            m_surfacing_flag.data[1] = 1;  
            // printf("bhv_surfacing: DVL triggered state change \r\n");
            // RCLCPP_WARN(m_logger, "No DVL triggered state change");
            RCLCPP_WARN_STREAM_THROTTLE(m_logger, steady_clock, 1000, std::string("Surfacing_bhv: No DVL timeout triggered state change"));


            change_state(u_navigation_fail_state);
            return false;

        }
    }


    if(!m_set_point_pub)
    {
        return false;
    }

    auto temp_setpoint = std::make_shared<mvp_msgs::msg::ControlProcess>();

    // transform_control_process_msg(m_bhv_setpoint, temp_setpoint, get_helm_world_link(), get_helm_child_link());

    *set_point = m_bhv_setpoint;
   
    return true;
}

/**
 * @brief Behavior must export the class to the Plugin library.
 */
 #include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(helm::Surfacing, helm::BehaviorBase)