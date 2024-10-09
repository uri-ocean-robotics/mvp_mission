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


#include "bhv_teleoperation/bhv_teleoperation.h"

using namespace helm;

Teleoperation::Teleoperation() {
    m_use_joy = false;
    std::cout << "A message from the teleoperation" << std::endl;
}

Teleoperation::~Teleoperation() {}

void Teleoperation::initialize(const rclcpp::Node::WeakPtr &parent) {

    /*************************************************************************/
    /* get the node */

    m_node = parent;
    auto node = m_node.lock();
    m_logger = node->get_logger();

    /*************************************************************************/
    /* Load Parameters for ROS2 */

    // Load max parameters
    std::string prefix = get_name() + "/";
    std::string m_ns;
    m_ns = node->get_namespace();
    if (!m_ns.empty() && m_ns[0] == '/') {
        m_ns = m_ns.substr(1);
    }
    // Load increments in control
    node->declare_parameter(prefix + "tele_s_surge", 1.0);
    node->get_parameter(prefix + "tele_s_surge", m_tele_s_surge);

    node->declare_parameter(prefix + "tele_s_sway", 1.0);
    node->get_parameter(prefix + "tele_s_sway", m_tele_s_sway); 

    node->declare_parameter(prefix + "tele_d_yaw", 1.0);
    node->get_parameter(prefix + "tele_d_yaw", m_tele_d_yaw);

    node->declare_parameter(prefix + "tele_d_pitch", 1.0);
    node->get_parameter(prefix + "tele_d_pitch", m_tele_d_pitch);

    node->declare_parameter(prefix + "tele_d_depth", 1.0);
    node->get_parameter(prefix + "tele_d_depth", m_tele_d_depth);

    //load max values
    node->declare_parameter(prefix + "max_z", 100.0);
    node->get_parameter(prefix + "max_z", m_max_z);

    node->declare_parameter(prefix + "max_roll", M_PI_2);
    node->get_parameter(prefix + "max_roll", m_max_roll);

    node->declare_parameter(prefix + "max_pitch", M_PI_2);
    node->get_parameter(prefix + "max_pitch", m_max_pitch);

    node->declare_parameter(prefix + "max_surge", 1.0);
    node->get_parameter(prefix + "max_surge", m_max_surge);

    node->declare_parameter(prefix + "max_sway", 1.0);
    node->get_parameter(prefix + "max_sway", m_max_sway);

    // Load  enable/disable control
    node->declare_parameter(prefix + "ctrl_disable_srv", "controller/disable");
    node->get_parameter(prefix + "ctrl_disable_srv", m_ctrl_disable);
    m_ctrl_disable = "/" + m_ns + "/" + m_ctrl_disable;
    
    node->declare_parameter(prefix + "ctrl_enable_srv", "controller/enable");
    node->get_parameter(prefix + "ctrl_enable_srv", m_ctrl_enable);
    m_ctrl_enable = "/" + m_ns + "/" + m_ctrl_enable;

    node->declare_parameter(prefix + "no_joy_timeout", 3.0);
    node->get_parameter(prefix + "no_joy_timeout", m_no_joy_timeout);

    /*************************************************************************/
    /* Setup ROS2 sub/pub/srv/... */

    //! NOTE: the joy node should be launch on topside so that the namesapce will
    //        not the same on the vehilce, just the direct topic
    // joystick sub
    m_joy_sub = node->create_subscription<sensor_msgs::msg::Joy>(
        "~/"+ prefix + "joy", rclcpp::SystemDefaultsQoS(),
        std::bind(&Teleoperation::f_tele_op,
        this, std::placeholders::_1));

    // controller srv
    m_disable_ctrl_client = node->create_client<std_srvs::srv::Empty>(m_ctrl_disable);

    m_enable_ctrl_client = node->create_client<std_srvs::srv::Empty>(m_ctrl_enable);

    while (!m_disable_ctrl_client->wait_for_service(2s)) {
        RCLCPP_WARN(m_logger, 
            "service(%s) not available, waiting again...", m_ctrl_disable.c_str());
    }

    while (!m_enable_ctrl_client->wait_for_service(2s)) {
        RCLCPP_WARN(m_logger, 
            "service(%s) not available, waiting again...", m_ctrl_enable.c_str());
    }

    /*************************************************************************/
    /* Declare the degree of freedoms to be controlled by the behavior */

    BehaviorBase::m_dofs = decltype(m_dofs){
        // for poistion
        mvp_msgs::msg::ControlMode::DOF_Z,
        // for orientation 
        mvp_msgs::msg::ControlMode::DOF_ROLL,
        mvp_msgs::msg::ControlMode::DOF_PITCH,
        mvp_msgs::msg::ControlMode::DOF_YAW,
        // for velocity
        mvp_msgs::msg::ControlMode::DOF_U,
        mvp_msgs::msg::ControlMode::DOF_V,
    };


    /////initialize the desired pose first
    m_desired_roll = 0;
    m_desired_pitch = 0;
    m_desired_yaw = 0;
    m_desired_z = 0;
    m_desired_surge = 0;
    m_desired_sway = 0;
}

//tele op is good for control surge, pitch, depth and  heading
void Teleoperation::f_tele_op(const sensor_msgs::msg::Joy::SharedPtr msg) {

    //LB is the safety button and the joy is true
    if(msg->buttons[4]==1 && m_use_joy)
    {
        //left axis up and down
        m_desired_surge = msg->axes[1] * m_tele_s_surge;

        //left axis up and down 
        m_desired_sway = msg->axes[0] * m_tele_s_sway; 

        //X button decrease heading B button increase heading
        m_desired_yaw = 
            m_desired_yaw + m_tele_d_yaw/180*M_PI * 
            (-msg->buttons[0] + msg->buttons[2]); 
        
        //wrap yaw into -pi to pi.
        m_desired_yaw = 
            (fmod(m_desired_yaw + std::copysign(M_PI, m_desired_yaw), 2*M_PI) 
            - std::copysign(M_PI, m_desired_yaw));        

        //Y->decrease A->increase
        m_desired_pitch = 
            m_desired_pitch + m_tele_d_pitch/180*M_PI * 
            (-msg->buttons[3] + msg->buttons[1]); 

        //RB depth decrease, RT depth increase
        m_desired_z = 
            m_desired_z + m_tele_d_depth * (-msg->buttons[5] + msg->buttons[7]); 

        //saturation
        m_desired_roll = std::min(std::max(m_desired_roll, -m_max_roll), m_max_roll);
        m_desired_pitch = std::min(std::max(m_desired_pitch, -m_max_pitch), m_max_pitch);
        //no limit for yaw.
        m_desired_surge = std::min(std::max(m_desired_surge, -m_max_surge), m_max_surge);
        m_desired_sway = std::min(std::max(m_desired_sway, -m_max_sway), m_max_sway);
        m_desired_z = std::min(std::max(m_desired_z, -m_max_z), m_max_z);

    }

    //use back button to call disable controller service
    if(msg->buttons[8]==1)
    {
        //! TODO: change the state if failed

        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        
        auto resp = m_disable_ctrl_client->async_send_request(request);

        //! TODO: weakptr has no accesss to the node interface, check the result of srv 

        m_use_joy = false;
        RCLCPP_WARN(m_logger, "teleop disabled !");
    }

    //use start button to call enable controller service
    if(msg->buttons[9]==1)
    {
        //! TODO: change the state if failed

        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        
        auto resp = m_enable_ctrl_client->async_send_request(request);

    }

    // the following two button won't affect the controller.
    //set tele-op to false
    if(msg->buttons[6]==1)
    {
        // first time enable joystick and record vehicle pose
        m_desired_pitch = 0;
        m_desired_roll = 0;
        m_desired_yaw = BehaviorBase::m_process_values.orientation.z;
        m_desired_z = BehaviorBase::m_process_values.position.z;
        m_desired_surge = 0;
        m_desired_sway = 0;
        m_use_joy = true;
        RCLCPP_WARN(m_logger, "teleop enabled !");
    }
    
    //record the timeout
    if(m_use_joy)
    {
        m_last_joy_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
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
    mvp_msgs::msg::ControlProcess *set_point) {

    if( !m_use_joy ) {
        return false;
    }

    //timeout
    if(rclcpp::Clock(RCL_ROS_TIME).now().seconds()-m_last_joy_time > m_no_joy_timeout)
    {
        RCLCPP_WARN(m_logger, "No joy command for %.1lf second.", m_no_joy_timeout);
        RCLCPP_WARN(m_logger, "Teleop is disabled, please enable it again using the joystick");
        m_use_joy = false;
        return false;
    }

    //set point /heder/frame_id and child frame id will be the same as the helm setting (not additional setting here).
    // Set Position
    set_point->position.z = m_desired_z;

    // Set orientation
    set_point->orientation.x = m_desired_roll;
    set_point->orientation.y = m_desired_pitch;
    set_point->orientation.z = m_desired_yaw;

    // Set velocity
    set_point->velocity.x = m_desired_surge;
    set_point->velocity.y = m_desired_sway;
   

    return true;
}

/**
 * @brief Behavior must export the class to the Plugin library.
 */
 #include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(helm::Teleoperation, helm::BehaviorBase)