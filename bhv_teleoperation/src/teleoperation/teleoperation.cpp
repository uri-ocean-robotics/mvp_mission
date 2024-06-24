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

Teleoperation::Teleoperation(){
//   : m_use_joy(false), m_last_yaw (false), m_last_pitch(false) {
    m_use_joy = false;
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
    m_transform_listener.reset(new
        tf2_ros::TransformListener(m_transform_buffer)
    );

    // ROS related: load parameters, setup sub/pub

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

    m_pnh->param<double>("tele_d_yaw", m_tele_d_yaw, 1.0);
    m_pnh->param<double>("tele_s_surge", m_tele_s_surge, 1.0);
    m_pnh->param<double>("tele_s_sway", m_tele_s_sway, 1.0);
    m_pnh->param<double>("tele_d_pitch", m_tele_d_pitch, 1.0);
    m_pnh->param<double>("tele_d_depth", m_tele_d_depth, 1.0);


    //robot mvp_controller service
    m_pnh->param<std::string>("ctrl_disable_srv", m_ctrl_disable, "controller/disable");
    //
    m_pnh->param<std::string>("ctrl_enable_srv", m_ctrl_enable, "controller/enable");


    m_joy_sub = m_pnh->subscribe("joy", 100, &Teleoperation::f_tele_op, this);

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

//tele op is good for control surge, pitch, depth and  heading
void Teleoperation::f_tele_op(const sensor_msgs::Joy::ConstPtr& msg) {
    // printf("disbale: %d\n", msg->buttons[8]);
    
    //LB is the safety button and the joy is true
    if(msg->buttons[4]==1 & m_use_joy){
        
        m_desired_surge = msg->axes[1] * m_tele_s_surge; //left axis up and down
        m_desired_sway = msg->axes[0] * m_tele_s_sway; //left axis up and down


        m_desired_yaw = m_desired_yaw + m_tele_d_yaw/180*M_PI * (-msg->buttons[0] + msg->buttons[2]); //X button decrease heading B button increase heading
        
        //wrap yaw into -pi to pi.
        m_desired_yaw = (fmod(m_desired_yaw + std::copysign(M_PI, m_desired_yaw), 2*M_PI) 
                - std::copysign(M_PI, m_desired_yaw));        


        m_desired_pitch = m_desired_pitch + m_tele_d_pitch/180*M_PI *(-msg->buttons[3] + msg->buttons[1]); //Y->decrease A->increase

        m_desired_z = m_desired_z + m_tele_d_depth * (-msg->buttons[5] + msg->buttons[7]); //RB depth decrease, RT depth increase

        //saturation
        m_desired_pitch = std::min(std::max(m_desired_pitch, -m_max_pitch), m_max_pitch);
        m_desired_yaw = std::min(std::max(m_desired_yaw, -m_max_yaw), m_max_yaw);
        m_desired_surge = std::min(std::max(m_desired_surge, -m_max_surge), m_max_surge);
        m_desired_z = std::min(std::max(m_desired_z, -m_max_z), m_max_z);
    }

    //use back button to call disable controller service
    if(msg->buttons[8]==1)
    {
        //check disable controller service
        if(!ros::service::exists(m_ctrl_disable, false)) {
            std::cout << "The service " << m_ctrl_disable << " is not available"<<std::endl;
        }
        std_srvs::Empty srv;
        if(!ros::service::call(m_ctrl_disable, srv)) {
            std::cout << "Failed to disable the controller" << std::endl;
            // change the state if failed
        }
        m_use_joy = false;
    }

    //use start button to call enable controller service
    if(msg->buttons[9]==1)
    {
        //check disable controller service
        if(!ros::service::exists(m_ctrl_enable, false)) {
            std::cout << "The service " << m_ctrl_enable << " is not available"<<std::endl;
        }
        std_srvs::Empty srv;
        if(!ros::service::call(m_ctrl_enable, srv)) {
            std::cout << "Failed to disable the controller" << std::endl;
            // change the state if failed
        }

    }

    // the following two button won't affect the controller.
    //set tele-op to false
    if(msg->buttons[6]==1)
    {
        // first time enable joystick and record vehicle pose
        // if(!m_use_joy) {
            // record global information
        m_desired_pitch = BehaviorBase::m_process_values.orientation.y;
        m_desired_yaw = BehaviorBase::m_process_values.orientation.z;
        m_desired_z = BehaviorBase::m_process_values.position.z;
        m_desired_surge = 0;
        // }

        m_use_joy = true; //!m_use_joy;
        // printf("m_use_joy = %s\r\n", m_use_joy ? "yes":"no");
        // printf("depth =%lf || %lf\r\n", m_desired_z, BehaviorBase::m_process_values.position.z);

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

    // printf("set point will be set \r\n");
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

    // printf("set point depth =%lf\r\n", set_point->position.z);

    return true;
}

/**
 * @brief Behavior must export the class to the Plugin library.
 */
PLUGINLIB_EXPORT_CLASS(helm::Teleoperation, helm::BehaviorBase)