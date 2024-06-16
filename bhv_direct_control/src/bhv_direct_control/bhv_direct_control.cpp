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
    m_max = Eigen::VectorXd::Zero(SETPOINT_DOF_LENGTH); //12 DOF
    //use array instead of 12 variables
    m_desired_value = Eigen::VectorXd::Zero(SETPOINT_DOF_LENGTH);  //12 DOF

    //load max
    node->declare_parameter("max_x", 5.0);
    node->get_parameter("max_x", m_max(DOF::X));
    node->declare_parameter("max_y", 5.0);
    node->get_parameter("max_y", m_max(DOF::Y));
    node->declare_parameter("max_z", 5.0);
    node->get_parameter("max_z", m_max(DOF::Z));

    node->declare_parameter("max_roll", M_PI_2);
    node->get_parameter("max_roll", m_max(DOF::ROLL));
    node->declare_parameter("max_pitch", M_PI_2);
    node->get_parameter("max_pitch", m_max(DOF::PITCH));
    node->declare_parameter("max_yaw", M_PI);
    node->get_parameter("max_yaw", m_max(DOF::YAW));
    
    node->declare_parameter("max_u", 1.0);
    node->get_parameter("max_u", m_max(DOF::U));
    node->declare_parameter("max_v", 1.0);
    node->get_parameter("max_v", m_max(DOF::V));
    node->declare_parameter("max_w", 1.0);
    node->get_parameter("max_w", m_max(DOF::W));

    node->declare_parameter("max_p", 1.0);
    node->get_parameter("max_p", m_max(DOF::P));
    node->declare_parameter("max_q", 1.0);
    node->get_parameter("max_q", m_max(DOF::Q));
    node->declare_parameter("max_r", 1.0);
    node->get_parameter("max_r", m_max(DOF::R));

    //load desired
    node->declare_parameter("desired_x", 0.0);
    node->get_parameter("desired_x", m_desired_value(DOF::X));
    node->declare_parameter("desired_y", 0.0);
    node->get_parameter("desired_y", m_desired_value(DOF::Y));
    node->declare_parameter("desired_z", 0.0);
    node->get_parameter("desired_z", m_desired_value(DOF::Z));
    
    node->declare_parameter("desired_roll", 0.0);
    node->get_parameter("desired_roll", m_desired_value(DOF::ROLL));
    node->declare_parameter("desired_pitch", 0.0);
    node->get_parameter("desired_pitch", m_desired_value(DOF::PITCH));
    node->declare_parameter("desired_yaw", 0.0);
    node->get_parameter("desired_yaw", m_desired_value(DOF::YAW));
    
    node->declare_parameter("desired_u", 0.0);
    node->get_parameter("desired_u", m_desired_value(DOF::U));
    node->declare_parameter("desired_v", 0.0);
    node->get_parameter("desired_v", m_desired_value(DOF::V));
    node->declare_parameter("desired_w", 0.0);
    node->get_parameter("desired_w", m_desired_value(DOF::W));

    node->declare_parameter("desired_p", 0.0);
    node->get_parameter("desired_p", m_desired_value(DOF::P));
    node->declare_parameter("desired_q", 0.0);
    node->get_parameter("desired_q", m_desired_value(DOF::Q));
    node->declare_parameter("desired_r", 0.0);
    node->get_parameter("desired_r", m_desired_value(DOF::R));

    ///topics
    m_setpoint_sub = node->create_subscription<mvp_msgs::msg::ControlProcess>("desired_setpoints", 100, 
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
        // for angular velocity
        mvp_msgs::msg::ControlMode::DOF_P,
        mvp_msgs::msg::ControlMode::DOF_Q,
        mvp_msgs::msg::ControlMode::DOF_R,
    };

}


void DirectControl::m_setpoint_callback(const mvp_msgs::msg::ControlProcess::SharedPtr msg)
{
    //saturation
    bhv_global_link = msg->header.frame_id;
    bhv_child_link = msg->child_frame_id;
    ///linear velocity
    m_desired_value(DOF::U) = std::min( std::max(msg->velocity.x, -m_max(DOF::U)), m_max(DOF::U) );
    m_desired_value(DOF::V) = std::min( std::max(msg->velocity.x, -m_max(DOF::V)), m_max(DOF::V) );
    m_desired_value(DOF::W) = std::min( std::max(msg->velocity.x, -m_max(DOF::W)), m_max(DOF::W) );


    ///angular velocity
    m_desired_value(DOF::P) = std::min( std::max(msg->velocity.x, -m_max(DOF::P)), m_max(DOF::P) );
    m_desired_value(DOF::Q) = std::min( std::max(msg->velocity.x, -m_max(DOF::Q)), m_max(DOF::Q) );
    m_desired_value(DOF::R) = std::min( std::max(msg->velocity.x, -m_max(DOF::R)), m_max(DOF::R) );

    //Position
    m_desired_value(DOF::X) = std::min( std::max(msg->velocity.x, -m_max(DOF::X)), m_max(DOF::X) );
    m_desired_value(DOF::Y) = std::min( std::max(msg->velocity.x, -m_max(DOF::Y)), m_max(DOF::Y) );
    m_desired_value(DOF::Z) = std::min( std::max(msg->velocity.x, -m_max(DOF::Z)), m_max(DOF::Z) );

    //euler angle
    m_desired_value(DOF::ROLL) = std::min( std::max(msg->velocity.x, -m_max(DOF::ROLL)), m_max(DOF::ROLL) );
    m_desired_value(DOF::PITCH) = std::min( std::max(msg->velocity.x, -m_max(DOF::PITCH)), m_max(DOF::PITCH) );
    m_desired_value(DOF::YAW) = std::min( std::max(msg->velocity.x, -m_max(DOF::YAW)), m_max(DOF::YAW) );

    transform_setpoint();
}


void DirectControl::transform_setpoint()
{
    auto steady_clock = rclcpp::Clock();
    //convert m_desired_value from bhv frames into helm frames
    Eigen::Vector3d xyz_helm, ypr_helm, uvw_helm, pqr_helm;
    //this portion will be later moved into bevhavior_base.hpp so all behvaior can use the same function to do transformation.
    try{
        //get tf from bhv world to helm world
        geometry_msgs::msg::TransformStamped tf_bw_hw = m_transform_buffer->lookupTransform(
            get_helm_global_link(),
            bhv_global_link,
            tf2::TimePointZero,
            10ms
        );
        //transform the xyz set point
        //P_h = R_bw^hw * P_bhv + T_bw^hw
        auto tf_eigen = tf2::transformToEigen(tf_bw_hw);
        xyz_helm = tf_eigen.rotation() * 
                        Eigen::Vector3d(m_desired_value(DOF::X),
                                        m_desired_value(DOF::Y), 
                                        m_desired_value(DOF::Z))
                        + tf_eigen.translation();

        //transform the euler angle
        //step 1 get helm_global to bhv_global
         geometry_msgs::msg::TransformStamped tf_hg_bg = m_transform_buffer->lookupTransform(
            bhv_global_link,
            get_helm_global_link(),
            tf2::TimePointZero,
            10ms
        );
        auto tf_hgbg_eigen = tf2::transformToEigen(tf_hg_bg);
        //step 2 compute the bhv_global to bhv_global set point pose.
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(m_desired_value(DOF::YAW), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(m_desired_value(DOF::PITCH), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(m_desired_value(DOF::ROLL), Eigen::Vector3d::UnitX());

        // rotation matrix from the helm global to bhv_setpoint pose 
        Eigen::Matrix3d R_helm_global =  tf_hgbg_eigen.rotation() *R;
        ypr_helm = R_helm_global.eulerAngles(2, 1, 0);
        
        // ypr_helm.y() = asin(-R_helm_global(2, 0));

        // // Calculate yaw (rotation about Z-axis)
        // ypr_helm.z() = atan2(R_helm_global(1, 0), R_helm_global(0, 0));

        // // Calculate roll (rotation about X-axis)
        // ypr_helm.x() = atan2(R_helm_global(2, 1), R_helm_global(2, 2));

        //computet he bhv_local to helm local
        geometry_msgs::msg::TransformStamped tf_bl_hl = m_transform_buffer->lookupTransform(
            get_helm_local_link(),
            bhv_child_link,
            tf2::TimePointZero,
            10ms
        );
        auto tf_blhl_eigen = tf2::transformToEigen(tf_bl_hl);
        
        ///velocity
        uvw_helm = tf_blhl_eigen.rotation() *
                    Eigen::Vector3d(m_desired_value(DOF::U),
                                    m_desired_value(DOF::V), 
                                    m_desired_value(DOF::W));

        tf2::Quaternion quat;
        quat.setW(tf_bl_hl.transform.rotation.w);
        quat.setX(tf_bl_hl.transform.rotation.x);
        quat.setY(tf_bl_hl.transform.rotation.y);
        quat.setZ(tf_bl_hl.transform.rotation.z);

        Eigen::VectorXd process_values = Eigen::VectorXd::Zero(SETPOINT_DOF_LENGTH);
        tf2::Matrix3x3(quat).getRPY(
            process_values(DOF::ROLL),
            process_values(DOF::PITCH),
            process_values(DOF::YAW)
        );
        Eigen::Matrix3d ang_vel_tranform = Eigen::Matrix3d::Identity();
        ang_vel_tranform = f_angular_velocity_transform(process_values);

        pqr_helm = ang_vel_tranform *
                    Eigen::Vector3d(m_desired_value(DOF::P),
                                    m_desired_value(DOF::Q), 
                                    m_desired_value(DOF::R));


    } catch (const tf2::TransformException & e) {
            RCLCPP_WARN_STREAM_THROTTLE(m_logger, steady_clock, 10, std::string("Can't compute thruster tf between ") + e.what());
            RCLCPP_INFO( m_logger, "Could not transform %s to %s: %s",
                         get_helm_global_link().c_str(), bhv_global_link.c_str(), e.what() ); 
          return;

    }

    m_desired_value(DOF::X) = xyz_helm.x();
    m_desired_value(DOF::Y) = xyz_helm.y();
    m_desired_value(DOF::Z) = xyz_helm.z();
    m_desired_value(DOF::U) = uvw_helm.x();
    m_desired_value(DOF::V) = uvw_helm.y();
    m_desired_value(DOF::W) = uvw_helm.z();
    m_desired_value(DOF::ROLL) = ypr_helm(2);
    m_desired_value(DOF::PITCH) = ypr_helm(1);
    m_desired_value(DOF::YAW) = ypr_helm(0);
    m_desired_value(DOF::P) = pqr_helm.x();
    m_desired_value(DOF::Q) = pqr_helm.y();
    m_desired_value(DOF::R) = pqr_helm.z();
}


Eigen::MatrixXd DirectControl::f_angular_velocity_transform(const Eigen::VectorXd& orientation) {
    Eigen::Matrix3d transform = Eigen::Matrix3d::Zero();

    // 85 < pitch < 95, -95 < pitch < -85 
    if( (orientation(DOF::PITCH) >  1.483529839 && orientation(DOF::PITCH) <  1.658062761) ||
        (orientation(DOF::PITCH) > -1.658062761 && orientation(DOF::PITCH) < -1.483529839) ) {
        transform(0,0) = 1.0;
        transform(0,1) = 0.0;
        transform(0,2) = 0.0;
        transform(1,0) = 0.0;
        transform(1,1) = cos(orientation(DOF::ROLL));
        transform(1,2) = -sin(orientation(DOF::ROLL));
        transform(2,0) = 0.0;
        transform(2,1) = 0.0;
        transform(2,2) = 0.0;
    }
    else {
        transform(0,0) = 1.0;
        transform(0,1) = sin(orientation(DOF::ROLL)) * tan(orientation(DOF::PITCH));
        transform(0,2) = cos(orientation(DOF::ROLL)) * tan(orientation(DOF::PITCH));
        transform(1,0) = 0.0;
        transform(1,1) = cos(orientation(DOF::ROLL));
        transform(1,2) = -sin(orientation(DOF::ROLL));
        transform(2,0) = 0.0;
        transform(2,1) = sin(orientation(DOF::ROLL)) / cos(orientation(DOF::PITCH));
        transform(2,2) = cos(orientation(DOF::ROLL)) / cos(orientation(DOF::PITCH));
        RCLCPP_WARN(m_logger, "the angular transform matrix is approaching the singularity point");
    }    

    return transform;
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
}


/**
 * @brief Behavior must export the class to the Plugin library.
 */
 #include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(helm::DirectControl, helm::BehaviorBase)