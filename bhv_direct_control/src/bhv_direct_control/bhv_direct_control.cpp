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

    std::string prefix = get_name() + "/";

    //load initial desired values
    node->declare_parameter(prefix + "desired_x", 0.0);
    node->get_parameter(prefix + "desired_x", m_desired_value(DOF::X));
    node->declare_parameter(prefix + "desired_y", 0.0);
    node->get_parameter(prefix + "desired_y", m_desired_value(DOF::Y));
    node->declare_parameter(prefix + "desired_z", 0.0);
    node->get_parameter(prefix + "desired_z", m_desired_value(DOF::Z));
    
    node->declare_parameter(prefix + "desired_roll", 0.0);
    node->get_parameter(prefix + "desired_roll", m_desired_value(DOF::ROLL));
    node->declare_parameter(prefix + "desired_pitch", 0.0);
    node->get_parameter(prefix + "desired_pitch", m_desired_value(DOF::PITCH));
    node->declare_parameter(prefix + "desired_yaw", 0.0);
    node->get_parameter(prefix + "desired_yaw", m_desired_value(DOF::YAW));
    
    node->declare_parameter(prefix + "desired_u", 0.0);
    node->get_parameter(prefix + "desired_u", m_desired_value(DOF::U));
    node->declare_parameter(prefix + "desired_v", 0.0);
    node->get_parameter(prefix + "desired_v", m_desired_value(DOF::V));
    node->declare_parameter(prefix + "desired_w", 0.0);
    node->get_parameter(prefix + "desired_w", m_desired_value(DOF::W));

    std::string node_name = node->get_name();
    std::string ns = node->get_namespace();
    if (!ns.empty() && ns[0] == '/') {
        ns = ns.substr(1);
    }

    std::string global_link, child_link;
    node->declare_parameter(prefix + "default_bhv_world_link", "world_ned");
    node->get_parameter(prefix + "default_bhv_world_link", global_link);

    node->declare_parameter(prefix + "default_bhv_child_link", "cg_link");
    node->get_parameter(prefix + "default_bhv_child_link", child_link);
    
    bhv_global_link = ns + "/" + global_link;
    bhv_child_link = ns + "/" + child_link;

    ///topics
    m_setpoint_sub = node->create_subscription<mvp_msgs::msg::ControlProcess>("~/"+ prefix + "desired_setpoints", 100, 
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
    //saturation
    bhv_global_link = msg->header.frame_id;
    bhv_child_link = msg->child_frame_id;
    ///linear velocity
    m_desired_value(DOF::U) = msg->velocity.x;
    m_desired_value(DOF::V) = msg->velocity.y;
    m_desired_value(DOF::W) = msg->velocity.z;

    //Position
    m_desired_value(DOF::X) = msg->position.x;
    m_desired_value(DOF::Y) = msg->position.y;
    m_desired_value(DOF::Z) = msg->position.z;

    //euler angle
    m_desired_value(DOF::ROLL) = msg->orientation.x;
    m_desired_value(DOF::PITCH) = msg->orientation.y;
    m_desired_value(DOF::YAW) = msg->orientation.z;
    
    transform_setpoint();

}


void DirectControl::transform_setpoint()
{
    auto steady_clock = rclcpp::Clock();
    //convert m_desired_value from bhv frames into helm frames
    
    //this portion will be later moved into bevhavior_base.hpp so all behvaior can use the same function to do transformation.
    try{
        //get tf from bhv world to helm world
        geometry_msgs::msg::TransformStamped tf_bw_hw = m_transform_buffer->lookupTransform(
            get_helm_world_link(),
            bhv_global_link,
            tf2::TimePointZero,
            10ms
        );
        //transform the xyz set point
        geometry_msgs::msg::PoseStamped setpoint_pose_bhv, setpoint_pose_helm;

        setpoint_pose_bhv.header.frame_id = bhv_global_link;
        setpoint_pose_bhv.pose.position.x = m_desired_value(DOF::X);
        setpoint_pose_bhv.pose.position.y = m_desired_value(DOF::Y);
        setpoint_pose_bhv.pose.position.z = m_desired_value(DOF::Z);
        
        tf2::Quaternion q;
        q.setRPY(m_desired_value(DOF::ROLL), m_desired_value(DOF::PITCH), m_desired_value(DOF::YAW));
        setpoint_pose_bhv.pose.orientation.x = q.x();
        setpoint_pose_bhv.pose.orientation.y = q.y();
        setpoint_pose_bhv.pose.orientation.z = q.z();
        setpoint_pose_bhv.pose.orientation.w = q.w();

        //convert setpoint pose
        setpoint_pose_helm.header.frame_id = get_helm_world_link();

        tf2::doTransform(setpoint_pose_bhv, setpoint_pose_helm, tf_bw_hw);
        tf2::Quaternion quat;
        quat.setW(setpoint_pose_helm.pose.orientation.w);
        quat.setX(setpoint_pose_helm.pose.orientation.x);
        quat.setY(setpoint_pose_helm.pose.orientation.y);
        quat.setZ(setpoint_pose_helm.pose.orientation.z);

        m_desired_value(DOF::X) = setpoint_pose_helm.pose.position.x;
        m_desired_value(DOF::Y) = setpoint_pose_helm.pose.position.y;
        m_desired_value(DOF::Z) = setpoint_pose_helm.pose.position.z;

        tf2::Matrix3x3(quat).getRPY(
            m_desired_value(DOF::ROLL),
            m_desired_value(DOF::PITCH),
            m_desired_value(DOF::YAW)
        );
        
        //computet he bhv_local to helm local
        geometry_msgs::msg::TransformStamped tf_bl_hl = m_transform_buffer->lookupTransform(
            get_helm_child_link(),
            bhv_child_link,
            tf2::TimePointZero,
            10ms
        );

        // printf("helm_local = %s, bhv_child = %s\r\n", get_helm_child_link().c_str(), bhv_child_link.c_str());
        auto tf_blhl_eigen = tf2::transformToEigen(tf_bl_hl);

        Eigen::Vector3d uvw_helm;

        ///velocity
        uvw_helm = tf_blhl_eigen.rotation() *
                    Eigen::Vector3d(m_desired_value(DOF::U),
                                    m_desired_value(DOF::V), 
                                    m_desired_value(DOF::W));

        m_desired_value(DOF::U) = uvw_helm.x();
        m_desired_value(DOF::V) = uvw_helm.y();
        m_desired_value(DOF::W) = uvw_helm.z();

    } catch (const tf2::TransformException & e) {
            RCLCPP_WARN_STREAM_THROTTLE(m_logger, steady_clock, 10, std::string("Can't compute tf in direct contro: ") + e.what());
            RCLCPP_INFO( m_logger, "Could not transform %s to %s: %s",
                         get_helm_world_link().c_str(), bhv_global_link.c_str(), e.what() ); 
          return;

    }

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
    
    set_point->position.x = m_desired_value(DOF::X);
    set_point->position.y = m_desired_value(DOF::Y);
    set_point->position.z = m_desired_value(DOF::Z);

    // printf("set_point z = %lf\r\n", set_point->position.z);
    // Set orientation
    set_point->orientation.x = m_desired_value(DOF::ROLL);
    set_point->orientation.y = m_desired_value(DOF::PITCH);
    set_point->orientation.z = m_desired_value(DOF::YAW);

    // Set velocity
    set_point->velocity.x = m_desired_value(DOF::U);
    set_point->velocity.y = m_desired_value(DOF::V);
    set_point->velocity.z = m_desired_value(DOF::W);

    return true;
}


/**
 * @brief Behavior must export the class to the Plugin library.
 */
 #include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(helm::DirectControl, helm::BehaviorBase)