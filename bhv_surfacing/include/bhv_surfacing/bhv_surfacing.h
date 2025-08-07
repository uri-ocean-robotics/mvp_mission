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


#pragma once

#include "atomic"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "behavior_interface/behavior_base.h"
#include "mvp_msgs/msg/control_process.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_listener.h"

#include "Eigen/Dense"

namespace helm {

using namespace std::chrono_literals;  //NOLINT

class Surfacing : public BehaviorBase {
private:
    /**
        * @brief Destroy the Teleoperation object
        */
    ~Surfacing() override;

    /**
        * @brief Initialize function
        *
        * @details This function initializes the behavior. It is called by the
        * helm. It is a pure virtual function and it must be implemented in the
        * behavior. If left unimplemented, code will not compile. In this
        * function, user is responsible for proper initalization of the
        * behavior. This function must be unblocking, otherwise, every other
        * behavior may wait this function to return. In this function user
        * should create #ros::NodeHandle, and define controlled degrees of
        * freedom. Below is a trivial implementation of this function
        *
        */
    void initialize(const rclcpp::Node::WeakPtr &parent) override;

    /**
        * @brief This function is inherited from #BehaviorBase
        */
    void activated() override;

    /**
        * @brief This function is inherited from #BehaviorBase
        */
    void disabled() override;
    /**
     * @brief ros related 
     */
    rclcpp::Node::WeakPtr m_node;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_ctrl_set_client;    

    rclcpp::Logger m_logger{rclcpp::get_logger("mvp2_mission_bhv_surfacing")};


    double m_last_gps_time; //last time was surfaced with gps

    double m_first_gps_time; //inital gps time

    double m_last_comm_time;   //later we need a heartbeat from c2.

    double u_submerged_period_with_no_gps; //the timeout for triggering the surfacing

    double u_submerged_period_with_no_comm;  //the timeout for triggering the surfacing

    double m_last_imu_time;

    double m_last_dvl_time;

    double u_no_imu_timeout;

    double u_no_dvl_timeout;

    std::string u_navigation_fail_state;

    double u_surfacing_duration; //how long it will stay at the surface

    double c_surfacing_depth; //the surfacing depth

    bool u_floating_to_surface_flag; //floating to surface? 

    bool m_gps_flag = false;

    bool m_imu_flag = true;

    bool m_dvl_flag= true;

    bool m_comm_flag = false;

    bool m_set_point_pub = false;

    bool m_active_flag = false;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_gps_fix_subscriber;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr m_dvl_sub;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_dive_trigger_srv;

    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr m_surfacing_flag_pub;

    std_msgs::msg::Int8MultiArray m_surfacing_flag;

    void f_cb_gps_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    void f_cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg);

    void f_cb_dvl(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);


    void f_dive_trigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    std::string m_ctrl_set_srv;

    std::string bhv_global_link;

    std::string bhv_child_link;

    mvp_msgs::msg::ControlProcess m_bhv_setpoint; 

    void transform_setpoint();

    //! @brief Transform buffer for TF2
    std::unique_ptr<tf2_ros::Buffer> m_transform_buffer;

    //! @brief Transform listener for TF2
    std::unique_ptr<tf2_ros::TransformListener> m_transform_listener;

public:

    /**
        * @brief Trivial constructor
        */
    Surfacing();

    /**
        * @brief Request set point from the behavior. It is consumed by helm.
        *
        * @param msg Result value of the behavior. This value is written by the
        *            Behavior. Helm uses this variable to generate set_point
        *            for the controller.
        * @return true if you want helm to use the result.
        * @return false if you don't want helm to use the result.
        */
    bool request_set_point(mvp_msgs::msg::ControlProcess *set_point) override;

};

}
