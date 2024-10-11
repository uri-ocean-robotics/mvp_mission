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
    Email: linzhao@uri.edu
    Year: 2024

    Copyright (C) 2024 Smart Ocean Systems Laboratory

    This program will guide the AUV depth to perform depth keeping
    at different time intervals.
*/

#pragma once

#include "atomic"
#include "rclcpp/rclcpp.hpp"
#include "behavior_interface/behavior_base.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_listener.h"

namespace helm {

using namespace std::chrono_literals;  //NOLINT

class AltitudeTracking : public BehaviorBase {
private:
    /**
        * @brief Destroy the Teleoperation object
        */
    ~AltitudeTracking() override;

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

    /**
        * @brief desired setpoint values
     */
    double m_desired_altitude;

    double m_altitude;

    int m_altitude_mode;  //-1: disabled, 0: safety_mode, 1:continuous

    //altitude topic from dvl
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_altitude_sub;

    //allowing user to change desired altitude on-the-fly
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_desired_altitude_sub;
    
    void f_m_altitude_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    void f_c_altitude_cb(const std_msgs::msg::Float64::SharedPtr msg);


    //! @brief Transform buffer for TF2
    std::unique_ptr<tf2_ros::Buffer> m_transform_buffer;

    //! @brief Transform listener for TF2
    std::unique_ptr<tf2_ros::TransformListener> m_transform_listener;

    rclcpp::Logger m_logger{rclcpp::get_logger("mvp2_mission_bhv_direct_control")};


    
public:

    /**
        * @brief Trivial constructor
        */
    AltitudeTracking();

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
