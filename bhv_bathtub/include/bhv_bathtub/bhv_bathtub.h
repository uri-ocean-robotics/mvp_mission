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
#include "mvp_msgs/msg/control_process.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_listener.h"
#include "Eigen/Dense"


namespace helm {

using namespace std::chrono_literals;  //NOLINT

class Bathtub : public BehaviorBase {
private:
    /**
        * @brief Destroy the Teleoperation object
        */
    ~Bathtub() override;

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
        * @brief global link id
    */
    std::string bhv_global_link;

    /**
        * @brief timer
     */
    rclcpp::Time bhv_timer;

    /**
        * @brief desired setpoint values
     */
    double m_desired_depth;

    double m_desired_pitch;
    /**
        * @brief depth segments
     */
    std::vector<double> m_depth_list;

    /**
        * @brief diving angle segments
     */
    std::vector<double> m_dive_ang_list;

    /**
        * @brief climbing angle segments
     */
    std::vector<double> m_climb_ang_list;

    /**
        * @brief depth segments time
     */
    std::vector<double> m_depth_time_list;

    /**
        * @brief inactive time
     */
    double m_no_depth_time;

     /**
        * @brief current depth index
     */
    int m_depth_index;

    void transform_setpoint();

    //! @brief Transform buffer for TF2
    std::unique_ptr<tf2_ros::Buffer> m_transform_buffer;

    //! @brief Transform listener for TF2
    std::unique_ptr<tf2_ros::TransformListener> m_transform_listener;

    rclcpp::Logger m_logger{rclcpp::get_logger("mvp2_mission_bhv_direct_control")};


    
public:

    /**
        * @brief Trivial constructor
        */
    Bathtub();

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
