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

    Author: Emir Cem Gezer
    Email: emircem@uri.edu;emircem.gezer@gmail.com
    Year: 2022

    Copyright (C) 2022 Smart Ocean Systems Laboratory
*/

#pragma once

#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

#include "behavior_interface/behavior_base.h"

namespace helm 
{

using namespace std::chrono_literals;  //NOLINT

class BehaviorTemplate : public BehaviorBase
{
private:
    /**
     * @brief Initialize function
     *
     * @details This function initializes the behavior. 
     *
     * @param  parent pointer to user's node
     * @param  name The name of this planner
     *
     */
    void initialize(const rclcpp::Node::WeakPtr &parent) override;

    /**
     * @brief This function is triggered when a behavior gets activated
     * A plugin may or may not override this function.
     */
    void activated() override;

    /**
     * @brief This function is triggered when a behavior gets disabled
     * A plugin may or may not override this function.
     */
    void disabled() override;

    /**
     * @brief Callback function for Float64 msg subscriber
     * @param msg received std_msgs/msg/Float64 message
     */
    void templateCallback(const std_msgs::msg::Float64::SharedPtr msg);

    /**
     * @brief ros related 
     */
    rclcpp::Node::WeakPtr m_node;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pub;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_sub;

    rclcpp::Logger m_logger{rclcpp::get_logger("mvp2_mission_template_bhv")};

    /**
     * @brief variables get from callback, but will used for request_set_point
     * 
     * @details the shared value need be accessed by multi-thread ( one thread 
     *          from self behavior through callback(), one from behaviors management 
     *          from helm through request_set_point() ). So the shared data need 
     *          protection theoretically. atomic is the simplest way.  
     *          check: https://en.cppreference.com/w/cpp/atomic/atomic
     */
    std::atomic_long m_input_value;
    /**
     * @brief parameters load ros parameters
     */
    double m_param_double;

    // double m_test_double;
public:
    /**
     * @brief Trivial constructor
     */
    BehaviorTemplate();

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

} // namespace helm