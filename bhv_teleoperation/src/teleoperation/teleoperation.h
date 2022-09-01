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

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Joy.h"

#include "behavior_interface/behavior_base.h"
#include "mvp_msgs/ControlProcess.h"

namespace helm {

    class Teleoperation : public BehaviorBase {
    private:
        /**
         * @brief Destroy the Teleoperation object
         */
        ~Teleoperation() override;

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
        void initialize() override;

        /**
         * @brief trivial node handler for load parameters
         */
        ros::NodeHandlePtr m_pnh;

        /**
         * @brief trivial node handler for subscribe and publish
         */
        ros::NodeHandlePtr m_nh;

        /**
         * @brief This function is inherited from #BehaviorBase
         */
        void activated() override;

        /**
         * @brief This function is inherited from #BehaviorBase
         */
        void disabled() override;

        /**
         * @brief Joystick subscriber
         */
        ros::Subscriber m_joy_sub;

        /**
         * @brief Trivial joy stick callback
         *
         * @details 1)record joystick input; 2)record gloabl information
         *          (e.g. pitch and yaw angle) if it's trigged by Joystick Enabled
         *          or Orientation Update by Joystick
         *
         * @param m Message
         */
        void f_joy_cb(const sensor_msgs::Joy::ConstPtr &m);

        /**
         * @brief Joystick value for surge (velocity in body frame x-axis)
         */
        std::atomic<double> m_joy_surge;

        /**
         * @brief Joystick value for yaw angular velocity
         */
        std::atomic<double> m_joy_yaw_rate;

        /**
         * @brief Joystick value for pitch angular velocity
         */
        std::atomic<double> m_joy_pitch_rate;

        /**
         * @brief Max surge value from joystick input
         */
        double m_max_surge;

        /**
         * @brief Max picth rate value from joystick input
         */
        double m_max_pitch_rate;

        /**
         * @brief Max yaw rate value from joystick input
         */
        double m_max_yaw_rate;

        /**
         * @brief ROS rostopic for joystick node
         */
        std::string m_joy_topic_name;

        /**
         * @brief Joystick map for surge control stick
         */
        int m_axes_surge;

        /**
         * @brief Joystick map for pitch control stick
         */
        int m_axes_pitch;

        /**
         * @brief Joystick map for yaw control stick
         */
        int m_axes_yaw;

        /**
         * @brief Joystick map for enable button
         */
        int m_button_enable;

        /**
         * @brief Value to indicate joystick is enabled or not
         */
        std::atomic<bool> m_use_joy;

        /**
         * @brief Value to indicate joystick has pitch input previously
         */
        bool m_last_pitch;

        /**
         * @brief Value to indicate joystick has yaw input previously
         */
        bool m_last_yaw;

        /**
         * @brief Value of recorded picth angle in global frame
         */
        std::atomic<double> m_recorded_picth;

        /**
         * @brief Value of recorded yaw angle in global frame
         */
        std::atomic<double> m_recorded_yaw;

    public:

        /**
         * @brief Trivial constructor
         */
        Teleoperation();

        /**
         * @brief Request set point from the behavior. It is consumed by helm.
         *
         * @param msg Result value of the behavior. This value is written by the
         *            Behavior. Helm uses this variable to generate set_point
         *            for the controller.
         * @return true if you want helm to use the result.
         * @return false if you don't want helm to use the result.
         */
        bool request_set_point(mvp_msgs::ControlProcess *msg) override;

    };
}