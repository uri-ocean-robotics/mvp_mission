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
    Year: 2023

    Copyright (C) 2023 Smart Ocean Systems Laboratory
*/

#pragma once

#include "ros/ros.h"

#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"
#include "behavior_interface/behavior_base.h"
#include "mvp_msgs/ControlProcess.h"
#include "sensor_msgs/Joy.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_listener.h"


namespace helm {

    class DirectControlCont : public BehaviorBase {
    private:
        /**
         * @brief Destroy the DirectControlCont object
         */
        ~DirectControlCont() override;

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
         * @brief subscriber for new values
         */
        ros::Subscriber continuous_command_sub;

        /**
         * @brief subscriber for joy command
         */
        ros::Subscriber joy_sub;

        /**
         * @brief mvp message
         */
        void continuous_update(const mvp_msgs::ControlProcess::ConstPtr& new_values);

        /**
         * @brief joy message
         */
        // void tele_op(const sensor_msgs::Joy::ConstPtr& msg);
        /**
         * @brief global link id
        */
        std::string global_link;

        /**
         * @brief local link id
        */
        std::string local_link;

        /**
         * @brief Max value for x
        */
        double m_max_x;

        /**
         * @brief Max value for y
        */
        double m_max_y;

        /**
         * @brief Max value for z
        */
        double m_max_z;

        /**
         * @brief Max value for roll
        */
        double m_max_roll;

        /**
         * @brief Max value for pitch
        */
        double m_max_pitch;

        /**
         * @brief Max value for yaw
        */
        double m_max_yaw;

        /**
         * @brief Max value for surge
        */
        double m_max_surge;

        /**
         * @brief Max value for sway
        */
        double m_max_sway;

        /**
         * @brief Max value for heave
        */
        double m_max_heave;

        /**
         * @brief Max value for roll rate
        */
        double m_max_roll_rate;

        /**
         * @brief Max value for pitch rate
        */
        double m_max_pitch_rate;

        /**
         * @brief Max value for yaw rate
        */
        double m_max_yaw_rate;

        /**
         * @brief Desired value for x
        */
        double m_desired_x;

        /**
         * @brief Desired value for y
        */
        double m_desired_y;

        /**
         * @brief Desired value for z
        */
        double m_desired_z;

        /**
         * @brief Desired value for roll
        */
        double m_desired_roll;

        /**
         * @brief Desired value for pitch
        */
        double m_desired_pitch;

        /**
         * @brief Desired value for yaw
        */
        double m_desired_yaw;

        /**
         * @brief Desired value for surge
        */
        double m_desired_surge;

        /**
         * @brief Desired value for sway
        */
        double m_desired_sway;

        /**
         * @brief Desired value for heave
        */
        double m_desired_heave;

        /**
         * @brief Desired value for roll rate
        */
        double m_desired_roll_rate;

        /**
         * @brief Desired value for pitch rate
        */
        double m_desired_pitch_rate;

        /**
         * @brief Desired value for yaw rate
        */
        double m_desired_yaw_rate;

        /**
         * @brief teleop yaw increments
        */
        double m_tele_d_yaw;
        /**
         * @brief teleop pitch increments
        */
        double m_tele_d_pitch;
        /**
         * @brief teleop z increments
        */
        double m_tele_d_depth;

        /**
         * @brief teleop surge scale
        */
        double m_tele_s_surge;
        /**
         * @brief teleop sway scale
        */
        double m_tele_s_sway;


        //! @brief Transform buffer for TF2
        tf2_ros::Buffer m_transform_buffer;
        
        /**
         * @brief Transform listener for TF2
         */
        std::shared_ptr<tf2_ros::TransformListener> m_transform_listener;

        /**
         * @brief for calling controller service
         */
        std::string m_ctrl_disable;
        std::string m_ctrl_enable;

    public:

        /**
         * @brief Trivial constructor
         */
        DirectControlCont();

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