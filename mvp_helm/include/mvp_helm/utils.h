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

#include "array"
#include "mvp_msgs/msg/control_process.h"

namespace helm {

    namespace utils {
        /**
         * @brief
         * @todo Write the documentation
         *
         * @param msg
         * @return std::array<double, 12>
         */
        std::array<double, 12> control_process_to_array(
            const mvp_msgs::msg::ControlProcess *msg)
        {
            std::array<double, 12> a {};
            a[mvp_msgs::msg::ControlMode::DOF_X] = msg->position.x;
            a[mvp_msgs::msg::ControlMode::DOF_Y] = msg->position.y;
            a[mvp_msgs::msg::ControlMode::DOF_Z] = msg->position.z;

            a[mvp_msgs::msg::ControlMode::DOF_PITCH] = msg->orientation.y;
            a[mvp_msgs::msg::ControlMode::DOF_YAW] = msg->orientation.z;

            a[mvp_msgs::msg::ControlMode::DOF_U] = msg->velocity.x;
            a[mvp_msgs::msg::ControlMode::DOF_V] = msg->velocity.y;
            a[mvp_msgs::msg::ControlMode::DOF_W] = msg->velocity.z;

            a[mvp_msgs::msg::ControlMode::DOF_P] = msg->angular_rate.x;
            a[mvp_msgs::msg::ControlMode::DOF_Q] = msg->angular_rate.y;
            a[mvp_msgs::msg::ControlMode::DOF_R] = msg->angular_rate.z;

            return  a;
        }

        /**
         * @brief
         * @todo Write the documentation
         *
         * @param a
         * @return mvp_msgs::ControlProcess
         */
        mvp_msgs::msg::ControlProcess array_to_control_process_msg(
            const std::array<double, 12>& a)
        {
            mvp_msgs::msg::ControlProcess msg;
            msg.position.x = a[mvp_msgs::msg::ControlMode::DOF_X];
            msg.position.y = a[mvp_msgs::msg::ControlMode::DOF_Y];
            msg.position.z = a[mvp_msgs::msg::ControlMode::DOF_Z];

            msg.orientation.x = a[mvp_msgs::msg::ControlMode::DOF_ROLL];
            msg.orientation.y = a[mvp_msgs::msg::ControlMode::DOF_PITCH];
            msg.orientation.z = a[mvp_msgs::msg::ControlMode::DOF_YAW];

            msg.velocity.x = a[mvp_msgs::msg::ControlMode::DOF_U];
            msg.velocity.y = a[mvp_msgs::msg::ControlMode::DOF_V];
            msg.velocity.z = a[mvp_msgs::msg::ControlMode::DOF_W];

            msg.angular_rate.x = a[mvp_msgs::msg::ControlMode::DOF_P];
            msg.angular_rate.y = a[mvp_msgs::msg::ControlMode::DOF_Q];
            msg.angular_rate.z = a[mvp_msgs::msg::ControlMode::DOF_R];
            return msg;
        }
    }

}