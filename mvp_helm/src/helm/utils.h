#pragma once

#include "array"
#include "mvp_msgs/ControlProcess.h"

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
            const mvp_msgs::ControlProcess& msg)
        {
            std::array<double, 12> a {};
            a[mvp_msgs::ControlMode::DOF_X] = msg.position.x;
            a[mvp_msgs::ControlMode::DOF_Y] = msg.position.y;
            a[mvp_msgs::ControlMode::DOF_Z] = msg.position.z;

            a[mvp_msgs::ControlMode::DOF_ROLL] = msg.orientation.x;
            a[mvp_msgs::ControlMode::DOF_PITCH] = msg.orientation.y;
            a[mvp_msgs::ControlMode::DOF_YAW] = msg.orientation.z;

            a[mvp_msgs::ControlMode::DOF_SURGE] = msg.velocity.x;
            a[mvp_msgs::ControlMode::DOF_SWAY] = msg.velocity.y;
            a[mvp_msgs::ControlMode::DOF_HEAVE] = msg.velocity.z;

            a[mvp_msgs::ControlMode::DOF_ROLL_RATE] = msg.angular_rate.x;
            a[mvp_msgs::ControlMode::DOF_PITCH_RATE] = msg.angular_rate.y;
            a[mvp_msgs::ControlMode::DOF_YAW_RATE] = msg.angular_rate.z;

            return  a;
        }

        /**
         * @brief
         * @todo Write the documentation
         *
         * @param a
         * @return mvp_msgs::ControlProcess
         */
        mvp_msgs::ControlProcess array_to_control_process_msg(
            const std::array<double, 12>& a)
        {
            mvp_msgs::ControlProcess msg;
            msg.position.x = a[mvp_msgs::ControlMode::DOF_X];
            msg.position.y = a[mvp_msgs::ControlMode::DOF_Y];
            msg.position.z = a[mvp_msgs::ControlMode::DOF_Z];

            msg.orientation.x = a[mvp_msgs::ControlMode::DOF_ROLL];
            msg.orientation.y = a[mvp_msgs::ControlMode::DOF_PITCH];
            msg.orientation.z = a[mvp_msgs::ControlMode::DOF_YAW];

            msg.velocity.x = a[mvp_msgs::ControlMode::DOF_SURGE];
            msg.velocity.y = a[mvp_msgs::ControlMode::DOF_SWAY];
            msg.velocity.z = a[mvp_msgs::ControlMode::DOF_HEAVE];

            msg.angular_rate.x = a[mvp_msgs::ControlMode::DOF_ROLL_RATE];
            msg.angular_rate.y = a[mvp_msgs::ControlMode::DOF_PITCH_RATE];
            msg.angular_rate.z = a[mvp_msgs::ControlMode::DOF_YAW_RATE];
            return msg;
        }
    }

}