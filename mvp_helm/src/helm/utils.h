#pragma once

#include "array"
#include "mvp_control/dictionary.h"
#include "mvp_control/ControlProcess.h"

namespace helm {

    namespace utils {
        std::array<double, ctrl::CONTROLLABLE_DOF_LENGTH> control_process_to_array(
            const mvp_control::ControlProcess& msg)
        {
            std::array<double, ctrl::CONTROLLABLE_DOF_LENGTH> a {};
            a[ctrl::DOF::X] = msg.position.x;
            a[ctrl::DOF::Y] = msg.position.y;
            a[ctrl::DOF::Z] = msg.position.z;

            a[ctrl::DOF::ROLL] = msg.orientation.x;
            a[ctrl::DOF::PITCH] = msg.orientation.y;
            a[ctrl::DOF::YAW] = msg.orientation.z;

            a[ctrl::DOF::SURGE] = msg.velocity.x;
            a[ctrl::DOF::SWAY] = msg.velocity.y;
            a[ctrl::DOF::HEAVE] = msg.velocity.z;

            a[ctrl::DOF::ROLL_RATE] = msg.angular_rate.x;
            a[ctrl::DOF::PITCH_RATE] = msg.angular_rate.y;
            a[ctrl::DOF::YAW_RATE] = msg.angular_rate.z;

            return  a;
        }

        mvp_control::ControlProcess array_to_control_process_msg(
            const std::array<double, ctrl::CONTROLLABLE_DOF_LENGTH>& a)
        {
            mvp_control::ControlProcess msg;
            msg.position.x = a[ctrl::DOF::X];
            msg.position.y = a[ctrl::DOF::Y];
            msg.position.z = a[ctrl::DOF::Z];

            msg.orientation.x = a[ctrl::DOF::ROLL];
            msg.orientation.y = a[ctrl::DOF::PITCH];
            msg.orientation.z = a[ctrl::DOF::YAW];

            msg.velocity.x = a[ctrl::DOF::SURGE];
            msg.velocity.y = a[ctrl::DOF::SWAY];
            msg.velocity.z = a[ctrl::DOF::HEAVE];

            msg.angular_rate.x = a[ctrl::DOF::ROLL_RATE];
            msg.angular_rate.y = a[ctrl::DOF::PITCH_RATE];
            msg.angular_rate.z = a[ctrl::DOF::YAW_RATE];
            return msg;
        }
    }

}