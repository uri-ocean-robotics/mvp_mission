#pragma once

#include "array"
#include "mvp_control/dictionary.h"
#include "mvp_control/ControlState.h"

namespace helm {

    namespace utils {
        std::array<double, ctrl::STATE_DOF_SIZE> control_state_msg_to_array(
            const mvp_control::ControlState& msg)
        {
            std::array<double, ctrl::STATE_DOF_SIZE> a {};
            a[ctrl::DOF::X] = msg.position.x;
            a[ctrl::DOF::Y] = msg.position.y;
            a[ctrl::DOF::Z] = msg.position.z;

            a[ctrl::DOF::ROLL] = msg.orientation.x;
            a[ctrl::DOF::PITCH] = msg.orientation.y;
            a[ctrl::DOF::YAW] = msg.orientation.z;

            a[ctrl::DOF::SURGE] = msg.velocity.x;
            a[ctrl::DOF::SWAY] = msg.velocity.y;
            a[ctrl::DOF::HEAVE] = msg.velocity.z;
            return  a;
        }

        mvp_control::ControlState array_to_control_state_msg(
            const std::array<double, ctrl::STATE_DOF_SIZE>& a)
        {
            mvp_control::ControlState msg;
            msg.position.x = a[ctrl::DOF::X];
            msg.position.y = a[ctrl::DOF::Y];
            msg.position.z = a[ctrl::DOF::Z];

            msg.orientation.x = a[ctrl::DOF::ROLL];
            msg.orientation.y = a[ctrl::DOF::PITCH];
            msg.orientation.z = a[ctrl::DOF::YAW];

            msg.velocity.x = a[ctrl::DOF::SURGE];
            msg.velocity.y = a[ctrl::DOF::SWAY];
            msg.velocity.z = a[ctrl::DOF::HEAVE];
            return msg;
        }
    }

}