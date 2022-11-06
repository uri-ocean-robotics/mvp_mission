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

#include "bhv_hold_position.h"
#include "pluginlib/class_list_macros.h"

using namespace helm;

void HoldPosition::initialize() {

    m_pnh.reset(
        new ros::NodeHandle(ros::this_node::getName() + "/" + get_name())
    );

    BehaviorBase::m_dofs = decltype(m_dofs){
        mvp_msgs::ControlMode::DOF_X,
        mvp_msgs::ControlMode::DOF_Y,
        mvp_msgs::ControlMode::DOF_Z,
        mvp_msgs::ControlMode::DOF_YAW,
        mvp_msgs::ControlMode::DOF_SURGE
    };

}

HoldPosition::HoldPosition() {
    std::cout << "A message from the hold position behavior" << std::endl;
}

void HoldPosition::activated() {
    m_desired = m_process_values;
}

bool HoldPosition::request_set_point(
    mvp_msgs::ControlProcess *set_point) {

    mvp_msgs::ControlProcess p;

    p.position.x = m_desired.position.x;
    p.position.y = m_desired.position.y;
    p.position.z = 0;

    p.orientation = m_desired.orientation;

    *set_point = p;

    return true;
}

/**
 * @brief Behavior must export the class to the Plugin library.
 */
PLUGINLIB_EXPORT_CLASS(helm::HoldPosition, helm::BehaviorBase)