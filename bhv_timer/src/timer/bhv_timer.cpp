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


#include "bhv_timer.h"
#include "pluginlib/class_list_macros.h"

using namespace helm;

void Timer::initialize() {

    m_pnh.reset(
        new ros::NodeHandle(ros::this_node::getName() + "/" + get_name())
    );

    if(!m_pnh->hasParam("duration")) {
        // ill configuration
    }

    m_pnh->param<double>("duration", m_duration, 0.0);

    // String: A state to be requested after a failed execution
    m_pnh->param<std::string>("transition_to", m_transition_to, "");

}

Timer::Timer() {

}

void Timer::activated() {

    m_t = ros::Time::now();

}

bool Timer::request_set_point(mvp_msgs::ControlProcess *set_point) {

    if(m_duration != 0.0 && !m_transition_to.empty()) {

        if((ros::Time::now() - m_t).toSec() > m_duration) {
            change_state(m_transition_to);
        }
    }

    return false;
}

/**
 * @brief Behavior must export the class to the Plugin library.
 */
PLUGINLIB_EXPORT_CLASS(helm::Timer, helm::BehaviorBase)