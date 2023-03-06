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


#include "periodic_surface.h"
#include "pluginlib/class_list_macros.h"

using namespace helm;

void PeriodicSurface::initialize() {

    m_pnh.reset(
        new ros::NodeHandle(ros::this_node::getName() + "/" + get_name())
    );

    BehaviorBase::m_dofs = decltype(m_dofs){
        mvp_msgs::ControlMode::DOF_PITCH,
        mvp_msgs::ControlMode::DOF_Z
    };

    m_pnh->param("forward_distance", m_fwd_distance, 3.0);

    m_pnh->param<double>("max_pitch", m_max_pitch, M_PI_2);
    m_pnh->param<double>("surface_interval", m_surface_interval, 10.0); //seconds
    m_pnh->param<double>("surface_duration", m_surface_duration, 10.0); //seconds

    m_activated = false;

    m_bhv_state = BhvState::DISABLED;

}


/**
 * @brief Construct a new Periodic Surface:: Periodic Surface object
 *
 */
PeriodicSurface::PeriodicSurface()
{
    std::cout << "a message from periodic surface" << std::endl;
}

void PeriodicSurface::activated() {

    // This will be triggered when the behaviour is activated.
    m_start_time = ros::Time::now();

    m_bhv_state = BhvState::ENABLED;

    m_activated = true;

}

void PeriodicSurface::disabled() {

    m_bhv_state = BhvState::DISABLED;

    m_activated = false;

}

/**
 * @brief Commands the controller with a set point from the behavior
 *
 * @param set_point
 * @return true
 * @return false
 */
bool PeriodicSurface::request_set_point(mvp_msgs::ControlProcess *set_point)
{

    if(!m_activated) {
        return false;
    }

    //check depth to set m_surfaced_time

    if(m_bhv_state == BhvState::ENABLED) {

        if(BehaviorBase::m_process_values.position.z < 0.5){
            m_bhv_state = BhvState::WAITING;
            m_surfaced_time = ros::Time::now();
        }

    } else if (m_bhv_state == BhvState::WAITING) {

        if(ros::Time::now().sec - m_surfaced_time.sec > m_surface_duration) {
            m_bhv_state = BhvState::DISABLED;
            m_start_time = ros::Time::now();

            return false;
        }

    } else if (m_bhv_state == BhvState::DISABLED) {

        if(ros::Time::now().sec - m_start_time.sec > m_surface_interval) {
            m_bhv_state = BhvState::ENABLED;
        } else {
            return false;
        }

    }

    double pitch = atan(BehaviorBase::m_process_values.position.z / m_fwd_distance);

    if(fabs(pitch) > m_max_pitch) {
        set_point->orientation.y = pitch >= 0 ? m_max_pitch : -m_max_pitch;
    } else {
        set_point->orientation.y = pitch;
    }

    set_point->position.z = 0;

    return true;
}


PLUGINLIB_EXPORT_CLASS(helm::PeriodicSurface, helm::BehaviorBase)