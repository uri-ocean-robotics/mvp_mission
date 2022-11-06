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


#include "bhv_sawtooth_wave.h"
#include "pluginlib/class_list_macros.h"

using namespace helm;

void SawtoothWave::initialize() {

    m_pnh.reset(
        new ros::NodeHandle(ros::this_node::getName() + "/" + get_name())
    );

    m_pnh->param("min_depth", m_min_depth, 0.0); // meters

    m_pnh->param("max_depth", m_max_depth, 5.0); // meters

    m_pnh->param("surge_velocity", m_surge_velocity, 0.65); // m/s

    m_pnh->param("heading", m_heading, 0.0); // radians

    // radians. default: 22.5 degrees
    m_pnh->param("pitch", m_pitch, 0.39269908169872414);

    BehaviorBase::m_dofs = decltype(m_dofs){
        mvp_msgs::ControlMode::DOF_SURGE,
        mvp_msgs::ControlMode::DOF_PITCH,
        mvp_msgs::ControlMode::DOF_YAW
    };

    m_bhv_state = BHV_STATE::IDLE;

}

SawtoothWave::SawtoothWave() {

}

void SawtoothWave::activated() {

    if(BehaviorBase::m_process_values.position.z > m_max_depth) {
        m_bhv_state = BHV_STATE::ASCENDING;
    }

    if(BehaviorBase::m_process_values.position.z < m_min_depth) {
        m_bhv_state = BHV_STATE::DESCENDING;
    }
}

void SawtoothWave::disabled() {

    m_bhv_state = BHV_STATE::IDLE;

}


bool SawtoothWave::request_set_point(
    mvp_msgs::ControlProcess *set_point) {

    set_point->orientation.z = m_heading;

    set_point->velocity.x = m_surge_velocity;

    if(m_bhv_state == BHV_STATE::ASCENDING) {
        if(BehaviorBase::m_process_values.position.z < m_min_depth) {
            m_bhv_state = BHV_STATE::DESCENDING;
            return false;
        }
        set_point->orientation.y = m_pitch;


    } else if (m_bhv_state == BHV_STATE::DESCENDING){
        if(BehaviorBase::m_process_values.position.z > m_max_depth) {
            m_bhv_state = BHV_STATE::ASCENDING;
            return false;
        }
        set_point->orientation.y = -m_pitch;
    }

    return true;
}

PLUGINLIB_EXPORT_CLASS(helm::SawtoothWave, helm::BehaviorBase)