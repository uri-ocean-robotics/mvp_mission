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

#include "motion_evaluation.h"
#include "pluginlib/class_list_macros.h"
#include "memory"
#include "vector"
#include "thread"
#include "dynamic_reconfigure/server.h"
#include "chrono"

using namespace helm;

MotionEvaluation::MotionEvaluation() : BehaviorBase() {

}

void MotionEvaluation::initialize() {

    m_pnh.reset(
        new ros::NodeHandle(ros::this_node::getName() + "/" + get_name())
    );

    BehaviorBase::m_dofs = decltype(m_dofs){
        mvp_msgs::ControlMode::DOF_SURGE,
        mvp_msgs::ControlMode::DOF_YAW_RATE,
        mvp_msgs::ControlMode::DOF_PITCH_RATE,
        mvp_msgs::ControlMode::DOF_YAW,
        mvp_msgs::ControlMode::DOF_PITCH
    };

    m_dynconf_server.setCallback(
        std::bind(&MotionEvaluation::f_dynconf_freqmag_cb, this,
                  std::placeholders::_1,
                  std::placeholders::_2
        )
    );

    m_pnh->param<bool>("square_wave", m_square_wave, false);

    m_surge_phase = 0;
    m_pitch_rate_phase = 0;
    m_yaw_rate_phase = 0;
    m_yaw_phase = 0;
    m_pitch_phase = 0;

}

void MotionEvaluation::f_dynconf_freqmag_cb(
    bhv_motion_evaluation::FreqMagConfig &conf, uint32_t level)
{
    std::scoped_lock lock(m_config_mutex);

    m_config = conf;

}

bool MotionEvaluation::request_set_point(mvp_msgs::ControlProcess *set_point)
{
    std::scoped_lock lock(m_config_mutex);

    /*
     * Decide the action needs to be taken
     */
    m_cmd.header.frame_id = BehaviorBase::m_process_values.header.frame_id;

    if(m_config.surge_frequency != 0) {
        m_surge_phase += M_PI / (get_helm_frequency() / m_config.surge_frequency);

        double w = sin(m_surge_phase);
        if(m_square_wave) {
            w = (w > 0 ? 1 : -1);
        }

        m_cmd.velocity.x = w * m_config.surge_magnitude;
    } else {
        m_cmd.velocity.x = m_config.surge_magnitude;
    }

    if(m_config.yaw_rate_frequency != 0) {
        m_yaw_rate_phase += M_PI / (get_helm_frequency() / m_config.yaw_rate_frequency);

        double w = sin(m_yaw_rate_phase);
        if(m_square_wave) {
            w = (w > 0 ? 1 : -1);
        }

        m_cmd.angular_rate.z = w * m_config.yaw_rate_magnitude;
    } else {
        m_cmd.angular_rate.z = m_config.yaw_rate_magnitude;
    }

    if(m_config.pitch_rate_frequency != 0) {
        m_pitch_rate_phase += M_PI / (get_helm_frequency() / m_config.pitch_rate_frequency);

        double w = sin(m_pitch_rate_phase);
        if(m_square_wave) {
            w = (w > 0 ? 1 : -1);
        }

        m_cmd.angular_rate.y = w * m_config.pitch_rate_magnitude;
    } else {
        m_cmd.angular_rate.y = m_config.pitch_rate_magnitude;
    }

    if(m_config.yaw_frequency != 0) {
        m_yaw_phase += M_PI / (get_helm_frequency() / m_config.yaw_frequency);

        double w = sin(m_yaw_phase);
        if(m_square_wave) {
            w = (w > 0 ? 1 : -1);
        }

        m_cmd.orientation.z = w * m_config.yaw_magnitude;
    } else {
        m_cmd.orientation.z = m_config.yaw_magnitude;
    }

    if(m_config.pitch_frequency != 0) {
        m_pitch_phase += M_PI / (get_helm_frequency() / m_config.pitch_frequency);

        double w = sin(m_pitch_phase);
        if(m_square_wave) {
            w = (w > 0 ? 1 : -1);
        }

        m_cmd.orientation.y = w * m_config.pitch_magnitude;
    } else {
        m_cmd.orientation.y = m_config.pitch_magnitude;
    }
    /*
     * Command it to the helm
     */
    *set_point = m_cmd;

    return true;
}

PLUGINLIB_EXPORT_CLASS(helm::MotionEvaluation, helm::BehaviorBase)