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

#include "behavior_interface/behavior_base.h"
#include "ros/ros.h"
#include "mvp_msgs/ControlProcess.h"
#include "geometry_msgs/PolygonStamped.h"
#include "bhv_motion_evaluation/FreqMagConfig.h"
#include "mutex"
#include "dynamic_reconfigure/server.h"

namespace helm {

    class MotionEvaluation : public BehaviorBase {
    private:

        void initialize() override;

        /***********************************************************************
         * ROS
         */

        /**
         * @brief Trivial node handle
         */
        ros::NodeHandlePtr m_pnh;

        /**
         * @brief Control Process command message
         */
        mvp_msgs::ControlProcess m_cmd;

        bhv_motion_evaluation::FreqMagConfig m_config;

        std::recursive_mutex m_config_mutex;

        dynamic_reconfigure::Server<bhv_motion_evaluation::FreqMagConfig>
            m_dynconf_server;

        void f_dynconf_freqmag_cb(bhv_motion_evaluation::FreqMagConfig& conf,
                                  uint32_t level);

        double m_surge_phase;

        double m_yaw_rate_phase;

        double m_pitch_rate_phase;

        double m_yaw_phase;

        double m_pitch_phase;

        bool m_square_wave;

    public:

        MotionEvaluation();


        bool request_set_point(mvp_msgs::ControlProcess *msg) override;

    };
}