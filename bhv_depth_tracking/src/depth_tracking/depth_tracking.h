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
#include "std_msgs/Float64.h"

namespace helm {

    class DepthTracking : public BehaviorBase {
    private:
        /**
         * @brief
         * @todo To be implemented
         */
        void initialize() override;

        /**
         * @brief trivial node handler
         */
        ros::NodeHandlePtr m_nh;

        /**
         * @brief requested depth
         */
        double m_requested_depth;

        /**
         * @brief Trivial subscriber object
         */
        ros::Subscriber m_sub;

        /**
         * @brief Trvial subscriber callback
         *
         * @param msg
         */
        void f_cb_sub(const std_msgs::Float64::ConstPtr& msg);

        double m_max_pitch;

        double m_fwd_distance;

        bool m_use_heave_velocity;
        bool m_pitch_enabled;
        virtual auto configure_dofs() -> decltype(m_dofs) final;

    public:

        /**
         * @brief Trivial constructor
         */
        DepthTracking();

        ~DepthTracking();

        bool request_set_point(mvp_msgs::ControlProcess *msg) override;

    };
}