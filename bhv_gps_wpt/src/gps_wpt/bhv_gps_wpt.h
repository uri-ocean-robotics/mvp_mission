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
#include "vector"

namespace helm {

    class GpsWaypoint : public BehaviorBase {
    private:

        struct ll_t {
            double latitude;
            double longitude;
        } typedef ll_t;

        void initialize() override;

        ros::NodeHandlePtr m_pnh;

        void activated() override;

        void f_parse_ll_wpts();

        std::vector<ll_t> m_latlong_points;

        std::string m_state_fail;

        std::string m_target_topic;

        std::string m_fromll_service;

        ros::Publisher m_poly_pub;

        std::string m_target_frame_id;

    public:

        /**
         * @brief Trivial constructor
         */
        GpsWaypoint();

        /**
         * @brief Request set point from the behavior. It is consumed by helm.
         *
         * @param msg Result value of the behavior. This value is written by the
         *            Behavior. Helm uses this variable to generate set_point
         *            for the controller.
         * @return true if you want helm to use the result.
         * @return false if you don't want helm to use the result.
         */
        bool request_set_point(mvp_msgs::ControlProcess *msg) override;

    };
}