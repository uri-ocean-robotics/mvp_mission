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

    class SawtoothWave : public BehaviorBase {
    private:

        enum class BHV_STATE : int{
            IDLE,
            ASCENDING,
            DESCENDING
        };

        void initialize() override;

        double m_min_depth;

        double m_max_depth;

        double m_pitch;

        double m_surge_velocity;

        double m_heading;

        BHV_STATE m_bhv_state;

        /**
         * @brief trivial node handler
         */
        ros::NodeHandlePtr m_pnh;

        void activated() override;

        void disabled() override;

    public:


        SawtoothWave();

        bool request_set_point(mvp_msgs::ControlProcess *msg) override;

    };
}