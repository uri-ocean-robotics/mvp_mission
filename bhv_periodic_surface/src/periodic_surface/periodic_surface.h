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


namespace helm {

    class PeriodicSurface : public BehaviorBase {
    private:
        enum class BhvState : int {
            DISABLED,
            ENABLED,
            WAITING
        };

        BhvState m_bhv_state;

        /**
         * @brief
         * @todo To be implemented
         */
        void initialize() override;

        /**
         * @brief Trivial node handler
         */
        ros::NodeHandlePtr m_pnh;

        /**
         * @brief Forward distance of the surfacing behaviour
         *
         */
        double m_fwd_distance;

        /**
         * @brief maximum pitch that the behavior will command. in radians
         */
        double m_max_pitch;

        /**
         * @brief Surfacing period. in seconds
         *
         * A surfacing period ends when the vehicle is surfaced and it begins
         * when the #PeriodicSurface::m_surface_duration ends.
         */
        double m_surface_interval;

        /**
         * @brief Surfacing duration, in seconds
         *
         * Dictates the duration of the surfacing
         */
        double m_surface_duration;

        /**
         * @brief Unix time stamp of the time when behavior is activated again.
         *
         * When it activated, it indicates the time when the vehicle starts to
         * climb up again.
         */
        ros::Time m_start_time;

        /**
         * @brief Unix time stamp of the time when the vehicle is first surfaced
         */
        ros::Time m_surfaced_time;

        /**
         * @brief Implementation of #BehaviorBase::activated
         */
        void activated() override;

        /**
         * @brief Implementation of #BehaviorBase::disabled
         */
        void disabled() override;

    public:

        PeriodicSurface();

        bool request_set_point(mvp_msgs::ControlProcess *msg) override;

    };
}