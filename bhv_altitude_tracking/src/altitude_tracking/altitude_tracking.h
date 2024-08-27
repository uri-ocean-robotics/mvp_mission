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
#include "geometry_msgs/PointStamped.h"
#include "tf2_ros/transform_listener.h"

namespace helm {

    class AltitudeTracking : public BehaviorBase {
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
         * @brief Trivial node handler
         */
        ros::NodeHandlePtr m_pnh;


        /**
         * @brief requested depth
         */
        double m_requested_depth;

        /**
         * @brief minimum altitude
         */
        double m_min_altitude;

        /**
          * @brief altitude flag
        */
        bool m_altitude_safety_flag = false;

        /**
         * @brief world_ned altitude
         */
        double m_altitude;

         /**
         * @brief world_ned altitude
         */
        int m_altitude_mode;  //-1: disabled, 0: safety, 1:continuous

        ros::Subscriber m_altitude_sub;

        /**
         * @brief altitude topic
         */
        std::string m_altitude_topic_name;

        void f_altitude_cb(const geometry_msgs::PointStamped::ConstPtr &m);

        /**
         * @brief Transform buffer for TF2
         */
        tf2_ros::Buffer m_transform_buffer;

        /**
         * @brief Transform listener for TF2
         */
        std::shared_ptr<tf2_ros::TransformListener> m_transform_listener;


    public:

        /**
         * @brief Trivial constructor
         */
        AltitudeTracking();

        ~AltitudeTracking();

        bool request_set_point(mvp_msgs::ControlProcess *msg) override;

    };
}