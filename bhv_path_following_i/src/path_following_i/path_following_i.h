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
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/Marker.h"


namespace helm {

    class PathFollowingI : public BehaviorBase {
    private:
        /**
         * @brief
         * @todo To be implemented
         */
        void initialize() override;

        /***********************************************************************
         * ROS
         */

        /**
         * @brief Trivial node handler
         */
        ros::NodeHandlePtr m_pnh;

        /**
         * @brief Trivial node handler
         */
        ros::NodeHandlePtr m_nh;

        /**
         * @brief Control Process command message
         */
        mvp_msgs::ControlProcess m_cmd;

        /**
         * @brief Trivial update waypoint subscriber
         */
        ros::Subscriber m_update_waypoint_sub;

        /**
         * @brief Trivial append waypoint subscriber
         */
        ros::Subscriber m_append_waypoint_sub;

        /**
         * @brief Trivial marker publisher
         */
        ros::Publisher m_full_trajectory_publisher;

        /**
         * @brief Trajectory segment publisher
         */
        ros::Publisher m_trajectory_segment_publisher;

        /**
         * @brief Waypoints to be traversed
         */
        geometry_msgs::PolygonStamped m_waypoints;

        geometry_msgs::PolygonStamped m_transformed_waypoints;

        /**
         * @brief Frame id of the points name
         */
        std::string m_frame_id;
        /**
         * @brief Index of the lines
         */
        int m_line_index;

        /**
         * @brief Acceptance radius in meters
         */
        double m_acceptance_radius;

        /**
         * @brief Lookahead distance in meters
         */
        double m_lookahead_distance;

        /**
         * @brief Overshoot timeout in seconds
         */
        double m_overshoot_timeout;

        /**
         * @brief Surge velocity for the behavior
         */
        double m_surge_velocity;

        /**
         * @brief experimental integral gain
         */
        double m_sigma;
        double m_yint;
        double m_beta_gain;

        /**
         * @brief Overshoot timer
         * This variable will hold the time it passed since the overshoot.
         */
        ros::Time m_overshoot_timer;

        /**
         * @brief Done state
         * Behavior will request a state change to helm with the value this
         * variable holds.
         */
        std::string m_state_done;

        /**
         * @brief Fail state
         * Behavior will request a state change to helm with the value this
         * variable holds.
         */
        std::string m_state_fail;

        /**
         * @brief First point in the active line segment
         */
        geometry_msgs::Point32 m_wpt_first;

        /**
         * @brief Second point in the active line segment
         */
        geometry_msgs::Point32 m_wpt_second;

        /**
         * @brief Transform buffer for TF2
         */
        tf2_ros::Buffer m_transform_buffer;

        /**
         * @brief Transform listener for TF2
         */
        std::shared_ptr<tf2_ros::TransformListener> m_transform_listener;


        /**
         * @brief Parses waypoints from ROS parameter server
         */
        void f_parse_param_waypoints();

        /**
         * @brief Transform waypoints to #target_frame
         *
         * @param target_frame
         * @param in
         * @param out
         */
        void f_transform_waypoints(
            const std::string &target_frame,
            const geometry_msgs::PolygonStamped &in,
            geometry_msgs::PolygonStamped *out
        );

        /**
         * @brief Trivial waypoint callback
         *
         * @param m Message
         * @param append Append if true, replace if false
         */
        void f_waypoint_cb(const geometry_msgs::PolygonStamped::ConstPtr &m,
                           bool append);

        /**
         * @brief Progress to the next line segment
         */
        void f_next_line_segment();

        /**
         * @brief Sends visualization messages to RViZ.
         *
         * @param clear Clears if true, publishes otherwise
         */
        void f_visualize_path(bool clear = false);

        /**
         * @brief Sends visualization messages to RViZ.
         *
         * @param clear Clears if true, publishes otherwise
         */
        void f_visualize_segment(bool clear = false);

        /**
         * @brief Destroy the Path Following object
         */
        ~PathFollowingI() override;

        /**
         * @brief This function is inherited from #BehaviorBase
         */
        void activated() override;

        /**
         * @brief This function is inherited from #BehaviorBase
         */
        void disabled() override {}

        void resume_or_start();

    public:

        /**
         * @brief trivial constructor
         */
        PathFollowingI();

        /**
         * @brief This function is inherited from #BehaviorBase
         * @param msg
         * @return
         */
        bool request_set_point(mvp_msgs::ControlProcess *msg) override;


    };
}