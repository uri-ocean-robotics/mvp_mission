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


#include "bhv_path_following/bhv_path_following.h"

using namespace helm;

void PathFollowing::f_visualize_path(bool clear) {

    visualization_msgs::msg::Marker marker;
    marker.header = m_transformed_waypoints.header;
    marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    if(clear) {
        marker.action = visualization_msgs::msg::Marker::DELETEALL;
    } else {
        marker.action = visualization_msgs::msg::Marker::MODIFY;
        for (const auto &i: m_transformed_waypoints.polygon.points) {
            geometry_msgs::msg::Point p;
            p.x = i.x;
            p.y = i.y;
            p.z = i.z;
            marker.points.emplace_back(p);
        }
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
    }
    rclcpp::Duration duration(0, 0);  // 2 seconds
    marker.lifetime.sec = duration.seconds();         // Set seconds
    marker.lifetime.nanosec = duration.nanoseconds() % (1000 * 1000 * 1000);  // Set nanoseconds

    m_full_trajectory_publisher->publish(marker);

}

void PathFollowing::f_visualize_segment(bool clear) {
    visualization_msgs::msg::Marker marker;
    marker.header = m_transformed_waypoints.header;
    marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

    if(clear) {
        marker.action = visualization_msgs::msg::Marker::DELETEALL;
    } else {
        marker.action = visualization_msgs::msg::Marker::MODIFY;
        for(const auto& i : {m_wpt_first, m_wpt_second}) {
            geometry_msgs::msg::Point p;
            p.x = i.x;
            p.y = i.y;
            p.z = i.z;
            marker.points.emplace_back(p);
        }

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

    }
    rclcpp::Duration duration(0, 0);  // 2 seconds
    marker.lifetime.sec = duration.seconds();         // Set seconds
    marker.lifetime.nanosec = duration.nanoseconds() % (1000 * 1000 * 1000);  // Set nanoseconds

    m_trajectory_segment_publisher->publish(marker);

}