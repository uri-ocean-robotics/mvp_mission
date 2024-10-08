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

    Author: Lin Zhao
    Email: linzhao@uri.edu
    Year: 2022

    Copyright (C) 2022 Smart Ocean Systems Laboratory
*/


#pragma once

#include "atomic"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "behavior_interface/behavior_base.h"
#include "mvp_msgs/msg/control_process.hpp"
#include "mvp_msgs/srv/get_waypoints.hpp"
#include "mvp_msgs/srv/load_waypoint.hpp"
#include "mvp_msgs/srv/send_waypoints.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "robot_localization/srv/to_ll.hpp"
#include "robot_localization/srv/from_ll.hpp"
#include "yaml-cpp/yaml.h"

namespace helm {

using namespace std::chrono_literals;  //NOLINT

class PathFollowing : public BehaviorBase {
private:
    /**
        * @brief Destroy the Teleoperation object
        */
    ~PathFollowing() override;

    /**
        * @brief Initialize function
        *
        * @details This function initializes the behavior. It is called by the
        * helm. It is a pure virtual function and it must be implemented in the
        * behavior. If left unimplemented, code will not compile. In this
        * function, user is responsible for proper initalization of the
        * behavior. This function must be unblocking, otherwise, every other
        * behavior may wait this function to return. In this function user
        * should create #ros::NodeHandle, and define controlled degrees of
        * freedom. Below is a trivial implementation of this function
        *
        */
    void initialize(const rclcpp::Node::WeakPtr &parent) override;

    /**
        * @brief This function is inherited from #BehaviorBase
        */
    void activated() override;

    /**
        * @brief This function is inherited from #BehaviorBase
        */
    void disabled() override;


    /**
     * @brief ros related 
     */
    rclcpp::Node::WeakPtr m_node;

    rclcpp::Logger m_logger{rclcpp::get_logger("mvp2_mission_bhv_path_following")};
    
    
    ////////////////////////Subscribers////////////////////////
    /**
        * @brief Trivial update waypoint subscriber
    */
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr m_update_waypoint_sub;

    /**
        * @brief Trivial append waypoint subscriber
    */
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr m_append_waypoint_sub;

    /**
        * @brief Trivial update surge subscriber
    */
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_update_surge_sub;


    ////////////////////////Publisher////////////////////////
    /**
    * @brief Trivial marker publisher
    */
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_full_trajectory_publisher;

    /**
    * @brief Trajectory segment publisher
    */
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_trajectory_segment_publisher;

    ////////////////////////Service////////////////////////
    //! @brief Get next waypoint
    rclcpp::Service<mvp_msgs::srv::GetWaypoints>::SharedPtr get_next_waypoints_server;

    //! @brief load waypoints
    rclcpp::Service<mvp_msgs::srv::LoadWaypoint>::SharedPtr load_waypoint_server;

    //! @brief update waypoints
    rclcpp::Service<mvp_msgs::srv::SendWaypoints>::SharedPtr update_waypoints_server;

    rclcpp::Client<robot_localization::srv::ToLL>::SharedPtr m_toll_client;

    rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr m_fromll_client;
    ////////variables//////////
    std::string m_frame_id;

    std::string  m_enu_frame; 

    std::string m_fromll_service;

    std::string m_toll_service;

    std::string waypoint_path;

    std::string m_state_done;

    std::string m_state_fail;

    std::string m_ns;

    double m_acceptance_radius;

    double m_lookahead_distance;

    double m_overshoot_timeout;

    double  m_overshoot_timer;

    double m_surge_velocity;

    double m_pitch;

    double m_sigma;

    double m_beta_gain;

    double m_lookahead_max, m_lookahead_min, m_lookahead_gamma;
    
    bool m_lookahead_adaptive;

    double m_yint;

    int m_line_index;

    geometry_msgs::msg::Point32 m_wpt_first;

    geometry_msgs::msg::Point32 m_wpt_second;

    geometry_msgs::msg::PolygonStamped m_waypoints;

    geometry_msgs::msg::PolygonStamped m_transformed_waypoints;

    // mvp_msgs::msg::ControlProcess m_cmd;
    // ///////TF//////////////////////////
    std::unique_ptr<tf2_ros::Buffer> m_transform_buffer;
    std::unique_ptr<tf2_ros::TransformListener> m_transform_listener;



    // /////////callbacks
    void f_waypoint_cb(const geometry_msgs::msg::PolygonStamped::SharedPtr m, bool append);

    void f_surge_cb(const std_msgs::msg::Float64::SharedPtr m);


    bool f_cb_srv_get_next_waypoints(
            const std::shared_ptr<mvp_msgs::srv::GetWaypoints::Request> request,
            const std::shared_ptr<mvp_msgs::srv::GetWaypoints::Response> response);

    bool f_cb_srv_load_waypoint(
            const std::shared_ptr<mvp_msgs::srv::LoadWaypoint::Request> request,
            const std::shared_ptr<mvp_msgs::srv::LoadWaypoint::Response> response);

    bool f_cb_srv_update_waypoints(
            const std::shared_ptr<mvp_msgs::srv::SendWaypoints::Request> request,
            const std::shared_ptr<mvp_msgs::srv::SendWaypoints::Response> response);
    

    void f_transform_waypoints(
        const std::string &target_frame,
        const geometry_msgs::msg::PolygonStamped &in,
        geometry_msgs::msg::PolygonStamped *out);

    void resume_or_start();

    void f_next_line_segment();


    //misc
    void f_visualize_path(bool clear = false);

    void f_visualize_segment(bool clear = false);

public:

    /**
        * @brief Trivial constructor
        */
    PathFollowing();

    /**
        * @brief Request set point from the behavior. It is consumed by helm.
        *
        * @param msg Result value of the behavior. This value is written by the
        *            Behavior. Helm uses this variable to generate set_point
        *            for the controller.
        * @return true if you want helm to use the result.
        * @return false if you don't want helm to use the result.
        */
    bool request_set_point(mvp_msgs::msg::ControlProcess *set_point) override;

};

}
