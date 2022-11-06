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


#include "path_following.h"
#include "pluginlib/class_list_macros.h"
#include "memory"
#include "vector"
#include "thread"
#include "functional"
#include "cmath"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace helm;

PathFollowing::PathFollowing() : BehaviorBase() {

    m_line_index = 0;

}

PathFollowing::~PathFollowing() {

    m_update_waypoint_sub.shutdown();

    m_append_waypoint_sub.shutdown();

}

void PathFollowing::initialize() {

    m_pnh.reset(
        new ros::NodeHandle(ros::this_node::getName() + "/" + get_name())
    );

    m_nh.reset(new ros::NodeHandle());

    BehaviorBase::m_dofs = decltype(m_dofs){
        mvp_msgs::ControlMode::DOF_SURGE,
        mvp_msgs::ControlMode::DOF_YAW,
    };

    std::string update_topic_name;

    std::string append_topic_name;

    m_pnh->param<std::string>(
        "update_topic", update_topic_name, "update_waypoints");

    m_pnh->param<std::string>(
        "append_topic", append_topic_name, "append_waypoints");

    m_pnh->param<std::string>("frame_id", m_frame_id, "frame_id");


    // Meters
    m_pnh->param<double>("acceptance_radius", m_acceptance_radius, 1.0);

    // Meters
    m_pnh->param<double>("lookahead_distance", m_lookahead_distance, 2.0);

    // Seconds
    m_pnh->param<double>("overshoot_timeout", m_overshoot_timeout, 30);

    // Meter/Seconds
    m_pnh->param<double>("surge_velocity", m_surge_velocity, 0.5);

    // Arbitrary constant
    m_pnh->param<double>("beta_gain", m_beta_gain, 1.0);

    // String: A state to be requested after a successful execution
    m_pnh->param<std::string>("state_done", m_state_done, "");

    // String: A state to be requested after a failed execution
    m_pnh->param<std::string>("state_fail", m_state_fail, "");

    f_parse_param_waypoints();

    m_update_waypoint_sub = m_pnh->subscribe<geometry_msgs::PolygonStamped>(
        update_topic_name,
        10,
        std::bind(
            &PathFollowing::f_waypoint_cb,
            this,
            std::placeholders::_1,
            false
        )
    );

    m_append_waypoint_sub = m_pnh->subscribe<geometry_msgs::PolygonStamped>(
        append_topic_name,
        10,
        std::bind(
            &PathFollowing::f_waypoint_cb,
            this,
            std::placeholders::_1,
            true
        )
    );

    m_full_trajectory_publisher = m_pnh->advertise<visualization_msgs::Marker>(
        "path", 0);

    m_trajectory_segment_publisher =
        m_pnh->advertise<visualization_msgs::Marker>("segment", 0);


    m_transform_listener.reset(new
        tf2_ros::TransformListener(m_transform_buffer)
    );

}

void PathFollowing::f_waypoint_cb(
        const geometry_msgs::PolygonStamped::ConstPtr &m, bool append)
{
    if(m->header.frame_id.empty()) {
        // no decision can be made
        ROS_WARN_STREAM("no frame id provided for the waypoints!");
        return;
    }

    if(append) {

        // append
        for(const auto& i : m->polygon.points) {
            m_waypoints.polygon.points.emplace_back(i);
        }

    } else {

        // replace
        m_waypoints = *m;

        m_line_index = 0;

        resume_or_start();
    }
}

void PathFollowing::f_parse_param_waypoints() {
    XmlRpc::XmlRpcValue l;
    if(!m_pnh->getParam("waypoints", l)) {
        return;
    }

    if(l.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("waypoints are not in type array format.");
        return;
    }

    for(uint32_t i = 0 ; i < l.size() ; i++) {
        for(const auto& key : {"x", "y"}) {
            if(l[i][key].getType() != XmlRpc::XmlRpcValue::TypeInvalid) {
                break;
            }
        }
    }


    for(uint32_t i = 0; i < l.size() ; i++) {
        std::map<std::string, double> mp;
        for(const auto& key : {"x", "y"}) {
            if (l[i][key].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                mp[key] = static_cast<double>(l[i][key]);
            } else if (l[i][key].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                mp[key] = static_cast<int>(l[i][key]);
            }
        }
        geometry_msgs::Point32 gp;
        gp.x = static_cast<float>(mp["x"]);
        gp.y = static_cast<float>(mp["y"]);

        m_waypoints.polygon.points.emplace_back(gp);
    }

    m_waypoints.header.frame_id = m_frame_id;

}

void
PathFollowing::f_transform_waypoints(
    const std::string &target_frame,
    const geometry_msgs::PolygonStamped &in,
    geometry_msgs::PolygonStamped *out)
{

    geometry_msgs::PolygonStamped tm;

    tm.header.stamp = ros::Time::now();
    tm.header.frame_id = target_frame;

    try {

        for (const auto &pt: in.polygon.points) {
            geometry_msgs::PointStamped ps;

            if(in.header.frame_id[0] == '/') {
                ps.header.frame_id =
                    in.header.frame_id.substr(1, in.header.frame_id.length());
            } else {
                ps.header.frame_id = in.header.frame_id;
            }

            ps.point.x = pt.x;
            ps.point.y = pt.y;

            auto t = m_transform_buffer.transform(
                ps, target_frame, ros::Duration(1.0));

            geometry_msgs::Point32 p;
            p.x = static_cast<float>(t.point.x);
            p.y = static_cast<float>(t.point.y);

            tm.polygon.points.emplace_back(p);
        }

        *out = tm;
    } catch(const tf2::TransformException& e) {
        ROS_ERROR_STREAM(e.what()) ;
    }
}

void PathFollowing::f_next_line_segment() {
    auto length = m_transformed_waypoints.polygon.points.size();

    m_wpt_first =
        m_transformed_waypoints.polygon.points[m_line_index % length];
    m_wpt_second =
        m_transformed_waypoints.polygon.points[(m_line_index + 1) % length];

    m_line_index++;

    if(m_line_index == length) {
        change_state(m_state_done);
        m_line_index = 0;
    }

}

void PathFollowing::activated() {

    std::cout << "path following (" << get_name() << ") activated!" << std::endl;

    if(!m_waypoints.polygon.points.empty()) {
        resume_or_start();
    }

}

void PathFollowing::resume_or_start() {
    // Transform all the points into controller's frame
    geometry_msgs::PolygonStamped poly;
    f_transform_waypoints(
        m_process_values.header.frame_id,
        m_waypoints,
        &m_transformed_waypoints
    );

    // Push robots position as the first point
    geometry_msgs::Point32 p;
    p.x = static_cast<float>(m_process_values.position.x);
    p.y = static_cast<float>(m_process_values.position.y);
    m_wpt_first = p;


    // Select second waypoint to be the next point in the way point list
    m_wpt_second = m_transformed_waypoints.polygon.points[
        m_line_index % m_transformed_waypoints.polygon.points.size()
    ];


}

bool PathFollowing::request_set_point(mvp_msgs::ControlProcess *set_point) {


    // Clear the path segment and the path if the behavior is not active
    if(!m_activated) {
        f_visualize_segment(true);
        f_visualize_path(true);
        return false;
    }

    // Warn the user if there is less than 2 points in the waypoint list
    if(m_waypoints.polygon.points.size() < 2) {
        return false;
    }

    // Visualize the path and the segment
    f_visualize_path();
    f_visualize_segment();

    // Acquire vehicle position from the controller process
    double x = BehaviorBase::m_process_values.position.x;
    double y = BehaviorBase::m_process_values.position.y;

    // Compute horizontal path-tangential angle
    double gamma_p = std::atan2(m_wpt_second.y - m_wpt_first.y,
                                m_wpt_second.x - m_wpt_first.x);

    // Compute along track errors
    double Xe = (x - m_wpt_first.x) * cos(gamma_p) +
        (y - m_wpt_first.y) * sin(gamma_p);
    double Ye = -(x - m_wpt_first.x) * sin(gamma_p) +
        (y - m_wpt_first.y) * cos(gamma_p);
    double Xke = (x - m_wpt_second.x) * cos(gamma_p) +
        (y - m_wpt_second.y) * sin(gamma_p);

    // Check of overshoot
    double lookahead = m_lookahead_distance;
    if(Xke > 0 ) {
        // overshoot detected
        ROS_WARN_THROTTLE(5, "Overshoot detected!");

        // Look back
        lookahead = -lookahead;

        // we are at the opposite side now
        gamma_p = gamma_p + M_PI;

        // record the time
        auto t = ros::Time::now();

        // if overshoot timer is not set, set it now.
        if((t - m_overshoot_timer).toSec() == t.toSec()) {
            m_overshoot_timer = ros::Time::now();
        }

        // check if overshoot timer passed the timeout.
        if((t - m_overshoot_timer).toSec() > m_overshoot_timeout) {
            ROS_ERROR_THROTTLE(10, "Overshoot abort!");
            change_state(m_state_fail);
            return false;
        }

    }


    // Calculate side slip angle
    double beta = 0;
    if(BehaviorBase::m_process_values.velocity.x != 0) {
        beta =
            atan2(
                BehaviorBase::m_process_values.velocity.y,
                BehaviorBase::m_process_values.velocity.x);
    }

    beta *= m_beta_gain;

    // set an arbitrary velocity
    m_cmd.velocity.x = m_surge_velocity;

    // set the heading for line of sight
    m_cmd.orientation.z = gamma_p + atan( - Ye / lookahead) - beta;

    // check the acceptance radius
    auto dist = std::sqrt(Xke * Xke + Ye*Ye);
    if(dist < m_acceptance_radius ) {
        f_next_line_segment();
        m_overshoot_timer.fromSec(0);
    }

    /*
     * Command it to the helm
     */
    *set_point = m_cmd;

    /*
     * Use the result from the behavior
     */
    return true;
}

PLUGINLIB_EXPORT_CLASS(helm::PathFollowing, helm::BehaviorBase)
