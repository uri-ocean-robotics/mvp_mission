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


#include "waypoint_tracking.h"
#include "pluginlib/class_list_macros.h"
#include "geometry_msgs/PointStamped.h"

using namespace helm;

WaypointTracking::WaypointTracking() : BehaviorBase() {

    m_wpt_index = 0;

    std::cout << "A message from the waypoint tracking" << std::endl;

}

WaypointTracking::~WaypointTracking() {

    m_update_waypoint_sub.shutdown();

    m_append_waypoint_sub.shutdown();

}

void WaypointTracking::initialize() {

    m_pnh.reset(
        new ros::NodeHandle(ros::this_node::getName() + "/" + get_name())
    );

    m_nh.reset(new ros::NodeHandle());

    BehaviorBase::m_dofs = decltype(m_dofs){
        mvp_msgs::ControlMode::DOF_SURGE,
        mvp_msgs::ControlMode::DOF_YAW
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

    // Meter/Seconds
    m_pnh->param<double>("surge_velocity", m_surge_velocity, 0.5);

    // String: A state to be requested after a successful execution
    m_pnh->param<std::string>("state_done", m_state_done, "");

    f_parse_param_waypoints();

    m_waypoint_viz_pub = m_pnh->advertise<visualization_msgs::Marker>(
        "waypoints", 0);

    m_update_waypoint_sub = m_pnh->subscribe<geometry_msgs::PolygonStamped>(
        update_topic_name,
        10,
        std::bind(
            &WaypointTracking::f_waypoint_cb,
            this,
            std::placeholders::_1,
            false
        )
    );

    m_append_waypoint_sub = m_pnh->subscribe<geometry_msgs::PolygonStamped>(
        append_topic_name,
        10,
        std::bind(
            &WaypointTracking::f_waypoint_cb,
            this,
            std::placeholders::_1,
            true
        )
    );

    m_transform_listener.reset(new
        tf2_ros::TransformListener(m_transform_buffer)
    );

}

void WaypointTracking::f_waypoint_cb(
        const geometry_msgs::PolygonStamped::ConstPtr &m, bool append)
{
    if(m->header.frame_id.empty()) {
        ROS_WARN_STREAM("no frame id provided for the waypoints!");
        return;
    }

    if(append) {
        for(const auto& i : m->polygon.points) {
            m_waypoints.polygon.points.emplace_back(i);
        }
    } else { /* replace */
        m_waypoints = *m;
        m_wpt_index = 0;
    }
}

void WaypointTracking::f_parse_param_waypoints() {
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
WaypointTracking::f_transform_waypoints(
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
            ps.header.frame_id = in.header.frame_id;
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

void WaypointTracking::activated() {

    std::cout << "path following (" << get_name() << ") activated!" << std::endl;

    if(!m_waypoints.polygon.points.empty()) {
        resume_or_start();
    }

}

void WaypointTracking::resume_or_start() {
    // Transform all the points into controller's frame
    geometry_msgs::PolygonStamped poly;
    f_transform_waypoints(
        m_process_values.header.frame_id,
        m_waypoints,
        &m_transformed_waypoints
    );

}

bool WaypointTracking::request_set_point(mvp_msgs::ControlProcess *set_point) {

    if(m_transformed_waypoints.polygon.points.empty()) {
        return false;
    }

    f_visualize_waypoints();

    auto wpt = m_transformed_waypoints.polygon.points[m_wpt_index];

    auto dist_x = wpt.x - m_process_values.position.x;
    auto dist_y = wpt.y - m_process_values.position.y;

    auto dist = sqrt(dist_y * dist_y + dist_x * dist_x);

    if(dist < m_acceptance_radius) {
        m_wpt_index++;

        if(m_transformed_waypoints.polygon.points.size() == m_wpt_index) {
            change_state(m_state_done);
            m_wpt_index = 0;
        }

        // skip for this loop
        return false;
    }

    set_point->orientation.z  = atan2(dist_y, dist_x);
    set_point->velocity.x = m_surge_velocity;

    /*
     * Use the result from the behavior
     */
    return true;
}

PLUGINLIB_EXPORT_CLASS(helm::WaypointTracking, helm::BehaviorBase)
