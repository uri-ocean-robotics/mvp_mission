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


#include "bhv_gps_wpt.h"
#include "pluginlib/class_list_macros.h"
#include "robot_localization/FromLL.h"
#include "geometry_msgs/PolygonStamped.h"

using namespace helm;

void GpsWaypoint::initialize() {

    m_pnh.reset(
        new ros::NodeHandle(ros::this_node::getName() + "/" + get_name())
    );

    m_pnh->param<std::string>("state_fail", m_state_fail, "");

    m_pnh->param<std::string>("target_topic", m_target_topic, "");

    m_pnh->param<std::string>("target_frame_id", m_target_frame_id, "odom");

    m_pnh->param<std::string>("fromll_service", m_fromll_service, "fromll");

    BehaviorBase::m_dofs = decltype(m_dofs){};

    m_poly_pub = m_pnh->advertise<geometry_msgs::PolygonStamped>(
        m_target_topic, 100);

    f_parse_ll_wpts();

}

GpsWaypoint::GpsWaypoint() {
    std::cout << "A message from the template behavior" << std::endl;
}

void GpsWaypoint::activated() {

    /**
     * Wait for the "/fromLL" service from navsat_transform_node in
     * robot_localization package. If the node is not there, or is not properly
     * set up, "/fromLL" service will not be available. If so, change state
     * appropriately.
     */

    std::cout << "The behavior (" << get_name() << ") is calculating GPS transforms" << std::endl;
    if(!ros::service::exists(m_fromll_service, false)) {
        change_state(m_state_fail);
        std::cout << "The behavior (" << get_name() << ") can't call the service: " << m_fromll_service << std::endl;
    }

    geometry_msgs::PolygonStamped poly;

    for(const auto& ll : m_latlong_points) {
        // Prepare the request
        robot_localization::FromLL ser;
        ser.request.ll_point.latitude = ll.latitude;
        ser.request.ll_point.longitude = ll.longitude;
        ser.request.ll_point.altitude = 0;

        // call the service
        // ROS_INFO("%s\n", m_fromll_service.c_str());
        if(!ros::service::call(m_fromll_service, ser)) {
            std::cout << "The behavior (" << get_name() << ") failed to compute GPS transforms" << std::endl;

            // change the state if failed
            change_state(m_state_fail);
            return;
        }

        geometry_msgs::Point32 pt;

        pt.x = ser.response.map_point.x;
        pt.y = ser.response.map_point.y;

        poly.polygon.points.push_back(pt);

    }

    poly.header.frame_id = m_target_frame_id;
    ROS_INFO("%s",m_target_topic.c_str());

    m_poly_pub.publish(poly);
    std::cout << "The behavior (" << get_name() << ") completed GPS transforms and update " << m_target_topic << std::endl;

}

void GpsWaypoint::f_parse_ll_wpts() {

    XmlRpc::XmlRpcValue l;
    if(!m_pnh->getParam("waypoints", l)) {
        return;
    }

    if(l.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("waypoints are not in type array format.");
        return;
    }

    for(uint32_t i = 0 ; i < l.size() ; i++) {
        for(const auto& key : {"lat", "long"}) {
            if(l[i][key].getType() != XmlRpc::XmlRpcValue::TypeInvalid) {
                break;
            }
        }
    }

    for(uint32_t i = 0; i < l.size() ; i++) {
        std::map<std::string, double> mp;
        for(const auto& key : {"lat", "long"}) {
            if (l[i][key].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                mp[key] = static_cast<double>(l[i][key]);
            } else if (l[i][key].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                mp[key] = static_cast<int>(l[i][key]);
            }
        }

        ll_t ll;
        ll.latitude = static_cast<double>(mp["lat"]);
        ll.longitude = static_cast<double>(mp["long"]);
        m_latlong_points.push_back(ll);
    }

}

bool GpsWaypoint::request_set_point(mvp_msgs::ControlProcess *set_point) {
    


    return false;
}

/**
 * @brief Behavior must export the class to the Plugin library.
 */
PLUGINLIB_EXPORT_CLASS(helm::GpsWaypoint, helm::BehaviorBase)
