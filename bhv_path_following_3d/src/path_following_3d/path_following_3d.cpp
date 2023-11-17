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


#include "path_following_3d.h"
#include "pluginlib/class_list_macros.h"
#include "memory"
#include "vector"
#include "thread"
#include "functional"
#include "cmath"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "robot_localization/ToLL.h"

using namespace helm;

PathFollowing3D::PathFollowing3D() : BehaviorBase() {

    m_line_index = 0;

}

PathFollowing3D::~PathFollowing3D() {

    m_update_waypoint_sub.shutdown();

    m_append_waypoint_sub.shutdown();

}

void PathFollowing3D::initialize() {

    m_pnh.reset(
        new ros::NodeHandle(ros::this_node::getName() + "/" + get_name())
    );

    m_nh.reset(new ros::NodeHandle());

    BehaviorBase::m_dofs = decltype(m_dofs){
        mvp_msgs::ControlMode::DOF_SURGE,
        mvp_msgs::ControlMode::DOF_YAW,
        mvp_msgs::ControlMode::DOF_Z
    };

    std::string update_topic_name;

    std::string append_topic_name;

    m_pnh->param<std::string>(
        "update_topic", update_topic_name, "update_waypoints");

    m_pnh->param<std::string>(
        "append_topic", append_topic_name, "append_waypoints");

    m_pnh->param<std::string>("frame_id", m_frame_id, "world");

    m_pnh->param<std::string>("enu_frame", m_enu_frame, "world");
    // Meters
    m_pnh->param<double>("acceptance_radius", m_acceptance_radius, 1.0);

    // Meters
    m_pnh->param<double>("lookahead_distance", m_lookahead_distance, 2.0);

    // Seconds
    m_pnh->param<double>("overshoot_timeout", m_overshoot_timeout, 30);

    // Meter/Seconds
    m_pnh->param<double>("surge_velocity", m_surge_velocity, 0.5);

    //robot localization to ll service name
    m_pnh->param<std::string>("toll_service", m_toll_service, "toLL");

    // Arbitrary constant
    m_pnh->param<double>("sigma", m_sigma, 1.0);
    
    m_pnh->param<double>("beta_gain", m_beta_gain, 0.0);

    //file path to the waypoint folder
    m_pnh->param<std::string>("waypoint_path", waypoint_path, "~/go_to_list");

    // String: A state to be requested after a successful execution
    m_pnh->param<std::string>("state_done", m_state_done, "");

    // String: A state to be requested after a failed execution
    m_pnh->param<std::string>("state_fail", m_state_fail, "");
    
    f_parse_param_waypoints();

    m_update_waypoint_sub = m_pnh->subscribe<geometry_msgs::PolygonStamped>(
        update_topic_name,
        10,
        std::bind(
            &PathFollowing3D::f_waypoint_cb,
            this,
            std::placeholders::_1,
            false
        )
    );

    m_append_waypoint_sub = m_pnh->subscribe<geometry_msgs::PolygonStamped>(
        append_topic_name,
        10,
        std::bind(
            &PathFollowing3D::f_waypoint_cb,
            this,
            std::placeholders::_1,
            true
        )
    );

    m_full_trajectory_publisher = m_pnh->advertise<visualization_msgs::Marker>(
        "path", 0);

    m_trajectory_segment_publisher =
        m_pnh->advertise<visualization_msgs::Marker>("segment", 0);


    /**
     * Initialize services
     */
    get_next_waypoint_server = m_pnh->advertiseService
        <mvp_msgs::GetWaypoint::Request,
        mvp_msgs::GetWaypoint::Response>
    (
        "get_next_waypoint",
        std::bind(
            &PathFollowing3D::f_cb_srv_get_next_waypoint,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    // Load waypoints service
    load_waypoint_server = m_pnh->advertiseService
        <mvp_msgs::LoadWaypoint::Request,
        mvp_msgs::LoadWaypoint::Response>
    (
        "load_waypoints",
        std::bind(
            &PathFollowing3D::f_cb_srv_load_waypoint,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    m_transform_listener.reset(new
        tf2_ros::TransformListener(m_transform_buffer)
    );

}


//Get next waypoint callback

bool PathFollowing3D::f_cb_srv_get_next_waypoint(
        mvp_msgs::GetWaypoint::Request &req, mvp_msgs::GetWaypoint::Response &resp) {
            
    //m_wpt_second is in the mvp_control's frame.
    resp.wpt.header.stamp = ros::Time::now();
    resp.wpt.header.frame_id = m_process_values.header.frame_id;
    
    resp.wpt.wpt.x = m_wpt_second.x;
    resp.wpt.wpt.y = m_wpt_second.y;
    resp.wpt.wpt.z = m_wpt_second.z;

    // printf("wpt=%f,%f,%f\r\n",m_wpt_second.x, m_wpt_second.y, m_wpt_second.z);
    //we need to call the service to convert to lat and lon
    //call the service
    std::cout << "Converting next wpt into geopoint" << std::endl;

    //convert the resp.wpt from the waypoint frame id into the ENU (world)
    Eigen::Vector3d p_world;
    try {
    auto tf_wpt_world = m_transform_buffer.lookupTransform(
        m_enu_frame,
        resp.wpt.header.frame_id,
        ros::Time::now(),
        ros::Duration(1.0)
    );
    
    auto tf_eigen = tf2::transformToEigen(tf_wpt_world);

    p_world = tf_eigen.rotation() * 
                                Eigen::Vector3d(resp.wpt.wpt.x, resp.wpt.wpt.y, resp.wpt.wpt.z)
                                + tf_eigen.translation();

    } catch(tf2::TransformException &e) {
        ROS_WARN_STREAM_THROTTLE(10, std::string("Can't transform the p and rpy to the global!") + e.what());
        return false;
    }

    //check he to_LL service
    if(!ros::service::exists(m_toll_service, false)) {
        change_state(m_state_fail);
        std::cout << "The To_LL service is not available from " << m_fromll_service << std::endl;
    }
    //call the service 
    robot_localization::ToLL ser;
    ser.request.map_point.x = p_world.x();
    ser.request.map_point.y = p_world.y();
    ser.request.map_point.z = p_world.z();

    if(!ros::service::call(m_toll_service, ser)) {
        std::cout << "Failed to compute the latitude and longitude for the next waypoint" << std::endl;
        // change the state if failed
        change_state(m_state_fail);
        return false;
    }
    //map response into our service response.
    resp.wpt.ll_wpt.latitude = ser.response.ll_point.latitude;
    resp.wpt.ll_wpt.longitude = ser.response.ll_point.longitude;
    resp.wpt.ll_wpt.altitude = ser.response.ll_point.altitude;
    // printf("wpt=%f,%f,%f\r\n", resp.wpt.x, resp.wpt.y, resp.wpt.z);
    return true;
}

bool PathFollowing3D::f_cb_srv_load_waypoint(mvp_msgs::LoadWaypoint::Request &req, mvp_msgs::LoadWaypoint::Response &resp)
{
    geometry_msgs::PolygonStamped temp_waypoints;
    std::string filename  = waypoint_path + req.file + ".yaml";

    YAML::Node map = YAML::LoadFile(filename);

    bool good_frame = false;
    bool good_waypoint = false;

    if(map["frame_id"])
    {
        std::string ns = ros::this_node::getNamespace();
        temp_waypoints.header.frame_id = ns + "/" + map["frame_id"].as<std::string>();
        good_frame = true;
    }

    if(map["waypoints"])
    {
        for(uint32_t i = 0; i < map["waypoints"].size() ; i++) {
            std::map<std::string, double> mp;
            for(const auto& key : {"x", "y", "z"})
            {
                mp[key] = map["waypoints"][i][key].as<float>();
            }

            geometry_msgs::Point32 gp;
            gp.x = static_cast<float>(mp["x"]);
            gp.y = static_cast<float>(mp["y"]);
            gp.z = static_cast<float>(mp["z"]);

            temp_waypoints.polygon.points.emplace_back(gp);

            good_waypoint = true;

        }
    }

    if(good_frame && good_waypoint)
    {
        m_waypoints = temp_waypoints;
        resp.success=true;
    }
    else
    {
        resp.success=false;
    }

    
    return true;
}




void PathFollowing3D::f_waypoint_cb(
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

void PathFollowing3D::f_parse_param_waypoints() {
    XmlRpc::XmlRpcValue l;
    if(!m_pnh->getParam("waypoints", l)) {
        return;
    }

    if(l.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("waypoints are not in type array format.");
        return;
    }

    for(uint32_t i = 0 ; i < l.size() ; i++) {
        for(const auto& key : {"x", "y", "z"}) {
            if(l[i][key].getType() != XmlRpc::XmlRpcValue::TypeInvalid) {
                break;
            }
        }
    }


    for(uint32_t i = 0; i < l.size() ; i++) {
        std::map<std::string, double> mp;
        for(const auto& key : {"x", "y", "z"}) {
            if (l[i][key].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                mp[key] = static_cast<double>(l[i][key]);
            } else if (l[i][key].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                mp[key] = static_cast<int>(l[i][key]);
            }
        }
        geometry_msgs::Point32 gp;
        gp.x = static_cast<float>(mp["x"]);
        gp.y = static_cast<float>(mp["y"]);
        gp.z = static_cast<float>(mp["z"]);

        m_waypoints.polygon.points.emplace_back(gp);
    }

    m_waypoints.header.frame_id = m_frame_id;

}

void PathFollowing3D::f_transform_waypoints(
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
            ps.point.z = pt.z;

            auto t = m_transform_buffer.transform(
                ps, target_frame, ros::Duration(1.0));

            geometry_msgs::Point32 p;
            p.x = static_cast<float>(t.point.x);
            p.y = static_cast<float>(t.point.y);
            p.z = static_cast<float>(t.point.z);

            tm.polygon.points.emplace_back(p);
        }

        *out = tm;
    } catch(const tf2::TransformException& e) {
        ROS_ERROR_STREAM(e.what()) ;
    }
}

void PathFollowing3D::f_next_line_segment() {
    auto length = m_transformed_waypoints.polygon.points.size();
    //start point
    m_wpt_first =
        m_transformed_waypoints.polygon.points[m_line_index % length];
    //end point
    m_wpt_second =
        m_transformed_waypoints.polygon.points[(m_line_index + 1) % length];
    m_yint = 0;  //reset the integral?
    m_line_index++;

    if(m_line_index == length) {
        change_state(m_state_done);
        m_line_index = 0;
    }

}

void PathFollowing3D::activated() {

    std::cout << "path following (" << get_name() << ") activated!" << std::endl;

    if(!m_waypoints.polygon.points.empty()) {
        resume_or_start();
    }

}

void PathFollowing3D::resume_or_start() {
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
    // p.z = static_cast<float>(m_process_values.position.z);
    m_wpt_first = p;


    // Select second waypoint to be the next point in the way point list
    m_wpt_second = m_transformed_waypoints.polygon.points[
        m_line_index % m_transformed_waypoints.polygon.points.size()
    ];


}

bool PathFollowing3D::request_set_point(mvp_msgs::ControlProcess *set_point) {


    // Clear the path segment and the path if the behavior is not active
    if(!m_activated) {
        f_visualize_segment(true);
        f_visualize_path(true);
        return false;
    }

    // Warn the user if there is less than 2 points in the waypoint list
    if(m_waypoints.polygon.points.size() < 1) {
        return false;
    }

    // Visualize the path and the segment
    f_visualize_path();
    f_visualize_segment();

    // Acquire vehicle position from the controller process
    double x = BehaviorBase::m_process_values.position.x;
    double y = BehaviorBase::m_process_values.position.y;
    double dx1 = x - m_wpt_first.x;
    double dy1 = y - m_wpt_first.y;
    double dx2 = x - m_wpt_second.x;
    double dy2 = y - m_wpt_second.y;

    // Compute horizontal path-tangential angle
    double gamma_p = std::atan2(m_wpt_second.y - m_wpt_first.y,
                                m_wpt_second.x - m_wpt_first.x);


    // Compute along track errors
    double Xe =  dx1 * cos(gamma_p) + dy1 * sin(gamma_p);
    double Ye = -dx1 * sin(gamma_p) + dy1 * cos(gamma_p);

    double Xke = (x - m_wpt_second.x) * cos(gamma_p) +
        (y - m_wpt_second.y) * sin(gamma_p);

    // Check of overshoot
    double lookahead = m_lookahead_distance;
     // Check of overshoot
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
    

    // Calculate integral
    double denumerator;
    denumerator = (Ye + m_sigma*m_yint)*(Ye + m_sigma*m_yint) + lookahead*lookahead;
    m_yint += lookahead * Ye / denumerator;

    
    // Calculate the vehicle's cross-track velocity for sideslip compenstation
    // double beta = 0;
    double u = BehaviorBase::m_process_values.velocity.x;
    double v = BehaviorBase::m_process_values.velocity.y;
    double yaw = BehaviorBase::m_process_values.orientation.z;
    double ye_dot = -u * sin(-yaw+gamma_p) + v * cos(-yaw + gamma_p);


    // set the surge velocity
    m_cmd.velocity.x = m_surge_velocity;

    // set the heading for line of sight
    m_cmd.orientation.z = gamma_p - atan( (Ye + m_sigma*m_yint)/ lookahead   + ye_dot*m_beta_gain);
    double aoa = atan( Ye / lookahead); 
    // ROS_INFO("%lf, %lf, %lf, %lf\n", m_yint, gamma_p*180/3.1415926, ye_dot, m_cmd.orientation.z*180/3.1415926);
    // check the acceptance radius
    
    auto dist = std::sqrt(dx2 * dx2 + dy2*dy2);
    if(dist < m_acceptance_radius ) {
        f_next_line_segment();
        m_overshoot_timer.fromSec(0);
    }

    //Set the direct z set point
    m_cmd.position.z = m_wpt_second.z;

    /*
     * Command it to the helm
     */
    *set_point = m_cmd;

    /*
     * Use the result from the behavior
     */
    return true;
}

PLUGINLIB_EXPORT_CLASS(helm::PathFollowing3D, helm::BehaviorBase)
