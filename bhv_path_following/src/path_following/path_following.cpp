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
        new ros::NodeHandle(ros::this_node::getName() + "/" + m_name)
    );

    m_nh.reset(new ros::NodeHandle());

    m_dofs = decltype(m_dofs){
        ctrl::DOF::SURGE,
        ctrl::DOF::YAW,
    };

    std::string update_topic_name;

    std::string append_topic_name;

    m_pnh->param<std::string>(
        "update_topic",
        update_topic_name,
        "update_waypoints");

    m_pnh->param<std::string>(
        "update_topic",
        append_topic_name,
        "append_waypoints");

    m_pnh->param<std::string>(
        "frame_id",
        m_frame_id,
        "frame_id");

    m_pnh->param<double>("acceptance_radius", m_acceptance_radius, 1.0);

    m_pnh->param<double>("lookahead_distance", m_lookahead_distance, 2.0);

    m_pnh->param<double>("overshoot_timeout", m_overshoot_timeout, 30);

    m_pnh->param<double>("surge_velocity", m_surge_velocity, 0.5); // m/s

    m_pnh->param<double>("beta_gain", m_beta_gain, 1.0);

    m_pnh->param<std::string>("state_done", m_state_done, "");

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

    bool local_frame = false;
    for(uint32_t i = 0 ; i < l.size() ; i++) {
        for(const auto& key : {"x", "y", "z"}) {
            if(!l[i][key].getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
                local_frame = true;
                break;
            }
        }
    }

    bool gps_frame = false;
    for(uint32_t i = 0 ; i < l.size() ; i++) {
        for(const auto& key : {"lat", "long"}) {
            if(!l[i][key].getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
                gps_frame = true;
                break;
            }
        }
    }

    if(local_frame && gps_frame) {
        throw std::runtime_error("mixed waypoint types!");
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
            ps.header.frame_id = in.header.frame_id;
            ps.point.x = pt.x;
            ps.point.y = pt.y;

            auto t = m_transform_buffer.transform(ps, target_frame, ros::Duration(1.0));

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

    m_wpt_first = m_transformed_waypoints.polygon.points[m_line_index % length];
    m_wpt_second = m_transformed_waypoints.polygon.points[(m_line_index + 1) % length];

    m_line_index++;

    if(m_line_index == length) {
        f_change_state(m_state_done);
        m_line_index = 0;
    }

}

void PathFollowing::activated() {

    std::cout << "path following (" << m_name << ") activated!" << std::endl;

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

bool PathFollowing::request_set_point(mvp_control::ControlProcess *set_point) {

    if(!m_activated) {
        f_visualize_segment(true);
        f_visualize_path(true);
        return false;
    }

    if(m_waypoints.polygon.points.size() < 2) {
        ROS_ERROR_STREAM_THROTTLE(30,
            "behavior (" << m_name << "): there has to be more "
            "than 2 waypoints");
        return false;
    }

    /*
     * Decide the action needs to be taken
     */
    f_visualize_path();

    f_visualize_segment();

    std::cout << "POLYGON!\n" << m_transformed_waypoints << std::endl;

    double x = m_process_values.position.x;

    double y = m_process_values.position.y;

    double Yp = std::atan2(
        m_wpt_second.y - m_wpt_first.y,
        m_wpt_second.x - m_wpt_first.x);

    double Xke = (x - m_wpt_second.x) * cos(Yp) + (y - m_wpt_second.y) * sin(Yp);
    double Xe = (x - m_wpt_first.x) * cos(Yp) + (y - m_wpt_first.y) * sin(Yp);
    double Ye = -(x - m_wpt_first.x) * sin(Yp) + (y - m_wpt_first.y) * cos(Yp);

    double lookahead = m_lookahead_distance;
    if(Xke > 0 ) {
        // overshoot detected
        ROS_WARN_THROTTLE(5, "Overshoot detected!");

        // Look back
        lookahead = -lookahead;

        // we are at the opposite side now
        Yp = Yp + M_PI;

        // record the time
        auto t = ros::Time::now();

        // if overshoot timer is not set, set it now.
        if((t - m_overshoot_timer).toSec() == t.toSec()) {
            m_overshoot_timer = ros::Time::now();
        }

        // check if overshoot timer passed the timeout.
        if((t - m_overshoot_timer).toSec() > m_overshoot_timeout) {
            ROS_ERROR_THROTTLE(10, "Overshoot abort!");
            f_change_state(m_state_fail);
            return false;
        }

    }


    double beta = 0;
    // Calculate side slip angle
    if(m_process_values.velocity.x != 0) {
        beta =
            atan(m_process_values.velocity.y / m_process_values.velocity.x);
    }

    beta *= m_beta_gain;

    // set an arbitrary velocity
    m_cmd.velocity.x = m_surge_velocity;

    // set the heading for line of sight
    m_cmd.orientation.z = Yp + atan( - Ye / lookahead) - beta;

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

    return true;
}

PLUGINLIB_EXPORT_CLASS(helm::PathFollowing, helm::BehaviorBase)