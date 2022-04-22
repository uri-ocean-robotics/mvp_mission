#include "path_following.h"
#include "pluginlib/class_list_macros.h"
#include "memory"
#include "vector"
#include "thread"
#include "functional"
#include "cmath"


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
        "world_frame",
        m_world_frame,
        "world");

    m_pnh->param<double>("acceptance_radius", m_acceptance_radius, 1.0);

    m_pnh->param<double>("lookahead_distance", m_lookahead_distance, 2.0);

    m_pnh->param<double>("overshoot_timeout", m_overshoot_timeout, 30);

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

    m_trajectory_segment_publisher = m_pnh->advertise<visualization_msgs::Marker>(
        "segment", 0);


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

    geometry_msgs::PolygonStamped a;
    f_transform_waypoints(*m, &a);

    if(append) {

        // append
        for(const auto& i : a.polygon.points) {
            m_waypoints.polygon.points.emplace_back(i);
        }

    } else {

        // replace
        m_waypoints = a;

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

    m_waypoints.header.frame_id = m_world_frame;

}

void
PathFollowing::f_transform_waypoints(const geometry_msgs::PolygonStamped &in,
                                     geometry_msgs::PolygonStamped *out) {

    f_transform_waypoints(m_world_frame ,in, out);

}

void
PathFollowing::f_transform_waypoints(
    const std::string &target_frame,
    const geometry_msgs::PolygonStamped &in,
    geometry_msgs::PolygonStamped *out)
{

    geometry_msgs::PolygonStamped tm;

    tm.header.stamp = ros::Time::now();
    tm.header.frame_id = m_world_frame;

    try {
        for (const auto &pt: in.polygon.points) {

            auto w_pt = m_transform_buffer.lookupTransform(
                target_frame,
                in.header.frame_id,
                ros::Time::now(),
                ros::Duration(1.0)
            );

            geometry_msgs::Point32 p;
            p.x = static_cast<float>(w_pt.transform.translation.x);
            p.y = static_cast<float>(w_pt.transform.translation.y);
            p.z = static_cast<float>(w_pt.transform.translation.z);

            tm.polygon.points.emplace_back(p);
        }

        *out = tm;
    } catch(const tf2::TransformException& e) {
        ROS_ERROR_STREAM(e.what()) ;
    }
}

void PathFollowing::f_next_line_segment() {

    auto length = m_waypoints.polygon.points.size();

    m_wpt_first = m_waypoints.polygon.points[m_line_index % length];
    m_wpt_second = m_waypoints.polygon.points[(m_line_index + 1) % length];

    m_line_index++;

    if(m_line_index == length) {
        f_change_state(m_state_done);
        m_line_index = 0;
    }

}

void PathFollowing::activated() {

    geometry_msgs::Point32 p;

    p.x = static_cast<float>(m_process_values.position.x);

    p.y = static_cast<float>(m_process_values.position.y);

    m_wpt_first = p;
    m_wpt_second = m_waypoints.polygon.points[
        m_line_index % m_waypoints.polygon.points.size()
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

    geometry_msgs::PolygonStamped poly;
    // todo: i don't like the placement of this

    f_transform_waypoints(
        m_process_values.header.frame_id,
        m_waypoints,
        &poly
    );

    std::cout << poly << std::endl;

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

    // Calculate side slip angle
    double beta =
        atan2(m_process_values.velocity.y, m_process_values.velocity.x);

    // set an arbitrary velocity
    m_cmd.velocity.x = 0.7;

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