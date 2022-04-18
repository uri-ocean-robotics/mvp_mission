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

void PathFollowing::f_2point_to_line(double y2, double x2, double y1, double x1,
                                     double *a, double *b, double *c)
{
    *a = y1 - y2;
    *b = x2 - x1;
    *c = (x1 - x2) * y1 + (y2 - y1) * x1;
}

void
PathFollowing::f_shortest_dist_to_line(double a, double b, double c,
                                       double x0, double y0,
                                       double *dist)
{
    *dist = std::fabs(a * x0 + b * y0 + c) / std::sqrt(a * a + b * b);
}

void
PathFollowing::f_closest_point_to_line(double a, double b, double c,
                                       double x0, double y0,
                                       double *x, double *y)
{
    *x = (b * (b * x0 - a * y0) - a * c) / (a * a + b * b);
    *y = (a * (-b* x0 + a * y0) - b * c) / (a * a + b * b);
}

int
PathFollowing::f_direction_of_point(double ax, double ay,
                                    double bx, double by,
                                    double x, double y)
{
    return std::signbit((bx - ax) * (y - ay) - (by - ay) * (x - ax)) ? 1 : -1;
}

void
PathFollowing::f_proceed_to_next_line() {

}

bool PathFollowing::request_set_point(mvp_control::ControlProcess *set_point) {

    if(m_waypoints.polygon.points.size() < 2) {
        ROS_ERROR_STREAM_THROTTLE(30,
            "behavior (" << m_name << "): there has to be more "
            "than 2 waypoints");
        return false;
    }

    /*
     * Decide the action needs to be taken
     */

    geometry_msgs::PolygonStamped poly;
    // todo: i don't like the placement of this

    f_transform_waypoints(
        m_process_values.header.frame_id,
        m_waypoints,
        &poly
    );

    auto fpt = m_waypoints.polygon.points.at(m_line_index);
    auto spt = m_waypoints.polygon.points.at(m_line_index + 1);

    double a,b,c;
    f_2point_to_line(spt.y, spt.x, fpt.y, fpt.x,
                     &a, &b, &c);

    std::cout << "a: " << a << " b: " << b << " c: " << c << std::endl;

    double dist;
    f_shortest_dist_to_line(a, b, c,
                            m_process_values.position.x,
                            m_process_values.position.y,
                            &dist);

    std::cout << "dist: " << dist << std::endl;

    dist *=
        f_direction_of_point(
            spt.x, spt.y, fpt.x, fpt.y,
            m_process_values.position.x,
            m_process_values.position.y);

    std::cout << "dist(2): " << dist << std::endl;

    double dh = 2; // meters

    double yp = - std::atan2(a, b);

    double yaw_desired = yp + atan2(- dist, dh);

    m_cmd.orientation.z = yaw_desired;
    std::cout << "yaw_desired: " << yaw_desired << std::endl;

    m_cmd.velocity.x = 0.2;

    /*
     * Command it to the helm
     */
    *set_point = m_cmd;

    return true;
}

PLUGINLIB_EXPORT_CLASS(helm::PathFollowing, helm::BehaviorBase)