#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "bhv_path_following/bhv_path_following.h"
#include "tf2_eigen/tf2_eigen.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;

using namespace helm;

PathFollowing::PathFollowing()
{
    std::cout << "A message from the path_following" << std::endl;

}

void PathFollowing::initialize(const rclcpp::Node::WeakPtr &parent) 
{
     /*************************************************************************/
    /* get the node */

    m_node = parent;
    auto node = m_node.lock();
    m_logger = node->get_logger();

    m_ns = node->get_namespace();

    /*************************************************************************/
    /* Load Parameters for ROS2 */
    std::string update_topic_name;

    std::string append_topic_name;

    std::string update_geopath_topic_name;

    std::string surge_topic_name;

    std::vector<double> m_wpt_x;

    std::vector<double> m_wpt_y;

    std::vector<double> m_wpt_z;


    //topic and frame params
    node->declare_parameter("update_topic", "update_waypoints");
    node->get_parameter("update_topic", update_topic_name);

    node->declare_parameter("append_topic", "append_waypoints");
    node->get_parameter("append_topic", append_topic_name);

    node->declare_parameter("surge_topic", "update_surge");
    node->get_parameter("surge_topic", surge_topic_name);

    node->declare_parameter("frame_id", "world");
    node->get_parameter("frame_id", m_frame_id);

    node->declare_parameter("enu_frame", "world");
    node->get_parameter("enu_frame", m_enu_frame);

    node->declare_parameter("toll_service", "toLL");
    node->get_parameter("toll_service", m_toll_service);

    node->declare_parameter("fromll_service", "fromLL");
    node->get_parameter("fromll_service", m_fromll_service);

    node->declare_parameter("waypoint_path", "~/go_to_list");
    node->get_parameter("waypoint_path", waypoint_path);

    node->declare_parameter("state_done", "");
    node->get_parameter("state_done", m_state_done);

    node->declare_parameter("state_fail", "");
    node->get_parameter("state_fail", m_state_fail);

    //LOS params
    node->declare_parameter("acceptance_radius", 1.0);
    node->get_parameter("acceptance_radius", m_acceptance_radius);

    node->declare_parameter("lookahead_distance", 2.0);
    node->get_parameter("lookahead_distance", m_lookahead_distance);

    node->declare_parameter("overshoot_timeout", 1.0);
    node->get_parameter("overshoot_timeout", m_overshoot_timeout);

    node->declare_parameter("surge_velocity", 1.0);
    node->get_parameter("surge_velocity", m_surge_velocity);

    node->declare_parameter("pitch_angle", 0.0);
    node->get_parameter("pitch_angle", m_pitch);

    node->declare_parameter("sigma", 0.0);
    node->get_parameter("sigma", m_sigma);

    node->declare_parameter("beta_gain", 0.0);
    node->get_parameter("beta_gain", m_beta_gain);
    
    node->declare_parameter("lookahead_adaptive_flag", false);
    node->get_parameter("lookahead_adaptive_flag", m_lookahead_adaptive);

    node->declare_parameter("lookahead_max", 10.0);
    node->get_parameter("lookahead_max", m_lookahead_max);

    node->declare_parameter("lookahead_min", 1.0);
    node->get_parameter("lookahead_min", m_lookahead_min);

    node->declare_parameter("lookahead_gamma", 0.01);
    node->get_parameter("lookahead_gammaq", m_lookahead_gamma);

    node->declare_parameter("waypoint_x", m_wpt_x);
    node->get_parameter("waypoint_x", m_wpt_x);

    node->declare_parameter("waypoint_y", m_wpt_y);
    node->get_parameter("waypoint_y", m_wpt_y);

    node->declare_parameter("waypoint_z", m_wpt_z);
    node->get_parameter("waypoint_z", m_wpt_z);

    //put waypoints into the variable.
    m_waypoints.polygon.points.clear();

    if (m_wpt_x.size() == m_wpt_y.size() && m_wpt_y.size() == m_wpt_z.size()) {
        for (size_t i = 0; i < m_wpt_x.size(); ++i) 
        {
            geometry_msgs::msg::Point32 point;
            point.x = m_wpt_x[i];
            point.y = m_wpt_y[i];
            point.z = m_wpt_z[i];
            m_waypoints.polygon.points.push_back(point);
        }
        m_waypoints.header.frame_id = m_frame_id;
    } 
    else 
    {
        printf("### Waypoint size checking failed!!!\r\n");
    }


    /*************************************************************************/
    /* Setup ROS2 sub/pub/srv/... */
    ///Pubs & subs
    m_update_waypoint_sub = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
                                                    update_topic_name, 10, 
                                                    [this](const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
                                                        this->f_waypoint_cb(msg, false);
                                                        });

   
    m_append_waypoint_sub = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
                                                    append_topic_name, 10, 
                                                    [this](const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
                                                        this->f_waypoint_cb(msg, true);
                                                        });


    m_update_surge_sub = node->create_subscription<std_msgs::msg::Float64>(
                                                    surge_topic_name, 10, 
                                                    std::bind(&PathFollowing::f_surge_cb, 
                                                                this,_1));


    m_full_trajectory_publisher = node->create_publisher<visualization_msgs::msg::Marker>("path", 0);

    m_trajectory_segment_publisher = node->create_publisher<visualization_msgs::msg::Marker>("segment", 0);

    /////services////////////
    get_next_waypoints_server = node->create_service<mvp_msgs::srv::GetWaypoints>("get_next_waypoints",
        std::bind(&PathFollowing::f_cb_srv_get_next_waypoints, this, _1, _2));

    load_waypoint_server = node->create_service<mvp_msgs::srv::LoadWaypoint>("load_waypoints",
        std::bind(&PathFollowing::f_cb_srv_load_waypoint, this, _1, _2));

    update_waypoints_server = node->create_service<mvp_msgs::srv::SendWaypoints>("update_waypoints",
        std::bind(&PathFollowing::f_cb_srv_update_waypoints, this, _1, _2));


    m_toll_client = node->create_client<robot_localization::srv::ToLL>(m_toll_service);

    m_fromll_client = node->create_client<robot_localization::srv::FromLL>(m_fromll_service);

    while (!m_toll_client->wait_for_service(2s)) {
        RCLCPP_WARN(m_logger, 
            "service(%s) not available, waiting again...", m_toll_service.c_str());
    }

    while (!m_fromll_client->wait_for_service(2s)) {
        RCLCPP_WARN(m_logger, 
            "service(%s) not available, waiting again...", m_fromll_service.c_str());
    }

    ////////////control DOFs////////////
    BehaviorBase::m_dofs = decltype(m_dofs){
        mvp_msgs::msg::ControlMode::DOF_U,
        mvp_msgs::msg::ControlMode::DOF_PITCH,
        mvp_msgs::msg::ControlMode::DOF_YAW,
        mvp_msgs::msg::ControlMode::DOF_Z
    };

    m_transform_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    m_transform_listener = std::make_unique<tf2_ros::TransformListener>(*m_transform_buffer);
 
    
}


void PathFollowing::f_surge_cb(const std_msgs::msg::Float64::SharedPtr m)
{
    m_surge_velocity = m->data;

}

void PathFollowing::f_transform_waypoints(
        const std::string &target_frame,
        const geometry_msgs::msg::PolygonStamped &in,
        geometry_msgs::msg::PolygonStamped *out)
{
    geometry_msgs::msg::PolygonStamped tm;

    tm.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    tm.header.frame_id = target_frame;
    // printf("target_frame = %s\r\n", target_frame.c_str());

    try {
        for (const auto &pt: in.polygon.points) {
            geometry_msgs::msg::PointStamped ps;

            if(in.header.frame_id[0] == '/') {
                ps.header.frame_id =
                    in.header.frame_id.substr(1, in.header.frame_id.length());
            } else {
                ps.header.frame_id = in.header.frame_id;
            }
            ps.point.x = pt.x;
            ps.point.y = pt.y;
            ps.point.z = pt.z;
            geometry_msgs::msg::PointStamped t;
            t = m_transform_buffer->transform(ps, target_frame);

            geometry_msgs::msg::Point32 p;
            p.x = static_cast<float>(t.point.x);
            p.y = static_cast<float>(t.point.y);
            p.z = static_cast<float>(t.point.z);

            tm.polygon.points.emplace_back(p);
        }

    }
    catch(tf2::TransformException &e) {
        RCLCPP_ERROR_STREAM(m_logger,e.what()) ;
    }
}


void PathFollowing::resume_or_start() {
    // Transform all the points into controller's frame
    geometry_msgs::msg::PolygonStamped poly;
    f_transform_waypoints(
        // m_process_values.header.frame_id,
        get_helm_global_link(),
        m_waypoints,
        &m_transformed_waypoints
    );

    // Push robots position as the first point
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(m_process_values.position.x);
    p.y = static_cast<float>(m_process_values.position.y);
    // p.z = static_cast<float>(m_process_values.position.z);
    m_wpt_first = p;


    // Select second waypoint to be the next point in the way point list
    m_wpt_second = m_transformed_waypoints.polygon.points[
        m_line_index % m_transformed_waypoints.polygon.points.size()
    ];

}

void PathFollowing::f_next_line_segment() {
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

void PathFollowing::f_waypoint_cb(const geometry_msgs::msg::PolygonStamped::SharedPtr m, bool append)
{
    if(m->header.frame_id.empty()) {
        // no decision can be made
        // ROS_WARN_STREAM("no frame id provided for the waypoints!");
        RCLCPP_WARN(m_logger, "no frame id provided for the waypoints!");
        return;
    }

    if(append)
    {
        for(const auto& i : m->polygon.points) {
            m_waypoints.polygon.points.emplace_back(i);
        }
        //transform the waypoints
        f_transform_waypoints(get_helm_global_link(), m_waypoints, &m_transformed_waypoints);

    }
    else
    {
         // replace
        m_waypoints = *m;

        m_line_index = 0;

        resume_or_start();
    }

}

bool PathFollowing::f_cb_srv_get_next_waypoints(
            const std::shared_ptr<mvp_msgs::srv::GetWaypoints::Request> request,
            const std::shared_ptr<mvp_msgs::srv::GetWaypoints::Response> response)
{
     auto length = m_waypoints.polygon.points.size();
    if (length == 0)
    {
        RCLCPP_WARN(m_logger, "No waypoint programmed");
    }

    int num = request->count.data;
    if (num == 0)
    {
        num = length - m_line_index+1;
    }

    //resize the num variable if it is larger than the actual lenght of the waypoint
    if (num > length) {
        num = length+1;
    }

    response->wpt.resize(num);

    //begging query waypoints
    for (int i =0 ; i< num; i++)
    {
        if(i == 0)
        {
            response->wpt[i].header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            response->wpt[i].header.frame_id = get_helm_global_link();
            response->wpt[i].wpt.x = m_wpt_first.x;
            response->wpt[i].wpt.y = m_wpt_first.y;
            response->wpt[i].wpt.z = m_wpt_first.z;
        }
        else{
            response->wpt[i].header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            response->wpt[i].header.frame_id = m_waypoints.header.frame_id;
            response->wpt[i].wpt.x = static_cast<double>(m_waypoints.polygon.points[m_line_index + i-1].x);
            response->wpt[i].wpt.y = static_cast<double>(m_waypoints.polygon.points[m_line_index + i-1].y);
            response->wpt[i].wpt.z = static_cast<double>(m_waypoints.polygon.points[m_line_index + i-1].z);
        }

        // printf("waypoint xyz got\r\n");
        //we need to call the service to convert to lat and lon
        //call the service
        // std::cout << "Converting future waypoints into geopoint" << std::endl;
        //convert the resp.wpt from the waypoint frame id into the ENU (world)
        Eigen::Vector3d p_world;
        try {

            geometry_msgs::msg::TransformStamped tf_wpt_world = m_transform_buffer->lookupTransform(
                    m_enu_frame,
                    response->wpt[i].header.frame_id,
                    tf2::TimePointZero,
                    10ms
                );
            
            auto tf_eigen = tf2::transformToEigen(tf_wpt_world);
            p_world = tf_eigen.rotation() * 
                                Eigen::Vector3d(response->wpt[i].wpt.x, 
                                                response->wpt[i].wpt.y, 
                                                response->wpt[i].wpt.z)
                                + tf_eigen.translation();


            //check to_LL service
            // robot_localization::srv::ToLL ser;
            auto srv_req = std::make_shared<robot_localization::srv::ToLL::Request>();
            srv_req->map_point.x = p_world.x();
            srv_req->map_point.y = p_world.y();
            srv_req->map_point.z = p_world.z();

            auto node = m_node.lock();
            // m_toll_client
            auto result = m_toll_client->async_send_request(srv_req);
            // Wait for the result.
            if (rclcpp::spin_until_future_complete(node, result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                auto result_value = result.get();
                response->wpt[i].ll_wpt.latitude = result_value->ll_point.latitude;
                response->wpt[i].ll_wpt.longitude = result_value->ll_point.longitude;
                response->wpt[i].ll_wpt.altitude = result_value->ll_point.altitude;
            } 
            else 
            {
                std::cout << "Failed to compute the latitude and longitude for the next waypoint" << std::endl;
                // change the state if failed
                change_state(m_state_fail);
                return false;
            }

        } catch(tf2::TransformException &e) {
            RCLCPP_WARN(m_logger, "Can't transform the p and rpy to the global");
            return false;
        }

    }

    return true;

}

bool PathFollowing::f_cb_srv_load_waypoint(
        const std::shared_ptr<mvp_msgs::srv::LoadWaypoint::Request> request,
        const std::shared_ptr<mvp_msgs::srv::LoadWaypoint::Response> response)
{
    geometry_msgs::msg::PolygonStamped temp_waypoints;
    std::string filename  = waypoint_path + request->file + ".yaml";

    YAML::Node map = YAML::LoadFile(filename);

    bool good_frame = false;
    bool good_waypoint = false;

    if(map["frame_id"])
    {
        temp_waypoints.header.frame_id = m_ns + "/" + map["frame_id"].as<std::string>();
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

            geometry_msgs::msg::Point32 gp;
            gp.x = static_cast<float>(mp["x"]);
            gp.y = static_cast<float>(mp["y"]);
            gp.z = static_cast<float>(mp["z"]);

            temp_waypoints.polygon.points.emplace_back(gp);

            good_waypoint = true;

        }
    }


    if(map["ll_waypoints"])
    {
        for(uint32_t i = 0; i < map["ll_waypoints"].size() ; i++) {
            std::map<std::string, double> mp;
            for(const auto& key : {"lat", "lon", "alt"})
            {
                mp[key] = map["ll_waypoints"][i][key].as<float>();
            }

            geographic_msgs::msg::GeoPoint geop; 
            geop.latitude = static_cast<float>(mp["lat"]);
            geop.longitude = static_cast<float>(mp["lon"]);
            geop.altitude = static_cast<float>(mp["alt"]);

            //convert into lat lon
            auto srv_req = std::make_shared<robot_localization::srv::FromLL::Request>();
            srv_req->ll_point.latitude = geop.latitude;
            srv_req->ll_point.longitude = geop.longitude;
            srv_req->ll_point.altitude = geop.altitude;


            auto node = m_node.lock();
            auto result = m_fromll_client->async_send_request(srv_req);
            // Wait for the result.
            if (rclcpp::spin_until_future_complete(node, result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                auto result_value = result.get();
                // response->wpt[i].ll_wpt.latitude = result_value->ll_point.latitude;
                // response->wpt[i].ll_wpt.longitude = result_value->ll_point.longitude;
                // response->wpt[i].ll_wpt.altitude = result_value->ll_point.altitude;
                geometry_msgs::msg::Point32 gp;
                gp.x = result_value->map_point.x;
                gp.y = result_value->map_point.y;
                gp.z = result_value->map_point.z;
                temp_waypoints.polygon.points.emplace_back(gp);

                good_waypoint = true;
            } 
            else 
            {
                std::cout << "Failed to compute the latitude and longitude for the next waypoint" << std::endl;
                // change the state if failed
                change_state(m_state_fail);
                return false;
            }
            
        }

    }

    if(good_frame && good_waypoint)
    {
        m_waypoints = temp_waypoints;
        printf("waypoint updated from loading file service \r\n");
        m_line_index = 0;
        resume_or_start();
        
        response->success=true;
    }
    else
    {
        response->success=false;
    }

    
    return true;

}

bool PathFollowing::f_cb_srv_update_waypoints(
        const std::shared_ptr<mvp_msgs::srv::SendWaypoints::Request> request,
        const std::shared_ptr<mvp_msgs::srv::SendWaypoints::Response> response)
{
    geometry_msgs::msg::PolygonStamped temp_waypoints;
     //for latlon type
    if(strcmp(request->type.c_str(), "geopath") == 0)
    {
        // printf("updating waypoints \r\n");
        temp_waypoints.header.frame_id = m_frame_id; //set to world by default
        for(const auto& i : request->wpt) {
            

            auto srv_req = std::make_shared<robot_localization::srv::FromLL::Request>();
            srv_req->ll_point.latitude = i.ll_wpt.latitude;
            srv_req->ll_point.longitude = i.ll_wpt.latitude;
            srv_req->ll_point.altitude = i.ll_wpt.altitude;

            auto node = m_node.lock();
            auto result = m_fromll_client->async_send_request(srv_req);
            // Wait for the result.
            if (rclcpp::spin_until_future_complete(node, result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                auto result_value = result.get();
                // response->wpt[i].ll_wpt.latitude = result_value->ll_point.latitude;
                // response->wpt[i].ll_wpt.longitude = result_value->ll_point.longitude;
                // response->wpt[i].ll_wpt.altitude = result_value->ll_point.altitude;
                geometry_msgs::msg::Point32 gp;
                gp.x = result_value->map_point.x;
                gp.y = result_value->map_point.y;
                gp.z = result_value->map_point.z;
                temp_waypoints.polygon.points.emplace_back(gp);
            } 
            else 
            {
                std::cout << "Failed to compute the latitude and longitude for the next waypoint" << std::endl;
                // change the state if failed
                change_state(m_state_fail);
                return false;
            }
        }
        m_waypoints = temp_waypoints;
        printf("geopath type waypoint updated from the service \r\n");
        // printf("m_waypoints size = %d; temp_waypoints = %d\r\n", m_waypoints.polygon.points.size(), temp_waypoints.polygon.points.size());
        m_line_index = 0;
        resume_or_start();
        response->success = true;
        return true;
    }


}


void PathFollowing::activated() {
    /**
     * @brief This function is called when the behavior gets activated.
     *
     * @note This function is an inherited virtual function. It may left
     * unimplemented.
     *
     * @details This function is called when the behavior internal state
     * defined by #BehaviorBase::m_actived changes to true.
     */
    std::cout << "path following behavior is activated!" << std::endl;

    if(!m_waypoints.polygon.points.empty()) {
        resume_or_start();
    }
}

void PathFollowing::disabled() {
    /**
     * @brief This function is called when the behavior gets disabled.
     *
     * @note This function is an inherited virtual function. It may left
     * unimplemented.
     *
     * @details This function is called when the behavior internal state
     * defined by #BehaviorBase::m_actived changes to false.
     */
    std::cout << "path following behavior is disabled!" << std::endl;
}

//! NOTE: for the pitch and yaw, we can't direct assign the joystick value as desired_value,
//!       because pitch and yaw are in the global frame, but surge is ok. it's in the body frame.
bool PathFollowing::request_set_point(mvp_msgs::msg::ControlProcess *set_point) 
{
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
    
    //
    double Xke = dx2 *cos(gamma_p) + dy2 *sin(gamma_p);
    // double Xke = (x - m_wpt_second.x) * cos(gamma_p) +
        // (y - m_wpt_second.y) * sin(gamma_p);

    // Check of overshoot
    double lookahead = m_lookahead_distance;

    if(Xke > 0 ) {
        auto node = m_node.lock();
        // overshoot detected
        // ROS_WARN_THROTTLE(5, "Overshoot detected!");
        RCLCPP_WARN_THROTTLE(m_logger, *node->get_clock(), 5000, "Overshoot detected!");
        // Look back
        lookahead = -lookahead;

        // we are at the opposite side now
        gamma_p = gamma_p + M_PI;

        // record the time
        auto t = rclcpp::Clock(RCL_ROS_TIME).now();

        // if overshoot timer is not set, set it now.
        if( (t.seconds() - m_overshoot_timer) == t.seconds())
        {
            m_overshoot_timer = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
        }

        // check if overshoot timer passed the timeout.
        if((t.seconds() - m_overshoot_timer) > m_overshoot_timeout) {
            // ROS_ERROR_THROTTLE(10, "Overshoot abort!");
            RCLCPP_WARN_THROTTLE(m_logger, *node->get_clock(), 10000, "Overshoot abort!");

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

    

    // set the heading for line of sight
    // m_cmd.orientation.z = gamma_p - atan( (Ye + m_sigma*m_yint)/ lookahead   + ye_dot*m_beta_gain);
    if(m_lookahead_adaptive)
    {
         //adapitve lookahead
        // lookahead = m_lookahead_max - (m_lookahead_max - m_lookahead_min)*exp(-m_lookahead_gamma*fabs(Ye));
        lookahead = (m_lookahead_max - m_lookahead_min)*exp(-m_lookahead_gamma*fabs(Ye)) + m_lookahead_min;

        // printf("lookahead distance = %f\r\n", lookahead);
        
        // m_cmd.orientation.z = gamma_p - atan(Ye/lookahead);

    }
    /// compute desired set point now
    // set the surge velocity

    set_point->velocity.x = m_surge_velocity;

    set_point->orientation.z = gamma_p - atan( Ye/ lookahead   + ye_dot*m_beta_gain + m_sigma*m_yint/lookahead);

    //Set the direct z set point
    set_point->position.z = m_wpt_second.z;

    set_point->orientation.y = m_pitch;

    // check the acceptance radius
    
    auto dist = std::sqrt(dx2 * dx2 + dy2*dy2);
    if(dist < m_acceptance_radius ) {
        f_next_line_segment();
        m_overshoot_timer = 0;
    }

    
    return true;
}

/**
 * @brief Behavior must export the class to the Plugin library.
 */
 #include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(helm::PathFollowing, helm::BehaviorBase)