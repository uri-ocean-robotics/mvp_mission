#pragma once

#include "behavior_interface/behavior_base.h"
#include "ros/ros.h"
#include "mvp_control/ControlProcess.h"
#include "geometry_msgs/PolygonStamped.h"
#include "tf2_ros/transform_listener.h"

namespace helm {

    class PathFollowing : public BehaviorBase {
    private:
        /**
         * @brief
         * @todo To be implemented
         */
        void initialize() override;

        /***********************************************************************
         * utils
         */
        void f_2point_to_line(double y2, double x2, double y1, double x1,
                              double *a, double *b, double *c);

        void f_shortest_dist_to_line(double a, double b, double c, double x0,
                                     double y0, double *dist);

        void f_closest_point_to_line(double a, double b, double c, double x0,
                                     double y0, double *x, double *y);

        int f_direction_of_point(double ax, double ay,
                                  double bx, double by,
                                  double x, double y);

        /***********************************************************************
         * ROS
         */

        /**
         * @brief Trivial node handle
         */
        ros::NodeHandlePtr m_pnh;

        /**
         * @brief Control Process command message
         */
        mvp_control::ControlProcess m_cmd;

        /**
         * @brief Trivial update waypoint subscriber
         */
        ros::Subscriber m_update_waypoint_sub;

        /**
         * @brief Trivial append waypoint subscriber
         */
        ros::Subscriber m_append_waypoint_sub;

        geometry_msgs::PolygonStamped m_waypoints;

        int m_line_index;

        //! @brief Transform buffer for TF2
        tf2_ros::Buffer m_transform_buffer;

        //! @brief Transform listener for TF2
        std::shared_ptr<tf2_ros::TransformListener> m_transform_listener;

        std::string m_world_frame;

        void f_parse_param_waypoints();

        void f_transform_waypoints(const geometry_msgs::PolygonStamped &in,
                                   geometry_msgs::PolygonStamped *out);

        void f_transform_waypoints(
            const std::string &target_frame,
            const geometry_msgs::PolygonStamped &in,
            geometry_msgs::PolygonStamped *out
        );

        void f_waypoint_cb(const geometry_msgs::PolygonStamped::ConstPtr &m,
                           bool append);

        void f_proceed_to_next_line();

        ~PathFollowing() override;

    public:

        /**
         * @brief trivial constructor
         */
        PathFollowing();

        /**
         * @brief
         * @param msg
         * @return
         */
        bool request_set_point(mvp_control::ControlProcess *msg) override;

    };
}