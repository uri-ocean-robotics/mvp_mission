#pragma once

#include "behavior_interface/behavior_base.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace helm {

    class DepthTracking : public BehaviorBase {
    private:
        /**
         * @brief
         * @todo To be implemented
         */
        void initialize() override;

        /**
         * @brief trivial node handler
         */
        ros::NodeHandlePtr m_nh;

        /**
         * @brief requested depth
         */
        double m_requested_depth{};

        ros::Subscriber m_sub;

        void f_cb_sub(const std_msgs::Float64::ConstPtr& msg);

        double m_p_gain;

        double m_d_gain;

        double m_max_pitch;

    public:

        /**
         * @brief Trivial constructor
         */
        DepthTracking();

        ~DepthTracking();

        bool request_set_point(mvp_control::ControlProcess *msg) override;

    };
}