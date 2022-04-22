#pragma once

#include "behavior_interface/behavior_base.h"
#include "ros/ros.h"
#include "mvp_control/ControlProcess.h"
#include "geometry_msgs/PolygonStamped.h"
#include "motion_evaluation/FreqMagConfig.h"
#include "mutex"
#include "dynamic_reconfigure/server.h"

namespace helm {

    class MotionEvaluation : public BehaviorBase {
    private:

        void initialize() override;

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

        motion_evaluation::FreqMagConfig m_config;

        std::recursive_mutex m_config_mutex;

        dynamic_reconfigure::Server<motion_evaluation::FreqMagConfig>
            m_dynconf_server;

        void f_dynconf_freqmag_cb(motion_evaluation::FreqMagConfig& conf,
                                  uint32_t level);

        double m_surge_phase;

        double m_yaw_rate_phase;

        double m_pitch_rate_phase;

        void activated() override {};

        void disabled() override {};

    public:

        MotionEvaluation();


        bool request_set_point(mvp_control::ControlProcess *msg) override;

    };
}