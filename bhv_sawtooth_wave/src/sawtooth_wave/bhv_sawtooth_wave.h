#pragma once

#include "behavior_interface/behavior_base.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace helm {

    class SawtoothWave : public BehaviorBase {
    private:

        enum class BHV_STATE : int{
            IDLE,
            ASCENDING,
            DESCENDING
        };

        void initialize() override;

        double m_min_depth;

        double m_max_depth;

        double m_pitch;

        double m_surge_velocity;

        double m_heading;

        BHV_STATE m_bhv_state;

        /**
         * @brief trivial node handler
         */
        ros::NodeHandlePtr m_pnh;

        void activated() override;

        void disabled() override;

    public:


        SawtoothWave();

        bool request_set_point(mvp_msgs::ControlProcess *msg) override;

    };
}