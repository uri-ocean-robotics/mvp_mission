#pragma once

#include "behavior_interface/behavior_base.h"
#include "ros/ros.h"


namespace helm {

    class PeriodicSurface : public BehaviorBase {
    private:
        /**
         * @brief
         * @todo To be implemented
         */
        void initialize() override;

        ros::NodeHandlePtr m_nh;

    public:

        PeriodicSurface();

        bool request_set_point(mvp_control::ControlProcess *msg) override;

    };
}