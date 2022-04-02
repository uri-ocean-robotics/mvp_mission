#pragma once

#include "behavior_interface/behavior_base.h"
#include "ros/ros.h"


namespace helm {

    class PathFollowing : public BehaviorBase {
    private:
        /**
         * @brief
         * @todo To be implemented
         */
        void initialize() override;

        ros::NodeHandlePtr m_nh;

    public:

        PathFollowing();

        bool request_set_point(mvp_control::ControlProcess *msg) override;

    };
}