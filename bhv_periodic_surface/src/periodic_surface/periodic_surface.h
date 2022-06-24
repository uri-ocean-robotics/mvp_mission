#pragma once

#include "behavior_interface/behavior_base.h"
#include "ros/ros.h"


namespace helm {

    class PeriodicSurface : public BehaviorBase {
    private:
        enum class BhvState : int {
            DISABLED,
            ENABLED,
            WAITING
        };

        BhvState m_bhv_state;


        /**
         * @brief
         * @todo To be implemented
         */
        void initialize() override;

        ros::NodeHandlePtr m_nh;

        /**
         * @brief Forward distance of the surfacing behaviour
         *
         */
        double m_fwd_distance;

        double m_max_pitch;
        double m_surface_period;
        double m_surface_duration;

        ros::Time m_start_time;
        ros::Time m_surfaced_time;
        bool m_surfaced;

        void activated();

        void disabled();


        bool m_bhv_active;

    public:

        PeriodicSurface();

        bool request_set_point(mvp_control::ControlProcess *msg) override;

    };
}