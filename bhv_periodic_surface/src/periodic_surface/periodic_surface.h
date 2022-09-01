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

        /**
         * @brief Trivial node handler
         */
        ros::NodeHandlePtr m_pnh;

        /**
         * @brief Forward distance of the surfacing behaviour
         *
         */
        double m_fwd_distance;

        /**
         * @brief maximum pitch that the behavior will command. in radians
         */
        double m_max_pitch;

        /**
         * @brief Surfacing period. in seconds
         *
         * A surfacing period ends when the vehicle is surfaced and it begins
         * when the #PeriodicSurface::m_surface_duration ends.
         */
        double m_surface_period;

        /**
         * @brief Surfacing duration, in seconds
         *
         * Dictates the duration of the surfacing
         */
        double m_surface_duration;

        /**
         * @brief Unix time stamp of the time when behavior is activated again.
         *
         * When it activated, it indicates the time when the vehicle starts to
         * climb up again.
         */
        ros::Time m_start_time;

        /**
         * @brief Unix time stamp of the time when the vehicle is first surfaced
         */
        ros::Time m_surfaced_time;

        /**
         * @brief Implementation of #BehaviorBase::activated
         */
        void activated() override;

        /**
         * @brief Implementation of #BehaviorBase::disabled
         */
        void disabled() override;

    public:

        PeriodicSurface();

        bool request_set_point(mvp_msgs::ControlProcess *msg) override;

    };
}