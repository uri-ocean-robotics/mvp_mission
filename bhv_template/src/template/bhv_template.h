#pragma once

#include "behavior_interface/behavior_base.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace helm {

    class BehaviorTemplate : public BehaviorBase {
    private:
        /**
         * @brief Initialize function
         *
         * @details This function initializes the behavior. It is called by the
         * helm. It is a pure virtual function and it must be implemented in the
         * behavior. If left unimplemented, code will not compile. In this
         * function, user is responsible for proper initalization of the
         * behavior. This function must be unblocking, otherwise, every other
         * behavior may wait this function to return. In this function user
         * should create #ros::NodeHandle, and define controlled degrees of
         * freedom. Below is a trivial implementation of this function
         *
         * @code{.cpp}
         * void BehaviorTemplate::initalize() {
         *   m_pnh.reset(
         *     new ros::NodeHandle(ros::this_node::getName() + "/" +
         *       BehaviorBase::m_name)
         *   );
         *
         *   BehaviorBase::m_dofs = decltype(m_dofs){
         *     ctrl::DOF::SURGE
         *   };
         * }
         * @endcode
         */
        void initialize() override;

        /**
         * @brief trivial node handler
         */
        ros::NodeHandlePtr m_pnh;

        /**
         * @brief This function is inherited from #BehaviorBase
         */
        void activated() override;

        /**
         * @brief This function is inherited from #BehaviorBase
         */
        void disabled() override;

    public:

        /**
         * @brief Trivial constructor
         */
        BehaviorTemplate();

        /**
         * @brief Request set point from the behavior. It is consumed by helm.
         *
         * @param msg Result value of the behavior. This value is written by the
         *            Behavior. Helm uses this variable to generate set_point
         *            for the controller.
         * @return true if you want helm to use the result.
         * @return false if you don't want helm to use the result.
         */
        bool request_set_point(mvp_control::ControlProcess *msg) override;

    };
}