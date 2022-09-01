/*
    This file is part of MVP-Mission program.

    MVP-Mission is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MVP-Mission is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MVP-Mission.  If not, see <https://www.gnu.org/licenses/>.

    Author: Emir Cem Gezer
    Email: emircem@uri.edu;emircem.gezer@gmail.com
    Year: 2022

    Copyright (C) 2022 Smart Ocean Systems Laboratory
*/


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
        bool request_set_point(mvp_msgs::ControlProcess *msg) override;

    };
}