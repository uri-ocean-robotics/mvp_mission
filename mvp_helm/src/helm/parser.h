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

/*******************************************************************************
 * ROS
 */
#include "ros/ros.h"

/*******************************************************************************
 * STD
 */
#include "string"
#include "vector"
#include "memory"
#include "functional"

/*******************************************************************************
 * Helm
 */
#include "obj.h"
#include "behavior_container.h"

namespace helm {

    class Parser : public HelmObj {
    private:

        /**
         * @brief Parses behavior components in the helm yaml configuration
         *
         * It reads and parses the yaml file. After parsing, it calls operation
         * defined by #Parser::m_op_behavior_component.
         */
        void f_parse_behavior_components();

        /**
         * @brief Parses State Machine components in the helm yaml configuration
         *
         * It reads and parses the yaml file. After parsing, it calls operation
         * defined by #Parser::m_op_sm_component.
         */
        void f_parse_sm_components();

        /**
         * @brief Parses Helm Configuration components in the yaml configuration
         *
         * It reads and parses the yaml file. After parsing, it calls operation
         * defined by #Parser::m_op_helmconf_component.
         */
        void f_parse_helm_configuration();

        /**
         * @brief Behavior component operation
         */
        std::function<void(behavior_component_t)> m_op_behavior_component;

        /**
         * @brief State machine component operation
         */
        std::function<void(sm_state_t)> m_op_sm_component;

        /**
         * @brief Helm Configuration component operation
         */
        std::function<void(helm_configuration_t)> m_op_helmconf_component;

        static void f_load_ros_param(const std::string& launch);
    public:

        Parser();

        void initialize();

        void set_op_behavior_component(decltype(m_op_behavior_component));

        void set_op_sm_component(decltype(m_op_sm_component) f);

        void set_op_helmconf_component(decltype(m_op_helmconf_component) f);

        typedef std::shared_ptr<Parser> Ptr;

    };
}