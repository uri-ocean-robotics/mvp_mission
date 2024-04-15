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
#include "rclcpp/rclcpp.hpp"

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

// #include "behavior_container.hpp"
#include "dictionary.h"

namespace helm {

    class Parser {
    private:

        rclcpp::Logger m_logger{rclcpp::get_logger("helm_parser")};

         //! @brief Transform prefix
        // std::string m_tf_prefix;

        //! @brief helm configuration yaml file
        std::string m_helm_config_file;

        // //! @brief helm_frequency
        // double m_frequency;

        // //! @brief global link
        // std::string m_global_link;

        // //! @brief local link
        // std::string m_local_link;

        /**
         * @brief Parses Helm Configuration components in the yaml configuration
         *
         * It reads and parses the yaml file. After parsing, it calls operation
         * defined by #Parser::m_op_helmconf_component.
         */
        // void f_parse_helm_config();

        /**
         * @brief Parses behavior components in the helm yaml configuration
         *
         * It reads and parses the yaml file. After parsing, it calls operation
         * defined by #Parser::m_op_behavior_component.
         */
        void f_parse_bhv_components();
        
        /**
         * @brief Parses State Machine components in the helm yaml configuration
         *
         * It reads and parses the yaml file. After parsing, it calls operation
         * defined by #Parser::m_op_sm_component.
         */
        void f_parse_sm_components();

        /**
         * @brief Behavior component operation
         */
        std::function<void(behavior_component_t)> m_op_behavior_component = [](behavior_component_t){};

        /**
         * @brief State machine component operation
         */
        std::function<void(sm_state_t)> m_op_sm_component = [](sm_state_t){};

        /**
         * @brief Helm Configuration component operation
         */
        // std::function<void(helm_configuration_t)> m_op_helmconf_component = [](helm_configuration_t){};

    public:

        Parser(std::string config_file);

        void initialize();

        void set_op_behavior_component(decltype(m_op_behavior_component));

        void set_op_sm_component(decltype(m_op_sm_component) f);

        // void set_op_helmconf_component(decltype(m_op_helmconf_component) f);

        typedef std::shared_ptr<Parser> Ptr;

    };
}