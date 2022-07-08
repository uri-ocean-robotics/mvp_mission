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