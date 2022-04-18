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

/*******************************************************************************
 * Mist
 */
#include "tinyxml2.h"

namespace helm {

    class Parser : public HelmObj {
    private:

        //! @brief Root xml element for accessing elements
        tinyxml2::XMLElement * m_xml_root{};

        /**
         * @brief Parses behavior components in the helm xml configuration
         *
         * It reads and parses the XML file. After parsing, it calls operation
         * defined by #Parser::m_op_behavior_component.
         */
        void f_parse_behavior_components();

        /**
         * @brief Parses State Machine components in the helm xml configuration
         *
         * It reads and parses the XML file. After parsing, it calls operation
         * defined by #Parser::m_op_sm_component.
         */
        void f_parse_sm_components();

        /**
         * @brief Parses Helm Configuration components in the xml configuration
         *
         * It reads and parses the XML file. After parsing, it calls operation
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

        template <typename T>
        static void
        f_read_generic_param(tinyxml2::XMLElement *e, T *p, std::string *name);


    public:

        Parser();

        void initialize();

        void set_op_behavior_component(decltype(m_op_behavior_component));

        void set_op_sm_component(decltype(m_op_sm_component) f);

        void set_op_helmconf_component(decltype(m_op_helmconf_component) f);

        typedef std::shared_ptr<Parser> Ptr;

    };
}