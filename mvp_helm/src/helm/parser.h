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
         * @brief
         */
        void f_parse_behavior_components();

        void f_parse_sm_components();

        void f_parse_helm_configuration();

        std::function<void(behavior_component_t)> m_op_behavior_component;

        std::function<void(sm_state_t)> m_op_sm_component;

        std::function<void(helm_configuration_t)> m_op_helmconf_component;

    public:

        Parser();

        void initialize();

        void set_op_behavior_component(decltype(m_op_behavior_component));

        void set_op_sm_component(decltype(m_op_sm_component) f);

        void set_op_helmconf_component(decltype(m_op_helmconf_component) f);

        typedef std::shared_ptr<Parser> Ptr;

    };
}