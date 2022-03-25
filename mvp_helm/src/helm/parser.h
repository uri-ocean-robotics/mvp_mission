#pragma once

#include "ros/ros.h"
#include "string"
#include "vector"
#include "memory"
#include "functional"
#include "obj.h"
#include "behavior_container.h"
#include "tinyxml2.h"

namespace helm {

    class Parser : public HelmObj {
    private:

        tinyxml2::XMLDocument m_doc;

        tinyxml2::XMLElement * m_xml_root;

        void f_query_behavior_components();

        void f_query_fsm_components();

        std::function<
            void (
                std::string name,
                std::string type,
                std::map<std::string, int> states
            )
        > m_op_behavior_component;

        std::function<
            void (
                std::string name,
                std::string mode
            )
        > m_op_fsm_component;

    public:

        Parser();

        void initialize();

        void set_op_behavior_component(decltype(m_op_behavior_component));

        void set_op_fsm_component(decltype(m_op_fsm_component));

        typedef std::shared_ptr<Parser> Ptr;

    };
}