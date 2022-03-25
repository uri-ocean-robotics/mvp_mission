
#include "parser.h"
#include "exception.h"
#include "dictionary.h"

#include "tinyxml2.h"
#include "behavior_container.h"

using namespace helm;


Parser::Parser() : HelmObj() {

}

void Parser::initialize() {

    if(!m_pnh->hasParam(CONF_CONFIGURATION)) {
        throw HelmException("No helm configuration is provided!");
    }

    std::string xml_string;

    m_pnh->getParam(CONF_CONFIGURATION, xml_string);

    m_doc.Parse(xml_string.c_str(), xml_string.length());

    m_xml_root = m_doc.FirstChildElement(xml::TAG);

    if(m_xml_root == nullptr) {
        throw HelmException("Check formatting of your xml file!");
    }

    f_query_behavior_components();


    f_query_fsm_components();

}



void Parser::f_query_behavior_components() {

    for(auto * xml_bhv = m_xml_root->FirstChildElement(xml::bhvconf::behavior::TAG);
        xml_bhv != nullptr;
        xml_bhv = xml_bhv->NextSiblingElement(xml::bhvconf::behavior::TAG) )
    {

        auto bhv_name = xml_bhv->Attribute(xml::bhvconf::behavior::ATTRS::NAME);

        if(bhv_name == nullptr) {
            throw HelmException("A behavior without name!");
        }

        auto bhv_plugin = xml_bhv->Attribute(xml::bhvconf::behavior::ATTRS::PLUGIN);
        if(bhv_plugin == nullptr) {
            throw HelmException("A behavior without plugin!");
        }

        std::map<std::string, int> bhv_states;
        for(auto * xml_state = xml_bhv->FirstChildElement(xml::bhvconf::behavior::state::TAG);
            xml_state != nullptr;
            xml_state = xml_state->NextSiblingElement(xml::bhvconf::behavior::state::TAG))
        {

            auto state_name = xml_state->Attribute(xml::bhvconf::behavior::state::ATTRS::NAME);

            if(state_name == nullptr) {
                throw HelmException("A state without name!");
            }

            auto state_priority =
                xml_state->Attribute(xml::bhvconf::behavior::state::ATTRS::PRIORITY);



            if(state_priority == nullptr) {
                throw HelmException("A state without priority!");
            }
            bhv_states[state_name] =
                xml_state->IntAttribute(xml::bhvconf::behavior::state::ATTRS::PRIORITY);
        }

        m_op_behavior_component(bhv_name, bhv_plugin, bhv_states);

    }

}

void Parser::f_query_fsm_components() {
    for(auto * xml_state = m_xml_root->FirstChildElement(xml::fsm::TAG);
        xml_state != nullptr;
        xml_state = xml_state->NextSiblingElement(xml::fsm::state::TAG) )
    {

        auto state_name = xml_state->Attribute(xml::fsm::state::ATTRS::NAME);

        if(state_name == nullptr) {
            throw HelmException("A FSM state without name!");
        }

        auto state_mode = xml_state->Attribute(xml::fsm::state::ATTRS::MODE);
        if(state_mode == nullptr) {
            throw HelmException("A FSM state without low level controller mode!");
        }

        m_op_fsm_component(state_name, state_mode);

    }
}

void Parser::set_op_behavior_component(decltype(m_op_behavior_component) f) {
    m_op_behavior_component = f;
}

void Parser::set_op_fsm_component(decltype(m_op_fsm_component) f) {
    m_op_fsm_component = f;
}
