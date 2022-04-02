
#include "parser.h"

#include <utility>
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

    tinyxml2::XMLDocument doc;

    doc.Parse(xml_string.c_str(), xml_string.length());

    m_xml_root = doc.FirstChildElement(xml::TAG);

    if(m_xml_root == nullptr) {
        throw HelmException("Check formatting of your xml file!");
    }

    f_parse_behavior_components();

    f_parse_sm_components();

}



void Parser::f_parse_behavior_components() {

    for(auto * xml_bhv = m_xml_root->FirstChildElement(xml::bhvconf::behavior::TAG);
        xml_bhv != nullptr;
        xml_bhv = xml_bhv->NextSiblingElement(xml::bhvconf::behavior::TAG) )
    {

        behavior_component_t the_behavior;

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
            the_behavior.states[state_name] =
                xml_state->IntAttribute(xml::bhvconf::behavior::state::ATTRS::PRIORITY);
        }

        the_behavior.name = std::string(bhv_name);
        the_behavior.plugin = std::string(bhv_plugin);
        the_behavior.states = bhv_states;

        m_op_behavior_component(the_behavior);

    }

}

void Parser::f_parse_sm_components() {
    for(auto * xml_state = m_xml_root->FirstChildElement(xml::sm::TAG);
        xml_state != nullptr;
        xml_state = xml_state->NextSiblingElement(xml::sm::state::TAG) )
    {
        sm_state_t the_state;

        auto state_name = xml_state->Attribute(xml::sm::state::ATTRS::NAME);

        if(state_name == nullptr) {
            throw HelmException("A state machine state without name!");
        }

        auto state_mode = xml_state->Attribute(xml::sm::state::ATTRS::MODE);
        if(state_mode == nullptr) {
            throw HelmException("A state machine state without low level controller mode!");
        }

        auto state_init = xml_state->Attribute(xml::sm::state::ATTRS::INITIAL);
        if(state_init != nullptr) {
            the_state.initial = xml_state->BoolAttribute(xml::sm::state::ATTRS::INITIAL);
        }

        for(auto *xml_transition = xml_state->FirstChildElement() ;
            xml_transition != nullptr ;
            xml_transition = xml_transition->NextSiblingElement(
                xml::sm::state::transition::TAG
            ))
        {
            the_state.transitions.emplace_back(
                xml_transition->Attribute(
                    xml::sm::state::transition::ATTRS::TO
                )
            );
        }

        // This will be called after each state machine state appears in XML
        m_op_sm_component(the_state);

    }
}

void Parser::f_parse_helm_configuration() {

    helm_configuration_t conf{};

    auto xml_freq = m_xml_root->FirstChildElement(xml::helmconf::frequency::TAG);
    if(xml_freq != nullptr) {
        double a;
        auto error = xml_freq->QueryDoubleText(&a);

        if(error == tinyxml2::XML_SUCCESS) {
            conf.frequency = a;
        } else {
            conf.frequency = DEFAULT_HELM_FREQ;
        }

    }

    m_op_helmconf_component(conf);

}

void Parser::set_op_behavior_component(decltype(m_op_behavior_component) f) {
    m_op_behavior_component = std::move(f);
}

void Parser::set_op_sm_component(decltype(m_op_sm_component) f) {
    m_op_sm_component = std::move(f);
}

void Parser::set_op_helmconf_component(decltype(m_op_helmconf_component) f) {
    m_op_helmconf_component = std::move(f);
}

