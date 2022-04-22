
#include "parser.h"

#include "utility"
#include "exception.h"
#include "dictionary.h"

#include "tinyxml2.h"
#include "behavior_container.h"
#include "ros/package.h"

#include "unistd.h"
#include "cstdlib"
#include "cstdio"
#include "sys/wait.h"

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

    f_parse_helm_configuration();

    f_parse_behavior_components();

    f_parse_sm_components();


}



void Parser::f_parse_behavior_components() {

    auto xml_bhvs = m_xml_root->FirstChildElement(xml::bhvconf::TAG);


    if(xml_bhvs == nullptr) {
        throw HelmException("no <" + std::string(xml::bhvconf::TAG) + "/>  is provided");
    }

    for(auto * xml_bhv = xml_bhvs->FirstChildElement(xml::bhvconf::behavior::TAG);
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
            bhv_states[state_name] =
                xml_state->IntAttribute(xml::bhvconf::behavior::state::ATTRS::PRIORITY);
        }

        tinyxml2::XMLElement* bhv_parameters = xml_bhv->FirstChildElement(
            xml::bhvconf::behavior::parameters::TAG
        );

        tinyxml2::XMLPrinter param_printer;
        if(bhv_parameters != nullptr) {
            auto type = bhv_parameters->Attribute(
                xml::bhvconf::behavior::parameters::ATTRS::TYPE);

            if(type != nullptr) {
                if(std::strcmp(type, xml::bhvconf::behavior::parameters::ATTRS::OPTIONS_TYPE::ROS) == 0) {
                    // load ros param

                    tinyxml2::XMLDocument doc;


                    auto * launch = doc.NewElement("launch");
                    auto * group = doc.NewElement("group");
                    group->SetAttribute("ns",
                                        (ros::this_node::getName() + "/" + std::string(bhv_name)).c_str());
                    doc.InsertEndChild(launch);
                    launch->InsertEndChild(group);
                    for(auto * e = bhv_parameters->FirstChild();
                        e != nullptr;
                        e = e->NextSiblingElement())
                    {
                        group->InsertEndChild(e->DeepClone(&doc));
                    }
                    doc.Accept(&param_printer);

                    f_load_ros_param(param_printer.CStr());

                }
            }

        }


        the_behavior.name = std::string(bhv_name);
        the_behavior.plugin = std::string(bhv_plugin);
        the_behavior.states = bhv_states;
        the_behavior.params = std::string(param_printer.CStr());
        m_op_behavior_component(the_behavior);

    }

}

void Parser::f_parse_sm_components() {

    auto xml_states = m_xml_root->FirstChildElement(xml::smconf::TAG);

    if(xml_states == nullptr) {
        throw HelmException("no <" + std::string(xml::smconf::TAG) + "/>  is provided");
    }

    for(auto * xml_state = xml_states->FirstChildElement(xml::smconf::state::TAG);
        xml_state != nullptr;
        xml_state = xml_state->NextSiblingElement(xml::smconf::state::TAG) )
    {
        sm_state_t the_state;

        auto state_name = xml_state->Attribute(xml::smconf::state::ATTRS::NAME);

        if(state_name == nullptr) {
            throw HelmException("A state machine state without name!");
        }
        the_state.name = std::string(state_name);

        auto state_mode = xml_state->Attribute(xml::smconf::state::ATTRS::MODE);
        if(state_mode == nullptr) {
            throw HelmException("A state machine state without low level controller mode!");
        }
        the_state.mode = std::string(state_mode);

        auto state_init = xml_state->Attribute(xml::smconf::state::ATTRS::INITIAL);
        the_state.initial = false;
        if(state_init != nullptr) {
            the_state.initial = xml_state->BoolAttribute(xml::smconf::state::ATTRS::INITIAL);

            std::cout << "initial state: " << the_state.name << std::endl;
        }

        for(auto *xml_transition = xml_state->FirstChildElement();
            xml_transition != nullptr ;
            xml_transition = xml_transition->NextSiblingElement(
                xml::smconf::state::transition::TAG
            ))
        {
            the_state.transitions.emplace_back(
                xml_transition->Attribute(
                    xml::smconf::state::transition::ATTRS::TO
                )
            );
        }

        // This will be called after each state machine state appears in XML
        m_op_sm_component(the_state);

    }
}

void Parser::f_parse_helm_configuration() {

    helm_configuration_t conf{};

    auto xml_helm_conf = m_xml_root->FirstChildElement(xml::helmconf::TAG);

    if(xml_helm_conf == nullptr) {
        throw HelmException("no <" + std::string(xml::helmconf::TAG) + "/>  is provided");
    }

    for(auto * xml_param = xml_helm_conf->FirstChildElement(xml::generic::param::TAG);
        xml_param != nullptr;
        xml_param = xml_param->NextSiblingElement(xml::generic::param::TAG) )
    {
        auto name = xml_param->Attribute(xml::generic::param::ATTRS::NAME);

        if(strcmp(xml::helmconf::param_types::FREQUENCY, name) == 0) {
            double value;
            xml_param->QueryAttribute(xml::generic::param::ATTRS::VALUE, &value);
            conf.frequency = value;
        } else {
            // rest of the configuration variables will go here
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

void Parser::f_load_ros_param(const std::string& launch) {

    int pfd[2];

    int the_stdout = dup(STDOUT_FILENO);

    pid_t cpid;

    if(pipe(pfd) == -1) {
        fprintf(stderr, "Pipe error\n");
        exit(-1);
    }
    cpid = fork();
    if(cpid == -1) {
        fprintf(stderr, "Fork error\n");
        exit(-1);
    }
    if( cpid == 0 ) {
        close(pfd[1]);
        close(STDIN_FILENO);
        int new_stdin = dup(pfd[0]);

        execlp("roslaunch",
               "roslaunch","-","--wait", "--no-summary", "--disable-title",
               nullptr);

        close(pfd[0]);
        close(new_stdin);
        exit(0);
    } else {
        /* Father */
        close(pfd[0]);
        close(STDOUT_FILENO);

        int new_stdout = dup(pfd[1]);

        std::cout << launch << std::endl;
        close(pfd[1]);
        close(new_stdout);
        /* waiting for child */
        wait(nullptr);
    }

    dup2(the_stdout, STDOUT_FILENO);
    close(the_stdout);

}