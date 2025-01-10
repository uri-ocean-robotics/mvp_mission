#pragma once

#include <vector>
#include <string>
#include <map>

#define CONST_STRING static constexpr const char *

namespace helm {

    /***************************************************************************
     * Constants
     */

    static constexpr double DEFAULT_HELM_FREQ = 50;


   /****************************************************************************
    * structs and types
    */
    struct sm_state_t {
        bool initial;
        std::string name;
        std::string control_mode;
        std::vector<std::string> transitions;
        std::string set_point_world_frame;
        std::string set_point_child_frame;
    };

    struct behavior_sm_state_t{
        std::string name;
        int priority;
    };

    struct behavior_component_t{
        std::string name;
        std::string plugin;
        std::map<std::string, int> states;
        std::string params;
    };

    struct helm_configuration_t{
        double frequency;
        std::string world_link;
        std::string child_link;
    };

    CONST_STRING CONF_HELM = "helm_configuration";
    CONST_STRING CONF_HELM_FREQ = "frequency";
    CONST_STRING CONF_HELM_WORLD_LINK_DEFAULT = "default_world_link";
    CONST_STRING CONF_HELM_CHILD_LINK_DEFAULT = "default_child_link";
    CONST_STRING CONF_TF_PREFIX = "tf_prefix";
    CONST_STRING CONF_HELM_FILE = "helm_config_file";
    CONST_STRING CONF_FROMLL_SRV_NAME = "helm_fromLL_srv_name";
    CONST_STRING CONF_TOLL_SRV_NAME = "helm_toLL_srv_name";

    CONST_STRING CONF_FSM = "finite_state_machine";
    CONST_STRING CONF_FSM_NAME = "state_name";
    CONST_STRING CONF_FSM_MODE = "control_mode";
    CONST_STRING CONF_FSM_INITIAL = "initial";
    CONST_STRING CONF_FSM_TRANSITIONS = "transitions";
    CONST_STRING CONF_FSM_WORLD_FRAME = "set_point_world_frame";
    CONST_STRING CONF_FSM_CHILD_FRAME = "set_point_child_frame";

    CONST_STRING CONF_BHV = "behaviors";
    CONST_STRING CONF_BHV_NAME = "name";
    CONST_STRING CONF_BHV_PLUGIN = "plugin";
    CONST_STRING CONF_BHV_STATES = "states";
    CONST_STRING CONF_BHV_STATES_NAME = "name";
    CONST_STRING CONF_BHV_STATES_PRIORITY = "priority";
}