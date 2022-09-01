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


#include "parser.h"

#include "utility"
#include "exception.h"
#include "dictionary.h"

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

    if(!m_pnh->hasParam("finite_state_machine")) {
        throw HelmException(
            "No finite state machine configuration is provided!");
    }

    if(!m_pnh->hasParam("behaviors")) {
        throw HelmException("No behavior configuration is provided!");
    }


    f_parse_helm_configuration();

    f_parse_behavior_components();

    f_parse_sm_components();

}

void Parser::f_parse_helm_configuration() {

    XmlRpc::XmlRpcValue helm_config;

    m_pnh->getParam(CONF_HELM, helm_config);

    m_op_helmconf_component(
        {
            .frequency = static_cast<double>(helm_config[CONF_HELM_FREQ])
        }
    );
}

void Parser::f_parse_behavior_components() {

    XmlRpc::XmlRpcValue bhv_list;

    m_pnh->getParam(CONF_BHV, bhv_list);

    ROS_ASSERT(bhv_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for(int32_t i = 0 ; i< bhv_list.size() ; i++) {

        std::map<std::string, int> states;
        for(int32_t j = 0 ; j < bhv_list[i][CONF_BHV_STATES].size() ; j++) {
            states[bhv_list[i][CONF_BHV_STATES][j][CONF_BHV_STATES_NAME]] =
                bhv_list[i][CONF_BHV_STATES][j][CONF_BHV_STATES_PRIORITY];
        }


        m_op_behavior_component(
            {
                .name = bhv_list[i][CONF_BHV_NAME],
                .plugin = bhv_list[i][CONF_BHV_PLUGIN],
                .states = states
            }
        );
    }
}

void Parser::f_parse_sm_components() {

    XmlRpc::XmlRpcValue fsm_list;

    m_pnh->getParam(CONF_FSM, fsm_list);

    ROS_ASSERT(fsm_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for(int32_t i = 0 ; i < fsm_list.size() ; i++) {

        bool initial;
        if(fsm_list[i].hasMember(CONF_FSM_INITIAL)) {
            initial = fsm_list[i][CONF_FSM_INITIAL];
        }

        std::vector<std::string> transitions;
        if(fsm_list[i].hasMember(CONF_FSM_TRANSITIONS)) {
            for(int32_t j = 0 ; j < fsm_list[i][CONF_FSM_TRANSITIONS].size() ; j++)
            transitions.emplace_back(fsm_list[i][CONF_FSM_TRANSITIONS][j]);
        }

        m_op_sm_component(
            {
                .initial = initial,
                .name = fsm_list[i][CONF_FSM_NAME],
                .mode = fsm_list[i][CONF_FSM_MODE],
                .transitions = transitions
            }
        );
    }

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
