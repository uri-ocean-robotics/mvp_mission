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


#include "mvp_helm/parser.h"
#include "mvp_helm/exception.h"
#include "mvp_helm/dictionary.h"

// #include "mvp_helm/behavior_container.h"
#include "utility"

#include "unistd.h"
#include "cstdlib"
#include "cstdio"
#include "sys/wait.h"
#include "yaml-cpp/yaml.h"

using namespace helm;

Parser::Parser(std::string config_file): m_helm_config_file(config_file)
{}

void Parser::initialize() {

    f_parse_sm_components();

    f_parse_bhv_components();
}

void Parser::f_parse_sm_components()
{
    RCLCPP_INFO(m_logger, "Parsing Finite State Machines configuration from file:%s", m_helm_config_file.c_str());

    //parse state machine
    YAML::Node map = YAML::LoadFile(m_helm_config_file);

    //if the file has FSM?
    if(map[CONF_FSM])
    {
        for(YAML::const_iterator it=map[CONF_FSM].begin();it != map[CONF_FSM].end(); ++it) 
        {
            std::string state_name = it->first.as<std::string>();       // <- states

            //check initial
            bool initial=false;
            if(map[CONF_FSM][state_name]["initial"])
            {
                initial = map[CONF_FSM][state_name]["initial"].as<bool>();
            }

            //check control mode
            std::string control_mode; 
            control_mode = map[CONF_FSM][state_name][CONF_FSM_MODE].as<std::string>();

            //transitions
            std::vector<std::string> transitions;
            transitions = map[CONF_FSM][state_name][CONF_FSM_TRANSITIONS].as<std::vector<std::string>>();

            m_op_sm_component({initial, state_name, control_mode, transitions});

            // RCLCPP_INFO(m_logger, "initial=%d, state_name =%s, control_mode=%s", initial, state_name.c_str(), control_mode.c_str());
            // RCLCPP_INFO(m_logger, "transitions");
            // for(const auto& v : transitions){
            //     printf("v:%s \n", v.c_str());
            //     RCLCPP_INFO(m_logger, "v:%s", v.c_str());
            // }
        }
    }  
    else
    {
        // warning
        RCLCPP_INFO(m_logger, "!!No finite_state_machine configured in the file: %s", m_helm_config_file.c_str());
    }

    RCLCPP_INFO(m_logger, "Parsing FSM success");
}

void Parser::f_parse_bhv_components()
{
    //parse state machine
    YAML::Node map = YAML::LoadFile(m_helm_config_file);
    RCLCPP_INFO(m_logger, "Parsing behavior configuration from file %s", m_helm_config_file.c_str());

    //if the file has FSM?
    if(map[CONF_BHV])
    {   
        for(YAML::const_iterator it=map[CONF_BHV].begin(); it != map[CONF_BHV].end(); ++it) 
        {
            std::string bhv_name = it->first.as<std::string>();       // <- behavior name
            std::string bhv_plugin = map[CONF_BHV][bhv_name][CONF_BHV_PLUGIN].as<std::string>();
            // RCLCPP_INFO(m_logger, "bhv_name=%s, bhv_plug=%s", bhv_name.c_str(), bhv_plugin.c_str());

            //iterating through state and priorities
            std::map<std::string, int> bhv_states;
            for(YAML::const_iterator ii=map[CONF_BHV][bhv_name][CONF_BHV_STATES_PRIORITY].begin(); 
                                     ii != map[CONF_BHV][bhv_name][CONF_BHV_STATES_PRIORITY].end(); 
                                     ++ii) 
            {    
                std::string state_name = ii->first.as<std::string>();
                // int priority = map[CONF_BHV][bhv_name][CONF_BHV_STATES_PRIORITY][state_name].as<int>();
                bhv_states[state_name] = map[CONF_BHV][bhv_name][CONF_BHV_STATES_PRIORITY][state_name].as<int>();

                // RCLCPP_INFO(m_logger, "bhv_state=%s, priority=%d", state_name.c_str(), bhv_states[state_name]);
            }
           
            m_op_behavior_component({bhv_name, bhv_plugin, bhv_states});
        }
        
    }
    else
    {
        // warning
        RCLCPP_INFO(m_logger, "!!No behavior configured in the file: %s", m_helm_config_file.c_str());
    }

    RCLCPP_INFO(m_logger, "Parsing behavior success");
}

void Parser::set_op_behavior_component(decltype(m_op_behavior_component) f) {
    m_op_behavior_component = std::move(f);
}

void Parser::set_op_sm_component(decltype(m_op_sm_component) f) {
    m_op_sm_component = std::move(f);
}
