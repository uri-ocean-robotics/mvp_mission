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


#include "sm.h"
#include "algorithm"

using namespace helm;

void StateMachine::append_state(const sm_state_t& state) {
    m_states.emplace_back(state);
}

auto StateMachine::translate_to(const std::string& state_name) -> bool {

    auto state_idx = std::find_if(
        m_states.begin(),
        m_states.end(),
        [state_name](const sm_state_t& val) {
            return val.name == state_name;
        }
    );

    auto transition_idx = std::find_if(
        m_active_state.transitions.begin(),
        m_active_state.transitions.end(),
        [state_name](const auto& val) {
            return val == state_name;
        }
    );

    if(transition_idx == std::end(m_active_state.transitions)) {
        // illegal state transition requested
        return false;
    }

    if(state_idx != std::end(m_states)) {
        m_active_state = sm_state_t (*state_idx);
        return true;
    } else {
        return false;
    }

}

auto StateMachine::get_active_state() -> decltype(m_active_state) {
    return m_active_state;
}

void StateMachine::initialize() {

    auto initial_state = std::find_if(
        m_states.begin(),
        m_states.end(),
        [](const sm_state_t& t)
        {
            return t.initial;
        }
    );

    if(initial_state != m_states.end()) {
        m_active_state = sm_state_t(*initial_state);
    } else {
        m_active_state = m_states.front();
    }

}

auto StateMachine::get_state(const std::string &name,
                             sm_state_t *state) -> bool {

    auto state_idx = std::find_if(
        m_states.begin(),
        m_states.end(),
        [name](const sm_state_t& val) {
            return val.name == name;
        }
    );

    if(state_idx != m_states.end()) {
        *state = *state_idx;
        return true;
    }

    return false;
}
