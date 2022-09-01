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


#pragma once

/*******************************************************************************
 * STD
 */
#include "string"
#include "map"
#include "memory"
#include "cinttypes"
#include "vector"

/*******************************************************************************
 * Helm
 */
#include "dictionary.h"

namespace helm {

    class StateMachine {
    private:

        std::vector<sm_state_t> m_states;

        sm_state_t m_active_state;

    public:

        typedef std::shared_ptr<StateMachine> Ptr;

        StateMachine() = default;

        void initialize();

        void append_state(const sm_state_t& state);

        auto translate_to(const std::string& state_name) -> bool;

        auto get_active_state() -> decltype(m_active_state);

        auto get_state(const std::string &name, sm_state_t *state) -> bool;

        auto get_states() -> decltype(m_states) { return m_states; }

    };
}