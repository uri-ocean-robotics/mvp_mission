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