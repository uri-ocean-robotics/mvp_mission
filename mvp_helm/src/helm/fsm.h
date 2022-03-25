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

    class FiniteStateMachine {
    private:

        std::vector<fsm_state_t> m_states;

        fsm_state_t m_active_state;

    public:

        typedef std::shared_ptr<FiniteStateMachine> Ptr;

        FiniteStateMachine() = default;

        void append_state(fsm_state_t state);

        auto translate_to(const std::string& state_name) -> bool;

        auto get_active_state() -> decltype(m_active_state);

    };
}