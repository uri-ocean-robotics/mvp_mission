#include "sm.h"
#include "algorithm"

using namespace helm;

void StateMachine::append_state(sm_state_t state) {
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

    /**
     * todo: implement and check transition table
     */

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
