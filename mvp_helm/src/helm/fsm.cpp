#include "fsm.h"
#include "algorithm"

using namespace helm;

void FiniteStateMachine::append_state(fsm_state_t state) {
    m_states.emplace_back(state);
}



auto FiniteStateMachine::translate_to(const std::string& state_name) -> bool {

    auto state_idx = std::find_if(
        m_states.begin(),
        m_states.end(),
        [state_name](const fsm_state_t& val) {
            return val.name == state_name;
        }
    );

    /**
     * todo: implement and check transition table
     */

    if(state_idx != std::end(m_states)) {
        m_active_state = fsm_state_t (*state_idx);
        return true;
    } else {
        return false;
    }

}

auto FiniteStateMachine::get_active_state() -> decltype(m_active_state) {
    return m_active_state;
}
