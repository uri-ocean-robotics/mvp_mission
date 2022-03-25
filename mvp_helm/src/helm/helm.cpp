
/*******************************************************************************
 * STD
 */
#include "functional"
#include "utility"

/*******************************************************************************
 * ROS
 */
#include "mvp_control/GetControlModes.h"
#include "mvp_control/dictionary.h"

/*******************************************************************************
 * Helm
 */
#include "helm.h"
#include "behavior_container.h"
#include "dictionary.h"
#include "utils.h"
#include "exception.h"

/*******************************************************************************
 * namespaces
 */
using namespace helm;

/*******************************************************************************
 * Implementations
 */

Helm::Helm() : HelmObj() {

};

void Helm::initialize() {

    /***************************************************************************
     * Initialize objects
     */

    m_parser.reset(new Parser());

    m_finite_state_machine.reset(new FiniteStateMachine());

    /***************************************************************************
     * Parse mission file
     */

    m_parser->set_op_behavior_component(std::bind(
        &Helm::f_generate_behaviors, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
    ));

    m_parser->set_op_fsm_component(std::bind(
        &Helm::f_generate_fsm_states, this,
        std::placeholders::_1, std::placeholders::_2
    ));
    m_parser->initialize();

    /***************************************************************************
     * Initialize subscriber
     *
     */
    m_sub_controller_current_state = m_nh->subscribe(
        ctrl::TOPIC_CONTROL_STATE_CURRENT,
        100,
        &Helm::f_cb_controller_state,
        this
    );

    m_pub_controller_desired_state = m_nh->advertise<mvp_control::ControlState>(
        ctrl::TOPIC_CONTROL_STATE_DESIRED,
        100
    );

    /***************************************************************************
     * Initialize behavior plugins
     */
    f_initialize_behaviors();

}

void Helm::f_initialize_behaviors() {

    for(const auto& i : m_behavior_containers) {
        i->initialize();
    }

}

void Helm::f_get_controller_modes() {

    auto client = m_pnh->serviceClient<mvp_control::GetControlModes>(
        ctrl::SERVICE_GET_CONTROL_MODES
    );

    while(!client.waitForExistence(ros::Duration(5))) {
        ROS_WARN_STREAM(
            "Service " << client.getService() << " can not be found!"
        );
    }

    mvp_control::GetControlModes srv;
    client.call(srv);
    m_controller_modes.modes = srv.response.modes;

}

void Helm::f_generate_behaviors(
    std::string name,
    std::string type,
    std::map<std::string, int> states)
{

    std::vector<fsm_state_priority_t> fsm_states;

    for(const auto& i : states) {
        fsm_states.emplace_back(fsm_state_priority_t {
            i.first,
            i.second
        });
    }

    BehaviorContainer::Ptr b = std::make_shared<BehaviorContainer>(
        name, type, fsm_states);

    m_behavior_containers.emplace_back(b);

}

void Helm::f_generate_fsm_states(std::string name, std::string mode) {

    fsm_state_t s{
        std::move(name), std::move(mode)
    };

    m_finite_state_machine->append_state(s);

}

void Helm::f_cb_controller_state(
    const mvp_control::ControlState::ConstPtr& msg) {
    m_controller_state = *msg;
}

void Helm::f_iterate() {

    /**
     * Acquire state information from finite state machine. Get state name and
     * respective mode to that state.
     */
    auto active_state = m_finite_state_machine->get_active_state();

    /**
     * Create holders for priorities and control inputs.
     */
    std::array<double, ctrl::STATE_DOF_SIZE> dof_ctrl{};
    std::array<int, ctrl::STATE_DOF_SIZE> dof_priority{};

    for(const auto& i : m_behavior_containers) {

        // find the behavior
        auto bhv_state = std::find_if(
            i->get_states().begin(),
            i->get_states().end(),
            [active_state](const fsm_state_priority_t& s) {
                return active_state.name == s.name;
            }
        );

        /**
         * Update the system state inside behavior
         */
        i->get_behavior()->register_state(m_controller_state);

        /**
         * If the behavior is not defined for the state we are in not, continue.
         */
        if(bhv_state == std::end(i->get_states())) {
            continue;
        }


        /**
         * Request control command from the behavior
         */
        mvp_control::ControlState control_command;
        if(!i->get_behavior()->request_control(&control_command)) {
            // todo: do something about dysfunctional behavior
            continue;
        }

        /**
         * Turn requested control command into an array so that it can be
         * processed by iterating degree of freedoms.
         */
        auto bhv_control_array =
            utils::control_state_msg_to_array(control_command);

        for(const auto& dof : i->get_behavior()->get_dofs()) {
            if(bhv_state->priority > dof_priority[dof]) {
                dof_ctrl[dof] = bhv_control_array[dof];
                dof_priority[dof] = bhv_state->priority;
            }
        }

    }

    /**
     * Push commands to low level controller
     */
    auto msg = utils::array_to_control_state_msg(dof_ctrl);
    msg.control_mode = active_state.mode;
    msg.header.stamp = ros::Time::now();

    m_pub_controller_desired_state.publish(msg);

}
