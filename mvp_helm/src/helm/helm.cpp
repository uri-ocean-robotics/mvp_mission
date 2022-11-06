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

/*******************************************************************************
 * STD
 */
#include "functional"
#include "utility"

/*******************************************************************************
 * ROS
 */
#include "mvp_msgs/GetControlModes.h"
#include "ros/callback_queue.h"

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


/*******************************************************************************
 * Public methods
 */

Helm::Helm() : HelmObj() {

};

Helm::~Helm() {

    m_sub_controller_process_values.shutdown();

    m_pub_controller_set_point.shutdown();

    m_get_states_srv.shutdown();

    m_get_state_srv.shutdown();

    m_change_state_srv.shutdown();

}

void Helm::initialize() {

    /***************************************************************************
     * Initialize objects
     */

    m_parser.reset(new Parser());

    m_state_machine.reset(new StateMachine());

    /***************************************************************************
     * Parse mission file
     */

    m_parser->set_op_behavior_component(std::bind(
        &Helm::f_generate_behaviors, this, std::placeholders::_1
    ));

    m_parser->set_op_sm_component(std::bind(
        &Helm::f_generate_sm_states, this, std::placeholders::_1
    ));

    m_parser->set_op_helmconf_component(std::bind(
        &Helm::f_configure_helm, this, std::placeholders::_1
    ));

    m_parser->initialize();

    /***************************************************************************
     * Initialize subscriber
     *
     */
    m_sub_controller_process_values = m_nh->subscribe(
        "controller/process/value",
        100,
        &Helm::f_cb_controller_process,
        this
    );

    m_pub_controller_set_point = m_nh->advertise<mvp_msgs::ControlProcess>(
        "controller/process/set_point",
        100
    );

    /***************************************************************************
     * Initialize ros services
     */

    m_change_state_srv = m_pnh->advertiseService(
        "change_state",
        &Helm::f_cb_change_state,
        this);

    m_get_state_srv = m_pnh->advertiseService(
        "get_state",
        &Helm::f_cb_get_state,
        this
    );

    m_get_states_srv = m_pnh->advertiseService(
        "get_states",
        &Helm::f_cb_get_states,
        this
    );

    /***************************************************************************
     * Initialize state machine
     */
    m_state_machine->initialize();

    /***************************************************************************
     * Initialize behavior plugins
     */
    f_initialize_behaviors();


    /***************************************************************************
     * setup connection with low level controller
     */
    f_get_controller_modes();

}

void Helm::run() {

    std::thread helm_loop([this] { f_helm_loop(); });

    ros::spin();

    helm_loop.join();
}

/*******************************************************************************
 * Private methods
 */

void Helm::f_initialize_behaviors() {

    for(const auto& i : m_behavior_containers) {

        i->initialize();

        i->get_behavior()->f_change_state =
            std::bind(&Helm::f_change_state, this, std::placeholders::_1);

        i->get_behavior()->m_helm_frequency = m_helm_freq;
    }

}

void Helm::f_get_controller_modes() {

    auto client = m_nh->serviceClient
        <mvp_msgs::GetControlModes>("controller/get_modes");

    while(!client.waitForExistence(ros::Duration(5))) {
        ROS_WARN_STREAM(
            "Waiting for service: " << client.getService()
        );

        if(ros::isShuttingDown()) {
            return;
        }
    }

    mvp_msgs::GetControlModes srv;

    client.call(srv);

    m_controller_modes.modes = srv.response.modes;

}

void Helm::f_generate_behaviors(const behavior_component_t& component)
{
    /**
     * This function is called each time parser reads a tag defined by
     * #helm::xml::bhvconf::behavior::TAG. This function is defined so that
     * initiating of the objects are done by an #helm::Helm object.
     */

    BehaviorContainer::Ptr b = std::make_shared<BehaviorContainer>(
        component
    );

    m_behavior_containers.emplace_back(b);

}

void Helm::f_generate_sm_states(const sm_state_t& state) {

    m_state_machine->append_state(state);

}

void Helm::f_configure_helm(helm_configuration_t conf) {

    m_helm_freq = conf.frequency;

}

void Helm::f_cb_controller_process(
    const mvp_msgs::ControlProcess::ConstPtr& msg) {
    m_controller_process_values = msg;
}

void Helm::f_iterate() {
    if(m_controller_process_values == nullptr) {
        return;
    }
    /**
     * Acquire state information from finite state machine. Get state name and
     * respective mode to that state.
     */
    auto active_state = m_state_machine->get_active_state();

    auto active_mode = std::find_if(
        m_controller_modes.modes.begin(),
        m_controller_modes.modes.end(),
        [active_state](const mvp_msgs::ControlMode& mode){
            return mode.name == active_state.mode;
        }
    );

    if(active_mode == std::end(m_controller_modes.modes)) {
        ROS_WARN_STREAM_THROTTLE(10,
            "Active mode '" << active_state.mode << "' can not be found in low"
            " level controller configuration! Helm is skipping.");
        return;
    }
    // Type cast the vector
    std::vector<int> dofs;
    std::for_each(
        active_mode->dofs.begin(),
        active_mode->dofs.end(),
        [&](const auto & elem){
            dofs.emplace_back((int)elem);
        }
    );

    /**
     * Create holders for priorities and control inputs.
     */
    std::array<double, 12> dof_ctrl{};
    std::array<int, 12> dof_priority{};

    for(const auto& i : m_behavior_containers) {

        /**
         * Inform the behavior about the active DOFs
         */
        i->get_behavior()->m_active_dofs = dofs;

        /**
         * Update the system state inside behavior
         */

        i->get_behavior()->m_process_values = *m_controller_process_values;

        /*
         * Check if behavior should be active in active state
         */
        bool pass = false;
        if(!i->get_opts().states.count(active_state.name)) {
            i->get_behavior()->f_disable();
            pass = true;
        } else {
            i->get_behavior()->f_activate();
        }

        /**
         * Request control command from the behavior
         * TODO: request set point must be multithreaded with timeout
         */
        mvp_msgs::ControlProcess set_point;
        if(!i->get_behavior()->request_set_point(&set_point)) {
            // todo: do something about dysfunctional behavior
            continue;
        }

        /**
         * We still want to request an action from a behavior. A behavior may
         * implement its core functionality inside
         * #BevahiorBase::request_set_point. That functionality might be related
         * with safety. But at the same time, we want to tell the behavior
         * whether or not is active.
         */
        if(pass) {
            continue;
        }

        /**
         * Get the priority of the behavior in given state
         */
        auto priority = i->get_opts().states[active_state.name];

        /**
         * A behavior might only check for system state and take other actions
         * rather than communicating with the low level controller. Such as
         * drop weight, or cut all the power to the motors etc. In this case,
         * DOFs might not be defined in the behavior. A behavior can do whatever
         * it wants in #BehaviorBase::request_set_point method.
         */
        if(i->get_behavior()->get_dofs().empty()) {
            continue;
        }

        /**
         * Turn requested control command into an array so that it can be
         * processed by iterating degree of freedoms.
         */
        auto bhv_control_array =
            utils::control_process_to_array(set_point);

        for(const auto& dof : i->get_behavior()->get_dofs()) {
            /**
             * This is where the magic happens
             */
            if(priority > dof_priority[dof]) {
                dof_ctrl[dof] = bhv_control_array[dof];
                dof_priority[dof] = priority;
            }
        }

    }

    /**
     * Push commands to low level controller
     */
    auto msg = utils::array_to_control_process_msg(dof_ctrl);

    msg.control_mode = active_state.mode;
    msg.header.stamp = ros::Time::now();
    m_pub_controller_set_point.publish(msg);

}

void Helm::f_helm_loop() {

    ros::Rate r(m_helm_freq);
    while(ros::ok() && !ros::isShuttingDown()) {
        f_iterate();
        r.sleep();
    }

}

bool Helm::f_cb_change_state(mvp_msgs::ChangeState::Request &req,
                             mvp_msgs::ChangeState::Response &resp) {

    if(f_change_state(req.state)) {

        sm_state_t s;
        m_state_machine->get_state(req.state, &s);

        resp.state.name = s.name;
        resp.state.mode = s.mode;
        resp.state.transitions = s.transitions;
        resp.status = true;

        return true;
    }

    resp.state.name = m_state_machine->get_active_state().name;
    resp.state.mode = m_state_machine->get_active_state().mode;
    resp.state.transitions = m_state_machine->get_active_state().transitions;
    resp.status = false;

    return true;
}

bool Helm::f_cb_get_state(mvp_msgs::GetState::Request &req,
                          mvp_msgs::GetState::Response &resp) {

    if(req.name.empty()) {

        resp.state.name = m_state_machine->get_active_state().name;
        resp.state.mode = m_state_machine->get_active_state().mode;
        resp.state.transitions =
            m_state_machine->get_active_state().transitions;

        return true;
    }

    sm_state_t s;
    if (m_state_machine->get_state(req.name, &s)) {

        resp.state.name = s.name;
        resp.state.mode = s.mode;
        resp.state.transitions = s.transitions;

        return true;
    }
    throw HelmException("no state found with name " + req.name);
}


bool Helm::f_cb_get_states(mvp_msgs::GetStates::Request &req,
                           mvp_msgs::GetStates::Response &resp) {
    for(const auto& i : m_state_machine->get_states()) {
        mvp_msgs::HelmState s;
        s.mode = i.mode;
        s.name = i.name;
        s.transitions = i.transitions;

        resp.states.emplace_back(s);
    }

    return true;
}

bool Helm::f_change_state(const std::string& name) {
    return m_state_machine->translate_to(name);
}