
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

#include <utility>
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
        ctrl::TOPIC_CONTROL_PROCESS_VALUE,
        100,
        &Helm::f_cb_controller_state,
        this
    );

    m_pub_controller_set_point = m_nh->advertise<mvp_control::ControlProcess>(
        ctrl::TOPIC_CONTROL_PROCESS_SET_POINT,
        100
    );

    /***************************************************************************
     * Initialize behavior plugins
     */
    f_initialize_behaviors();

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
    }

}

void Helm::f_get_controller_modes() {

    auto client = m_pnh->serviceClient
        <mvp_control::GetControlModes>(ctrl::SERVICE_GET_CONTROL_MODES);

    while(!client.waitForExistence(ros::Duration(5))) {
        ROS_WARN_STREAM(
            "Service " << client.getService() << " can not be found!"
        );
    }

    mvp_control::GetControlModes srv;
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

void Helm::f_generate_sm_states(sm_state_t state) {

    m_state_machine->append_state(std::move(state));

}

void Helm::f_configure_helm(helm_configuration_t conf) {

    m_helm_freq = conf.frequency;

}

void Helm::f_cb_controller_state(
    const mvp_control::ControlProcess::ConstPtr& msg) {
    m_controller_process_values = *msg;
}

void Helm::f_iterate() {

    /**
     * Acquire state information from finite state machine. Get state name and
     * respective mode to that state.
     */
    auto active_state = m_state_machine->get_active_state();

    /**
     * Create holders for priorities and control inputs.
     */
    std::array<double, ctrl::CONTROLLABLE_DOF_LENGTH> dof_ctrl{};
    std::array<int, ctrl::CONTROLLABLE_DOF_LENGTH> dof_priority{};

    for(const auto& i : m_behavior_containers) {

        /**
         * Update the system state inside behavior
         */
        i->get_behavior()->register_process_values(m_controller_process_values);

        /**
         * Request control command from the behavior
         */
        mvp_control::ControlProcess set_point;
        if(!i->get_behavior()->request_set_point(&set_point)) {
            // todo: do something about dysfunctional behavior
            continue;
        }

        if(!i->get_opts().states.count(active_state.name)) {
            continue;
        }

        auto priority = i->get_opts().states[active_state.name];

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
    while(ros::ok()) {
        f_iterate();
        r.sleep();
    }

}
