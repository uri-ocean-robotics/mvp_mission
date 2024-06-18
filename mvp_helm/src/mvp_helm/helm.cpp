#include <memory>
#include <string>
#include <vector>
#include <utility>
#include "mvp_helm/helm.h"

using namespace std::chrono_literals;

namespace helm 
{

Helm::Helm(const rclcpp::NodeOptions & options)
: NodeWrapper("mvp_helm", "", options)
{
    RCLCPP_INFO(get_logger(), "helm constructor");

    // Load Helm parameters
    this->declare_parameter(CONF_HELM_FREQ, 10.0);
    this->get_parameter(CONF_HELM_FREQ, m_helm_freq);

    this->declare_parameter(CONF_HELM_GLOBAL, "world_ned");
    this->get_parameter(CONF_HELM_GLOBAL, m_global_link_id);
    std::cout<<"global_link: "<<m_global_link_id<<std::endl;

    this->declare_parameter(CONF_HELM_LOCAL, "cg_link");
    this->get_parameter(CONF_HELM_LOCAL, m_local_link_id);
    std::cout<<"local_link: "<<m_local_link_id<<std::endl;
    

    this->declare_parameter(CONF_HELM_FILE, "helm.yaml");
    this->get_parameter(CONF_HELM_FILE, m_helm_config_file);

    std::string tf_prefix;
    this->declare_parameter(CONF_TF_PREFIX, "");
    this->get_parameter(CONF_TF_PREFIX, tf_prefix);
    m_tf_prefix = tf_prefix.empty() ? "" : tf_prefix + "/";

    m_global_frame = m_tf_prefix + m_global_link_id;
    m_local_frame = m_tf_prefix+ m_local_link_id;
}

Helm::~Helm()
{
    rclcpp::shutdown();
}

void Helm::initialize() {

    /***************************************************************************
     * Initialize objects
     */
    m_parser.reset(new Parser(m_helm_config_file));

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

    m_parser->initialize();

    /***************************************************************************
     * Initialize subscriber
     *
     */
    m_sub_controller_process_values = this->create_subscription<mvp_msgs::msg::ControlProcess>(
        "controller/process/value",
        100,
        std::bind(&Helm::f_cb_controller_process, this, std::placeholders::_1)
    );

    m_pub_controller_set_point = this->create_publisher<mvp_msgs::msg::ControlProcess>(
        "controller/process/set_point",
        100
    );

    m_helm_state_change_caller = this->create_publisher<std_msgs::msg::String>(
        "helm/change_state_caller",
        100
    );
    

    m_helm_setpoint_bhv = this->create_publisher<mvp_msgs::msg::SetpointBehavior>(
        "helm/setpoint_bhv",
        100
    );

    /***************************************************************************
     * Initialize ros services
     */

    m_change_state_srv = this->create_service<mvp_msgs::srv::ChangeState>(
        "~/change_state",
        std::bind(&Helm::f_cb_change_state, this, std::placeholders::_1, std::placeholders::_2)
    );

    m_get_state_srv = this->create_service<mvp_msgs::srv::GetState>(
        "~/get_state",
        std::bind(&Helm::f_cb_get_state, this, std::placeholders::_1, std::placeholders::_2)
    );

    m_get_states_srv = this->create_service<mvp_msgs::srv::GetStates>(
        "~/get_states",
        std::bind(&Helm::f_cb_get_states, this, std::placeholders::_1, std::placeholders::_2)
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

    /***************************************************************************
     * setup behavior management thread
     */
    m_controller_worker = std::thread([this] { f_helm_loop(); });
    m_controller_worker.detach();    
}

void Helm::f_initialize_behaviors() {

    auto node = shared_from_this();

    for(const auto& i : m_behavior_containers) {

        i->initialize(node);

        i->get_behavior()->f_change_state =
            std::bind(&Helm::f_change_state, this, std::placeholders::_1);

        i->get_behavior()->m_helm_frequency = m_helm_freq;

        i->get_behavior()->m_local_link = m_local_frame;

        i->get_behavior()->m_global_link = m_global_frame;
    }
}

void Helm::f_get_controller_modes() {

    rclcpp::Client<mvp_msgs::srv::GetControlModes>::SharedPtr m_get_ctrl_mode_client = 
        this->create_client<mvp_msgs::srv::GetControlModes>("controller/get_modes");

    while (!m_get_ctrl_mode_client->wait_for_service(5s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    // mvp_msgs::msg::ControlMode resp;
    auto request = std::make_shared<mvp_msgs::srv::GetControlModes::Request>();
    
    auto resp = m_get_ctrl_mode_client->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), resp) 
        != rclcpp::FutureReturnCode::SUCCESS){

        RCLCPP_ERROR(this->get_logger(), "Failed to call service");
    }

    m_controller_modes.modes = resp.get()->modes;
}

void Helm::f_generate_behaviors(const behavior_component_t& component) {

    RCLCPP_INFO(get_logger(), "generate behaviors: name=%s", component.name.c_str());

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

    RCLCPP_INFO(get_logger(), "Test: generate state machines: name=%s", state.name.c_str());

    m_state_machine->append_state(state);
}

void Helm::f_cb_controller_process(const mvp_msgs::msg::ControlProcess::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Test: receive controller process: control_mode=%s", msg->control_mode.c_str());

    m_controller_process_values = msg;
}

bool Helm::f_cb_change_state(const std::shared_ptr<mvp_msgs::srv::ChangeState::Request> req,
                             const std::shared_ptr<mvp_msgs::srv::ChangeState::Response> resp) {
    RCLCPP_INFO(this->get_logger(), "change state");

    if(f_change_state(req->state)) {

        sm_state_t s;
        m_state_machine->get_state(req->state, &s);

        resp->state.name = s.name;
        resp->state.mode = s.control_mode;
        resp->state.transitions = s.transitions;
        resp->status = true;
        std_msgs::msg::String caller;
        caller.data=req->caller;
        m_helm_state_change_caller->publish(caller);

        return true;
    }

    resp->state.name = m_state_machine->get_active_state().name;
    resp->state.mode = m_state_machine->get_active_state().control_mode;
    resp->state.transitions = m_state_machine->get_active_state().transitions;
    resp->status = false;

    return true;
}

bool Helm::f_cb_get_state(const std::shared_ptr<mvp_msgs::srv::GetState::Request> req,
                          const std::shared_ptr<mvp_msgs::srv::GetState::Response> resp) {

    RCLCPP_INFO(this->get_logger(), "get state");

    if(req->name.empty()) {

        resp->state.name = m_state_machine->get_active_state().name;
        resp->state.mode = m_state_machine->get_active_state().control_mode;
        resp->state.transitions =
            m_state_machine->get_active_state().transitions;

        return true;
    }

    sm_state_t s;
    if (m_state_machine->get_state(req->name, &s)) {

        resp->state.name = s.name;
        resp->state.mode = s.control_mode;
        resp->state.transitions = s.transitions;

        return true;
    }
    throw HelmException("no state found with name " + req->name);
}

bool Helm::f_cb_get_states(const std::shared_ptr<mvp_msgs::srv::GetStates::Request> req,
                           const std::shared_ptr<mvp_msgs::srv::GetStates::Response> resp) {
    RCLCPP_INFO(this->get_logger(), "get states");

    for(const auto& i : m_state_machine->get_states()) {
        mvp_msgs::msg::HelmState s;
        s.mode = i.control_mode;
        s.name = i.name;
        s.transitions = i.transitions;

        resp->states.emplace_back(s);
    }

    return true;
}

bool Helm::f_change_state(const std::string& name) {
    RCLCPP_INFO(this->get_logger(), "state changed to: '%s'", name.c_str());

    return m_state_machine->translate_to(name);
}

void Helm::f_helm_loop() {

  rclcpp::Rate r(m_helm_freq);

  while(rclcpp::ok()) {
    // RCLCPP_INFO(get_logger(), "hi from helm loop");
    f_iterate();
    r.sleep();
  }
}

void Helm::f_iterate() {
    auto steady_clock = rclcpp::Clock();
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
        [active_state](const mvp_msgs::msg::ControlMode& mode){
            return mode.name == active_state.control_mode;
        }
    );

    if(active_mode == std::end(m_controller_modes.modes)) {
        RCLCPP_WARN_STREAM_THROTTLE(
            this->get_logger(), steady_clock, 10,
            "Active mode '" << active_state.control_mode << "' can not be found in low"
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
    mvp_msgs::msg::SetpointBehavior m_set_point_bhv;

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
        mvp_msgs::msg::ControlProcess set_point;
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
            utils::control_process_to_array(&set_point);

        for(const auto& dof : i->get_behavior()->get_dofs()) {
            /**
             * This is where the magic happens
             */
            if(priority > dof_priority[dof]) {
                dof_ctrl[dof] = bhv_control_array[dof];
                dof_priority[dof] = priority;
                m_set_point_bhv.behavior[dof] = i->get_behavior()->get_name();
            }
        }
    }

    /**
     * Push commands to low level controller
     */
    auto msg = utils::array_to_control_process_msg(dof_ctrl);

    

    m_set_point_bhv.control_mode = active_state.control_mode;
    m_set_point_bhv.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    m_set_point_bhv.header.frame_id = m_global_frame;
    m_helm_setpoint_bhv->publish(m_set_point_bhv);
    //only publish the set  point when there is a bhv setting the set point
    bool all_empty = std::all_of(m_set_point_bhv.behavior.begin(), m_set_point_bhv.behavior.end(), [](const std::string& s) {
        return s.empty();
    });
    if(all_empty == false) 
    {
        // makeup the message
        msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        msg.header.frame_id = m_global_frame;
        msg.control_mode = active_state.control_mode;
        msg.child_frame_id = m_local_frame;
        m_pub_controller_set_point->publish(msg);    
    }

}

} // namespace helm

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(helm::Helm)
