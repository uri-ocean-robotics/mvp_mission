#pragma once

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <thread>

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/string.hpp"

#include "mvp_msgs/msg/control_modes.hpp"
#include "mvp_msgs/msg/control_process.hpp"
#include "mvp_msgs/msg/setpoint_behavior.hpp"


#include "mvp_msgs/srv/get_control_modes.hpp"
#include "mvp_msgs/srv/get_state.hpp"
#include "mvp_msgs/srv/get_states.hpp"
#include "mvp_msgs/srv/change_state.hpp"

#include "mvp_helm/node_wrapper.h"
#include "mvp_helm/dictionary.h"
#include "mvp_helm/parser.h"
#include "mvp_helm/sm.h"
#include "mvp_helm/utils.h"
#include "mvp_helm/exception.h"
#include "mvp_helm/behavior_container.h"
#include "behavior_interface/behavior_base.h"
#include "robot_localization/srv/to_ll.hpp"
#include "robot_localization/srv/from_ll.hpp"


namespace helm
{

class Helm : public helm::NodeWrapper
{
public:
    /**
    * @brief A constructor for behavior_server::BehaviorServer
    * @param options Additional options to control creation of the node.
    */
    explicit Helm(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
    ~Helm();

    void initialize();

private:
    // friend class BehaviorBase;

    std::thread m_controller_worker;

    /**
      * @brief Controller state
      * This variable holds the state of the low level controller such as
      * position & orientation.
      */
    mvp_msgs::msg::ControlProcess::SharedPtr m_controller_process_values;

    /**
      * @brief Controller modes message object
      * Controller modes will be listened from low level controller and
      * stored in this variable
      */
    mvp_msgs::msg::ControlModes m_controller_modes;


    geographic_msgs::msg::GeoPoint m_datum;
    /**
    * @brief bhv container
    */
    std::vector<std::shared_ptr<BehaviorContainer>> m_behavior_containers;

    /**
      * @brief Gets controller modes from low level controller
      *
      */
    void f_get_controller_modes();

    /**
      * @brief Executes one iteration of helm
      *
      */
    void f_iterate();

    /**
      * @brief Runs the #Helm::f_iterate function defined by
      * #Helm::m_helm_freq.
      */
    void f_helm_loop();

    /***********************************************************************
    * ROS - Publishers, Subscribers, Services and callbacks
    */

    //! @brief Controller state subscriber
    rclcpp::Subscription<mvp_msgs::msg::ControlProcess>::SharedPtr 
        m_sub_controller_process_values;

    //! @brief datum subscriber
    rclcpp::Subscription<geographic_msgs::msg::GeoPoint>::SharedPtr 
        m_datum_sub;

    //! @brief Controller state request
    rclcpp::Publisher<mvp_msgs::msg::ControlProcess>::SharedPtr 
        m_pub_controller_set_point;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr 
        m_helm_state_change_caller;


    rclcpp::Service<mvp_msgs::srv::ChangeState>::SharedPtr 
        m_change_state_srv;

    rclcpp::Service<mvp_msgs::srv::GetStates>::SharedPtr 
        m_get_states_srv;

    rclcpp::Service<mvp_msgs::srv::GetState>::SharedPtr 
        m_get_state_srv;

    //! @brief bhv setpoint state request
    rclcpp::Publisher<mvp_msgs::msg::SetpointBehavior>::SharedPtr 
        m_helm_setpoint_bhv;

    /**
      * @brief Topic callback for state
      * @param msg
      */
    void f_cb_controller_process(
        const mvp_msgs::msg::ControlProcess::SharedPtr msg);


    bool f_cb_change_state(
        const std::shared_ptr<mvp_msgs::srv::ChangeState::Request> req,
        const std::shared_ptr<mvp_msgs::srv::ChangeState::Response> resp);

    bool f_cb_get_state(
        const std::shared_ptr<mvp_msgs::srv::GetState::Request> req,
        const std::shared_ptr<mvp_msgs::srv::GetState::Response> resp);

    bool f_cb_get_states(
        const std::shared_ptr<mvp_msgs::srv::GetStates::Request> req,
        const std::shared_ptr<mvp_msgs::srv::GetStates::Response> resp);

    bool f_change_state(const std::string& name);

    void f_cb_datum(const geographic_msgs::msg::GeoPoint::SharedPtr msg);

    void f_ll2dis(geographic_msgs::msg::GeoPoint ll_point, geometry_msgs::msg::Point::SharedPtr map_point);

    void f_dis2ll(geometry_msgs::msg::Point map_point, geographic_msgs::msg::GeoPoint::SharedPtr ll_point);
 

    /**
      * @brief Initiates the plugins
      *
      */
    void f_initialize_behaviors();

    /**
      *
      * @param behavior_component
      */
    void f_generate_behaviors(
        const behavior_component_t& behavior_component);

    /**
      * @param state
      */
    void f_generate_sm_states(const sm_state_t& state);

    /**
      * @brief Helm frequency in hertz
      */
    double m_helm_freq;

    /**
      * @brief Local link id (i.e., cg_link)
      */        
    std::string m_local_link_id;

    /**
      * @brief Global link id (i.e., world_ned)
      */           
    std::string m_global_link_id;

    /**
      * @brief helm file directory
      */           
    std::string m_helm_config_file;

    /**
      * @brief tf prefix
      */    
    std::string m_tf_prefix;

    /**
      * @brief Global frame_id
      */     
    std::string m_global_frame;

    /**
      * @brief Local frame_id
      */     
    std::string m_local_frame;
      
    /**
      * @brief Parse object
      *
      */
    Parser::Ptr m_parser;  

    /**
      * @brief State Machine object
      */
    StateMachine::Ptr m_state_machine;    
};

} // namespace helm
