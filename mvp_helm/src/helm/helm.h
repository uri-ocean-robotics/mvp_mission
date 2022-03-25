#pragma once

#include "memory"

#include "ros/ros.h"
#include "pluginlib/class_loader.h"

#include "behavior_container.h"
#include "obj.h"
#include "parser.h"
#include "fsm.h"

#include "mvp_control/ControlModes.h"
#include "mvp_control/ControlState.h"

namespace helm {

    class Helm : public HelmObj {
    private:

        /**
         * @brief Controller state
         */
        mvp_control::ControlState m_controller_state;

        /**
         * @brief Controller modes message object
         * Controller modes will be listened from low level controller and
         * stored in this variable
         */
        mvp_control::ControlModes m_controller_modes;

        /**
         * @brief Container for behaviors
         *
         */
        std::vector<std::shared_ptr<BehaviorContainer>> m_behavior_containers;

        /**
         * @brief Gets controller modes from low level controller
         *
         */
        void f_get_controller_modes();

        /**
         * @brief Generate behaviors as parser parses the XML helm configuration
         *
         * @param name Name of the behavior. Later be used as ros namespace
         * @param type Type of the behavior. Also referred as class name
         * @param states State definitions of behaviors. First element is state
         *               name, second element is priority.
         */
        void f_generate_behaviors(
            std::string name,
            std::string type,
            std::map<std::string, int> states);

        /**
         * @brief Generate Finite state machine states
         * @param name Name of the finite state machine
         * @param mode Low level controller of the mode
         */
        void f_generate_fsm_states(
            std::string name,
            std::string mode);

        /**
         * @brief Initiates the plugins
         *
         */
        void f_initialize_behaviors();

        /**
         * @brief Parse object
         *
         */
        Parser::Ptr m_parser;

        FiniteStateMachine::Ptr m_finite_state_machine;

        /**
         * @brief
         *
         */
        void f_iterate();



        /***********************************************************************
         * ROS - Publishers, Subscribers, Services and callbacks
         */

        //! @brief Controller state subscriber
        ros::Subscriber m_sub_controller_current_state;

        //! @brief Controller state request
        ros::Publisher m_pub_controller_desired_state;

        /**
         * @brief Topic callback for state
         * @param msg
         */
        void f_cb_controller_state(const mvp_control::ControlState::ConstPtr& msg);

    public:

        Helm();

        void initialize();

    };
}