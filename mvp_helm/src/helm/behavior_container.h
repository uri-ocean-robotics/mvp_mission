#pragma once

/*******************************************************************************
 * Behavior Interface
 */
#include "behavior_interface/behavior_base.h"

/*******************************************************************************
 * STD
 */
#include "memory"
#include "string"
#include "vector"
#include "map"

/*******************************************************************************
 * Boost
 */
#include "boost/shared_ptr.hpp"

/*******************************************************************************
 * ROS
 */
#include "pluginlib/class_loader.h"
#include "dictionary.h"

namespace helm {

    class BehaviorContainer {
    public:

        typedef std::shared_ptr<BehaviorContainer> Ptr;

    private:

        /**
         * @brief Behavior name
         * This name will be used as namespace in ROS ecosystem
         */
        std::string m_name;

        /**
         * @brief Class name for the behavior
         * Class name must match with the available plugins in the system
         */
        std::string m_type;

        /**
         * @brief Behavior defined by #m_class_name
         */
        boost::shared_ptr<BehaviorBase> m_behavior;

        /**
         * @brief Pluginlib class loader
         */
        boost::shared_ptr<pluginlib::ClassLoader<BehaviorBase>> m_class_loader;

        /**
         * @brief Finate machine states that the behavior will operate
         *
         * First element in the pair will hold the state name. Second element in
         * the pair will hold the priority of the behavior during the state.
         */
        std::vector<fsm_state_priority_t> m_states;

    public:


        /**
         * @brief Default constructor
         */
        BehaviorContainer() = default;

        /**
         * @brief BehaviorContainer constructor with all required values
         * @param name Name of the behavior
         * @param type Type or the class name of the behavior
         * @param states States in which the behavior will be active
         */
        BehaviorContainer(
            std::string name,
            std::string type,
            decltype(m_states) states
        );

        void initialize();

        ~BehaviorContainer();

        auto get_behavior() -> decltype(m_behavior) { return m_behavior; }

        auto get_states() -> decltype(m_states) { return m_states; }

    };

}