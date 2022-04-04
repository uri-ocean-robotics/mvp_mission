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

        //! @brief Trivial constructor
        typedef std::shared_ptr<BehaviorContainer> Ptr;

    private:

        behavior_component_t m_opts;

        /**
         * @brief Behavior defined by #m_class_name
         */
        boost::shared_ptr<BehaviorBase> m_behavior;

        /**
         * @brief Pluginlib class loader
         */
        std::shared_ptr<pluginlib::ClassLoader<BehaviorBase>> m_class_loader;


    public:

        /**
         * @brief Default constructor
         */
        BehaviorContainer() = default;

        explicit BehaviorContainer(behavior_component_t opts);

        ~BehaviorContainer();

        void initialize();

        auto get_behavior() -> decltype(m_behavior) { return m_behavior; }

        auto get_opts() -> decltype(m_opts) { return m_opts; }

    };

}