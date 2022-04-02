#pragma once

/*******************************************************************************
 * STD
 */
#include "cstdint"
#include "iostream"
#include "vector"
#include "functional"

/*******************************************************************************
 * MVP
 */
#include "mvp_control/dictionary.h"
#include "mvp_control/ControlProcess.h"

/*******************************************************************************
 * ROS
 */
#include "ros/ros.h"

namespace helm
{

    class BehaviorBase {
    protected:

        /**
         * @brief A vector holds controlled degrees of freedom
         * Each behavior must present the degrees of freedoms that they want
         * to control. Helm will be controlling this information during
         * execution.
         */
        std::vector<ctrl::DOF::IDX> m_dofs;

        /**
         * @brief Unique name for the behavior.
         * This name will later be used as namespace for ros node handler.
         */
        std::string m_name;

        /**
         * @brief Registered state of the of the low level controller
         */
        mvp_control::ControlProcess m_process_values;

        /**
         * @brief Construct a new Behavior Base object
         *
         */
        BehaviorBase() = default;

    public:

        typedef std::shared_ptr<BehaviorBase> Ptr;

        /**
         * @brief retrieve degrees of freedoms controlled by the behavior
         *
         * @return std::vector<ctrl::DOF::IDX>
         */
        virtual std::vector<ctrl::DOF::IDX> get_dofs() { return m_dofs; };

        /**
         * @brief Set the name of the behavior
         *
         * @param name
         */
        void set_name(const std::string name) { m_name = name; }

        /**
         * @brief Registers the state of the system to the behavior
         */
        void register_process_values(const mvp_control::ControlProcess& pv) {
            m_process_values = pv;
        }

        /**
         * @brief
         * @param set_point
         * @return true Success
         * @return false Failure
         */
        virtual bool request_set_point(
            mvp_control::ControlProcess* set_point) = 0;

        /**
         * @brief Initializer for behaviors
         *
         */
        virtual void initialize() = 0;

        virtual ~BehaviorBase() = default;

    };

}
