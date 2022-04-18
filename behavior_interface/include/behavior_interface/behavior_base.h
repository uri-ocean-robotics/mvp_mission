#pragma once

/*******************************************************************************
 * STD
 */
#include "cstdint"
#include "iostream"
#include "vector"
#include "functional"
#include "memory"

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
         * @brief A vector holds controlled DOFs by behavior
         * Each behavior must present the degrees of freedoms that they want
         * to control. Helm will be controlling this information during
         * execution.
         */
        std::vector<ctrl::DOF::IDX> m_dofs;
        /**
         * @brief A vector holds DOFs active at the moment by controller
         *
         */
        std::vector<ctrl::DOF::IDX> m_active_dofs;

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
         * @brief Frequency of the helm
         */
        double m_helm_frequency;
        /**
         * @brief Construct a new Behavior Base object
         *
         */
        BehaviorBase() = default;

    public:

        /**
         * @brief Trivial generic pointer type
         */
        typedef std::shared_ptr<BehaviorBase> Ptr;

        /**
         * @brief retrieve degrees of freedoms controlled by the behavior
         *
         * @return std::vector<ctrl::DOF::IDX>
         */
        auto get_dofs() -> const decltype(m_dofs)& {
            return m_dofs;
        };

        /**
         * @brief retrieve degrees of freedoms controlled by the behavior
         *
         * @return std::vector<ctrl::DOF::IDX>
         */
        virtual void set_active_dofs(const std::vector<ctrl::DOF::IDX>& dofs) {
            m_active_dofs = dofs;
        };

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

        void set_helm_frequency(const double& f) {
            m_helm_frequency = f;
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
