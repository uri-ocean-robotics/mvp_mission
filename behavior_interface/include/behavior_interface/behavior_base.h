#pragma once

/*******************************************************************************
 * STD
 */
#include <utility>

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
         * @brief Construct a new Behavior Base object
         *
         */
        BehaviorBase() = default;

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
         *
         */
        bool m_activated = false;

        /**
         * @brief This function is triggered when a behavior gets activated
         * A plugin may or may not override this function.
         */
        virtual void activated() {};

        /**
         * @brief This function is triggered when a behavior gets disabled
         * A plugin may or may not override this function.
         */
        virtual void disabled() {};

        /**
         * @brief
         *
         */
        std::function<bool(const std::string&)> f_change_state;

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
        virtual auto get_dofs() -> const decltype(m_dofs)& final
        {
            return m_dofs;
        }

        /**
         * @brief Set the name of the behavior
         *
         * @param name
         */
        virtual void set_name(const std::string name) final
        {
            m_name = name;
        }

        /**
         * @brief Registers the state of the system to the behavior
         */
        virtual auto register_process_values(
                const mvp_control::ControlProcess& pv) -> void final
        {
            m_process_values = pv;
        }

        virtual auto set_helm_frequency(const double& f) -> void final
        {
            m_helm_frequency = f;
        }

        virtual auto activate() -> void final
        {
            if(!m_activated) {
                activated();
            }
            m_activated = true;
        }

        virtual auto disable() -> auto final
        {
            if(m_activated) {
                disabled();
            }
            m_activated = false;
        }

        virtual auto set_state_change_function(
            decltype(f_change_state) f) -> void final
        {
            f_change_state = std::move(f);
        }

        virtual ~BehaviorBase() = default;

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


        /**
         * @brief retrieve degrees of freedoms controlled by the behavior
         *
         * @return std::vector<ctrl::DOF::IDX>
         */
        virtual void set_active_dofs(const std::vector<ctrl::DOF::IDX>& dofs)
        {
            m_active_dofs = dofs;
        }

    };

}
