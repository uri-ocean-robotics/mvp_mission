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

#pragma once

/*******************************************************************************
 * STD
 */

#include "cstdint"
#include "iostream"
#include "vector"
#include "functional"
#include "memory"
#include "exception"
#include "utility"

/*******************************************************************************
 * MVP
 */
#include "mvp_msgs/ControlProcess.h"
#include "mvp_msgs/ControlMode.h"
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
        std::vector<int> m_dofs;

        /**
         * @brief A vector holds DOFs active at the moment by controller
         *
         */
        std::vector<int> m_active_dofs;

        /**
         * @brief Unique name for the behavior.
         * This name will later be used as namespace for ros node handler.
         *
         * @todo A behavior may or may not use a parameter from ROS. However,
         *       this name is still a necessity.
         */
        std::string m_name;

        /**
         * @brief Registered state of the of the low level controller
         */
        mvp_msgs::ControlProcess m_process_values;

        /**
         * @brief Frequency of the helm
         */
        double m_helm_frequency;

        /**
         * @brief a member variable to hold activity state of the behavior
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
         * @brief Behaviors calls this function to request a state change from
         *        MVP-Helm.
         *
         * This function is set during the runtime to map one of the functions
         * from MVP-Helm.
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
                const mvp_msgs::ControlProcess& pv) -> void final
        {
            m_process_values = pv;
        }

        /**
         * @brief This function is called by the MVP-Helm to inform the behavior
         *        its frequency. This is useful for behaviors that are time
         *        depended.
         *
         * @todo Consider hiding it from the derived classes. No one has to know
         *       about this method.
         */
        virtual auto set_helm_frequency(const double& f) -> void final
        {
            m_helm_frequency = f;
        }

        /**
         * @brief This function is called by the MVP-Helm everytime if a
         *        behavior is active in the given state.
         */
        virtual void activate() final
        {
            if(!m_activated) {
                activated();
            }
            m_activated = true;
        }

        /**
         * @brief This function is called by the MVP-Helm everytime if a
         *        behavior is *not* active in the given state.
         */
        virtual void disable() final
        {
            if(m_activated) {
                disabled();
            }
            m_activated = false;
        }

        /**
         * @brief This funciton is for MVP-Helm to map its own function to the
         *        behavior. When a behavior calls this function, it directly
         *        calls the MVP-Helm function.
         * @todo Consider hiding from the derived class. No one needs to know
         *       about this method.
         */
        virtual auto set_state_change_function(
            decltype(f_change_state) f) -> void final
        {
            f_change_state = std::move(f);
        }

        virtual ~BehaviorBase() = default;

        /**
         * @brief
         * @param set_point
         * @return true if Behavior wants helm to use its result
         * @return false if Behavior doesn't want helm to use its result
         */
        virtual bool request_set_point(
            mvp_msgs::ControlProcess* set_point) = 0;

        /**
         * @brief Initializer for behaviors
         *
         */
        virtual void initialize() = 0;


        /**
         * @brief retrieve degrees of freedoms controlled by the behavior
         *
         * @todo Rename this function. Creates ambiguity.
         * @return std::vector<int>
         */
        virtual void set_active_dofs(const std::vector<int>& dofs)
        {
            m_active_dofs = dofs;
        }

    };

    class BehaviorException : public std::exception {
    protected:
        /** Error message.
         */
        std::runtime_error M;
    public:
        /** Constructor (C strings).
         *  @param message C-style string error message.
         *                 The string contents are copied upon construction.
         *                 Hence, responsibility for deleting the char* lies
         *                 with the caller.
         */
        explicit BehaviorException(const char* message)
            : M(message) {}

        /** Constructor (C++ STL strings).
         *  @param message The error message.
         */
        explicit BehaviorException(const std::string&  message)
            : M(message) {}

        /** Destructor.
         * Virtual to allow for subclassing.
         */
        ~BehaviorException() noexcept override = default;

        /** Returns a pointer to the (constant) error description.
         *  @return A pointer to a const char*. The underlying memory
         *          is in posession of the Exception object. Callers must
         *          not attempt to free the memory.
         */
        const char* what() const noexcept override {
        return M.what();
        }

    };
}
