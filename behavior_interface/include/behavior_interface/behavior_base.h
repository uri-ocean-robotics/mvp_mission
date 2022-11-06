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

namespace helm
{
    class Helm;

    class BehaviorBase {
    private:

        friend class Helm;

        friend class BehaviorContainer;

        /**
         * @brief Unique name for the behavior.
         * This name will later be used as namespace for ros node handler.
         *
         * @todo A behavior may or may not use a parameter from ROS. However,
         *       this name is still a necessity.
         */
        std::string m_name;

        /**
         * @brief Frequency of the helm
         */
        double m_helm_frequency;

        /**
         * @brief A string holds the active state name
         */
        std::string m_active_state;

        /**
         * @brief A vector holds DOFs active at the moment by controller
         *
         */
        std::vector<int> m_active_dofs;

        /**
         * @brief Behaviors calls this function to request a state change from
         *        MVP-Helm.
         *
         * This function is set during the runtime to map one of the functions
         * from MVP-Helm.
         */
        std::function<bool(const std::string&)> f_change_state;

        void f_set_active_state(const std::string& state) {
            m_active_state = state;
            state_changed(state);
        }

        /**
         * @brief This function is called by the MVP-Helm everytime if a
         *        behavior is active in the given state.
         */
        void f_activate()
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
        void f_disable()
        {
            if(m_activated) {
                disabled();
            }
            m_activated = false;
        }

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
         * @brief Registered state of the of the low level controller
         */
        mvp_msgs::ControlProcess m_process_values;

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
         * @brief
         *
         * @param state_name
        */
        virtual void state_changed(const std::string& state_name) {}

        virtual auto change_state(const std::string& state) -> bool final {
            return f_change_state(state);
        }

        virtual double get_helm_frequency() final { return m_helm_frequency; }

        virtual auto configure_dofs() -> decltype(m_dofs) {return decltype(m_dofs)();};

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

        virtual auto get_name() -> std::string final { return m_name; }

        virtual ~BehaviorBase() = default;

        /**
         * @brief
         * @param set_point
         * @return true if Behavior wants helm to use its result
         * @return false if Behavior doesn't want helm to use its result
         */
        virtual bool request_set_point(mvp_msgs::ControlProcess* set_point) = 0;

        /**
         * @brief Initializer for behaviors
         *
         */
        virtual void initialize() = 0;

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
