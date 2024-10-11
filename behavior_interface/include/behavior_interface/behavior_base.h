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
#include <cstdint>
#include <iostream>
#include <vector>
#include <functional>
#include <memory>
#include <exception>
#include <utility>

/*******************************************************************************
 * MVP
 */
#include "rclcpp/rclcpp.hpp"
#include "mvp_msgs/msg/control_process.hpp"
#include "mvp_msgs/msg/control_mode.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geographic_msgs/msg/geo_point.hpp"

namespace helm
{

class BehaviorBase {
private:
    friend class Helm;

    friend class BehaviorContainer;
    

    /**
     * @brief Frequency of the helm
     */
    double m_helm_frequency;

    /**
     * @brief Global link of helm
     */
    std::string m_global_link;

    /**
     * @brief Local link of helm
     */
    std::string m_local_link;

    /**
     * @brief A string holds the active state name
     */
    std::string m_active_state;

    /**
     * @brief A vector holds DOFs active at the moment by controller
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

    std::function<void(geometry_msgs::msg::Point map_point, geographic_msgs::msg::GeoPoint::SharedPtr ll_point)> f_dis2ll;
    std::function<void(geographic_msgs::msg::GeoPoint ll_point, geometry_msgs::msg::Point::SharedPtr map_point)> f_ll2dis;

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
     * @brief Unique name for the behavior.
     * This name will later be used as namespace for ros node handler.
     *
     * @todo A behavior may or may not use a parameter from ROS. However,
     *       this name is still a necessity.
     */
    std::string m_name;
    
    /**
     * @brief A vector holds controlled DOFs by behavior
     * Each behavior must present the degrees of freedoms that they want
     * to control. Helm will be controlling this information during
     * execution.
     */
    std::vector<int> m_dofs;

    int SETPOINT_DOF_LENGTH =12;

    struct DOF {
        //! @NOTE: ROLL_RATE, PITCH_RATE, YAW_RATE are controlle in global frame
        enum IDX : int {
            X =             mvp_msgs::msg::ControlMode::DOF_X,
            Y =             mvp_msgs::msg::ControlMode::DOF_Y,
            Z =             mvp_msgs::msg::ControlMode::DOF_Z,
            ROLL =          mvp_msgs::msg::ControlMode::DOF_ROLL,
            PITCH =         mvp_msgs::msg::ControlMode::DOF_PITCH,
            YAW =           mvp_msgs::msg::ControlMode::DOF_YAW,
            U =             mvp_msgs::msg::ControlMode::DOF_U,
            V =             mvp_msgs::msg::ControlMode::DOF_V,
            W =             mvp_msgs::msg::ControlMode::DOF_W,
            P =             mvp_msgs::msg::ControlMode::DOF_P,
            Q =             mvp_msgs::msg::ControlMode::DOF_Q,
            R =             mvp_msgs::msg::ControlMode::DOF_R,
        };
    };
    /**
     * @brief Registered state of the of the low level controller
     */
    mvp_msgs::msg::ControlProcess m_process_values;

    /**
     * @brief a member variable to hold activity state of the behavior
     */
    bool m_activated = false;

    /**
     * @brief This function is triggered when a behavior gets activated
     * A plugin may or may not override this function.
     */
    virtual void activated() = 0;

    /**
     * @brief This function is triggered when a behavior gets disabled
     * A plugin may or may not override this function.
     */
    virtual void disabled() = 0;

    /**
     * @brief
     *
     * @param state_name
     */
    virtual void state_changed(const std::string& state_name) {
        printf("name=%s\n", state_name.c_str());
    }

    /**
     * @brief 
     */
    virtual auto change_state(const std::string& state) -> bool final {
        return f_change_state(state);
    }

    virtual auto dis2ll(geometry_msgs::msg::Point map_point, geographic_msgs::msg::GeoPoint::SharedPtr ll_point) -> void final
    {
         return f_dis2ll(map_point, ll_point);
    }

    virtual auto ll2dis(geographic_msgs::msg::GeoPoint ll_point, geometry_msgs::msg::Point::SharedPtr map_point) -> void final
    {
         return f_ll2dis(ll_point, map_point);
    }
    /**
     * @brief 
     */
    virtual double get_helm_frequency() final { return m_helm_frequency; }

    /**
     * @brief 
     */
    virtual std::string get_helm_global_link() final { return m_global_link; }

    /**
     * @brief 
     */
    virtual std::string get_helm_local_link() final { return m_local_link; }
    
    /**
     * @brief 
     */
    virtual auto configure_dofs() -> decltype(m_dofs) {return decltype(m_dofs)();};

public:
    /**
     * @brief 
     */
    virtual ~BehaviorBase() = default; 

    /**
     * @brief Trivial generic pointer type
     */
    typedef std::shared_ptr<BehaviorBase> Ptr;

    /**
     * @brief retrieve degrees of freedoms controlled by the behavior
     *
     * @return std::vector<ctrl::DOF::IDX>
     */
    virtual auto get_dofs() -> const decltype(m_dofs)& final {
        return m_dofs;
    }

    /**
     * @brief 
     */
    virtual auto get_name() -> std::string final { 
        return m_name; 
    }

    /**
     * @brief
     * @param set_point
     * @return true if Behavior wants helm to use its result
     * @return false if Behavior doesn't want helm to use its result
     */
    virtual bool request_set_point(
        mvp_msgs::msg::ControlProcess *set_point) = 0;

    /**
     * @brief
     *
     * @param  parent pointer to user's node
     * @param  name The name of this planner
     */
    virtual void initialize(const rclcpp::Node::WeakPtr &parent) = 0;
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

} // namespace helm
