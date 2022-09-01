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

        /**
         * @brief Behavior component settings
         *
         */
        behavior_component_t m_opts;

        /**
         * @brief Behavior defined by #m_class_name
         */
        std::shared_ptr<BehaviorBase> m_behavior;

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