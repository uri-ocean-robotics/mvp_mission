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


#include "behavior_container.h"

#include "utility"
#include "exception.h"

namespace helm
{

    BehaviorContainer::BehaviorContainer(behavior_component_t opts) {
        m_opts = std::move(opts);

        m_class_loader.reset(
            new pluginlib::ClassLoader<BehaviorBase>(
                "behavior_interface", "helm::BehaviorBase"
            )
        );
    }

    void BehaviorContainer::initialize() {

        if(m_opts.name.empty()) {
            throw HelmException(
                "behavior can not be initialized without type name!"
            );
        }

        m_behavior.reset(
            m_class_loader->createUnmanagedInstance(m_opts.plugin)
        );

        m_behavior->m_name = m_opts.name;

        try {
            m_behavior->initialize();
        } catch ( BehaviorException &e) {
            ROS_ERROR_STREAM("Can not initialize behavior"
                "(" << m_opts.name << "): " << e.what());
        }

    }

    BehaviorContainer::~BehaviorContainer() {

        m_behavior.reset();

    }

} // namespace helm
