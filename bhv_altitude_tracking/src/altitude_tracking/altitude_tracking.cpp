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


#include "altitude_tracking.h"
#include "pluginlib/class_list_macros.h"

using namespace helm;

void AltitudeTracking::initialize() {

    m_pnh.reset(
        new ros::NodeHandle(ros::this_node::getName() + "/" + get_name())
    );

    m_nh.reset(new ros::NodeHandle());

    //! @par Declare the dofs to be controlled
    BehaviorBase::m_dofs = decltype(m_dofs){
        mvp_msgs::ControlMode::DOF_Z,
    };

    m_pnh->param<double>("m_min_altitude", m_min_altitude, -1.0);


    //altitude topic name
    m_pnh->param<std::string>("altitude_topic_name", m_altitude_topic_name,"");
    
    //altitude mode
    m_pnh->param<int>("altitude_tracking_mode", m_altitude_mode, 0);

    //altitude subscriber
    m_altitude_sub = m_pnh->subscribe<geometry_msgs::PointStamped>(
        m_altitude_topic_name,
        10,
        std::bind(
            &AltitudeTracking::f_altitude_cb,
            this,
            std::placeholders::_1
        )
    );
}



void AltitudeTracking::f_altitude_cb(const geometry_msgs::PointStamped::ConstPtr &m)
{
    //transform point stamp into world_ned
    geometry_msgs::PointStamped point_in_helm_global;

    try{
        point_in_helm_global = m_transform_buffer.transform(*m, get_helm_global_link());   
        m_altitude =  point_in_helm_global.point.z;
    }

    catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT transform: %s", ex.what());
    }
}

AltitudeTracking::AltitudeTracking() {
    std::cout << "a message from depth tracking" << std::endl;
}

AltitudeTracking::~AltitudeTracking() {
    m_altitude_sub.shutdown();
}

bool AltitudeTracking::request_set_point(mvp_msgs::ControlProcess *set_point) {

    switch(m_altitude_mode)
    {
        case 0:
            if(m_altitude < m_min_altitude)
            {
                set_point->position.z = BehaviorBase::m_process_values.position.z + m_altitude - m_min_altitude;
                return true;
            }
            break;

        case 1:
            set_point->position.z = BehaviorBase::m_process_values.position.z + m_altitude - m_min_altitude;
            return true;

        default:
            break;
    }

    return true;
}


PLUGINLIB_EXPORT_CLASS(helm::AltitudeTracking, helm::BehaviorBase)