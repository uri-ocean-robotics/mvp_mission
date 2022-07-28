#include "bhv_sawtooth_wave.h"
#include "pluginlib/class_list_macros.h"

using namespace helm;

void SawtoothWave::initialize() {

    m_pnh.reset(
        new ros::NodeHandle(ros::this_node::getName() + "/" +
            BehaviorBase::m_name)
    );

    m_pnh->param("min_depth", m_min_depth, 0.0); // meters

    m_pnh->param("max_depth", m_max_depth, 5.0); // meters

    m_pnh->param("surge_velocity", m_surge_velocity, 0.65); // m/s

    m_pnh->param("heading", m_heading, 0.0); // radians

    // radians. default: 22.5 degrees
    m_pnh->param("pitch", m_pitch, 0.39269908169872414);

    BehaviorBase::m_dofs = decltype(m_dofs){
        seal_msgs::ControlMode::DOF_SURGE,
        seal_msgs::ControlMode::DOF_PITCH,
        seal_msgs::ControlMode::DOF_YAW
    };

    m_bhv_state = BHV_STATE::IDLE;

}

SawtoothWave::SawtoothWave() {

}

void SawtoothWave::activated() {

    if(BehaviorBase::m_process_values.position.z > m_max_depth) {
        m_bhv_state = BHV_STATE::ASCENDING;
    }

    if(BehaviorBase::m_process_values.position.z < m_min_depth) {
        m_bhv_state = BHV_STATE::DESCENDING;
    }
}

void SawtoothWave::disabled() {

    m_bhv_state = BHV_STATE::IDLE;

}


bool SawtoothWave::request_set_point(
    seal_msgs::ControlProcess *set_point) {

    BehaviorBase::m_process_values.velocity.x;

    set_point->orientation.z = m_heading;

    set_point->velocity.x = m_surge_velocity;

    if(m_bhv_state == BHV_STATE::ASCENDING) {
        if(BehaviorBase::m_process_values.position.z < m_min_depth) {
            m_bhv_state = BHV_STATE::DESCENDING;
            return false;
        }
        set_point->orientation.y = m_pitch;


    } else if (m_bhv_state == BHV_STATE::DESCENDING){
        if(BehaviorBase::m_process_values.position.z > m_max_depth) {
            m_bhv_state = BHV_STATE::ASCENDING;
            return false;
        }
        set_point->orientation.y = -m_pitch;
    }

    return true;
}

PLUGINLIB_EXPORT_CLASS(helm::SawtoothWave, helm::BehaviorBase)