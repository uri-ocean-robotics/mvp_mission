#include "motion_evaluation.h"
#include "pluginlib/class_list_macros.h"
#include "memory"
#include "vector"
#include "thread"
#include "dynamic_reconfigure/server.h"
#include "chrono"

using namespace helm;

MotionEvaluation::MotionEvaluation() : BehaviorBase() {

}

void MotionEvaluation::initialize() {

    m_pnh.reset(
        new ros::NodeHandle(ros::this_node::getName() + "/" + m_name)
    );

    m_dofs = decltype(m_dofs){
        ctrl::DOF::SURGE,
        ctrl::DOF::YAW_RATE,
        ctrl::DOF::PITCH_RATE
    };

    m_dynconf_server.setCallback(
        std::bind(&MotionEvaluation::f_dynconf_freqmag_cb, this,
                  std::placeholders::_1,
                  std::placeholders::_2
        )
    );


    m_surge_phase = 0;
    m_pitch_rate_phase = 0;
    m_yaw_rate_phase = 0;

}

void MotionEvaluation::f_dynconf_freqmag_cb(
    motion_evaluation::FreqMagConfig &conf, uint32_t level)
{
    std::scoped_lock lock(m_config_mutex);

    m_config = conf;

}

bool MotionEvaluation::request_set_point(mvp_control::ControlProcess *set_point)
{
    std::scoped_lock lock(m_config_mutex);

    /*
     * Decide the action needs to be taken
     */
    m_cmd.header.frame_id = m_process_values.header.frame_id;

    if(m_config.surge_frequency != 0) {
        m_surge_phase += M_PI / (m_helm_frequency / m_config.surge_frequency);
        m_cmd.velocity.x = sin(m_surge_phase) * m_config.surge_magnitude;
    } else {
        m_cmd.velocity.x = m_config.surge_magnitude;
    }

    if(m_config.yaw_rate_frequency != 0) {
        m_yaw_rate_phase += M_PI / (m_helm_frequency / m_config.yaw_rate_frequency);
        m_cmd.angular_rate.z = sin(m_yaw_rate_phase) * m_config.yaw_rate_magnitude;
    } else {
        m_cmd.angular_rate.z = m_config.yaw_rate_magnitude;
    }

    if(m_config.pitch_rate_frequency != 0) {
        m_pitch_rate_phase += M_PI / (m_helm_frequency / m_config.pitch_rate_frequency);
        m_cmd.angular_rate.y = sin(m_pitch_rate_phase) * m_config.pitch_rate_magnitude;
    } else {
        m_cmd.angular_rate.y = m_config.pitch_rate_magnitude;
    }


    /*
     * Command it to the helm
     */
    *set_point = m_cmd;

    return true;
}

PLUGINLIB_EXPORT_CLASS(helm::MotionEvaluation, helm::BehaviorBase)