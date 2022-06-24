#include "periodic_surface.h"
#include "pluginlib/class_list_macros.h"

using namespace helm;

void PeriodicSurface::initialize() {

    m_nh.reset(
        new ros::NodeHandle(ros::this_node::getNamespace() + "/" + m_name)
    );
    m_dofs = decltype(m_dofs){
        ctrl::DOF::PITCH,
        ctrl::DOF::Z
    };
    m_nh->param("fwd_distance", m_fwd_distance, 3.0);

    m_nh->param("max_pitch", m_max_pitch, M_PI_2);
    m_nh->param("m_surface_period", m_surface_period, 10.0); //seconds
    m_nh->param("m_surface_duration", m_surface_duration, 10.0); //seconds

    m_activated = false;

    m_bhv_state = BhvState::DISABLED;

}


/**
 * @brief Construct a new Periodic Surface:: Periodic Surface object
 *
 */
PeriodicSurface::PeriodicSurface()
{
    std::cout << "a message from periodic surface" << std::endl;
}

void PeriodicSurface::activated() {

    // This will be triggered when the behaviour is activated.
    m_start_time = ros::Time::now();

    m_bhv_state = BhvState::ENABLED;

    m_activated = true;

}

void PeriodicSurface::disabled() {

    m_bhv_state = BhvState::DISABLED;

    m_activated = false;

}

/**
 * @brief Commands the controller with a set point from the behavior
 *
 * @param set_point
 * @return true
 * @return false
 */
bool PeriodicSurface::request_set_point(mvp_control::ControlProcess *set_point)
{

    if(!m_activated) {
        return false;
    }

    //check depth to set m_surfaced_time

    if(m_bhv_state == BhvState::ENABLED) {

        std::cout << "ENABLED" << std::endl;

        if(m_process_values.position.z < 0.5){
            m_bhv_state = BhvState::WAITING;
            m_surfaced_time = ros::Time::now();
        }

    } else if (m_bhv_state == BhvState::WAITING) {

        std::cout << "WAITING" << std::endl;

        if(ros::Time::now().sec - m_surfaced_time.sec > m_surface_duration) {
            m_bhv_state = BhvState::DISABLED;
            m_start_time = ros::Time::now();
        }

    } else if (m_bhv_state == BhvState::DISABLED) {

        std::cout << "DISABLED" << std::endl;

        if(ros::Time::now().sec - m_start_time.sec > m_surface_period) {
            m_bhv_state = BhvState::ENABLED;
        }

        return false;
    }




    if( m_bhv_state == BhvState::ENABLED || m_bhv_state == BhvState::WAITING)
    {
        double pitch = atan(m_process_values.position.z / m_fwd_distance);

        if(m_process_values.velocity.x != 0)  {

            pitch += atan(
                m_process_values.velocity.z / m_process_values.velocity.x);
        }

        if(fabs(pitch) > m_max_pitch) {
            set_point->orientation.y = pitch >= 0 ? m_max_pitch : -m_max_pitch;
        } else {
            set_point->orientation.y = pitch;
        }
    }



    return true;
}


PLUGINLIB_EXPORT_CLASS(helm::PeriodicSurface, helm::BehaviorBase)