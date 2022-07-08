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

        m_behavior->set_name(m_opts.name);

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
