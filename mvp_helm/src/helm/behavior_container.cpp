#include "behavior_container.h"

#include <utility>
#include "exception.h"

namespace helm
{

    BehaviorContainer::BehaviorContainer(
            std::string name,
            std::string type,
            decltype(m_states) states) {

        m_name = std::move(name);

        m_type = std::move(type);

        m_states = std::move(states);

    }

    void BehaviorContainer::initialize() {

        if(m_type.empty()) {
            throw HelmException(
                "behavior can not be initialized without type name!"
            );
        }

        m_class_loader =
            boost::make_shared<pluginlib::ClassLoader<BehaviorBase>>(
                "behavior_interface", "helm::BehaviorBase"
            );

        m_behavior = m_class_loader->createInstance(m_type);

        m_behavior->set_name(m_name);

    }

    BehaviorContainer::~BehaviorContainer() {

        m_behavior.reset();

    }

} // namespace helm
