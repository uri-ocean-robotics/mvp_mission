#include "obj.h"

namespace helm {
    HelmObj::HelmObj() {

        // Initialize node handler
        m_nh = std::make_shared<ros::NodeHandle>("");

        // Initialize node handler
        m_pnh = std::make_shared<ros::NodeHandle>("~");

    }
}