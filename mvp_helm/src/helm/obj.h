#pragma once

#include "ros/ros.h"
#include "memory"

namespace helm {

    class HelmObj {

    protected:

        std::shared_ptr<ros::NodeHandle> m_pnh;

        std::shared_ptr<ros::NodeHandle> m_nh;


    public:

        HelmObj();

        typedef std::shared_ptr<HelmObj> Ptr;

    };

}