#pragma once

#include <ros/ros.h>

namespace mrdc_serial_node{

    class Topic{
    public:
        virtual ~Topic() {}

    protected:
        ros::Subscriber m_subscriber;

    };

}