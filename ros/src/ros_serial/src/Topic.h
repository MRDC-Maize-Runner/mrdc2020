#pragma once

#include <ros/ros.h>

#include "SerialInterface.h"

namespace mrdc_serial_node{

    class Topic{
    public:
        virtual ~Topic() {}

    protected:
        SerialInterface *m_serialInterface;
        ros::Subscriber m_subscriber;

    };

}