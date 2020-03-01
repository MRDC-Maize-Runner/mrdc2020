#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "Topic.h"
#include "ConfigParser.h"

namespace mrdc_serial_node{

    class MotorTopic : public Topic{
    public:
        explicit MotorTopic(ros::NodeHandle &nodeHandle, SerialInterface *serialInterface, const ConfiguredMotor &motorConfiguration);
        void operator()(const std_msgs::Float32::ConstPtr &motorPower);

    private:
        int m_pinId;

    };

}