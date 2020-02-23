#include "MotorTopic.h"

using namespace mrdc_serial_node;

MotorTopic::MotorTopic(ros::NodeHandle &nodeHandle, const ConfiguredMotor &motorConfiguration) {
    m_pinId = motorConfiguration.m_pinNumber;
    m_subscriber = nodeHandle.subscribe(motorConfiguration.m_topicName, 1, &MotorTopic::operator(), this);
}

void MotorTopic::operator()(const std_msgs::Float32::ConstPtr &motorPower) {
    std::cout << "Received motor power message for pin " << m_pinId << " with intensity " << motorPower.get()->data << std::endl;
}