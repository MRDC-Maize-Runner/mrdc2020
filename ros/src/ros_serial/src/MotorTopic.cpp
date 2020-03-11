#include "MotorTopic.h"

#include <mutex>
#include <cmath>

#include "SerialInterface.h"
#include "protcol_constants.h"

using namespace mrdc_serial_node;

MotorTopic::MotorTopic(ros::NodeHandle &nodeHandle, SerialInterface *serialInterface, const ConfiguredMotor &motorConfiguration) {
    m_pinId = motorConfiguration.m_pinNumber;
    m_serialInterface = serialInterface;
    m_subscriber = nodeHandle.subscribe(motorConfiguration.m_topicName, 1, &MotorTopic::operator(), this);
}

void MotorTopic::operator()(const std_msgs::Float32::ConstPtr &motorPower) {
    constexpr int MAX = 255, MIN = 0;
    float f = motorPower->data;
    unsigned char scaled = static_cast<unsigned>(std::round((f*0.5f+0.5f)*(MAX-MIN)+MIN));

    { std::lock_guard<std::recursive_mutex> guard(m_serialInterface->mutex());
        m_serialInterface->send<std::uint8_t>(message_opcodes::MESSAGE_MOTOR);
        m_serialInterface->send<std::uint8_t>(m_pinId);
        m_serialInterface->send<std::uint8_t>(scaled);
    }

    std::cout << "Received motor power message for pin " << static_cast<std::uint32_t>(m_pinId) << " with intensity " << motorPower.get()->data << " = " << static_cast<std::uint32_t>(scaled) << std::endl;
}