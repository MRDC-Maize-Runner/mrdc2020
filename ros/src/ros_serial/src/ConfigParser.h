#pragma once

#include <string>
#include <cstdint>
#include <vector>

namespace mrdc_serial_node{

    struct ConfiguredMotor{
        explicit ConfiguredMotor(const std::string &topicName, std::uint8_t pinNumber)
            : m_topicName(topicName), m_pinNumber(pinNumber) {}

        std::string m_topicName;
        std::uint8_t m_pinNumber;
    };

    struct Config{
        std::vector<ConfiguredMotor> m_motors;
    };

    Config parseConfig(const std::string &file);

}