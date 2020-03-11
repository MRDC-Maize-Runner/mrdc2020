#pragma once

#include <fstream>
#include <mutex>
#include <vector>
#include <iostream>
#include <serial/serial.h>
#include <thread>

namespace mrdc_serial_node{

    class SerialInterface{
    public:
        explicit SerialInterface(const std::string &file);
        ~SerialInterface();

        template<typename T>
        void send(const T &t);
        std::recursive_mutex &mutex() noexcept { return m_mutex; }

    private:
        std::string m_file;
        std::recursive_mutex m_mutex;
        serial::Serial m_serialPort;
        std::thread m_serialReaderThread;

        void serialReaderThread();

    };

    template<typename T>
    void SerialInterface::send(const T &t) {
        std::lock_guard<std::recursive_mutex> guard(m_mutex);

        int tryCount = 1;
        auto currentByte = reinterpret_cast<const std::uint8_t*>(&t);
        auto endByte = currentByte+sizeof(T);
        while(currentByte < endByte){
            currentByte += m_serialPort.write(currentByte, endByte-currentByte);
        }

        m_serialPort.flushOutput();
    }

}