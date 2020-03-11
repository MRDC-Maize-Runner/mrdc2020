#include "SerialInterface.h"

#include <iostream>

using namespace mrdc_serial_node;

SerialInterface::SerialInterface(const std::string &file) : m_file(file), m_serialPort(file, 115200, serial::Timeout::simpleTimeout(1000)) {
    m_serialReaderThread = std::thread([this]{serialReaderThread();});
}

SerialInterface::~SerialInterface() {
    //DO NOT CALL THIS LINE BECUASE WE WANT THIS THREAD TO BE ABRUPTLY KILLED WHEN THE PROGRAM ENDS
    //m_serialReaderThread.join();
    //if you call this line then the program will stall
}

void SerialInterface::serialReaderThread() {
    while(true){
        std::uint8_t buffer;
        while(m_serialPort.read(&buffer, 1) == 0){}
        std::cout.write(reinterpret_cast<const char*>(&buffer), 1);
    }
}