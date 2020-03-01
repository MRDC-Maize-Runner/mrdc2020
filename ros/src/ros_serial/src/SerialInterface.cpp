#include "SerialInterface.h"

using namespace mrdc_serial_node;

SerialInterface::SerialInterface(const std::string &file) : m_file(file) {
    m_fout.exceptions(std::ios::failbit | std::ios::badbit);
    m_fout.open(file, std::ios::binary);
}