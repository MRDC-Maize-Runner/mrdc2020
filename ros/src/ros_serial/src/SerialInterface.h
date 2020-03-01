#pragma once

#include <fstream>
#include <mutex>
#include <vector>
#include <iostream>

namespace mrdc_serial_node{

    class SerialInterface{
    public:
        explicit SerialInterface(const std::string &file);

        template<typename T>
        void send(const T &t);
        std::recursive_mutex &mutex() noexcept { return m_mutex; }

    private:
        std::string m_file;
        std::recursive_mutex m_mutex;
        std::ofstream m_fout;

    };

    template<typename T>
    void SerialInterface::send(const T &t) {
        std::lock_guard<std::recursive_mutex> guard(m_mutex);
        try{
            m_fout.write(reinterpret_cast<const char*>(&t), sizeof(T));
            m_fout.flush();
        }catch(std::exception &e){
            std::cout << "snarl" << std::endl;
            m_fout.clear(std::ios::failbit | std::ios::badbit);
            m_fout.open(m_file, std::ios::binary);
            send(t);
        }
    }

}