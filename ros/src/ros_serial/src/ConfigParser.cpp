#include "ConfigParser.h"

#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <iostream>

using namespace mrdc_serial_node;

Config mrdc_serial_node::parseConfig(const std::string &file){
    auto rootNode = YAML::LoadFile(file);
    Config config;

    if(auto arduinoPath = rootNode["arduino_file"]){
        config.m_arduinoDevFile = arduinoPath.as<std::string>();
    }else{
        throw std::runtime_error("The arduino_file node must exist and be a string.");
    }

    if(rootNode["topics"]){
        auto topicsNode = rootNode["topics"];
        if(!topicsNode.IsMap()){
            throw std::runtime_error("The topics node *must* be a map");
        }

        for(const auto &it : topicsNode){
            auto name = it.first.as<std::string>();
            auto type = it.second["type"].as<std::string>();
            if(type == "motor"){
                config.m_motors.emplace_back(
                        name,
                        it.second["pin_id"].as<std::uint32_t>()
                );
                std::cout << "Found motor with name: " << name << std::endl;
            }else{
                throw std::runtime_error("Unknown topic type: " + type);
            }
        }
    }

    return config;
}