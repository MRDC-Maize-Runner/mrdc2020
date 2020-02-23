#include <ros/ros.h>
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>
#include <memory>

#include "ConfigParser.h"
#include "Topic.h"
#include "MotorTopic.h"

using namespace mrdc_serial_node;

void stringMessageCallback(const std_msgs::String::ConstPtr &msg){
    std::cout << "Received String Message: " << msg->data.c_str() << std::endl;
}

int main(int argc, char *argv[]){
    auto config = parseConfig("config.yml");

    ros::init(argc, argv, "mrdc_ros_serial");
    ros::NodeHandle nodeHandle;

    //make subscriptions, and leak memory like crazy
    std::vector<std::unique_ptr<Topic>> topics;

    for(auto &configuredMotor : config.m_motors){
        topics.push_back(std::make_unique<MotorTopic>(nodeHandle, configuredMotor));
    }

    ros::spin();

    return 0;
}