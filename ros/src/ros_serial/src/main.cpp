#include <ros/ros.h>
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>

void stringMessageCallback(const std_msgs::String::ConstPtr &msg){
    std::cout << "Received String Message: " << msg->data.c_str() << std::endl;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "mrdc_ros_serial");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber = nodeHandle.subscribe("/string_message_callback", 1000, &stringMessageCallback);
    ros::spin();
    return 0;
}