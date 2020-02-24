#include <string>
#include <stdio.h>
using namespace std;

#include <thread>
#include <chrono>

#include "ros/ros.h"
#include "ros/console.h"

#include "serial/serial.h"

#include "std_msgs/String.h"
#include "mrdc_xbee_communication/Analog_Controller.h"

#include "state.pb.h"
#include "status.pb.h"

serial::Serial ser;

bool message_done(int available, uint8_t length){return available>=length;}

//when a message is received
void status_call_back(const std_msgs::String::ConstPtr& msg){
    status::Status current_status;
    try{
        //build a message protobuf
        current_status.set_update(msg->data.c_str());
        //serialize it to a string
        string binary = current_status.SerializeAsString();
        //build message
        char size = (char) current_status.ByteSizeLong();
        string size_str(1, size);
        binary.insert(0, size_str);
        binary.insert(0,"Transmission");
        ser.write(binary);
        ROS_INFO("Sent Message: %s", msg);
    }catch (serial::IOException& e){
        ROS_WARN("Cant send status to controller");
    }
}

int main(int argc, char **argv){
    //initialize ROS
    ros::init(argc, argv, "xbee_receive");
    ros::NodeHandle n;

    ros::Rate loop_rate(100);

    //Initialize Protocol
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    state::State currentState;

    //serial setup with ros parameters
    if(!n.hasParam("mrdc_xbee_communication/port") || !n.hasParam("mrdc_xbee_communication/baud_rate")){throw "ROS Parameters not specified";}
    std::string port; int baud_rate;
    n.getParam("mrdc_xbee_communication/port", port);
    n.getParam("mrdc_xbee_communication/baud_rate", baud_rate);

    try{
        ser.setPort(port);ser.setBaudrate(baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR("Cant open xbee communication serial port");
    }

    //initialize ros channels
    ros::Publisher controller_analog = n.advertise<mrdc_xbee_communication::Analog_Controller>("mrdc_xbee_communication/controller_analog", 10);
    ros::Subscriber sub = n.subscribe("mrdc_xbee_communication/status_updates", 1000, status_call_back);
    //read through serial looking for a header, if found read the message and decode it, then send it off
    while (ros::ok()){
        //if the length of the header is available
        if(ser.available()>13){
            //read it
            std::string header = ser.read(12);
            //check if its the correct header.
            if(header=="Transmission"){
                //if so read the length (it has to be an array, just go with it)
                u_int8_t length [1] = {0};
                ser.read(length, 1);
                
                //wait for the rest of the message
                while(ser.available()<length[0]){
                    std::this_thread::sleep_for (std::chrono::microseconds(1000000*baud_rate/(8*length[0])));
                }
                
                //read the body
                uint8_t body [length[0]];
                ser.read(body, length[0]);
                //decode the body and if its valid, send it off
                if(currentState.ParseFromArray(body, length[0])){
                    mrdc_xbee_communication::Analog_Controller msg;
                    msg.forward = currentState.forward();
                    msg.turn = currentState.turn();
                    controller_analog.publish(msg);
                }else{
                    //its not valid post warning
                    ROS_WARN("Failed to decode protocol buffer.");
                }
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}