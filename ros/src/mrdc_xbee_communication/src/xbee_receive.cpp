#include <string>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

#include "ros/ros.h"
#include "ros/console.h"

#include "serial/serial.h"

#include "std_msgs/String.h"
#include "mrdc_xbee_communication/Analog_Controller.h"

#include "state.pb.h"

std::mutex mtx;
std::condition_variable cv;

bool message_done(int available, uint8_t length){return available>=length;}

int main(int argc, char **argv){
    //initialize ROS
    ros::init(argc, argv, "xbee_receive");
    ros::NodeHandle n;

    ros::Publisher controller_analog = n.advertise<mrdc_xbee_communication::Analog_Controller>("mrdc_xbee_communication/controller_analog", 10);
    ros::Rate loop_rate(100);

    //Initialize Protocol
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    state::State currentState;

    //serial setup with ros parameters
    try{
        if(!n.hasParam("mrdc_xbee_communication/port") || !n.hasParam("mrdc_xbee_communication/baud_rate")){throw "ROS Parameters not specified";}
        std::string port; int baud_rate;
        n.getParam("mrdc_xbee_communication/port", port);
        n.getParam("mrdc_xbee_communication/baud_rate", baud_rate);
        serial::Serial my_serial(port, baud_rate, serial::Timeout::simpleTimeout(1000));

        //read through serial looking for a header, if found read the message and decode it, then send it off
        while (ros::ok()){
            //if the length of the header is available
            if(my_serial.available()>13){
                //read it
                std::string header = my_serial.read(12);
                //check if its the correct header.
                if(header=="Transmission"){
                    //if so read the length (it has to be an array, just go with it)
                    u_int8_t length [1] = {0};
                    my_serial.read(length, 1);
                    
                    //wait for the rest of the message
                    std::unique_lock<std::mutex> lck(mtx);
                    cv.wait(lck, [&]{return message_done(my_serial.available(), length[0]);});
                    
                    //read the body
                    uint8_t body [length[0]];
                    my_serial.read(body, length[0]);
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
        }
    } catch (char* e){
        ROS_ERROR(e);
    }
    
}