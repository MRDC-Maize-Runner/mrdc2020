#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "mrdc_controls_processing/Analog_Controller.h"

#include <sstream>

ros::Publisher lf_pub; ros::Publisher lb_pub; ros::Publisher rf_pub; ros::Publisher rb_pub;

float deadzone;

//deal with messages
void process_analog(const mrdc_controls_processing::Analog_Controller& msg){
	std_msgs::Float32 left; std_msgs::Float32 right;
	left.data = 0.0; right.data = 0.0;
		//calculate the controls signals from the joystick
	if(msg.forward>deadzone || msg.forward<-1*deadzone){
		left.data += msg.forward;
		right.data += msg.forward;
	}
	if(msg.turn>deadzone || msg.turn<-1*deadzone){
		left.data += msg.turn;
		right.data -= msg.turn;
	}
	//make sure between -1 and 1
	left.data = std::clamp(left.data, (float) -1, (float) 1);
	right.data = std::clamp(right.data, (float) -1, (float) 1);

	//publish to channels
	lf_pub.publish(left); lb_pub.publish(left);
	rf_pub.publish(right); rb_pub.publish(right);
}

//set up ros and wait
int main(int argc, char **argv){
	//set up ros
	ros::init(argc, argv, "controls_processing");
	ros::NodeHandle n;

	//get ros params
	std::string Analog_Controller_Channel, Left_Front_Channel, Left_Back_Channel, Right_Front_Channel, Right_Back_Channel;
	n.param<std::string>("Analog_Controller_Channel", Analog_Controller_Channel, "mrdc_xbee_communication/controller_analog");
	n.param<std::string>("Left_Front_Channel", Left_Front_Channel, "mrdc_controls_processing/Left_Front");
	n.param<std::string>("Left_Back_Channel", Left_Back_Channel, "mrdc_controls_processing/Left_Back");
	n.param<std::string>("Right_Front_Channel", Right_Front_Channel, "mrdc_controls_processing/Right_Front");
	n.param<std::string>("Right_Back_Channel", Right_Back_Channel, "mrdc_controls_processing/Right_Back");
	n.param<float>("mrdc_controls_processing/deadzone", deadzone, 0.1);

	ros::Subscriber sub = n.subscribe(Analog_Controller_Channel, 1000, process_analog);

	
	//advertise the channels
	lf_pub = n.advertise<std_msgs::Float32>(Left_Front_Channel, 10);
	lb_pub = n.advertise<std_msgs::Float32>(Left_Back_Channel, 10);
	rf_pub = n.advertise<std_msgs::Float32>(Right_Front_Channel, 10);
	rb_pub = n.advertise<std_msgs::Float32>(Right_Back_Channel, 10);

	ros::spin();

	return 0;
}