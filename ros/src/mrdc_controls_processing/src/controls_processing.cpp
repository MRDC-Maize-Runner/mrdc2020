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

	ros::Subscriber sub = n.subscribe("mrdc_xbee_communication/controller_analog", 1000, process_analog);

	//get pararms
	if(!n.hasParam("mrdc_controls_processing/deadzone")){throw "ROS Parameters not specified";}
	n.getParam("mrdc_controls_processing/deadzone", deadzone);
	//advertise the channels
	lf_pub = n.advertise<std_msgs::Float32>("mrdc_controls_processing/Left_Front", 10);
	lb_pub = n.advertise<std_msgs::Float32>("mrdc_controls_processing/Left_Back", 10);
	rf_pub = n.advertise<std_msgs::Float32>("mrdc_controls_processing/Right_Front", 10);
	rb_pub = n.advertise<std_msgs::Float32>("mrdc_controls_processing/Right_Back", 10);

	ros::spin();

	return 0;
}