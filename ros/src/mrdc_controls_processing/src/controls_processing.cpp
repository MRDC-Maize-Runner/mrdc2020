#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "mrdc_controls_processing/Analog_Controller.h"

#include <sstream>

//ros::Publisher lf_pub; ros::Publisher lb_pub; ros::Publisher rf_pub; ros::Publisher rb_pub;
ros::Publisher data_publisher;

float deadzone;
int left_channel, right_channel, pwm_min, pwm_max, pwm_avg;

int transform(float value){return pwm_avg+(pwm_avg-pwm_min)*value;}

//deal with messages
void process_analog(const mrdc_controls_processing::Analog_Controller& msg){
	std_msgs::Int32MultiArray data;
	float left = 0.0; float right = 0.0;
		//calculate the controls signals from the joystick
	if(msg.forward>deadzone || msg.forward<-1*deadzone){
		left += msg.forward;
		right += msg.forward;
	}
	if(msg.turn>deadzone || msg.turn<-1*deadzone){
		left += msg.turn;
		right -= msg.turn;
	}
	//make sure between -1 and 1
	left = transform(std::clamp(left, (float) -1, (float) 1));
	right = transform(std::clamp(right, (float) -1, (float) 1));

	int message_arr[16] = {-1};
	message_arr[left_channel]=transform(left);
	message_arr[right_channel]=transform(right);
	data.data = std::vector<int> (std::begin(message_arr), std::end(message_arr));
	data_publisher.publish(data);
}

//set up ros and wait
int main(int argc, char **argv){
	//set up ros
	ros::init(argc, argv, "controls_processing");
	ros::NodeHandle n;

	//get ros params
	std::string Analog_Controller_Channel;//, Left_Front_Channel, Left_Back_Channel, Right_Front_Channel, Right_Back_Channel;
	n.param<std::string>("Analog_Controller_Channel", Analog_Controller_Channel, "mrdc_xbee_communication/controller_analog");
	/*n.param<std::string>("Left_Front_Channel", Left_Front_Channel, "mrdc_controls_processing/Left_Front");
	n.param<std::string>("Left_Back_Channel", Left_Back_Channel, "mrdc_controls_processing/Left_Back");
	n.param<std::string>("Right_Front_Channel", Right_Front_Channel, "mrdc_controls_processing/Right_Front");
	n.param<std::string>("Right_Back_Channel", Right_Back_Channel, "mrdc_controls_processing/Right_Back");*/
	std::string command_channel;
	n.param<std::string>("mrdc_controls_processing/command_channel", command_channel, "command");
	n.param<float>("mrdc_controls_processing/deadzone", deadzone, 0.1);
	n.param<int>("mrdc_controls_processing/left_pwm_channel", left_channel, 0);
	n.param<int>("mrdc_controls_processing/right_pwm_channel", right_channel, 0);
	n.param<int>("pwm_min", pwm_min, 2800);
	n.param<int>("pwm_max", pwm_max, 7600);
	pwm_avg = (pwm_min+pwm_max)/2;

	ros::Subscriber sub = n.subscribe(Analog_Controller_Channel, 1000, process_analog);

	
	//advertise the channels
	data_publisher = n.advertise<std_msgs::Int32MultiArray>(command_channel, 1);

	ros::spin();

	return 0;
}