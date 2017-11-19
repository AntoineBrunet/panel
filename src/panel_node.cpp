#include "ros/ros.h"
#include "cmg_msgs/State.h"
#include <fstream>
#include <iostream>

class Controller {
	public:
		Controller() : n("~"), cmd("/dev/pigpio"), res("/dev/pigout") {
			s = n.subscribe("/mae/state", 5, &Controller::state_change, this);
		}

		void state_change(const cmg_msgs::State::ConstPtr & msg) {
			for (int i = 0; i < 3; i++) {
				cmd << " p " << (int)state_led_ids[i] << " " << (int)state_led_values[msg->state][i];
			}
			cmd << std::endl;
			int response;
			res >> response;
			if (response != 0) {
				ROS_WARN("Pigpiod responded with: %d", response);
			}
		}
	private:
		const uint8_t state_led_ids[3] = {5,6,13};
		const uint8_t state_led_values[4][3] = {
			{255,  0,  0},
			{  0,255,  0},
			{  0,  0,255},
			{255, 80,125}
		};
		ros::NodeHandle n;
		ros::Subscriber s;
		std::ofstream cmd;
		std::ifstream res;	
};

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "panel");
	Controller c;
	ros::spin();
}
