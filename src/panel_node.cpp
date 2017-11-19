#include "ros/ros.h"
#include "fyt_mae/fyt_commons.h"
#include "cmg_msgs/State.h"
#include "cmg_msgs/Signal.h"
#include "fyt_mae/CheckState.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

#define NB_STATES 4
#define NB_OUT 4

typedef struct {
	uint16_t seq;
	uint16_t flags;
	uint32_t tick;
	uint32_t level;
} gpioReport_t;

class Controller {
	public:
		Controller() : n("~"), cmd("/dev/pigpio"), res("/dev/pigout") {
			s = n.subscribe("/mae/state", 5, &Controller::state_change, this);
			p = n.advertise<cmg_msgs::Signal>("/mae/signal", 1);
			cmd << "no" << std::endl;
			res >> in;
			std::ostringstream nbu_file;
			nbu_file << "/dev/pigpio" << in;
			std::string nbu_fname = nbu_file.str();
			ROS_INFO("Notification file is: %s", nbu_fname.c_str());
			nbu.open(nbu_fname, std::ifstream::binary);
			cmd << "nb " << in << " " << (1<<but_pin) << std::endl;
			int response;
			res >> response;
			if (response != 0) {
				ROS_WARN("Pigpiod responded to nb with: %d", response);
			}
			cmd << "fg " << (int)but_pin <<" 10000" << std::endl;
			res >> response;
			if (response != 0) {
				ROS_WARN("Pigpiod responded to fg with: %d", response);
			}

			ros::ServiceClient checkstate_client = n.serviceClient<fyt_mae::CheckState>("/mae/check_state");
			fyt_mae::CheckState state_msg;
			if (checkstate_client.call(state_msg)) {
				cmg_msgs::State s;
				s.state = state_msg.response.state;
				state_change(s);		
			}
		}

		void spin_button() {
			while (ros::ok() && nbu) {
				gpioReport_t report;
				nbu.read((char*)&report, sizeof(gpioReport_t));
				bool up = (report.level>>but_pin)&1;
				if (up) {
					cmg_msgs::Signal s;
					s.signal = signal_on_press;
					p.publish(s);
				}
				ROS_INFO("GOT REPORT: %d %s",
						report.seq,
						up?"UP":"DOWN"
				);
			}
		}

		void spin() {
			std::thread but_thread(&Controller::spin_button, this);
			ros::Rate r(10);
			while (ros::ok()) {
				ros::spinOnce();
				r.sleep();
			}
			cmd << "nc " << in << std::endl;
			but_thread.join();
		}

		void state_change(const cmg_msgs::State & msg) {
			for (int i = 0; i < NB_OUT; i++) {
				cmd << " p " << (int)state_led_ids[i] << " " << (int)state_led_values[msg.state][i];
			}
			cmd << std::endl;
			int response;
			res >> response;
			if (response != 0) {
				ROS_WARN("Pigpiod responded with: %d", response);
			}
			signal_on_press = state_sop[msg.state];
		}
	private:
		const uint8_t state_sop[4] = {SIG_GOOD, SIG_START, SIG_END, SIG_GOOD};
		uint8_t signal_on_press;
		const uint8_t state_led_ids[NB_OUT] = {5,6,13,19};
		const uint8_t state_led_values[NB_STATES][NB_OUT] = {
			{255,255,  0,  0},
			{  0,255,255,  0},
			{255, 80,125,255},
			{200,125,  0,  0}
		};
		const uint8_t but_pin = 26;
		ros::NodeHandle n;
		ros::Subscriber s;
		ros::Publisher p;
		std::ofstream cmd;
		std::ifstream res;	
		std::ifstream nbu;
		int in;
};

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "panel");
	Controller c;
	c.spin();
}
