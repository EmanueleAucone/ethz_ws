/*
 * Emanuele Aucone, ERL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Last modified on: 15.10.2020
 */

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "mav_msgs/RollPitchYawrateThrust.h"
#include <geometry_msgs/WrenchStamped.h>
#include <Eigen/Geometry>

//////////////////// Global variables ////////////////////

// Pubs and subs
ros::Publisher cmd_pub;
ros::Subscriber ft_sensor_sub;
ros::Publisher ft_pub;
const int pub_time = 1;

// Force/torque variables
geometry_msgs::Wrench ft_conv_msg;
Eigen::Vector3f raw_forces;
Eigen::Vector3f raw_momenta;

// Filter variables
const double beta = 0.15;
const int n = 1;
Eigen::Vector3f prev_forces;
Eigen::Vector3f prev_momenta;
Eigen::Vector3f forces;
Eigen::Vector3f momenta;

// Roll Pitch Yawrate Thrust controller variables
mav_msgs::RollPitchYawrateThrust cmd_msg;
double desired_roll;
double force_magnitude;

//////////////////// Functions ////////////////////

// Callback for force/torque measurement acquisition
void forceTorqueSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& ft_msg)
{
	// Store data for control strategy
	raw_forces[0] = (1000) * ft_msg->wrench.force.x;
	raw_forces[1] = (1000) * ft_msg->wrench.force.y;
	raw_forces[2] = (1000) * ft_msg->wrench.force.z;
	raw_momenta[0] = (1000) * ft_msg->wrench.torque.x;
	raw_momenta[1] = (1000) * ft_msg->wrench.torque.y;
	raw_momenta[2] = (1000) * ft_msg->wrench.torque.z;
}

// Compute the desired roll angle
double compute_desired_roll()
{
	desired_roll = (180/M_PI) * atan2(abs(ft_conv_msg.force.y),abs(ft_conv_msg.force.z));
	std::cout << "Desired roll angle: " << desired_roll << std::endl;

	force_magnitude = sqrt( ft_conv_msg.force.x*ft_conv_msg.force.x + 
				ft_conv_msg.force.y*ft_conv_msg.force.y + 
				ft_conv_msg.force.z*ft_conv_msg.force.z);
	std::cout << "Force magnitude: " << force_magnitude << std::endl;

	return desired_roll;
}

// Filter function
void filter_ft_sensor()
{
	int i;
	Eigen::Vector3f sum_forces(0,0,0);
	Eigen::Vector3f sum_momenta(0,0,0);
	
	// Low-Pass Filtering
	for(i = 0; i < n; i++)
	{
		ros::spinOnce();
		forces = (1 - beta)*prev_forces + beta*raw_forces;
		momenta = (1 - beta)*prev_momenta + beta*raw_momenta;

		prev_forces = forces;
		prev_momenta = momenta;

		sum_forces += forces;
		sum_momenta += momenta;	
	}

	// Publish converted measurements for visualization
	ft_conv_msg.force.x = sum_forces[0] / n; 
	ft_conv_msg.force.y = sum_forces[1] / n; 
	ft_conv_msg.force.z = sum_forces[2] / n; 
	ft_conv_msg.torque.x = sum_momenta[0] / n;
	ft_conv_msg.torque.y = sum_momenta[1] / n;
	ft_conv_msg.torque.z = sum_momenta[2] / n;
	ft_pub.publish(ft_conv_msg);
}

// Main function
int main(int argc, char** argv)
{
	ros::init(argc, argv, "high_level_roll_thrust_controller");
	ros::NodeHandle nh;
	ros::Rate loop(500);

	// Subscribers and Publishers 
	ft_sensor_sub = nh.subscribe("/haptic_drone/ft_sensor_topic", 10, &forceTorqueSensorCallback);
	cmd_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust>(mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 10);
	ft_pub = nh.advertise<geometry_msgs::Wrench>("/haptic_drone/ft_converted_measurements", 10);
	
	sleep(5);
		
	std::cout << "\r\n\n\n\033[32m\033[1mCLICK TO START \033[0m" << std::endl; 
	while ((getchar()) != '\n');	
	std::cout << "\r\n\n\n\033[32m\033[1mSTARTED! \033[0m" << std::endl; 
	
	ros::spinOnce();
	prev_forces = raw_forces;
	prev_momenta = raw_momenta;
	int iter = 0;
	double starting_time = ros::Time::now().toSec();

	cmd_msg.roll = 0.0;
	cmd_msg.pitch = 0.0;
	cmd_msg.yaw_rate = 0.0;
	cmd_msg.thrust.z = 16;
	cmd_pub.publish(cmd_msg);

	sleep(2);

	while(ros::ok())         
	{
		filter_ft_sensor();
		
		// Wait N seconds before publishing new position
		// Publish until the reference height is inside the limits		
		if((ros::Time::now().toSec() - starting_time) > pub_time) 
		{
			// Command thrust and roll
			cmd_msg.roll = compute_desired_roll();
			cmd_msg.pitch = 0.0;
			cmd_msg.yaw_rate = 0.0;
			if(iter%2 == 0)
				cmd_msg.thrust.z = 15.7;
			else
				cmd_msg.thrust.z = 15.8;
		
			ROS_INFO("Publishing new ROLL - THRUST: [%f, %f].", cmd_msg.roll, cmd_msg.thrust.z);
		  	cmd_pub.publish(cmd_msg);
		
			iter++;
			starting_time = ros::Time::now().toSec();
		}

		ros::spinOnce();
		loop.sleep();          
	}
	return 0;
}
