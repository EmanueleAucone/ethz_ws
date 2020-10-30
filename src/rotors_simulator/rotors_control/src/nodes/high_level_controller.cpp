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
#include <math.h>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <Eigen/Geometry>

using namespace std;

//////////////////// Global variables ////////////////////

// Pubs and subs
ros::Subscriber pos_sub;
ros::Publisher pos_pub;
ros::Subscriber ft_sensor_sub;
ros::Publisher ft_pub;
const int PUB_TIME = 2;				// New waypoint is published every 'PUB_TIME' seconds

// Force/torque variables
geometry_msgs::Wrench ft_conv_msg;
Eigen::Vector3f raw_forces;
Eigen::Vector3f raw_momenta;
double force_magnitude;
const double MAX_LATERAL_FORCE_THRESHOLD = 0.3; // [N]
const double DETECTED_CONTACT_THRESHOLD = 0.01; // [N]

// Position variables
Eigen::Vector3f pos;				// UAV global position

// Filter variables
const double BETA = 0.15;			// Low-Pass FIlter main parameter
const int n = 1;
Eigen::Vector3f prev_forces;
Eigen::Vector3f prev_momenta;
Eigen::Vector3f forces;
Eigen::Vector3f momenta;

// Position controller variables
geometry_msgs::PoseStamped pos_msg;
double desired_roll;
int effort;
bool slide_flag = false;
const int MAX_EFFORT = 4;	
const double PUSH = 0.1;
const double LIMIT_HEIGHT = 3.5;		// Max height waypoint [m]
const double CONTROL_RADIUS = 0.5;		// Gain: radius of the control circle
const double ROLL_THESHOLD = 0.0043633; 	// Min roll before switch to sliding mode [rad], it corresponds to 0.25 [deg]
const double MAX_ROLL = 0.05236; 		// Saturation value [rad], it corresponds to 3 [deg]


//////////////////// Functions ////////////////////

// Callback for drone position
void positionCallback(const geometry_msgs::PointStamped::ConstPtr& position_msg)
{
	// Store data about position for control strategy
	pos[0] = position_msg->point.x;
	pos[1] = position_msg->point.y;
	pos[2] = position_msg->point.z;
}

// Callback for force/torque measurement acquisition
void forceTorqueSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& ft_msg)
{
	// Store data about forces/torques for control strategy
	raw_forces[0] = (1000) * ft_msg->wrench.force.x;
	raw_forces[1] = (1000) * ft_msg->wrench.force.y;
	raw_forces[2] = (1000) * ft_msg->wrench.force.z;
	raw_momenta[0] = (1000) * ft_msg->wrench.torque.x;
	raw_momenta[1] = (1000) * ft_msg->wrench.torque.y;
	raw_momenta[2] = (1000) * ft_msg->wrench.torque.z;
}

// Compute the desired roll angle
void compute_desired_roll()
{
	force_magnitude = sqrt( ft_conv_msg.force.x*ft_conv_msg.force.x + 
				ft_conv_msg.force.y*ft_conv_msg.force.y + 
				ft_conv_msg.force.z*ft_conv_msg.force.z);
	cout << "Force along Y [N]: " << ft_conv_msg.force.y << endl;
	cout << "Force along Z [N]: " << ft_conv_msg.force.z << endl;
	cout << "Force magnitude [N]: " << force_magnitude << endl;

	desired_roll = atan2(ft_conv_msg.force.y, fabs(ft_conv_msg.force.z));
	// Saturation
	if(desired_roll > MAX_ROLL)
		desired_roll = MAX_ROLL;
	else if(desired_roll < -MAX_ROLL)
		desired_roll = -MAX_ROLL;
	cout << "Desired roll angle: " << (180/M_PI) * desired_roll << " [deg], " << desired_roll << " [rad]" << endl;
}

// Filtering force/torque sensor
void filter_ft_sensor()
{
	int i;
	Eigen::Vector3f sum_forces(0,0,0);
	Eigen::Vector3f sum_momenta(0,0,0);
	
	// Low-Pass Filtering
	for(i = 0; i < n; i++)
	{
		ros::spinOnce();
		forces = (1 - BETA)*prev_forces + BETA*raw_forces;
		momenta = (1 - BETA)*prev_momenta + BETA*raw_momenta;

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

// Fly up and push if a contact is detected
void fly_push()
{
	// Mantain you Y position
	pos_msg.pose.position.y = pos[1];
	
	// Contact detection checking
	if(fabs(ft_conv_msg.force.y) > DETECTED_CONTACT_THRESHOLD && fabs(ft_conv_msg.force.y) < MAX_LATERAL_FORCE_THRESHOLD)
	{
		cout << "CONTACT DETECTED: PUSH!" << endl;
		// Adjust the pushing force 	
		effort++;	
	}				
	else
		effort = 1;

	// Increasing the height waypoint				
	pos_msg.pose.position.z = pos[2] + effort*PUSH;	
	pos_pub.publish(pos_msg);

	slide_flag = true;
}

// Slide along the obstacle adjusting new position waypoint depending on the desired roll
void slide()
{
	// Change Y coordinate depending on the "circle formulation" 
	if(desired_roll > 0.0)
		pos_msg.pose.position.y = pos[1] - (CONTROL_RADIUS * cos(desired_roll));
	else
		pos_msg.pose.position.y = pos[1] + (CONTROL_RADIUS * cos(desired_roll));

	// Change Z coordinate depending on the "circle formulation" 
	pos_msg.pose.position.z = pos[2] + (CONTROL_RADIUS * sin(desired_roll));
	pos_pub.publish(pos_msg);
	cout << "BETTER STOP PUSHING. LET'S SLIDE!" << endl;

	slide_flag = false;
}

// Main function
int main(int argc, char** argv)
{
	ros::init(argc, argv, "high_level_controller");
	ros::NodeHandle nh;
	ros::Rate loop(500);

	// Subscribers and Publishers 
	ft_sensor_sub = nh.subscribe("/haptic_drone/ft_sensor_topic", 10, &forceTorqueSensorCallback);
	pos_sub = nh.subscribe("/haptic_drone/odometry_sensor1/position", 10, &positionCallback);
	pos_pub = nh.advertise<geometry_msgs::PoseStamped>(mav_msgs::default_topics::COMMAND_POSE, 10);
	ft_pub = nh.advertise<geometry_msgs::Wrench>("/haptic_drone/ft_converted_measurements", 10);
	
	sleep(5);

	ros::spinOnce();

	cout << "\r\n\n\n\033[32m\033[1mCLICK TO START \033[0m" << endl; 
	while ((getchar()) != '\n');	
	cout << "\r\n\n\n\033[32m\033[1mSTARTED! \033[0m" << endl; 
	
	effort = 1;
	prev_forces = raw_forces;
	prev_momenta = raw_momenta;
	double starting_time = ros::Time::now().toSec();
	
	while(ros::ok())         
	{
		ros::spinOnce();
		filter_ft_sensor();
				
		// Condition verified until the new height waypoint is inside the structure limits		
		if((pos[2] < LIMIT_HEIGHT) && ((ros::Time::now().toSec() - starting_time) > PUB_TIME)) 
		{
			// Compute desired roll needed to change the waypoint
			compute_desired_roll();

			// Command position
			pos_msg.header.stamp = ros::Time::now();
			pos_msg.pose.position.x = pos[0];
			
			// Fly up!
			if(fabs(desired_roll) < ROLL_THESHOLD && effort < MAX_EFFORT)
				fly_push();
			// Slide!
			else	
			{
				effort = 1;
				// To avoid undesired slides back
				if(slide_flag)
				{
					slide();
					sleep(4);
				}				
				else
					fly_push();
				
			}
			cout << "Current position = X: " << pos[0] << ", Y: " << pos[1] << ", Z: " << pos[2] << endl;
			cout << "Publishing new waypoint! X: " << pos_msg.pose.position.x << ", Y: " << pos_msg.pose.position.y << ", Z: " << pos_msg.pose.position.z << endl;
			cout << "--------------------------------------------------------------" << endl;
		  						
				
			starting_time = ros::Time::now().toSec();
			
		}
		loop.sleep();          
	}
	return 0;
}
