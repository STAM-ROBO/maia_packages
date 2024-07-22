#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <algorithm>
#include <ros/console.h>
#include <string>

/*
Perturbation node
Subscribes to:
- cmd_vel motor velocities from navigation controller
- [perturb_twist_name] topic providing the perturbatin velocities

Publishes the result of the sum

	cmd_vel + [perturb_twist_name]

to topic [ouput_topic] at a rate of 30Hz

*/

ros::Publisher twist_pub;

float pert_xlv = 0, pert_zrv = 0;
float cmd_xlv = 0, cmd_zrv = 0;
double pert_cb_last_call_secs = 0;
double cmd_cb_last_call_secs = 0;

#define MAX_TX_VEL 1.0f
#define MAX_RZ_VEL 1.0f

float joy_mapping(float x, float f3, float f1)
{
	return f3 * x * x * x + f1 * x;
}

void pert_callback(const geometry_msgs::Twist &msg)
{
	float alpha=0.1;
	pert_cb_last_call_secs = ros::Time::now().toSec();
	pert_xlv = (alpha) * joy_mapping(msg.linear.x, 0.9, 0.1) + (1-alpha) * pert_xlv;
	pert_zrv = (alpha) * joy_mapping(msg.linear.y, 0.9, 0.1) + (1-alpha) * pert_zrv;
}

void vel_callback(const geometry_msgs::Twist &msg)
{
	cmd_cb_last_call_secs = ros::Time::now().toSec();
	cmd_xlv = msg.linear.x;
	cmd_zrv = msg.angular.z;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "perturb_node");
	ros::NodeHandle nh("~");
	int isready=0;
	std::string perturb_topic_name;
	std::string output_topic_name;

	if (nh.getParam("perturb_topic", perturb_topic_name)) 
	{
		ros::Subscriber sub2 = nh.subscribe(perturb_topic_name, 10, pert_callback);
		ROS_INFO("Subscribed to perturbation topic: %s", perturb_topic_name.c_str());
		isready++;
	}

	if (nh.getParam("output_topic", output_topic_name)) 
	{
		twist_pub = nh.advertise<geometry_msgs::Twist>(output_topic_name, 10);
		ROS_INFO("Subscribed to output topic: %s", output_topic_name.c_str());
		isready++;
	}

	if(isready == 2)
	{
		// subscribe to cmd_vel
		ROS_INFO("PERTURBATOR READY");
		ros::Subscriber sub = nh.subscribe("cmd_vel", 10, vel_callback);
	}
	else
	{
		ROS_INFO("PERTURBATOR PARAM ERROR");
	}
	
	if(isready == 2)
	{
		ros::Rate loop_rate(30);
		double time_now = 0;
		ROS_INFO("*********** perturb_node started");
		while (ros::ok())
		{
			time_now = ros::Time::now().toSec();

			if (time_now - cmd_cb_last_call_secs > 0.5)
			{
				cmd_xlv = 0;
				cmd_zrv = 0;
				ROS_INFO_THROTTLE(10, "*********** cmd_vel data idle");
			}

			if (time_now - pert_cb_last_call_secs > 0.5)
			{
				pert_xlv = 0;
				pert_zrv = 0;
				ROS_INFO_THROTTLE(10, "*********** perturbation data idle");
			}

			geometry_msgs::Twist out_twist;
			out_twist.linear.x = cmd_xlv + pert_xlv;
			out_twist.angular.z = cmd_zrv + pert_zrv;
			twist_pub.publish(out_twist);

			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	else return 0;
}