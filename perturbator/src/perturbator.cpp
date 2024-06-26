#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <algorithm>
#include <ros/console.h>

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
	pert_cb_last_call_secs = ros::Time::now().toSec();
	pert_xlv = joy_mapping(msg.linear.x, 0.9, 0.2);
	pert_zrv = joy_mapping(-msg.linear.y, 0.5, 0.5);
}

void vel_callback(const geometry_msgs::Twist &msg)
{
	cmd_cb_last_call_secs = ros::Time::now().toSec();
	cmd_xlv = msg.linear.x*1.0;
	cmd_zrv = msg.angular.z*1.0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "perturb_node");
	ros::NodeHandle n;

	// publish perturbated values to topic cmd_vel_pert
	twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_pert", 10);

	// subscribe to cmd_vel
	ros::Subscriber sub = n.subscribe("cmd_vel", 10, vel_callback);
	ros::Subscriber sub2 = n.subscribe("spacenav/twist", 10, pert_callback);
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
			ROS_INFO_THROTTLE(5, "*********** cmd_vel data idle");
		}

		if (time_now - pert_cb_last_call_secs > 0.5)
		{
			pert_xlv = 0;
			pert_zrv = 0;
			ROS_INFO_THROTTLE(5, "*********** perturbation data idle");
		}

		geometry_msgs::Twist out_twist;
		out_twist.linear.x = cmd_xlv + pert_xlv;
		out_twist.angular.z = cmd_zrv + pert_zrv;
		twist_pub.publish(out_twist);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}