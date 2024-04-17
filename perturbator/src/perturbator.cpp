#include "ros/ros.h"
#include<geometry_msgs/Twist.h>
#include<math.h>
#include<algorithm>
#include <ros/console.h>

ros::Publisher twist_pub;

float p_xlv=0, p_zrv=0;
double pert_cb_last_call_secs=0;

#define MAX_TX_VEL 0.5
#define MAX_RZ_VEL 0.5

void pert_callback(const geometry_msgs::Twist& msg)
{
	p_xlv = std::min(MAX_TX_VEL, std::max(-MAX_TX_VEL, msg.linear.x));
	p_zrv = std::min(MAX_RZ_VEL, std::max(-MAX_RZ_VEL, msg.angular.z));
	pert_cb_last_call_secs=ros::Time::now().toSec();
}

void vel_callback(const geometry_msgs::Twist& msg)
{
	float xlv = msg.linear.x;
	float zrv = msg.angular.z;
	double time_now=ros::Time::now().toSec();
	
	/*
	inject the perturbation here
	TODO
	*/
	if(pert_cb_last_call_secs-time_now>1.0)
	{
		xlv += p_xlv;
		zrv += p_zrv;
	}
	else
	{
		ROS_INFO_THROTTLE(1, "pert_vel data not updated in the last 1.0 seconds. Suspending correction.");
	}
	
	geometry_msgs::Twist out_twist;
	out_twist.linear.x=xlv;
	out_twist.angular.z=zrv;
	twist_pub.publish(out_twist);
}
 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "perturb_node");
	ros::NodeHandle n;

	//publish perturbated values to topic cmd_vel_pert
	twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_pert", 10);
	
	//subscribe to cmd_vel
	ros::Subscriber sub = n.subscribe("cmd_vel", 10, vel_callback);
	ros::Subscriber sub2 = n.subscribe("pert_vel", 10, pert_callback);
	
	ROS_INFO("perturb_node started");
	ros::spin();
	return 0;
}