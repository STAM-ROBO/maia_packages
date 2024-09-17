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
ros::Subscriber pert_sub, cmd_vel_sub, gaze_vel_sub; 

float pert_xlv = 0, pert_zrv = 0, vertical_press, old_vertical_press;
float cmd_xlv = 0, cmd_zrv = 0;
float gaze_xlv = 0, gaze_zrv = 0;   //gaze velocity for hybrid control
double pert_cb_last_call_secs = 0;
double cmd_cb_last_call_secs = 0;
double gaze_cb_last_call_secs = 0;  //gaze velocity for hybrid control

#define MAX_TX_VEL 1.0f
#define MAX_RZ_VEL 1.0f

float joy_mapping(float x, float f3, float f1)
{
	return f3 * x * x * x + f1 * x;
}

void pert_callback(const geometry_msgs::Twist &msg)
{
	float alpha=0.05;
	pert_cb_last_call_secs = ros::Time::now().toSec();
	pert_xlv = (alpha) * joy_mapping(msg.linear.x/0.3, 0.3, 0.1) + (1-alpha) * pert_xlv;
	pert_zrv = (alpha) * joy_mapping(msg.linear.y/0.3, 0.35, 0.15) + (1-alpha) * pert_zrv;
	//ROS_INFO_THROTTLE(5, "pert_callback time called: %f", pert_cb_last_call_secs);

	vertical_press = msg.linear.z;
}

void vel_callback(const geometry_msgs::Twist &msg)
{
	cmd_cb_last_call_secs = ros::Time::now().toSec();
	cmd_xlv = msg.linear.x;
	cmd_zrv = msg.angular.z;
	//ROS_INFO_THROTTLE(5, "vel_callback time called: %f", cmd_cb_last_call_secs);
}

void gaze_vel_callback(const geometry_msgs::Twist &msg)
{
	float alpha=0.1;
	gaze_cb_last_call_secs = ros::Time::now().toSec();
	gaze_xlv = (alpha) * msg.linear.x + (1-alpha)*gaze_xlv;
	gaze_zrv = (alpha) * msg.angular.z+ (1-alpha)*gaze_zrv;
	//ROS_INFO_THROTTLE(5, "vel_callback time called: %f", cmd_cb_last_call_secs);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "perturb_node");
	ros::NodeHandle nh;
	int isready = 0;
	int go_stop = 0;
	std::string perturb_topic_name;
	std::string output_topic_name;

	pert_sub = nh.subscribe("spacenav/twist", 10, pert_callback);
	cmd_vel_sub = nh.subscribe("cmd_vel", 10, vel_callback);
	gaze_vel_sub = nh.subscribe("gaze_vel", 10, gaze_vel_callback);
	twist_pub = nh.advertise<geometry_msgs::Twist>("motor_vel", 10);
	isready=2;
		
	/*if (nh.getParam("perturb_topic", perturb_topic_name)) 
	{
		//ros::Subscriber pert_sub = nh.subscribe("spacenav/twist", 10, pert_callback);
		ros::Subscriber pert_sub = nh.subscribe(perturb_topic_name, 10, pert_callback);
		
		ROS_INFO("Subscribed to perturbation topic: %s", perturb_topic_name.c_str());
		isready++;
	}

	if (nh.getParam("output_topic", output_topic_name)) 
	{
		twist_pub = nh.advertise<geometry_msgs::Twist>(output_topic_name, 10);
		ROS_INFO("Publishing to output topic: %s", output_topic_name.c_str());
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
	*/
	
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

			if (time_now - gaze_cb_last_call_secs > 0.5)
			{
				gaze_xlv = 0;
				gaze_zrv = 0;
				ROS_INFO_THROTTLE(10, "*********** cmd_vel data idle");
			}
            
			// to use vertical signal as stop and go button
			if ((abs(vertical_press) > 0.6) && abs(vertical_press - old_vertical_press) > 0.2) 
			{	
				go_stop ++; 
			}
            
			old_vertical_press = vertical_press;

			
			if (go_stop%2 == 1)
			{
				cmd_xlv = 0;
				cmd_zrv = 0;
				gaze_xlv = 0;
				gaze_zrv = 0;
				//ROS_INFO("stop \n");
			} 

			geometry_msgs::Twist out_twist;
			//out_twist.linear.x = cmd_xlv + pert_xlv;
			//out_twist.angular.z = cmd_zrv + pert_zrv;
            out_twist.linear.x = fmin(0.25, fmax(-0.25, (cmd_xlv + pert_xlv + gaze_xlv)));
			out_twist.angular.z = fmin(0.5, fmax(-0.5,(cmd_zrv + pert_zrv + gaze_zrv)));

			twist_pub.publish(out_twist);

			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	else return 0;
}