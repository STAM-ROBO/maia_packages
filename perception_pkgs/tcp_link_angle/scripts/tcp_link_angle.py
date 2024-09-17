#!/usr/bin/env python

import rospy
import roslib
import socket
import math
import tf

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

def eye_gaze_angle_publisher():

    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.init_node('tcp_link_eye_angle')
    
    listener = tf.TransformListener()

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(("172.17.24.46", 12345)) #hololens IP
    
    gaze_angle = PoseStamped()
    cmd = Twist()

    count = 20
    rate = rospy.Rate(50) # 10hz
    cond = 0
    '''
    gaze_angle.header.frame_id = 'base_link'
    gaze_angle.header.stamp = rospy.Time.now()
    gaze_angle.header.seq = 1
            
    gaze_angle.pose.position.x = 1.0
    gaze_angle.pose.position.y = 0.0 #*math.tan(angle*math.pi/180)
    gaze_angle.pose.position.z = 0.0
    gaze_angle.pose.orientation.x = 0.0
    gaze_angle.pose.orientation.y = 0.0
    gaze_angle.pose.orientation.z = 0.0 #*math.sin(angle*math.pi/180/2)
    gaze_angle.pose.orientation.w = 1.0

    rospy.loginfo(gaze_angle)
        #print(angle)
    goal_publisher.publish(gaze_angle)
    '''        
    
    while not rospy.is_shutdown():
        
        response = client.recv(4096)
        eye_gaze = response.decode('utf-8')
        #rospy.loginfo(head_gaze)
        #print(eye_gaze)

        messages = eye_gaze.split('B')
        messages = messages[1:]
        
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/fiducial_0', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        head_angle = 180-(math.pi/2-math.asin(rot[2])*2)*180/math.pi
        print('angle')
        print(head_angle)
        #rate.sleep()
        '''
        cmd.linear.x = 0.15
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0  
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
                            
        #rospy.loginfo(gaze)
        #print(angle)
        cmd_publisher.publish(cmd)
        '''
        
        for message in messages:
            flag, message = message.split('E')
            angle, distance = message.split('D')
                
            if flag == '1': #for test
                
                print(flag)
                #angle = float(angle)
                
                #print(angle)
                distance = float(distance)
                
                gaze_angle.header.frame_id = 'base_link'   #or baselink
                gaze_angle.header.stamp = rospy.Time.now()
                gaze_angle.header.seq = count

                gaze_angle.pose.position.x = distance*math.cos(head_angle*math.pi/180*1)
                gaze_angle.pose.position.y = distance*math.sin(head_angle*math.pi/180*1)
                gaze_angle.pose.position.z = 0.0
                gaze_angle.pose.orientation.x = 0.0
                gaze_angle.pose.orientation.y = 0.0
                gaze_angle.pose.orientation.z = 0.0 #*math.sin(angle*math.pi/180/2)
                gaze_angle.pose.orientation.w = 1.0

                rospy.loginfo(gaze_angle)
                
                goal_publisher.publish(gaze_angle)
                count = count + 1
                cond = 1
                #rate.sleep()

            else:
                print(flag)
            
                '''
                if cond == 0
                    cmd.linear.x = 0.15
                    cmd.linear.y = 0.0
                    cmd.linear.z = 0.0
                    cmd.angular.x = 0.0  
                    cmd.angular.y = 0.0
                    cmd.angular.z = 0.0
                            
                #rospy.loginfo(gaze)
                #print(angle)
                    cmd_publisher.publish(cmd)

                #gaze_angle.header.frame_id = "/map"
                #gaze_angle.header.stamp = rospy.Time.now()
                
                #gaze_angle.pose.position.x = 0.0
                #gaze_angle.pose.position.y = 0.0
                #gaze_angle.pose.position.z = 0.0
                #gaze_angle.pose.orientation.x = 0.0
                #gaze_angle.pose.orientation.y = 0.0
                #gaze_angle.pose.orientation.z = 0.0
                #gaze_angle.pose.orientation.w = 1.0
                            
                #rospy.loginfo(gaze)
                #print(angle)
                #pub.publish(gaze_angle)
                    rate.sleep()
                '''  

        rate.sleep()  
            
    client.close()

if __name__ == "__main__":
    try:
        eye_gaze_angle_publisher()
    except rospy.ROSInterruptException:
        pass