#!/usr/bin/env python

import rospy
import socket
import math
import tf
import roslib

from std_msgs.msg import String
from geometry_msgs.msg import Twist

def eye_gaze_publisher():

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('tcp_link_eye')

    listener = tf.TransformListener()
    
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(("172.17.24.46", 12345)) #hololens IP
    
    gaze = Twist()
    
    alpha = 0.6
    alpha2 = 0.6
    alpha3 = 0.7

    old_angle = 0
    old_head_angle = 0
    old_angularZ = 0
    angularZ = 0
    old_angularZ_eye_head = 0

    rate = rospy.Rate(100) # 10hz

    #iter = 0

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

        if abs(head_angle-old_head_angle) > 30:
            alpha2 = 0.92
        else:
            alpha2 = 0.6
        
        head_angle = old_head_angle * alpha2 + (1-alpha2)*head_angle
        old_head_angle = head_angle
        
        #print('angle')
        #print(head_angle)
        #rate.sleep()

        gaze.linear.x = 0.15
        gaze.linear.y = 0.0
        gaze.linear.z = 0.0
        gaze.angular.x = 0.0  
        gaze.angular.y = 0.0
        gaze.angular.z = 0.0
                            
        #rospy.loginfo(gaze)
        #print(angle)
        pub.publish(gaze)
        #rate.sleep()
        #pub.publish(gaze)
        #pub.publish(gaze)
        
        for message in messages:
            flag, angle = message.split('E')

            angle = float(angle)
            angle = old_angle * alpha + (1-alpha)*angle
            #print(flag)

            if flag == '1': #for test
                print(flag)
                print('eye angle')
                print(angle)
                print('head angle')
                print(head_angle)

                gaze.linear.x = 0.15
                gaze.linear.y = 0.0
                gaze.linear.z = 0.0
                gaze.angular.x = 0.0  
                gaze.angular.y = 0.0

                angularZ = angle * -0.015
                angularZ = old_angularZ * alpha2 + (1-alpha2)*angularZ
                
                print('eye angular velocity')
                print(angularZ)
                
                angularZ_eye_head = (head_angle-angle)*0.011 #angularZ
                angularZ_eye_head = old_angularZ_eye_head * alpha3 + (1-alpha3)*angularZ_eye_head
                
                print('eye_head angular velocity')
                print(angularZ_eye_head)
                
                angularZ_eye_head = min((angularZ_eye_head, 0.20))
                angularZ_eye_head = max((angularZ_eye_head, -0.20))

                print('fileter eye_head angular velocity')
                print(angularZ_eye_head)

                gaze.angular.z = angularZ_eye_head
                #if angle > 3:
                #    gaze.angular.z = angle * -0.02
                #elif angle < -3:
                #    gaze.angular.z = angle * -0.02
                #else:
                #    gaze.angular.z = 0.0 
                
                #rospy.loginfo(gaze)
                #print(angle)
                pub.publish(gaze)
                #rospy.spin()
                rate.sleep()
                old_angle = angle
                old_angularZ = angularZ
                old_angularZ_eye_head = angularZ_eye_head

            else:
                print(flag)
            
                gaze.linear.x = 0.15
                gaze.linear.y = 0.0
                gaze.linear.z = 0.0
                gaze.angular.x = 0.0  
                gaze.angular.y = 0.0
                gaze.angular.z = 0.0
                            
                #rospy.loginfo(gaze)
                #print(angle)
                pub.publish(gaze)
                #rospy.spin()
                rate.sleep()
                old_angle = angle
                old_angularZ = angularZ
            
            #if flag == '1':
            #    pub.publish(gaze)
                            
    client.close()

if __name__ == "__main__":
    try:
        eye_gaze_publisher()
    except rospy.ROSInterruptException:
        pass