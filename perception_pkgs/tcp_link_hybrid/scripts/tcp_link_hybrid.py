#!/usr/bin/env python

import rospy
import roslib
import socket
import math
import tf

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

'''
def shutdown_hook(pub1):
    rospy.loginfo("Shutting down node...")
    zero_msg = Twist()
    gaze_vel.linear.x = 0.15
    gaze_vel.linear.y = 0.0
    gaze_vel.linear.z = 0.0
    gaze_vel.angular.x = 0.0  
    gaze_vel.angular.y = 0.0
    
    pub1.publish(zero_msg)
    rospy.sleep(1)  # Give some time to publish the messages
'''
motor_v = None

def callback_motor(vel):
    global motor_v
    motor_v = vel

def eye_gaze_hybrid_publisher():

    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    gaze_vel_publisher = rospy.Publisher('gaze_vel', Twist, queue_size=10)

    motor_vel_subscriber = rospy.Subscriber('/motor_vel', Twist, callback_motor)
    
    rospy.init_node('tcp_link_hybrid')
    
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(("172.17.24.46", 12345)) #hololens IP

    listener = tf.TransformListener()
    
    gaze_destination = PoseStamped()
    gaze_vel = Twist()

    count = 20
    rate = rospy.Rate(10) # 10hz
    cond = 0

    alpha = 0.6
    alpha2 = 0.6
    alpha3 = 0.7

    old_angle = 0
    old_head_angle = 0
    old_angularZ = 0
    angularZ = 0
    old_angularZ_eye_head = 0

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
    on_target = 0
    start = 1

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
        
        #head_angle = 0
        head_angle = 180-(math.pi/2-math.asin(rot[2])*2)*180/math.pi
        
        if abs(head_angle-old_head_angle) > 30:
            alpha2 = 0.92
        else:
            alpha2 = 0.6
        
        head_angle = old_head_angle * alpha2 + (1-alpha2)*head_angle
        old_head_angle = head_angle
        
        print('head angle')
        print(head_angle)
    
        for message in messages:
            flag, message = message.split('E')
            angle, distance = message.split('D')

            angle = float(angle)
            angle = old_angle * alpha + (1-alpha)*angle
            
            if flag == '2': #for test
                
                on_target = 1

                print(flag)
                #angle = float(angle)
                print(angle)
                distance = float(distance)
                
                gaze_destination.header.frame_id = 'base_link'
                gaze_destination.header.stamp = rospy.Time.now()
                gaze_destination.header.seq = count

                gaze_destination.pose.position.x = distance*math.cos(angle*math.pi/180*-1)
                gaze_destination.pose.position.y = distance*math.sin(angle*math.pi/180*-1)
                gaze_destination.pose.position.z = 0.0
                gaze_destination.pose.orientation.x = 0.0
                gaze_destination.pose.orientation.y = 0.0
                gaze_destination.pose.orientation.z = 0.0 #*math.sin(angle*math.pi/180/2)
                gaze_destination.pose.orientation.w = 1.0

                rospy.loginfo(gaze_destination)
                
                goal_publisher.publish(gaze_destination)
                count = count + 1

                old_angle = angle
                old_angularZ = angularZ
                #old_angularZ_eye_head = angularZ_eye_head

                #rate.sleep()

                gaze_vel.linear.x = 0.0
                gaze_vel.linear.y = 0.0
                gaze_vel.linear.z = 0.0
                gaze_vel.angular.x = 0.0  
                gaze_vel.angular.y = 0.0
                gaze_vel.angular.z = 0.0

                gaze_vel_publisher.publish(gaze_vel)

            elif flag == '1' and on_target == 0 :
                print(flag)
                gaze_vel.linear.x = 0.15
                gaze_vel.linear.y = 0.0
                gaze_vel.linear.z = 0.0
                gaze_vel.angular.x = 0.0  
                gaze_vel.angular.y = 0.0
                            
                #rospy.loginfo(gaze)
                #print(angle)

                print(flag)

                angularZ = angle * -0.015
                angularZ = old_angularZ * alpha2 + (1-alpha2)*angularZ
                
                #print('eye angular velocity')
                #print(angularZ)
                
                angularZ_eye_head = (head_angle-angle)*0.011 #angularZ
                angularZ_eye_head = old_angularZ_eye_head * alpha3 + (1-alpha3)*angularZ_eye_head
                
                #print('eye_head angular velocity')
                #print(angularZ_eye_head)
                
                angularZ_eye_head = min((angularZ_eye_head, 0.20))
                angularZ_eye_head = max((angularZ_eye_head, -0.20))

                #print('fileter eye_head angular velocity')
                #print(angularZ_eye_head)

                gaze_vel.angular.z = angularZ_eye_head
                #if angle > 3:
                #    gaze.angular.z = angle * -0.02
                #elif angle < -3:
                #    gaze.angular.z = angle * -0.02
                #else:
                #    gaze.angular.z = 0.0 
                
                #rospy.loginfo(gaze)
                #print(angle)
                gaze_vel_publisher.publish(gaze_vel)

                #rospy.spin()
                rate.sleep()
                old_angle = angle
                old_angularZ = angularZ
                old_angularZ_eye_head = angularZ_eye_head

            if (abs(motor_v.linear.x)+abs(motor_v.linear.y)+abs(motor_v.linear.z)) < 0.01:
                on_target = 0
                print(motor_v.linear.x)

    client.close()

if __name__ == "__main__":
    try:
        eye_gaze_hybrid_publisher()
    except rospy.ROSInterruptException:
        pass