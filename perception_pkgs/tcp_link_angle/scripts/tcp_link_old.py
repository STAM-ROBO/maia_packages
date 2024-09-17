#!/usr/bin/env python
'''
@package   :   tcp_link
@File      :   tcp_link
@Author    :   Je Hyung y Gianny
@Contact   :   jehyung.jung@tecnalia.com
@Copyright :   (C)Copyright Tecnalia 2023

@Desc      :   TCP/IP communication bewteen hololens 2 and ROS PC for eye gaze
'''

import rospy
import socket
import time
import numpy as np
import struct
import keyboard
import time
#import pandas as pd
import decimal

import torch

import sys
from select import select
from geometry_msgs.msg import PoseStamped
# from scipy.spatial.transform import Rotation  #change eulerangle to quaternion

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

import os

#path = 'C:/Users/g_brem02/sciebo/Promotion/Hololens/'
path = '/home/104135@TRI.LAN/ros/melodic/maia_compact/src/tcp_link/'

def vec_to_euler(x, y, z):
  yaw = np.arctan2(x, z)

  width =  np.sqrt(np.square(x) + np.square(z))
  pitch = np.arctan(y/width)
  return yaw*180/np.pi, pitch*180/np.pi

#Preparation
from sklearn.preprocessing import StandardScaler
from joblib import load
import torch.nn.functional as F

modelpath = path + 'model/' 
import sys
os.chdir(modelpath)
sys.path.append(os.getcwd())
import MODEL


def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

if __name__ == "__main__":

    settings = saveTerminalSettings()
    
    pub = rospy.Publisher("gaze_pos", PoseStamped, queue_size=10)
    rospy.init_node('tcp_link', anonymous=True)

    gaze = PoseStamped()
    D = decimal.Decimal
    #speed = 0.5
    key_timeout = 0.6

    #path = '~/ros/melodic/maia_compact/src/tcp_link/'

    data_to_save = np.empty((0,11), dtype="float64")
    all_messages = []

    #host, port = "10.67.30.36", 9090 #178 38#
    host, port = "192.168.1.120", 9090 #178 38 # at home
    #host, port = "172.17.24.46", 9090 #178 38 # WIFI_LAB  - change every time
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    
    checkpoint = torch.load(modelpath + 'transformer_eye.pth')
    model = MODEL.NN_Transformer_eye() #noEEG 
    model.load_state_dict(checkpoint)
    model.train(False)


    timestamp = np.arange(-2,0,0.1)

    eye_data = np.random.randn(1, 20, 2)
    eyes = torch.Tensor(eye_data)

    motion = torch.randn(1, 20, 2)
    eeg = torch.randn(1, 1) 
    waves = torch.randn(1, 1)

    #scaler_velocities = StandardScaler()
    #scaler_velocities = load(modelpath+  'scaler_velocities.bin')
    scaler_eyes = StandardScaler()
    scaler_eyes = load(modelpath+ 'scaler_eyes.bin')

############################################################
    current_data = "empty"
    lastOutput = 0
    ############################################################
    time.sleep(0.05)
    exp_running = True
    old_message = "empty"
    out_str = "Of" 
    q_pressed = False
    w_pressed = False
    old_x_value = -10000

    rate = rospy.Rate(10) # 10hz

    try:
        while (exp_running):

            key = getKey(settings, key_timeout)

            #if keyboard.is_pressed('q'):
            if key == 'q':
                if not q_pressed:
                    sock.sendall("On".encode())
                    print("on")
                    q_pressed = True
            else:
                q_pressed = False
            
            #if keyboard.is_pressed('w'):
            if key == 'w':
                if not w_pressed:
                    sock.sendall("Of".encode())
                    print("off")
                    w_pressed = True
            else:
                w_pressed = False
            
            receivedData = sock.recv(2048) #.decode("UTF-8") #receiveing data in Byte fron C#, and converting it to String
            receivedString = receivedData.decode()

            #rospy.loginfo(receivedString)
        
            if (receivedString == "Ending"):
                exp_running = False
                break
            else:
                messages = receivedString.split('T')
                messages = messages[1:]

                #rospy.loginfo(messages)

                for message in messages:
                    if ('R' in message): #Only full messages; better would be an end signal
                        unity_time, message = message.split('X')
                        print('000', message)
                        
                        #rospy.loginfo(message)
                        
                        if (message != old_message):
                            old_message = message 
                            x_value, message = message.split('Y')

                            print('xxx'+ x_value)

                            y_value, message = message.split('Z')
                            z_value, message = message.split('A')
                            a_value, message = message.split('B')
                            b_value, message = message.split('C')
                            c_value, message = message.split('P')
                            p_value, message = message.split('W') #euler angles
                            w_value, r_value = message.split('R')
                        
                            next_row = np.hstack((time.time(), unity_time, x_value, y_value, z_value, a_value, 
                                                  b_value, c_value, p_value, w_value, r_value))
                            data_to_save = np.append(data_to_save, np.expand_dims(next_row, axis=0), axis=0)
                            
                            gaze.header.frame_id = "/hololens"
                            gaze.header.stamp =  rospy.Time.now() #float(unity_time)
                            gaze.pose.position.x = float(a_value)
                            gaze.pose.position.y = float(b_value)
                            gaze.pose.position.z = float(c_value)
                            gaze.pose.orientation.x = 0   #quaternion - need to change euler angle to quaternion
                            gaze.pose.orientation.y = 0
                            gaze.pose.orientation.z = 0
                            gaze.pose.orientation.w = 0
                            
                            #rospy.loginfo(gaze)
                            #pub.publish(gaze)
                            #rate.sleep()
                            
                            x_value = float(x_value)

                            if (old_x_value != x_value):
                                print(x_value)
                                print('hhhhh') # for test

                            old_x_value = x_value
            if key == 'e':
                print("exit communication")
                exp_running = False                
    #except KeyboardInterrupt:  # as e:
    #    print("Ending program")
    #   # raise e

    except rospy.ROSInterruptException:
        pass

    #data_to_save = data_to_save.astype('float64')

    #f_data_to_save = data_to_save.ravel()
    #f_data_to_save = pd.to_numeric(f_data_to_save, errors='coerce')

    #np.savetxt(path + 'data/data' + time.strftime("%D_%H_%M_%S").replace('/', '_') + '.csv', f_data_to_save, delimiter=";", fmt='%f')
