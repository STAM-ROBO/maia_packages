
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
path = '/home/maia/maia_ws/src/perception_pkgs/tcp_link/'

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
    
    print('node started')
    settings = saveTerminalSettings()
    print('node started2')
    rospy.init_node('tcp_link_node', anonymous=True)
    print('node started3')
    pub = rospy.Publisher("gaze_pos", PoseStamped, queue_size=10)
    print('node started4')
    gaze = PoseStamped()
    print('node started5')
    D = decimal.Decimal
    #speed = 0.5
    key_timeout = 0.6
    print('node started6')
    #path = '~/ros/melodic/maia_compact/src/tcp_link/'

    data_to_save = np.empty((0,12), dtype="float64")
    print('node started7')
    all_messages = []
    

    print('node started8')
    #host, port = "10.67.30.36", 9090 #178 38#
    #host, port = "192.168.1.120", 9090 #178 38 # at home
    host, port = "172.17.24.47", 9090 #178 38 # WIFI_LAB  - change every time
    print(f'Trying to connect to HL at {host}:{port}')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print('connection ok')
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
    
    LOCOMOTION_INTENTION = False
    prediction = 77
    threshold = 0.5
    tolerance = 0.03
    historic_average = 0.5

    rate = rospy.Rate(10) # 10hz

    try:
        while (exp_running):
            '''
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
            
            '''
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
                        
                        #rospy.loginfo(message)
                        
                        if (message != old_message):
                            old_message = message 
                            x_value, message = message.split('Y')
                            y_value, message = message.split('Z')
                            z_value, message = message.split('A')
                            a_value, message = message.split('B')
                            b_value, message = message.split('C')
                            c_value, message = message.split('P')
                            p_value, message = message.split('W') #euler angles
                            w_value, r_value = message.split('R')

                            if r_value == "":
                                r_value = 0

                            next_row = np.hstack((time.time(), unity_time, x_value, y_value, z_value, a_value, 
                                                  b_value, c_value, p_value, w_value, r_value, prediction))
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
                            pub.publish(gaze)
                            #rate.sleep()
                            
                            old_x_value = x_value

            if len(data_to_save) >= 30:
                print(prediction)
                input_data = data_to_save[-30:, 2:5].astype(float) # last 2 sec at 66fps
                input_data = np.delete(input_data[-30:], np.arange(0, input_data.shape[0], 3), axis=0)
                eye_vector = vec_to_euler(input_data[:,0], input_data[:,1], input_data[:,2])
                eye_yaw = eye_vector[0]

                input_data = np.array(eye_vector).T.reshape(1, 20, -1)

                eyes = torch.Tensor(input_data)
                y_pred = model(motion, eyes, eeg, waves)
                prediction = F.sigmoid(y_pred).detach().cpu().numpy()[0,1] #Chance of this beeing a target
                #out_str = str(np.round(prediction))
            
                if prediction > threshold + tolerance:
                    if not LOCOMOTION_INTENTION:
                        sock.sendall("On".encode())
                        LOCOMOTION_INTENTION = True
                    
                if prediction < threshold - tolerance:
                    if LOCOMOTION_INTENTION:
                        sock.sendall("Of".encode())
                        LOCOMOTION_INTENTION = False
            
            if len(data_to_save) >= 530:
                historic_average = np.mean(data_to_save[30:,11].astype(float).round()[-500:])
            
                if historic_average > 0.9:
                    threshold = historic_average - 0.4
                elif historic_average < 0.45:
                    threshold = historic_average + 0.05
                else:
                    threshold = 0.5
            
            if len(data_to_save) > 3000:
                exp_running = False     
            #print('okokokokokok')
        
            '''
            if key == 'e':
                print("exit communication")
                exp_running = False     
            '''
                    
    #except KeyboardInterrupt:  # as e:
    #    print("Ending program")
    #   # raise e

    except rospy.ROSInterruptException:
        #pass
        print("Ending Program")

    data_to_save = data_to_save.astype('float64')

    #f_data_to_save = data_to_save.ravel()
    #f_data_to_save = pd.to_numeric(f_data_to_save, errors='coerce')

    np.savetxt(path + 'data/data' + time.strftime("%D_%H_%M_%S").replace('/', '_') + '.csv', data_to_save, delimiter=";", fmt='%f')
