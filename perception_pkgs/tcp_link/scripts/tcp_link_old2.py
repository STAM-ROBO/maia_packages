#!/usr/bin/env python
'''
@package   :   tcp_link_eye_gaze
@File      :   tcp_link_eye_gaze
@Author    :   Je Hyung y Gianny
@Contact   :   jehyung.jung@tecnalia.com
@Copyright :   (C)Copyright Tecnalia 2023

@Desc      :   TCP/IP communication bewteen hololens 2 and ROS PC for eye gaze
'''

#Preparation
import rospy
from geometry_msgs.msg import PoseStamped

import socket
import time
import numpy as np
import warnings
import os
path = '/home/maia/maia_ws/src/perception_pkgs/tcp_link/'
#AI Stuff
from sklearn.preprocessing import StandardScaler
from joblib import load
import torch
import torch.nn.functional as F
modelpath = path + 'model/' 
import sys
os.chdir(modelpath)
sys.path.append(os.getcwd())
import MODEL
checkpoint = torch.load(modelpath + 'transformer_eye.pth')
model = MODEL.NN_Transformer_eye() #noEEG 
model.load_state_dict(checkpoint)
model.train(False)
#scaler_velocities = StandardScaler()
#scaler_velocities = load(modelpath+  'scaler_velocities.bin')
scaler_eyes = StandardScaler()
scaler_eyes = load(modelpath+ 'scaler_eyes.bin')

print('Everything loaded successfully')

def vec_to_euler(x, y, z):
  yaw = np.arctan2(x, z)

  width =  np.sqrt(np.square(x) + np.square(z))
  pitch = np.arctan(y/width)
  return yaw*180/np.pi, pitch*180/np.pi

def eye_gaze_publisher(model, scaler_eyes):
    threshold = 0.5
    tolerance = 0.03

    pub = rospy.Publisher('eye_gaze_pos', PoseStamped, queue_size=10)
    rospy.init_node('tcp_link_eye')
    
    #Connection
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(("172.17.24.47", 9090))

    #Data
    data_to_save = np.empty((0,12), dtype="float64")
    binned_data = np.empty((0,12), dtype="float64")
    clear_binned_data = np.empty((0,12), dtype="float64")
    #AI
    eye_data = np.random.randn(1, 20, 2)
    eyes = torch.Tensor(eye_data)
    motion = torch.randn(1, 20, 2)
    eeg = torch.randn(1, 1) 
    waves = torch.randn(1, 1)

    LOCOMOTION_INTENTION = False
    prediction = 77
    highest_known_bin_edge = 0
    highest_known_bin = 0
    historic_average = np.copy(threshold)
    
    old_message = "empty"
    gaze = PoseStamped()
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        receivedData = sock.recv(2048) #.decode("UTF-8") #receiveing data in Byte fron C#, and converting it to String
        receivedString = receivedData.decode()
        #rospy.loginfo(head_gaze)
    
        messages = receivedString.split('T')
        messages = messages[1:]
         
        for message in messages:
            if ('R' in message): #Only full messages; better would be an end signal
                unity_time, message = message.split('X')
                if (message != old_message):
                    old_message = message 
                    x_value, message = message.split('Y')
                    y_value, message = message.split('Z')
                    z_value, message = message.split('A')
                    a_value, message = message.split('B')
                    b_value, message = message.split('C')
                    c_value, message = message.split('P')
                    p_value, message = message.split('W')
                    w_value, r_value = message.split('R')
                   
                    #Catch cut off messages    
                    if r_value == "":
                        r_value = "0"
                        
                    #Catch Connection loss
                    if len(data_to_save) > 5:
                        if (float(unity_time) - data_to_save[-1,1].astype('float64')) >= 500:
                            print("Connection Lost for over 1 second")
                        elif (float(unity_time) - data_to_save[-1,1].astype('float64')) >= 500:
                            print("Critical Package Loss")
                        elif (float(unity_time) - data_to_save[-1,1].astype('float64')) >= 200:
                            print("Severe Package Loss")
                            
                    next_row = np.hstack((time.time(), unity_time, x_value, y_value, z_value, a_value, 
                                          b_value, c_value, p_value, w_value, r_value, prediction))
                    data_to_save = np.append(data_to_save, np.expand_dims(next_row, axis=0 ), axis=0)
                    
                    
                    #bin new data
                    time_column = data_to_save[:,1].astype('float64')/100 #deciseconds
                    bin_edges = np.arange(np.floor(time_column.min()), time_column.max(), 1)*100
                    bin_indices = np.digitize(data_to_save[:,1], bin_edges) - 1 
                    with warnings.catch_warnings():
                        warnings.filterwarnings('ignore', r'Mean of empty slice.')
                        next_binned_row = []
                        for i in range(highest_known_bin_edge, len(bin_edges)-1):
                            new_chunk_to_bin = data_to_save[highest_known_bin:,:].astype('float64')[bin_indices[highest_known_bin:] == i, :]
                            #next_binned_row.append(np.mean(new_chunk_to_bin, axis=0))
                            #Would need to be the angular mean
                            if new_chunk_to_bin.size > 0:
                                middle_index = len(new_chunk_to_bin) // 2  
                                next_binned_row.append(new_chunk_to_bin[middle_index])
                        next_binned_row = np.array(next_binned_row)
                    
                    #concatenate binned data
                    if len(next_binned_row > 0):
                        binned_data = np.append(binned_data, next_binned_row, axis=0)
                        highest_known_bin_edge = len(bin_edges) - 1 #This has been settled
                        highest_known_bin = len(bin_indices) -1 #This has been settled
                    
                    
                    #clear binned data of Nans
                    if len(binned_data) > 20 and np.isnan(binned_data).any():
                        clear_binned_data = binned_data[~np.ma.fix_invalid(binned_data).mask.any(axis=1)]
                    else:
                        clear_binned_data = binned_data
                   
                    
                    #Ros Stuff
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
           
        if len(clear_binned_data) > 20:
            
            input_data = clear_binned_data[-20:, :].astype(float) # last 2 sec at 66fps

            #input_data = np.delete(input_data[-30:], np.arange(0, input_data.shape[0], 3), axis=0)
            eye_vector = vec_to_euler(input_data[:,2], input_data[:,3], input_data[:,4])
            e_input_data = np.array(eye_vector).T #.reshape(1, 20, -1)
            e_input_data[:,1] += input_data[:,8] #.reshape(-1, 20) #add head pitch
            e_input_data[e_input_data > 180] -= 360
            
            input_eyes_scaled = scaler_eyes.transform(e_input_data) #[0] .reshape(1, -1)
            input_eyes_scaled = input_eyes_scaled.reshape(1, 20, -1)
            
            eyes = torch.Tensor(input_eyes_scaled)
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
            
        if len(clear_binned_data) >= 150:
            historic_average = np.mean(clear_binned_data[30:,11].astype(float).round()[-150:]) #30s
            
            if historic_average > 0.85:
                threshold = historic_average - 0.35
            elif historic_average < 0.45:
                threshold = historic_average + 0.05
            else:
                threshold = 0.5
            
    sock.close()
    
    data_to_save = data_to_save.astype('float64')
    #f_data_to_save = data_to_save.ravel()
    #f_data_to_save = pd.to_numeric(f_data_to_save, errors='coerce')
    np.savetxt(path + 'data/data' + time.strftime("%D_%H_%M_%S").replace('/', '_') + '.csv', data_to_save, delimiter=";", fmt='%f')

if __name__ == "__main__":
    try:
        eye_gaze_publisher(model, scaler_eyes)
    except rospy.ROSInterruptException:
        pass