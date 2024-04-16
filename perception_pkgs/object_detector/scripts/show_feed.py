#!/home/administrator/anaconda3/envs/maia_ws/bin python
import cv2 
import numpy as np
import matplotlib.pyplot as plt

import rospy
import torch
import struct
from sensor_msgs.msg import Image,CameraInfo
#from PIL import Image as pil_image
from torchvision.io.image import read_image
from torchvision.models.detection import fasterrcnn_resnet50_fpn_v2, FasterRCNN_ResNet50_FPN_V2_Weights,FasterRCNN_MobileNet_V3_Large_320_FPN_Weights,fasterrcnn_mobilenet_v3_large_320_fpn
from torchvision.utils import draw_bounding_boxes
from torchvision.transforms.functional import to_pil_image

from mobile_net_backend.mobilenet_detector import MBNet_Detector
from yolov7_backend.yolov7_detector import YOLO7_Detector
import message_filters
from cv_bridge import CvBridge, CvBridgeError
class Detector:
    def __init__(self):
        import sys
        print(sys.executable)
        self.detector = YOLO7_Detector()
        image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        info = message_filters.Subscriber('/camera/color/camera_info', CameraInfo)

        ts = message_filters.TimeSynchronizer([image_sub,depth_sub,info], 5)
        ts.registerCallback(self.callback)
    
    def callback(self,rgb,depth,info):
        
        image_numpy = np.array(list(rgb.data),np.uint8)
        
        realsense_depth_scale = 0.001
        depth_img = np.frombuffer(depth.data, dtype=np.uint16).reshape((720,1280))*realsense_depth_scale
        
        
        
        #bridge = CvBridge()
        
        #print(depth_image.shape)
        #np.save('depth_bin.npy',depth)
        k = np.array(list(info.K)).reshape(3,3)
        inv_k = np.linalg.pinv(k)
        
        #print(k)
        image_numpy = image_numpy.reshape((720,1280,3))
        image_numpy = cv2.cvtColor(image_numpy,cv2.COLOR_RGB2BGR)
        boxes,labels = self.detector.detect(image_numpy)
        for box,label in zip(boxes,labels):
            xmin,ymin,xmax,ymax =[int(b) for b in box]
            cv2.rectangle(image_numpy,(xmin,ymin),(xmax,ymax),[255,255,0],2)
            center_x = int((xmin+xmax)/2)
            center_y = int((ymin+ymax)/2)
            z = depth_img[center_y,center_x]
            point2d_z = np.array([center_x*z,center_y*z,z])
            point_3d = np.dot(inv_k,point2d_z)
            print(point_3d)
            cv2.putText(image_numpy, '%.2fm'%(point_3d[2]),(xmin+20,ymin+20),cv2.FONT_HERSHEY_SIMPLEX, 2, 255)

        cv2.namedWindow('feed', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('feed', image_numpy)
        cv2.waitKey(1)


if __name__ == '__main__':
    
    rospy.init_node('listener', anonymous=True)
    d = Detector()

    rospy.spin()