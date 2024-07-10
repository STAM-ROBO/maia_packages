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

#from yolov7_backend.yolov7_detector import YOLO7_Detector
from yolov7_backend.yolo7_trt import YOLO7_TRT
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from msg_common.msg import Detections_3d,Object_3d
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import std_msgs
import time

def cv2_img2mg(img):
    img_msg = Image()
    img_msg.width = img.shape[1]
    img_msg.height = img.shape[0]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian=False
    img_msg.step = img_msg.width*img.shape[2]
    img_msg.data = img.tobytes()
    return img_msg

class Detector:
    def __init__(self):
        import sys
        print(sys.executable)
        self.detector = YOLO7_TRT()
        image_sub = message_filters.Subscriber('od_ns/camera/color/image_raw', Image)
        #depth_sub = message_filters.Subscriber('od_ns/camera/aligned_depth_to_color/image_raw', Image)
        depth_sub = message_filters.Subscriber('od_ns/camera/depth/image_rect_raw', Image)
        info = message_filters.Subscriber('od_ns/camera/color/camera_info', CameraInfo)
        self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub],5 , 0.05)

        self.pub = rospy.Publisher('detections_3d', Detections_3d, queue_size=180)
        
        self.names = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light', 
         'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 
         'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 
         'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 
         'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 
         'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 
         'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 
         'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 
         'hair drier', 'toothbrush']
        self.detector.warm_up()
        self.realsense_depth_scale = 0.001
        self.img_width = 640
        self.img_height = 480
        self.counter =0
        self.intrinsics_sub = rospy.Subscriber('od_ns/camera/color/camera_info',CameraInfo,self.do_initialization)
        self.vis_pub = rospy.Publisher('object_detector_feed',Image,queue_size=10)
        self.last_time=time.time()
        
    def do_initialization(self,info):
        self.k = np.array(list(info.K)).reshape(3,3)
        self.inv_k = np.linalg.pinv(self.k)
        self.intrinsics_sub.unregister()
        if(rospy.get_param('~debug')):
            self.ts.registerCallback(self.callback_debug)    
            rospy.loginfo('debug mode activated!!!!')
        else:
            self.ts.registerCallback(self.callback_debug)   
            #self.ts.registerCallback(self.callback_operation)
        print('do_initialization done')

    def create_obj3d(self, label, x,y,z):
        obj = Object_3d()
        obj.pt = Point(x,y,z)
        obj.obj_class = self.names[label]
        return obj

    def callback_operation(self,rgb,depth): 
        self.last_time=time.time()    
        image_numpy = np.frombuffer(rgb.data,np.uint8)
        depth_img = np.frombuffer(depth.data, dtype=np.uint16).reshape((self.img_height,self.img_width))*self.realsense_depth_scale       
        image_numpy = image_numpy.reshape((self.img_height,self.img_width,3))
        boxes,labels,scores = self.detector.detect(image_numpy)

        msg = Detections_3d()
        msg.header =  std_msgs.msg.Header()
        msg.header.stamp = rospy.Time.now()

        if len(boxes) == 0:
            #IF NO OBJS DETECTED
            msg.objects = []
        else:
            centers = np.array([[int((b[0]+b[2])/2),int((b[1]+b[3])/2)] for b in boxes])
            zs = [depth_img[i,j] for i,j in zip(centers[:,1],centers[:,0])]
            points2d_z = np.array([[cx*z,cy*z,z]for cx,cy,z in zip(centers[:,0],centers[:,1],zs)])
            points3d = np.dot(self.inv_k,points2d_z.T)
            objs = [self.create_obj3d(labels, x_3d, y_3d, z_3d) for labels,x_3d,y_3d,z_3d in zip(labels, points3d[0,:],points3d[1,:],points3d[2,:]) ]
            msg.objects = objs

        self.pub.publish(msg)
    
    def callback_debug(self,rgb,depth):
        print(f'callback_operation interval {int(1000*(time.time()-self.last_time))} ms')
        self.last_time=time.time() 
        image_numpy = np.frombuffer(rgb.data,np.uint8)
        depth_img = np.frombuffer(depth.data, dtype=np.uint16).reshape((self.img_height,self.img_width))*self.realsense_depth_scale
        image_numpy = image_numpy.reshape((self.img_height,self.img_width,3))
        image_numpy = cv2.cvtColor(image_numpy,cv2.COLOR_RGB2BGR)
        boxes,labels,scores = self.detector.detect(image_numpy)
        msg = Detections_3d()
        msg.header =  std_msgs.msg.Header()
        msg.header.stamp = rospy.Time.now()
        objs = []
        
        norm = cv2.normalize(depth_img, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        image_numpy = cv2.cvtColor(norm, cv2.COLOR_GRAY2BGR)

        msg.objects = []
        for box,label in zip(boxes,labels):            
            xmin,ymin,xmax,ymax =[int(b) for b in box]
            cv2.rectangle(image_numpy,(xmin,ymin),(xmax,ymax),[255,255,0],2)
            center_x = int((xmin+xmax)/2)
            center_y = int((ymin+ymax)/2)        
            depth_box=depth_img[ymin:ymax, xmin:xmax]    
            print(len(depth_box))
            if len(depth_box) > 0:       
                z = np.min(depth_box)
                if(z<0.01):
                    continue
                point2d_z = np.array([center_x*z,center_y*z,z])
                point_3d= np.dot(self.inv_k,point2d_z)
                
                cv2.putText(image_numpy, '%s'%(self.names[label]),(xmin+20,ymin+20),cv2.FONT_HERSHEY_SIMPLEX, .5, [0,0,255],2)
                cv2.putText(image_numpy, '%.2fm'%(point_3d[2]),(xmin+20,ymin+40),cv2.FONT_HERSHEY_SIMPLEX, .5, [0,0,255],2)
                obj = self.create_obj3d(label, point_3d[0], point_3d[1], point_3d[2])      
                objs.append(obj)

        msg.objects = objs
        self.pub.publish(msg)
      
        #cv2.imwrite('/home/maia/debug_img/frame_%s.png'%(str(self.counter).zfill(2)),image_numpy)
        self.vis_pub.publish(cv2_img2mg(image_numpy))
        self.counter+=1
        self.counter = self.counter %120


if __name__ == '__main__':
    
    rospy.init_node('listener', anonymous=True)
    d = Detector()

    rospy.spin()