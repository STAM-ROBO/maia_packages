#!/home/administrator/anaconda3/envs/maia_ws/bin python
import sys
import os

sys.path.append(os.getcwd()+'/src/perception_pkgs/object_detector/src/yolov7_backend/yolov7')
print(sys.path)
import rospy
from pathlib import Path
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from models.experimental import attempt_load
from utils.torch_utils import select_device,load_classifier,time_synchronized,TracedModel
import torch
from utils.general import non_max_suppression,scale_coords
class_id_to_string = {0: u'__background__',
 1: u'person',
 2: u'bicycle',
 3: u'car',
 4: u'motorcycle',
 5: u'airplane',
 6: u'bus',
 7: u'train',
 8: u'truck',
 9: u'boat',
 10: u'traffic light',
 11: u'fire hydrant',
 12: u'stop sign',
 13: u'parking meter',
 14: u'bench',
 15: u'bird',
 16: u'cat',
 17: u'dog',
 18: u'horse',
 19: u'sheep',
 20: u'cow',
 21: u'elephant',
 22: u'bear',
 23: u'zebra',
 24: u'giraffe',
 25: u'backpack',
 26: u'umbrella',
 27: u'handbag',
 28: u'tie',
 29: u'suitcase',
 30: u'frisbee',
 31: u'skis',
 32: u'snowboard',
 33: u'sports ball',
 34: u'kite',
 35: u'baseball bat',
 36: u'baseball glove',
 37: u'skateboard',
 38: u'surfboard',
 39: u'tennis racket',
 40: u'bottle',
 41: u'wine glass',
 42: u'cup',
 43: u'fork',
 44: u'knife',
 45: u'spoon',
 46: u'bowl',
 47: u'banana',
 48: u'apple',
 49: u'sandwich',
 50: u'orange',
 51: u'broccoli',
 52: u'carrot',
 53: u'hot dog',
 54: u'pizza',
 55: u'donut',
 56: u'cake',
 57: u'chair',
 58: u'couch',
 59: u'potted plant',
 60: u'bed',
 61: u'dining table',
 62: u'toilet',
 63: u'tv',
 64: u'laptop',
 65: u'mouse',
 66: u'remote',
 67: u'keyboard',
 68: u'cell phone',
 69: u'microwave',
 70: u'oven',
 71: u'toaster',
 72: u'sink',
 73: u'refrigerator',
 74: u'book',
 75: u'clock',
 76: u'vase',
 77: u'scissors',
 78: u'teddy bear',
 79: u'hair drier',
 80: u'toothbrush'}
class YOLO7_Detector:
    def __init__(self):
        print(torch.cuda.is_available())
        self.device = torch.device('cuda')
        model_weights = attempt_load('/home/maia_ws/src/perception_pkgs/object_detector/src/yolov7_backend/yolov7.pt')
        self.model = TracedModel(model_weights,self.device,640)

    def detect(self,image_numpy):
        img = image_numpy.copy()
        img = cv2.resize(img,(640,480))
        img = img[:,:,::-1].transpose(2,0,1)
        img = torch.from_numpy(img.copy()).to(self.device)
        img = img.float()
        img /= 255.0

        img = img.unsqueeze(0)
        with torch.no_grad():
            pred = self.model(img)[0]
        print(img.device)
        det = non_max_suppression(pred,0.4)[0]
        det[:,:4] = scale_coords(img.shape[2:],det[:,:4],image_numpy.shape).round()
        boxes = np.array([t.cpu().detach().numpy()[0:4].astype(int) for t in det])
        labels = np.array([t.cpu().detach().numpy()[-1].astype(int)+1 for t in det])
        scores = np.array([t.cpu().detach().numpy()[4].astype(float) for t in det])
        return boxes,labels,scores