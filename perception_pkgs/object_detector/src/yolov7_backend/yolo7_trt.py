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
import tensorrt as trt
from PIL import Image
from pathlib import Path
from collections import OrderedDict,namedtuple
class YOLO7_TRT:
    def __init__(self):
        print(torch.cuda.is_available())
        self.device = torch.device('cuda')
        model_weights = '/home/administrator/maia_ws/src/perception_pkgs/object_detector/src/yolov7_backend/yolo-nms.trt'
        Binding = namedtuple('Binding', ('name', 'dtype', 'shape', 'data', 'ptr'))
        logger = trt.Logger(trt.Logger.ERROR)
        trt.init_libnvinfer_plugins(logger, namespace="")
        with open(model_weights, 'rb') as f, trt.Runtime(logger) as runtime:
            model = runtime.deserialize_cuda_engine(f.read())
        self.bindings = OrderedDict()
        for index in range(model.num_bindings):
            name = model.get_tensor_name(index)
            dtype = trt.nptype(model.get_tensor_dtype(name))
            shape = tuple(model.get_tensor_shape(name))
            data = torch.from_numpy(np.empty(shape, dtype=np.dtype(dtype))).to(self.device)
            self.bindings[name] = Binding(name, dtype, shape, data, int(data.data_ptr()))
        self.binding_addrs = OrderedDict((n, d.ptr) for n, d in self.bindings.items())
        self.context = model.create_execution_context()
    def warm_up(self):
        for _ in range(60):
            tmp = torch.randn(1,3,640,640).to(self.device)
            self.binding_addrs['images'] = int(tmp.data_ptr())
            self.context.execute_v2(list(self.binding_addrs.values()))
    def letterbox(self,im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleup=True, stride=32):
        # Resize and pad image while meeting stride-multiple constraints
        shape = im.shape[:2]  # current shape [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:  # only scale down, do not scale up (for better val mAP)
            r = min(r, 1.0)

        # Compute padding
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding

        if auto:  # minimum rectangle
            dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding

        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
        return im, r, (dw, dh)
    def postprocess(self,boxes,r,dwdh):
        dwdh = torch.tensor(dwdh*2).to(boxes.device)
        boxes -= dwdh
        boxes /= r
        return boxes

    def detect(self,image_numpy):
        img = cv2.cvtColor(image_numpy, cv2.COLOR_BGR2RGB)
        image = img.copy()
        image, ratio, dwdh = self.letterbox(image, auto=False)
        image = image.transpose((2, 0, 1))
        image = np.expand_dims(image, 0)
        image = np.ascontiguousarray(image)

        im = image.astype(np.float32)
        im = torch.from_numpy(im).to(self.device)
        im/=255     

        self.binding_addrs['images'] = int(im.data_ptr())
        import time
        
        self.context.execute_v2(list(self.binding_addrs.values()))
        
        nums = self.bindings['num_dets'].data
        boxes = self.bindings['det_boxes'].data
        scores = self.bindings['det_scores'].data
        classes = self.bindings['det_classes'].data
  
        boxes = boxes[0,:nums[0][0]]
        scores = scores[0,:nums[0][0]].detach().cpu().numpy()
        classes = classes[0,:nums[0][0]].detach().cpu().numpy()
        boxs = [self.postprocess(b,ratio,dwdh).round().int() for b in boxes]
        boxes = [b.detach().cpu().numpy() for b in boxes]
        
        return boxes,classes,scores