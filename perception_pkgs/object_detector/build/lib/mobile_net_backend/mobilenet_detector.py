#!/home/administrator/anaconda3/envs/itt_sorter/bin python
from torchvision.models.detection import fasterrcnn_resnet50_fpn_v2, FasterRCNN_ResNet50_FPN_V2_Weights,FasterRCNN_MobileNet_V3_Large_320_FPN_Weights,fasterrcnn_mobilenet_v3_large_320_fpn
from torchvision.transforms.functional import to_pil_image
from PIL import Image as pil_image
import rospy
class MBNet_Detector:
    def __init__(self):
        self.weights = FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.COCO_V1
        self.model = fasterrcnn_mobilenet_v3_large_320_fpn(weights=self.weights, box_score_thresh=0.9)
        self.model.eval()
    def detect(self,image_numpy):
        img = pil_image.fromarray(image_numpy)

        preprocess = self.weights.transforms()
        batch = [preprocess(img)]
        prediction = self.model(batch)[0]
        labels = [self.weights.meta["categories"][i] for i in prediction["labels"]]
        boxes = prediction["boxes"].detach().cpu().numpy()
        return boxes,labels