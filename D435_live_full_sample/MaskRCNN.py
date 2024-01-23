from ast import If
import detectron2
import numpy as np
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2 import model_zoo

import os
import cv2
from os import listdir
from os.path import isfile, join


class Mask:
    def __init__(self):
        self.predictor = None
        self.create_predictor()

    def create_predictor(self):

        # 创建配置对象并加载预训练的模型配置
        cfg = get_cfg()
        cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))

        # print('网络模型构建成功')

        cfg.MODEL.ROI_HEADS.NUM_CLASSES = 2  # 两个类别，包括背景
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.9
        cfg.MODEL.WEIGHTS = "./pths/model_final.pth"   # 读取已经训练好的网络模型参数

        # print('网络参数读取成功')

        cfg.MODEL.DEVICE = "cuda"
        self.predictor = DefaultPredictor(cfg)

    def predict_result(self, imgPath, maskPath, showPath, show_result = False):

        colorName = imgPath + "color.png"
        # read and predict
        im = cv2.imread(colorName)
        outputs = self.predictor(im)
        masks = outputs["instances"].to("cpu").pred_masks.detach().numpy()
        masks = np.asarray(masks)

        
        for idx, mask in enumerate(masks):
            maskName = maskPath + "leaf_" + str(idx) + ".png" #每一个掩码图都是leaf_+编号.png的格式
            mask = (mask * 255).astype(np.uint8)
            mask = cv2.merge([mask, mask, mask]) 
            # 使用掩码与原始图像相交，得到实例分割结果图像
            result = cv2.bitwise_and(im, mask)
            cv2.imwrite(maskName, result)
            print("写入", maskName)
            
            if show_result == True:
                GREEN_COLOR = (0, 255, 0)  # Green color (BGR format)
                showName = showPath + "leaf_" + str(idx) + ".png" # 每一个效果图和掩码图编号一样
                # Convert the mask to grayscale
                mask_gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
                mask_gray = cv2.merge([mask_gray, mask_gray, mask_gray])
                # Combine the original image with the grayscale mask
                result = cv2.addWeighted(im, 0.4, mask_gray, 0.6, 0)
                # Draw contour lines based on class
                contours, _ = cv2.findContours(mask_gray[:, :, 0], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(result, contours, -1, GREEN_COLOR, thickness=3)
                cv2.imwrite(showName, result)
                print("写入", showName)
                
    


