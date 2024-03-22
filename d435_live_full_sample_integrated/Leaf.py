import D435 as D435
import MaskRCNN as mr
import os
import cv2
import shutil
import subprocess
from datetime import datetime
import pybeanbox

if __name__ == '__main__':


    d435 = D435.d435()


    plant = d435.get_d435_image()
    date_str = datetime.now().strftime("%Y-%m-%d-%H:%M:%S")
    print(plant.intrinsics2)
    imgPath = "/home/njau/auto_leaf/"+date_str+"/color_image/" # 彩色原图路径
    maskPath = "/home/njau/auto_leaf/"+date_str+"/fenge_image/" # 分割图保存路径
    showPath = "/home/njau/auto_leaf/"+date_str+"/show_image/" # 效果图路径
    dthPath = "/home/njau/auto_leaf/"+date_str+"/depth_image/" # 深度图保存路径
    date_outpath = "/home/njau/auto_leaf/"+date_str+"/date_outpath/"
    os.makedirs(imgPath, exist_ok=True)
    os.makedirs(maskPath, exist_ok=True)
    os.makedirs(showPath, exist_ok=True)
    os.makedirs(dthPath, exist_ok=True)
    os.makedirs(date_outpath, exist_ok=True)

    cv2.imwrite(imgPath + "color.png", plant.color_images)
    cv2.imwrite(dthPath + "depth.png", plant.depth_images)

    pre_mask = mr.Mask()
    pre_mask.predict_result(imgPath, maskPath, showPath, show_result = True)
    
    # use run_exe
    # subprocess.run(["/home/njau/auto_leaf/pcl_deal/beanbox", plant.intrinsics2, dthPath + "depth.png", maskPath, date_outpath])
    # use pybind11
    # 创建BeanBox2对象
    bean_box = pybeanbox.BeanBox2()
    # 调用run函数
    bean_box.run(plant.intrinsics2, dthPath + "depth.png", maskPath, date_outpath)

    # close the camera 
    d435.close_d435()