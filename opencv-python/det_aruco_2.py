# coding=utf-8
import cv2
import numpy as np

def detect_aruco_markers(image_path):
    # 加载图像
    img = cv2.imread(image_path)
    img = cv2.resize(img, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_CUBIC)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # 转换为灰度图
    # 初始化Aruco字典
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    
    # 检测参数
    parameters = cv2.aruco.DetectorParameters()
  
    # 检测Aruco标记
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # print("corners:")
    # print(corners)
    # print("corners(for):")
    # for i in corners:
    #     for j in i:
    #         print(j)
    # print("ids:")
    # print(ids)
    # print("rejectedImgPoints:")
    # print(rejectedImgPoints)
    # 如果检测到标记
    if ids is not None:
        # 遍历检测到的每个标记  
        for i in range(len(ids)):  
            # 绘制标记的边界
            cv2.aruco.drawDetectedMarkers(img, corners[i:i+1], borderColor=(0, 255, 0))
            cv2.putText(img,f"ID: {ids[i][0]}",(0,img.shape[0]),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
            # 打印标记的ID  
            print(f"Detected Aruco Marker ID: {ids[i][0]}")

        # 显示图像
        cv2.imshow('Detected Aruco Markers', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("No Aruco markers detected.")  
    print(corners)
# 调用函数，传入图片路径
detect_aruco_markers('../images/14.jpg')