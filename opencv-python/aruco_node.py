#!/usr/bin/env python3
#coding:utf-8
import cv2
from cv2 import aruco
import numpy as np
import math
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from cv_bridge import CvBridge
bridge = CvBridge()

def detect_aruco_tags(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 转换为灰度图
    # 初始化Aruco字典
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    # 检测参数
    parameters = cv2.aruco.DetectorParameters()
    # 检测Aruco标记
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if ids is not None:
        # 遍历检测到的每个标记
        for i in range(len(ids)):
            # 绘制标记的边界
            cv2.aruco.drawDetectedMarkers(frame, corners[i:i+1], borderColor=(0, 255, 0))
            cv2.putText(frame,f"ID: {ids[i][0]}",(0,frame.shape[0]),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
        cv2.imshow('Detected Aruco Markers', frame)
        cv2.waitKey(1)
    largest_area = 0
    largest_marker_id = -1
    for i, corner in enumerate(corners):
        marker_area = calculate_area(corner[i],frame)
        if marker_area > largest_area:
            largest_area = marker_area
            largest_marker_id = ids[i][0]
    return largest_marker_id,largest_area

def calculate_area(corners,frame):
    height,width = frame.shape[:2]
    x = corners[:, 0]
    y = corners[:, 1]
    S1 = x[0]*y[1] + x[1]*y[2] + x[2]*y[3] + x[3]*y[0]
    S2 = x[0]*y[3] + x[1]*y[0] + x[2]*y[1] + x[3]*y[2]
    area = abs(S1 - S2) / 2.0
    area_percent = area /(height*width)
    return area_percent

def callback(Image):
    frame = bridge.imgmsg_to_cv2(Image,"bgr8")
    result,area_percent = detect_aruco_tags(frame)
    pub.publish(result)
    pub2.publish(area_percent)

if __name__ == '__main__':
    rospy.init_node("aruco_node")
    pub = rospy.Publisher("aruco",Int32,queue_size=10)
    pub2 = rospy.Publisher("aruco_area",Float64,queue_size=10)
    sub = rospy.Subscriber("image2",Image,callback,queue_size=10)
    rospy.spin()