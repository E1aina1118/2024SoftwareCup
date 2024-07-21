#!/usr/bin/env python3  
# coding:utf-8  
import cv2  
from cv2 import aruco  
import numpy as np  
import rospy  
from sensor_msgs.msg import Image  
from std_msgs.msg import Int32  
from cv_bridge import CvBridge  
  
bridge = CvBridge()  

def detect_aruco_tags(frame):  
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 转换为灰度图  
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)  
    parameters = cv2.aruco.DetectorParameters()  
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)  
    largest_area = 0
    largest_marker_id = -1
    for i, corner in enumerate(corners):
        marker_area = calculate_area(corner)
        if marker_area > largest_area:
            largest_area = marker_area
            largest_marker_id = ids[i][0]
    cv2.aruco.drawDetectedMarkers(frame,corners,ids)
    cv2.putText(frame, f"ID: {ids[i][0]}, MAX_area:{largest_area}", (0, 0), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.imshow('Detected Aruco Markers', frame)
    cv2.waitKey(1)
    return largest_marker_id

def calculate_area(corners):  
    # 如果 corners 是三维的，取第一个二维数组
    if corners.ndim == 3 and corners.shape[0] == 1:  
        corners = corners[0]  
    # 确保 corners 是一个二维数组
    if corners.ndim != 2 or corners.shape[1] != 2:  
        print("corners must be a 2D array with shape (4, 2) or a 3D array with shape (1, 4, 2)")
    x = corners[:, 0]  
    y = corners[:, 1]  
    S1 = x[0]*y[1] + x[1]*y[2] + x[2]*y[3] + x[3]*y[0]  
    S2 = x[0]*y[3] + x[1]*y[0] + x[2]*y[1] + x[3]*y[2]  
    area = int(abs(S1 - S2) / 2.0)
    return area

def callback(image_msg):  
    frame = bridge.imgmsg_to_cv2(image_msg, "bgr8")  
    result = detect_aruco_tags(frame)   
    pub.publish(result)  
  
if __name__ == '__main__':  
    rospy.init_node("aruco_node")  
    pub = rospy.Publisher("aruco", Int32, queue_size=10)  
    sub = rospy.Subscriber("image", Image, callback, queue_size=10)  
    rospy.spin()

