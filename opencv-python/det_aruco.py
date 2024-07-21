# # coding=utf-8  
# import numpy as np  
# import cv2  
# import cv2.aruco as aruco  
  
# # ��ȡͼƬ  
# frame = cv2.imread('../images/17.jpg')  
# # �ҶȻ�  
# gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  

# # ����Ԥ������ֵ�
# aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
# parameters = aruco.DetectorParameters()

# # ʹ��aruco.detectMarkers()�������marker������ID�ͱ�־���4���ǵ�����  
# corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

# # �����⵽�˱�ǣ��򻭳�����  
# if ids is not None:  
#     aruco.drawDetectedMarkers(frame, corners, ids)

# # ��ʾͼ��  
# cv2.imshow("frame", frame)  
# cv2.waitKey(0)  
# cv2.destroyAllWindows()

#coding=utf-8
import cv2
from cv2 import aruco
import math

def calculate_area(corners):  
    # ��� corners ����ά�ģ�ȡ��һ����ά����
    if corners.ndim == 3 and corners.shape[0] == 1:  
        corners = corners[0]  
    # ȷ�� corners ��һ����ά����
    if corners.ndim != 2 or corners.shape[1] != 2:  
        raise ValueError("corners must be a 2D array with shape (4, 2) or a 3D array with shape (1, 4, 2)")
    x = corners[:, 0]  
    y = corners[:, 1]  
    S1 = x[0]*y[1] + x[1]*y[2] + x[2]*y[3] + x[3]*y[0]  
    S2 = x[0]*y[3] + x[1]*y[0] + x[2]*y[1] + x[3]*y[2]  
    area = int(abs(S1 - S2) / 2.0)
    return area

if __name__ == '__main__':
    image = cv2.imread("../images/17.jpg")
    dim = (int(image.shape[1] * 0.5), int(image.shape[0] * 0.5))
    frame = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
    # �ҶȻ�  
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # ����Ԥ������ֵ�
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print("corners:")
    print(corners)
    print("ids:")
    print(ids)
    largest_area = 0
    for i, corner in enumerate(corners):
        print(f"corner: {corner}")
        marker_area = calculate_area(corner)
        print(f"area:{marker_area}")
        if marker_area > largest_area:
            largest_area = marker_area
            largest_marker_id = ids[i][0]
    cv2.aruco.drawDetectedMarkers(frame,corners,ids)
    cv2.putText(frame, f"ID: {ids[i][0]}, area:{largest_area}", (0, frame.shape[0]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.imshow('Detected Aruco Markers', frame)
    cv2.waitKey(0)