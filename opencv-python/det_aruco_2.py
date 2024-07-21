# coding=utf-8
import cv2
import numpy as np

def detect_aruco_markers(image_path):
    # ����ͼ��
    img = cv2.imread(image_path)
    img = cv2.resize(img, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_CUBIC)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # ת��Ϊ�Ҷ�ͼ
    # ��ʼ��Aruco�ֵ�
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    
    # ������
    parameters = cv2.aruco.DetectorParameters()
  
    # ���Aruco���
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
    # �����⵽���
    if ids is not None:
        # ������⵽��ÿ�����  
        for i in range(len(ids)):  
            # ���Ʊ�ǵı߽�
            cv2.aruco.drawDetectedMarkers(img, corners[i:i+1], borderColor=(0, 255, 0))
            cv2.putText(img,f"ID: {ids[i][0]}",(0,img.shape[0]),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
            # ��ӡ��ǵ�ID  
            print(f"Detected Aruco Marker ID: {ids[i][0]}")

        # ��ʾͼ��
        cv2.imshow('Detected Aruco Markers', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("No Aruco markers detected.")  
    print(corners)
# ���ú���������ͼƬ·��
detect_aruco_markers('../images/14.jpg')