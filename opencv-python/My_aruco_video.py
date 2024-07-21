#coding=utf-8
import cv2
from cv2 import aruco
import math

def detect_aruco_tags(frame, aruco_type="DICT_ARUCO_ORIGINAL"):
    # Define the names of each possible ArUco tag that OpenCV supports
    ARUCO_DICT = {"DICT_4X4_50": cv2.aruco.DICT_4X4_50, "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
                  "DICT_4X4_250": cv2.aruco.DICT_4X4_250, "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
                  "DICT_5X5_50": cv2.aruco.DICT_5X5_50, "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
                  "DICT_5X5_250": cv2.aruco.DICT_5X5_250, "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
                  "DICT_6X6_50": cv2.aruco.DICT_6X6_50, "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
                  "DICT_6X6_250": cv2.aruco.DICT_6X6_250, "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
                  "DICT_7X7_50": cv2.aruco.DICT_7X7_50, "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
                  "DICT_7X7_250": cv2.aruco.DICT_7X7_250, "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
                  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
                  "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
                  "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
                  "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
                  "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11}

    # ��֤�ṩ��ArUco��ǩ�Ƿ���ڲ��ұ�OpenCV֧��
    # if ARUCO_DICT.get(aruco_type, None) is None:
    #     print("[INFO] ��֧��ArUCo��ǩ '{}'!".format(aruco_type))
    #     sys.exit(0)

    # ����ArUco�ֵ䲢��ȡArUco����
    # print("[INFO] detected '{}' tag".format(aruco_type))
    arucoDict = aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
    arucoParams = aruco.DetectorParameters()


    # ��֡�м��ArUco��ǩ
    (corners, ids, rejected) = aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)



    # ��֤���ټ�⵽һ��ArUco��ǩ
    if len(corners) > 0:
        # ��ArUco ID�б�չƽ
        ids = ids.flatten()

        # ѭ��������⵽��ArUco��ǩ
        for (markerCorner, markerID) in zip(corners, ids):
            # ��ȡ��ǩ�Ľǵ㣨ʼ�������ϣ����ϣ����£����µ�˳�򷵻أ�
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            # ��ÿ����x��y�������ת��Ϊ����
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # ����ArUco���ı߽��
            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

            # ���㲢����ArUco��ǩ�����ģ�x��y������
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
            circle_point = (cX, cY)
            print("[INFO] ArUco tag ID: {}".format(markerID))
            print(markerID)
            print((cX, cY))
            # ��֡�ϻ���ArUco��ǩID
            cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    else:
        markerID = None
        circle_point = None
    return markerID, circle_point, corners, ids

def calculate_area(corners):
    x = corners[:, 0]
    y = corners[:, 1]
    S1 = x[0]*y[1] + x[1]*y[2] + x[2]*y[3] + x[3]*y[0]  
    S2 = x[0]*y[3] + x[1]*y[0] + x[2]*y[1] + x[3]*y[2]  
    area = abs(S1 - S2) / 2.0  
    return area

if __name__ == '__main__':
    image = cv2.imread("../images/17.jpg")
    dim = (int(image.shape[1] * 0.5), int(image.shape[0] * 0.5))
    frame = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
    markerID, circle_point, corners, ids = detect_aruco_tags(frame, "DICT_4X4_50")
    area = calculate_area(corners)
    print("area:")
    print(area)
    print("corners:")
    print(corners)
    print("ids:")
    print(ids)
    cv2.imshow("frame",frame)
    cv2.waitKey(0)