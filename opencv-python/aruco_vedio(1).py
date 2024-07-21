import cv2
from cv2 import aruco
import imutils
import sys
import time
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

    # 验证提供的ArUco标签是否存在并且被OpenCV支持
    if ARUCO_DICT.get(aruco_type, None) is None:
        print("[INFO] 不支持ArUCo标签 '{}'！".format(aruco_type))
        sys.exit(0)

    # 加载ArUco字典并获取ArUco参数
    print("[INFO] 检测 '{}' 标签...".format(aruco_type))
    arucoDict = aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
    arucoParams = aruco.DetectorParameters()


    # 在帧中检测ArUco标签
    (corners, ids, rejected) = aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

    # 验证至少检测到一个ArUco标签
    if len(corners) > 0:
        # 将ArUco ID列表展平
        ids = ids.flatten()

        # 循环遍历检测到的ArUco标签
        for (markerCorner, markerID) in zip(corners, ids):
            # 提取标签的角点（始终以左上，右上，右下，左下的顺序返回）
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            # 将每个（x，y）坐标对转换为整数
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # 绘制ArUco检测的边界框
            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

            # 计算并绘制ArUco标签的中心（x，y）坐标
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
            circle_point = (cX, cY)
            print("[INFO] ArUco标签ID: {}".format(markerID))
            print(markerID)
            print((cX, cY))
            # 在帧上绘制ArUco标签ID
            cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    else:
        markerID = None
        circle_point = None

    # 显示输出帧
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    return markerID, circle_point

def state_flag(markerID, circle_point):
    # 初始化 markid 列表
    markerid = [1, 2]  # 用您实际的 markid 值替换此处的示例值

    if circle_point[0]>200 and circle_point[0]<400 and circle_point[1]>200 :
            if markerID in markerid:
                flag = 1
            else:
                flag = 0
    else :
        flag = None
    return flag

def state_detection(markerID, circle_point):
    if markerID == 1: #一号环岛
        one_flag = state_flag(markerID, circle_point)
        # print(one_flag)
        return one_flag

    elif markerID == 2: #二号环岛
        two_flag = state_flag(markerID, circle_point)
        return two_flag

    elif markerID == 3: #3号环岛
        three_flag = state_flag(markerID, circle_point)
        return three_flag

    elif markerID == 4: #四号环岛
        four_flag = state_flag(markerID, circle_point)
        return four_flag

    elif markerID == 5: #分岔路1
        five_flag = state_flag(markerID, circle_point)
        return five_flag

    elif markerID == 6: #分叉路2
        six_flag = state_flag(markerID, circle_point)
        return six_flag

    elif markerID == 7:#启停区
        seven_flag = state_flag(markerID, circle_point)
        return seven_flag

def angle (cx,cy):
    # 求出摄像头中线的向量
    v = (0, -240)
    u = (cx-319, cy-479)
    #求两个向量之间的夹角
    dot_product = u[0] * v[0] + u[1] * v[1] #通过计算点积的公式 u·v = u1 * v1 + u2 * v2，得到了向量 u 和 v 的点积 dot_product
    magnitude_u = math.sqrt(u[0]**2 + u[1]**2)
    magnitude_v = math.sqrt(v[0]**2 + v[1]**2) #计算了向量 u 和向量 v 的模长。模长的计算公式为 |u| = √(u1^2 + u2^2) 和 |v| = √(v1^2 + v2^2)
    cos_theta = dot_product / (magnitude_u * magnitude_v)#计算夹角的余弦值 cos_theta，它等于点积 dot_product 除以两个向量的模长的乘积
    theta = math.acos(cos_theta)#math.acos 函数计算了夹角的弧度值 theta，将余弦值转换为弧度值
    angle_degrees = math.degrees(theta) # 将弧度转换为角度
    if(u[0]<0):
        # angle=-angle_degrees
        angle = -theta
    else:
        # angle=angle_degrees
        angle = theta
    print(angle)
    #需要平移的距离
    if(angle<0.3):
        distance_x = cx - 320 #左移为负 右移为正
        print("distance_x:", distance_x)
    else:
        distance_x = 0
    return angle, distance_x

if __name__ == '__main__':
    # 初始化视频流
    print("[INFO] 启动视频流...")
    vs = cv2.VideoCapture(0)

    # 循环读取视频流中的帧
    while True:
        # 读取视频流的下一帧
        ret, frame = vs.read()
        if not ret:
            break
        # 调用函数进行视频中的 ArUco 标签检测
        markerID, circle_point = detect_aruco_tags(frame, "DICT_4X4_100")
        print(markerID)
        # if markerID is not None:
        #     print(state_detection(markerID, circle_point))
        #     cX = circle_point[0]
        #     cY = circle_point[1]
        #     theta, distance_x = angle(cX, cY)

    # 清理
    cv2.destroyAllWindows()
    vs.release()