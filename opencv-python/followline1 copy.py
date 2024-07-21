#!/usr/bin/env python3
#coding:utf-8
import cv2 
import numpy as np 
import math
import rospy 
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import String
from cv_bridge import CvBridge
bridge = CvBridge()

# #global variables
# get_times_global = 0
# previous_centroid_global = [0,0]
# exittime_threshold = 1000

def make_decision(image,stats,label_to_keep):
    i = label_to_keep
    x1 = stats[i][0]
    y1 = stats[i][1]
    x2 = stats[i][0] + stats[i][2]
    y2 = stats[i][1] + stats[i][3]
    imgh, imgw = image.shape[:2]
    if x1 == 0 and y1 > 0 and x2<imgw and y2 == imgh:
        if image[imgh-50][50] == 0 and image[imgh-50][imgw-50] == 0:
            sign = "left"
    elif x1 > 0 and y1 > 0 and x2 == imgw and y2 == imgh:
        if image[imgh-50][50] == 0 and image[imgh-50][imgw-50] == 0:
            sign = "right"
    elif x1 > 0 and x2 == imgw and y2 == imgh:
        if image[imgh-50][50] == 0 and image[imgh-50][imgw-50] == 0:
            sign = "angle_appeared"
    elif x1 == 0 and x2<imgw and y2 == imgh:
        if image[imgh-50][50] == 0 and image[imgh-50][imgw-50] == 0:
            sign = "angle_appeared"
    else:
        sign = "forward"
    return sign

# def select_init_centroid(img):
#     global previous_centroid_global
#     vanila_img_size = img.shape
#     height = vanila_img_size[0]
#     width = vanila_img_size[1]
#     previous_centroid_global = [width//2,height//2]

def process_img_twice(image):
    # 膨胀
    kernel_dilate = np.ones((5,5),np.uint8)
    dilated_img = cv2.dilate(image,kernel_dilate,iterations=6)

    # 腐蚀
    kernel_erode = np.ones((3,3),np.uint8)
    eroded_img = cv2.erode(dilated_img,kernel_erode,iterations=6)
    return eroded_img

def put_info(img,centroid,rad,condition):
    height, width= img.shape
    img_rgb = np.stack((img,)*3, axis=-1)
    color_1 = (255,191,0)
    color_2 = (0,0,255)
    start_point = (width//2,height)
    end_point_1 = (centroid[0],centroid[1])
    end_point_2 = (width//2,0)
    cv2.line(img_rgb,start_point,end_point_1,color_1,1)
    cv2.line(img_rgb,start_point,end_point_2,color_2,1)
    cv2.putText(img_rgb,f"degree:{180/math.pi*rad}",(0,height),cv2.FONT_HERSHEY_SIMPLEX,1,color_2,2)
    cv2.putText(img_rgb,f"{condition}",(width//2,height//2),cv2.FONT_HERSHEY_SIMPLEX,1,color_2,2)
    return img_rgb
    
def process_img(img): #输入：图片  输出：纯净引导线(二值化后)

    # global previous_centroid_global
    # global get_times_global
    # if get_times_global == 0:
    #     select_init_centroid(img)
    #     get_times_global += 1
    #     return 0.0

    # 高斯滤波
    blurred_img = cv2.GaussianBlur(img,(5,5),0)

    # 转为灰度图
    gray_img = cv2.cvtColor(blurred_img,cv2.COLOR_BGR2GRAY)

    # 颜色反转，二值化
    clcvt_img = cv2.bitwise_not(gray_img)
    _,thresh_img = cv2.threshold(clcvt_img,0,255,cv2.THRESH_OTSU)

    # cv2.imshow("thresh_img",thresh_img)
    # cv2.waitKey(1)

    # 图像腐蚀
    kernel_erode = np.ones((3,3),np.uint8)
    eroded_img = cv2.erode(thresh_img,kernel_erode,iterations=3)

    # 图像膨胀
    kernel_dilate = np.ones((3,3),np.uint8)
    dilated_img = cv2.dilate(eroded_img,kernel_dilate,iterations=3)

    # 获取连通域信息
    retvals,labels,stats,centroids = cv2.connectedComponentsWithStats(dilated_img,8)

    #--------------------------取距离上次连通域质心最短的连通域----------------------------#
    # ct_dis = []

    # for i in range(1,retvals):
    #     a_centroid = (int(centroids[i,0]),int(centroids[i,1]))
    #     dis = math.sqrt((a_centroid[0]-previous_centroid_global[0])**2+(a_centroid[1]-previous_centroid_global[1])**2)
    #     ct_dis.append((i,dis))

    # mindis = 999.0n
    # for i in ct_dis:
    #     if(i[1]<mindis):
    #         mindis = i[1]
    #         label_to_keep = i[0]
    #         previous_centroid_global = [int(centroids[i[0],0]),int(centroids[i[0],1])]
    #-----------------------------------------------------------------------------------#

    #--------------------------取面积最大的连通域-----------------------------------------#
    if retvals == 1:
        label_to_keep = 0
    else:
        max_connect = 0
        for i in range(1,retvals):
            if max_connect <= stats[i][4]:
                max_connect = stats[i][4]
                label_to_keep = i
    #-----------------------------------------------------------------------------------#

    pure_lane = np.zeros_like(dilated_img)
    pure_lane[labels == label_to_keep] = 255

    # 纯净轨道的二次处理(膨胀, 更多腐蚀(把引导线变细))
    pure_lane = process_img_twice(pure_lane)

    return pure_lane,stats,label_to_keep

def compute_target(pure_lane):
    # retvals_2,_,_,top_lane_pointf = cv2.connectedComponentsWithStats(pure_lane[0:height//5,0:width],connectivity = 8)
    height,width = pure_lane.shape[:2]
    for i in range(5):
        retvals_2,_,_,top_lane_pointf = cv2.connectedComponentsWithStats(pure_lane[height*i//5:height*(i+1)//5,0:width],connectivity = 8)
        if(retvals_2<=1):
            continue
        else:
            top_lane_point = [int(top_lane_pointf[1,0]),int(height*i//5+top_lane_pointf[1,1])]
            target_delta = math.atan((width/2-top_lane_point[0])/(height-top_lane_point[1]))
            break
    print(target_delta)
    return target_delta,top_lane_point

# -----------------------回调函数--------------------------- #
def img_callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg,"bgr8")
    pure_lane,stats,label_to_keep = process_img(cv_image)
    condition = make_decision(pure_lane,stats,label_to_keep)
    pub2.publish(condition)
    rospy.loginfo("condition published")
    if condition == "forward":
        result,top_point = compute_target(pure_lane)
        pub.publish(result)
        rospy.loginfo("yaw result published")
    else:
        top_point = (cv_image.shape[1]//2,cv_image.shape[0]//2)
        result = 0
        pub.publish(result)
    cv2.imshow("pure_lane_info",put_info(pure_lane,top_point,result,condition))
    cv2.waitKey(1)
# ---------------------------------------------------------- #
if __name__ == '__main__':
    rospy.init_node("followline")
    pub = rospy.Publisher("target_rad",Float64,queue_size=10)
    pub2 = rospy.Publisher("conditions",String,queue_size=10)
    sub = rospy.Subscriber("image",Image,img_callback,queue_size=10)
    rospy.spin()
