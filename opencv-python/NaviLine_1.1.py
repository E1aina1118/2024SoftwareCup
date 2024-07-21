#coding:utf-8
import cv2 # type: ignore
import numpy as np # type: ignore
import math
import rospy # type: ignore
from test_py.msg import Image # type: ignore
from cv_bridge import CvBridge,CvBridgeError # type: ignore


def callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
# 最初输入:vanila_img

# 读取原始图片
vanila_img = cv2.imread('../images/13.jpg')
vanila_img_size = vanila_img.shape
height = vanila_img_size[0]
width = vanila_img_size[1]

# 设置原始质心
last_centroid = (width*2//3,height//2)

# 高斯滤波
blurred_img = cv2.GaussianBlur(vanila_img,(5,5),0)

# 转为灰度图
gray_img = cv2.cvtColor(blurred_img,cv2.COLOR_BGR2GRAY)

# 颜色反转，二值化
clcvt_img = cv2.bitwise_not(gray_img)
_,thresh_img = cv2.threshold(clcvt_img,0,255,cv2.THRESH_OTSU)

# 图像腐蚀
kernel_erode = np.ones((3,3),np.uint8)
eroded_img = cv2.erode(thresh_img,kernel_erode,iterations=3)

# 图像膨胀
kernel_dilate = np.ones((3,3),np.uint8)
dilated_img = cv2.dilate(eroded_img,kernel_dilate,iterations=3)

# 获取连通域信息
retvals,labels,stats,centroids = cv2.connectedComponentsWithStats(dilated_img,8)

ct_dis = []

for i in range(1,retvals):
    a_centroid = (int(centroids[i,0]),int(centroids[i,1]))
    dis = math.sqrt((a_centroid[0]-last_centroid[0])**2+(a_centroid[1]-last_centroid[1])**2)
    ct_dis.append((i,dis))

# 初始化 然后找出需要保持的连通域的label
mindis = 999.0
for i in ct_dis:
    if(i[1]<mindis):
        mindis = i[1]
        label_to_keep = i[0]

pure_lane = np.zeros_like(dilated_img)
pure_lane[labels == label_to_keep] = 255
retvals_2,_,_,top_lane_pointf = cv2.connectedComponentsWithStats(pure_lane[0:height//5,0:width],connectivity = 8)

cv2.imshow('pure_lane[0:height//5,0:width]',pure_lane[0:height//5,0:width])

print(f"retvals_2 = {retvals_2}")
if retvals_2 >= 2:
    top_lane_point = (int(top_lane_pointf[1,0]),int(top_lane_pointf[1,1]))
    target_delta = math.atan((width/2-top_lane_point[0])/(height-top_lane_point[1]))
else:
    target_delta = "Error!"

print(f"target_delta = {target_delta}")

cv2.imshow('dilated_img',dilated_img)
cv2.imshow('pure_lane',pure_lane)
cv2.waitKey(0)


# 最后输出: pure_lane


