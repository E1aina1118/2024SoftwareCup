#coding=utf-8
import cv2 
import numpy as np 
import math

# def process_img(img): #输入：图片  输出：角度
#     vanila_img_size = img.shape
#     height = vanila_img_size[0]
#     width = vanila_img_size[1]

#     # 高斯滤波
#     blurred_img = cv2.GaussianBlur(img,(5,5),0)

#     # 转为灰度图
#     gray_img = cv2.cvtColor(blurred_img,cv2.COLOR_BGR2GRAY)

#     # 颜色反转，二值化
#     clcvt_img = cv2.bitwise_not(gray_img)
#     _,thresh_img = cv2.threshold(clcvt_img,0,255,cv2.THRESH_OTSU)

#     # 图像腐蚀
#     kernel_erode = np.ones((3,3),np.uint8)
#     eroded_img = cv2.erode(thresh_img,kernel_erode,iterations=3)

#     # 图像膨胀
#     kernel_dilate = np.ones((3,3),np.uint8)
#     dilated_img = cv2.dilate(eroded_img,kernel_dilate,iterations=3)

    # 获取连通域信息
    retvals,labels,stats,centroids = cv2.connectedComponentsWithStats(dilated_img,8)

    if retvals == 1:
        label_to_keep_1 = 0
    else:
        max_connect = 0
        for i in range(1,retvals):
            if max_connect <= stats[i][4]:
                max_connect = stats[i][4]
                label_to_keep_1 = i

    # ct_dis = []

    # for i in range(1,retvals):
    # a_centroid = (int(centroids[i,0]),int(centroids[i,1]))
    # dis = math.sqrt((a_centroid[0]-previous_centroid_global[0])**2+(a_centroid[1]-previous_centroid_global[1])**2)
    # ct_dis.append((i,dis))

    mindis = 999.0
    for i in ct_dis:
        if(i[1]<mindis):
            mindis = i[1]
            label_to_keep_2 = i[0]
            previous_centroid_global = [int(centroids[i[0],0]),int(centroids[i[0],1])]

    if label_to_keep_1 == label_to_keep_2:
        label_to_keep = label_to_keep_1
    else:
        label_to_keep = label_to_keep_1

    # pure_lane = np.zeros_like(dilated_img)
    # pure_lane[labels == label_to_keep] = 255
#     pure_lane = np.zeros_like(dilated_img)
#     pure_lane[labels == label_to_keep] = 255
#     print(stats)
#     cv2.imshow("pure",pure_lane)
#     cv2.imshow("thresh_img",thresh_img)
#     cv2.waitKey(0)
    
# image = cv2.imread("../images/15.png")
# process_img(image)



