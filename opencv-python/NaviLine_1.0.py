#-*-coding:utf-8 -*-
import cv2   # type: ignore
import numpy as np  #type: ignore 
import math

#原始图像
vanila_image = cv2.imread('../images/11.jpg')

# 使用灰度模式读取图像
gray_image = cv2.cvtColor(vanila_image, cv2.COLOR_BGR2GRAY)
height, width= gray_image.shape
# 颜色反转
img = cv2.bitwise_not(gray_image)

# 进行二值化
_, thresh = cv2.threshold(img, 127, 255, cv2.THRESH_OTSU)

# 图像腐蚀
kernel_erode = np.ones((6,6),np.uint8)
eroded_image = cv2.erode(thresh, kernel_erode, iterations=8)

# 调试腐蚀
# test_kernel = np.ones((7,7),np.uint8)
# for i in range(10):
#     test_img = thresh
#     cv2.imshow(f'ercoded_{i}',cv2.erode(thresh, test_kernel, iterations=i))

# 图像膨胀 
kernel_dilate = np.ones((6, 6), np.uint8)
dilated_image = cv2.dilate(eroded_image, kernel_dilate, iterations=8)

# 找到连通域和统计信息
num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(dilated_image, connectivity=8)

# 将二值化图像转化为BGR图像
rgb_image = cv2.cvtColor(np.dstack([dilated_image, dilated_image, dilated_image]), cv2.COLOR_BGR2RGB)

#去除多余元素，保留引导线
last_point = (width//2,0)
dis = []
min_dis_label = 1
min_dis = math.inf
for i in range(1,num_labels):
    centroid = (int(centroids[i, 0]), int(centroids[i, 1]))
    dis.append((int((centroids[i, 0]*centroids[i, 0]+centroids[i, 1]*centroids[i, 1])**0.5),i))
for i in dis:
    if(i[0]<min_dis):
        min_dis = i[0]
    min_dis_label = i[1]
pure_lane_image = np.zeros_like(thresh)
pure_lane_image[labels == min_dis_label] = 255

# 获得车道上部的质心，画线，计算转向角度
_,_,_,centroid_top_lane = cv2.connectedComponentsWithStats(pure_lane_image[0:height//5,0:width],connectivity = 8)
centroid_top_lane_point = (int(centroid_top_lane[1,0]),int(centroid_top_lane[1,1]))
cv2.circle(rgb_image,centroid_top_lane_point,3,(0,255,0),-1)
cv2.line(rgb_image,(width//2,height),centroid_top_lane_point,(102,255,178),2)#引导转向线
cv2.line(rgb_image,(width//2,height),(width//2,0),(102,255,255),1)#当前方向

angle_result = math.atan((width/2-centroid_top_lane[1,0])/(height-centroid_top_lane[1,1]))
# angle_result = math.atan((width/2-width)/(height-(height-1))) #test
print(f'width/2-centroid_top_lane[1,0]={width/2-centroid_top_lane[1,0]}')
print(f'height-centroid_top_lane[1,1]={height-centroid_top_lane[1,1]}')
print(f'angle = {angle_result} (rad)')
print(f'angle = {math.degrees(angle_result)} (degree)')


# 遍历所有连通域（不包括背景，背景标签为0） 
for i in range(1, num_labels):
    # 绘制质心，质心坐标是浮点数，转换为整数  
    centroid = (int(centroids[i, 0]), int(centroids[i, 1]))  
    # 绘制一个圆来表示质心，半径为3
    cv2.circle(rgb_image, centroid, 3, (255,0,0), -1)




# # 在图像上添加文字
# font = cv2.FONT_HERSHEY_SIMPLEX  # 选择字体  
# font_scale = 1  # 字体大小  
# font_color = (255, 0, 0)  # 文字颜色，这里是红色（BGR）  
# thickness = 2  # 文字粗细

# # 参数依次是：图像、文字内容、左下角坐标（x, y）、字体、字体大小、颜色、粗细、线型（可选）、自底向上（可选）
# print(num_labels)
# for i in range(1,num_labels):
#     if(int(centroids[i, 0])>height or int(centroids[i, 1])>width):
#         continue
#     text = f'label:{labels[int(centroids[i, 0]), int(centroids[i, 1])]}'
#     point_left_corner = (int(centroids[i, 0]), int(centroids[i, 1]))
#     cv2.putText(rgb_image, text, point_left_corner, font, font_scale, font_color, thickness)
#     print(f'round: {i} finished')

# 显示各种图像
cv2.imshow('pure_lane_image',pure_lane_image)
cv2.imshow('Thresholded Image', thresh)
cv2.imshow('Centroids', rgb_image)
cv2.imshow('eroded_image',eroded_image)
cv2.imshow('dilated_image',dilated_image)
cv2.imshow('vanila_image',vanila_image)

# 等待按键并关闭窗口  
cv2.waitKey(0)
cv2.destroyAllWindows()