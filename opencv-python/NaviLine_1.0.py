#-*-coding:utf-8 -*-
import cv2   # type: ignore
import numpy as np  #type: ignore 
import math

#ԭʼͼ��
vanila_image = cv2.imread('../images/11.jpg')

# ʹ�ûҶ�ģʽ��ȡͼ��
gray_image = cv2.cvtColor(vanila_image, cv2.COLOR_BGR2GRAY)
height, width= gray_image.shape
# ��ɫ��ת
img = cv2.bitwise_not(gray_image)

# ���ж�ֵ��
_, thresh = cv2.threshold(img, 127, 255, cv2.THRESH_OTSU)

# ͼ��ʴ
kernel_erode = np.ones((6,6),np.uint8)
eroded_image = cv2.erode(thresh, kernel_erode, iterations=8)

# ���Ը�ʴ
# test_kernel = np.ones((7,7),np.uint8)
# for i in range(10):
#     test_img = thresh
#     cv2.imshow(f'ercoded_{i}',cv2.erode(thresh, test_kernel, iterations=i))

# ͼ������ 
kernel_dilate = np.ones((6, 6), np.uint8)
dilated_image = cv2.dilate(eroded_image, kernel_dilate, iterations=8)

# �ҵ���ͨ���ͳ����Ϣ
num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(dilated_image, connectivity=8)

# ����ֵ��ͼ��ת��ΪBGRͼ��
rgb_image = cv2.cvtColor(np.dstack([dilated_image, dilated_image, dilated_image]), cv2.COLOR_BGR2RGB)

#ȥ������Ԫ�أ�����������
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

# ��ó����ϲ������ģ����ߣ�����ת��Ƕ�
_,_,_,centroid_top_lane = cv2.connectedComponentsWithStats(pure_lane_image[0:height//5,0:width],connectivity = 8)
centroid_top_lane_point = (int(centroid_top_lane[1,0]),int(centroid_top_lane[1,1]))
cv2.circle(rgb_image,centroid_top_lane_point,3,(0,255,0),-1)
cv2.line(rgb_image,(width//2,height),centroid_top_lane_point,(102,255,178),2)#����ת����
cv2.line(rgb_image,(width//2,height),(width//2,0),(102,255,255),1)#��ǰ����

angle_result = math.atan((width/2-centroid_top_lane[1,0])/(height-centroid_top_lane[1,1]))
# angle_result = math.atan((width/2-width)/(height-(height-1))) #test
print(f'width/2-centroid_top_lane[1,0]={width/2-centroid_top_lane[1,0]}')
print(f'height-centroid_top_lane[1,1]={height-centroid_top_lane[1,1]}')
print(f'angle = {angle_result} (rad)')
print(f'angle = {math.degrees(angle_result)} (degree)')


# ����������ͨ�򣨲�����������������ǩΪ0�� 
for i in range(1, num_labels):
    # �������ģ����������Ǹ�������ת��Ϊ����  
    centroid = (int(centroids[i, 0]), int(centroids[i, 1]))  
    # ����һ��Բ����ʾ���ģ��뾶Ϊ3
    cv2.circle(rgb_image, centroid, 3, (255,0,0), -1)




# # ��ͼ�����������
# font = cv2.FONT_HERSHEY_SIMPLEX  # ѡ������  
# font_scale = 1  # �����С  
# font_color = (255, 0, 0)  # ������ɫ�������Ǻ�ɫ��BGR��  
# thickness = 2  # ���ִ�ϸ

# # ���������ǣ�ͼ���������ݡ����½����꣨x, y�������塢�����С����ɫ����ϸ�����ͣ���ѡ�����Ե����ϣ���ѡ��
# print(num_labels)
# for i in range(1,num_labels):
#     if(int(centroids[i, 0])>height or int(centroids[i, 1])>width):
#         continue
#     text = f'label:{labels[int(centroids[i, 0]), int(centroids[i, 1])]}'
#     point_left_corner = (int(centroids[i, 0]), int(centroids[i, 1]))
#     cv2.putText(rgb_image, text, point_left_corner, font, font_scale, font_color, thickness)
#     print(f'round: {i} finished')

# ��ʾ����ͼ��
cv2.imshow('pure_lane_image',pure_lane_image)
cv2.imshow('Thresholded Image', thresh)
cv2.imshow('Centroids', rgb_image)
cv2.imshow('eroded_image',eroded_image)
cv2.imshow('dilated_image',dilated_image)
cv2.imshow('vanila_image',vanila_image)

# �ȴ��������رմ���  
cv2.waitKey(0)
cv2.destroyAllWindows()