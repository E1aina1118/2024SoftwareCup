# -*- coding:utf-8 -*-

import cv2
import numpy as np

image = cv2.imread('../images/20.png') # ����·����ȡһ��ͼƬ
cv2.imshow("BGR", image) # ��ʾͼƬ

hsv_low = np.array([0, 0, 0])
hsv_high = np.array([0, 0, 0])

# ���漸��������д���е�����

def h_low(value):
    hsv_low[0] = value

def h_high(value):
    hsv_high[0] = value

def s_low(value):
    hsv_low[1] = value

def s_high(value):
    hsv_high[1] = value

def v_low(value):
    hsv_low[2] = value

def v_high(value):
    hsv_high[2] = value

cv2.namedWindow('image',cv2.WINDOW_AUTOSIZE)
# �����Լ��趨��ʼֵ�����ֵ255����Ҫ����
cv2.createTrackbar('H low', 'image', 35, 255, h_low) 
cv2.createTrackbar('H high', 'image', 90, 255, h_high)
cv2.createTrackbar('S low', 'image', 43, 255, s_low)
cv2.createTrackbar('S high', 'image', 255, 255, s_high)
cv2.createTrackbar('V low', 'image', 35, 255, v_low)
cv2.createTrackbar('V high', 'image', 255, 255, v_high)

while True:
    dst = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # BGRתHSV
    dst = cv2.inRange(dst, hsv_low, hsv_high) # ͨ��HSV�ĸߵ���ֵ����ȡͼ�񲿷�����
    cv2.imshow('dst', dst)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()
