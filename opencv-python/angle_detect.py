# coding=utf-8

import cv2
import numpy as np

# stats�Ľṹ
#           0  1  2  3  4
# stats = [[x1,y1,w1,h1,area1],[x2,y2,w2,h2,area2],[...,...,...,...,...],...]

def angle_detect(image,stats,label_to_keep):
    i = label_to_keep
    x1 = stats[i][0]
    y1 = stats[i][1]
    x2 = stats[i][0] + stats[i][2]
    y2 = stats[i][1] + stats[i][3]
    imgh, imgw = image.shape[:2]
    print("x1:",x1)
    print("y1:",y1)
    print("x2:",x2)
    print("y2:",y2)
    print("imgw:",imgw)
    print("imgh:",imgh)
    if x1 == 0 and y1 > 0 and x2<imgw and y2 == imgh:
        sign = "left"
    else:
        sign = "NONE"

    return sign

img = cv2.imread("../images/16.jpg")
gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
retvals,labels,stats,_ = cv2.connectedComponentsWithStats(gray_img,8)
max_connect = 0
for i in range(1,retvals):
    if max_connect <= stats[i][4]:
        max_connect = stats[i][4]
        label_to_keep = i
pure_lane = np.zeros_like(gray_img)
pure_lane[labels == label_to_keep] = 255
print(stats)
print("stat of pure_lane")
print(stats[label_to_keep])
print(angle_detect(pure_lane,stats,label_to_keep))
cv2.imshow("pure_lane",pure_lane)
cv2.imshow("img",img)
cv2.waitKey(0)