# coding=utf-8
import cv2
import numpy as np

def process_img(input_image):
    hsv_img = cv2.cvtColor(input_image, cv2.COLOR_BGR2HSV)

    lower = np.array([92, 109, 47])
    upper = np.array([180, 255, 255])

    mask = cv2.inRange(hsv_img, lower, upper)

    # ≈Ú’Õ
    kernel_dilate = np.ones((3,3),np.uint8)
    mask = cv2.dilate(mask,kernel_dilate,iterations=3)

    # ∏Ø ¥
    kernel_erode = np.ones((3,3),np.uint8)
    mask = cv2.erode(mask,kernel_erode,iterations=3)
    return mask

if __name__ == "__main__":
    input_image = cv2.imread("../images/19.png")
    result = process_img(input_image)
    cv2.imshow("hsv red",result)
    cv2.waitKey(0)

