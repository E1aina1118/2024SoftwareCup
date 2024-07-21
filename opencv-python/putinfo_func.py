import cv2
import numpy as np
import math

def put_info(img,centroid,rad):
    height, width= img.shape
    img_rgb = np.stack((img,)*3, axis=-1)
    color_1 = (255,191,0)
    color_2 = (0,0,255)
    start_point = (width//2,height)
    end_point_1 = (centroid[0],centroid[1])
    end_point_2 = (width//2,0)
    cv2.line(img_rgb,start_point,end_point_1,color_1,2)
    cv2.line(img_rgb,start_point,end_point_2,color_2,2)
    cv2.putText(img_rgb,f"degree:{math.pi/180*rad}",(0,0),cv2.FONT_HERSHEY_SIMPLEX,1,color_2,2)
    return img_rgb