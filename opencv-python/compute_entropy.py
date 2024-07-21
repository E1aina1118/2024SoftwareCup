# coding=utf-8
import cv2  
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
bridge = CvBridge()


def image_variance(image):  
    # ��ͼ��ת��Ϊ�Ҷ�ͼ  
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  
    # ����Ҷ�ͼ��ķ���  
    variance = np.var(gray)  
    return variance  

def callback(Image):
    frame = bridge.imgmsg_to_cv2(Image,"bgr8")
    result = image_variance(frame)
    if result <= 400:
    

if __name__ == "__main__":
    pub = rospy.Publisher("aruco",Int32,queue_size=10)
    sub = rospy.Subscriber("image2",Image,callback,queue_size=10)


# ����ͼ��
# wood_texture = cv2.imread('../images/22.jpg')
# landscape = cv2.imread('../images/21.jpg')

# print(f"Wood Texture Variance: {variance_wood}")
