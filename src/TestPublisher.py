import rospy
import numpy as np
import cv2 as cv
from image_transport import ImageTransport

rospy.init_node("ma_node_pub")
pub = ImageTransport.advertise("mon_flux")

img = np.zeros((64,64,3),dtype=np.uint8)
img = cv.circle(
    img,
    (32,32),
    10,
    (0,0,255),
    -1
)

while True:
    pub.publish(img)
    rospy.sleep(0.1)
    if rospy.is_shutdown():
        break