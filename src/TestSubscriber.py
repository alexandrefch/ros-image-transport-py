#!/usr/bin/env python

import rospy
from image_transport.ImageTransport import ImageTransport

def callback(image):
    print(f"receive image of shape {image.shape}")

rospy.init_node("my_node_sub")
ImageTransport.subscribe("my_topic/image",callback)

while True:
    rospy.sleep(0.3)
    if rospy.is_shutdown():
        break