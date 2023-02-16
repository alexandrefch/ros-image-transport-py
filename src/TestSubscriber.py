import rospy
from image_transport import ImageTransport

def callback(image):
    print(f"receive image of shape {image.shape}")

rospy.init_node("ma_node_sub")
ImageTransport.subscribe("mon_flux",callback,3,'image_raw')

while True:
    rospy.sleep(0.3)
    if rospy.is_shutdown():
        break