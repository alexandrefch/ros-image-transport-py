<div align="center">

# ROS Image Transport Python

![Ros](https://img.shields.io/badge/Ros-Noetic-green?style=for-the-badge&logo=ROS)
![Python](https://img.shields.io/badge/Python-3.8-green?style=for-the-badge&logo=Python&logoColor=FFFFFF)

</div>

This ROS package aim to give the same facility of use of image topic as [image_transport](http://wiki.ros.org/image_transport) can does in c++. Moreover, this package is compatible with [image_transport](http://wiki.ros.org/image_transport), that mean that you can communicate to existing node that use image transport in c++. (Like [rqt](http://wiki.ros.org/rqt) or [RViz](http://wiki.ros.org/rviz))

## :rocket: Getting Started

### :gear: Compilation

<font size=2>

```shell
git clone https://github.com/alexandrefch/ros-image-transport-py.git
catkin build image_transport_py
```

> **Warning** <br>
> The catkin compilation might lead to some dependencies errors, do not hesitate to report them so they can be corrected.

</font>

### :computer: How to use

**Publisher**

<font size=2>

```py
# Publisher example

import rospy
from image_transport.ImageTransport import ImageTransport

rospy.init_node("my_node_publisher")
publisher = ImageTransport.advertise("my_topic/image")
publisher.publish(image)
```

</font>

Calling the function `ImageTransport.advertise()` will lead to the instantiation of an object `image_transport.Publisher` that will automaticly generate all supported transport type topic as below. (eg: rostopic list using `ImageTransport.advertise("my_topic/image")`)

<font size=2>

```txt
my_topic/image
my_topic/image/compressed
```

</font>

**Subscriber**

Now if you want to subscribe you simply need create call `ImageTransport.subscribe` and choose your desire format topic, the library will detect what type is it using topic name and pick the correct image decoder.
(eg : if you want to subscribe to a compressed topic simply use `ImageTransport.subscribe("my_topic/image/compressed",callback)`)

<font size=2>

```py
# Subscriber example

import rospy
from image_transport.ImageTransport import ImageTransport

def callback(image):
    print(f"receive image of shape {image.shape}")

rospy.init_node("my_node_subscriber")
ImageTransport.subscribe("my_topic/image/compressed",callback)
```

</font>

## :books: Supported transport type

- `image_raw`
- `compressed` (using jpeg compression)

<font size=2>

> **Note** <br>
> If you want to help and add new transport type feel free to fork this project and make a merge request.

</font>