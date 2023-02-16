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
from image_transport import ImageTransport

rospy.init_node("my_node_publisher")
publisher = ImageTransport.advertise("my_topic")
publisher.publish(image)
```

</font>

Calling the function `ImageTransport.advertise()` will lead to the instantiation of an object `image_transport.Publisher` that will automaticly generate all supported transport type topic as below. (eg: rostopic list using `ImageTransport.advertise("my_topic")`)

<font size=2>

```txt
my_topic/image/compressed
my_topic/image/image_raw
```

</font>

**Subscriber**

Same as above, here if you want for example subscribe to the topic named `my_topic/image/image_raw` inside `rostopic list`, simply use `my_topic` inside the function `ImageTransport.subscribe` and choose `image_raw` type like in example below. (By default the subscriber will subscribe to the `compressed` topic)

<font size=2>

```py
# Subscriber example

import rospy
from image_transport import ImageTransport

def callback(image):
    print(f"receive image of shape {image.shape}")

rospy.init_node("my_node_subscriber")
ImageTransport.subscribe("my_topic",callback,3,'image_raw')
```

</font>

## :books: Supported transport type

- `image_raw`
- `compressed` (using jpeg compression)

<font size=2>

> **Note** <br>
> If you want to help and add new transport type feel free to fork this project and make a merge request.

</font>