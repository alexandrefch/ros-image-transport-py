# Copyright (C) 2023 University of South Brittany, Lab-STICC UMR 6285 All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==================================================================================================
"""
ImageRaw TransportType, use sensor_msgs.msg.Image message to send image over ROS network.
"""

# ==================================================================================================
#                                          I M P O R T S
# ==================================================================================================

from cv_bridge import CvBridge
import numpy as np
import rospy
from sensor_msgs.msg import Image

from image_transport import TransportType, ImageType

# ==================================================================================================
#                                             C O D E
# ==================================================================================================

class ImageRaw(TransportType):

    _BRIDGE    = CvBridge()

    topic_uri = 'image_raw'

    def read_message(self, message : Image, image_type : str = ImageType.BGR8) -> np.ndarray:
        """
        Read an incoming message and return it's corresponding image.

        Parameters
        ----------
            message : Image
                Name of the transport type (eg:'compressed')
            image_type : str (default=ImageType.BGR8)
                Image type, look at image_transport.Imagetype for more option ('bgr8','rgb8',...)
        Return
        ------
            numpy.ndarray
        """
        image = ImageRaw._BRIDGE.imgmsg_to_cv2(message, image_type)
        return image

    def write_message(self, image : np.ndarray, image_type : str = ImageType.BGR8) -> Image:
        """
        Read an incoming image and return it's corresponding message.

        Parameters
        ----------
            image : numpy.ndarray
                Numpy image (same as OpenCv::Mat, no need to convert) that need to be convert
            image_type : str (default=ImageType.BGR8)
                Image type, look at image_transport.Imagetype for more option ('bgr8','rgb8',...)
        Return
        ------
            Image
        """
        chanel = 1 if image_type == ImageType.MONO8 else 3

        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.height       = image.shape[0]
        msg.width        = image.shape[1]
        msg.encoding     = image_type
        msg.is_bigendian = False
        msg.step         = msg.width * chanel
        msg.data         = np.reshape(image,(msg.height*msg.width*chanel)).tolist()
        return msg

    def get_message_type(self) -> type:
        """
        Return the ROS message type.

        Return
        ------
            type
        """
        return Image