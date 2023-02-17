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
Compressed TransportType, use sensor_msgs.msg.CompressedImage message with jpeg compression to send
image over ROS network.
"""

# ==================================================================================================
#                                          I M P O R T S
# ==================================================================================================

import cv2 as cv
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

from image_transport import TransportType, ImageType

# ==================================================================================================
#                                             C O D E
# ==================================================================================================

class Compressed(TransportType):

    topic_uri = 'compressed'

    def read_message(self, message : CompressedImage, image_type : str = ImageType.BGR8) -> np.ndarray:
        """
        Read an incoming message and return it's corresponding image.

        Parameters
        ----------
            message : CompressedImage
                Name of the transport type (eg:'compressed')
            image_type : str (default=ImageType.BGR8)
                Image type, look at image_transport.Imagetype for more option ('bgr8','rgb8',...)
        Return
        ------
            numpy.ndarray
        """
        buffer = np.frombuffer(message.data,dtype=np.uint8)
        flag   = cv.IMREAD_GRAYSCALE if image_type == ImageType.MONO8 else cv.IMREAD_COLOR
        image  = cv.imdecode(buffer, flag)

        if image_type == ImageType.RGB8:
            image = cv.cvtColor(image, cv.COLOR_BGR2RGB)

        return image

    def write_message(self, image : np.ndarray) -> CompressedImage:
        """
        Read an incoming image and return it's corresponding message.

        Parameters
        ----------
            image : numpy.ndarray
                Numpy image (same as OpenCv::Mat, no need to convert) that need to be convert
        Return
        ------
            CompressedImage
        """
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv.imencode('.jpg', image)[1]).tobytes()
        return msg

    def get_message_type(self) -> type:
        """
        Return the ROS message type.

        Return
        ------
            type
        """
        return CompressedImage