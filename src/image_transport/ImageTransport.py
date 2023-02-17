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
ImageTransport object is use to manage automaticly ROS image flux.
"""

# ==================================================================================================
#                                          I M P O R T S
# ==================================================================================================

from typing import Callable

from . import Publisher, ImageType, Subscriber

# ==================================================================================================
#                                             C O D E
# ==================================================================================================

class ImageTransport():
    """
    ImageTransport object use to manage automaticly ROS image flux.
    """

    @staticmethod
    def subscribe(topic_uri: str, callback: Callable, queue_size: int = 3, image_type: str = ImageType.BGR8):
        """
        Create a new subscriber

        Parameters
        ----------
            topic_uri : str
                Image topic name
            callback: Callable
                Function callback that will be call each time ImageTransport receive an image
            queue_size : int (default=3)
                Topic queue size
            image_type : str (default=ImageType.BGR8)
                Image type, look at image_transport.Imagetype for more option ('bgr8','rgb8',...)
        Return
        ------
            image_transport.Subscriber
        """
        return Subscriber(
            topic_uri           = ImageTransport.filter_uri(topic_uri),
            callback            = callback,
            queue_size          = queue_size,
            image_type          = image_type
        )

    @staticmethod
    def advertise(topic_uri: str, queue_size: int = 3) -> Publisher:
        """
        Create a new publisher

        Parameters
        ----------
            topic_uri : str
                Image topic name (eg: 'camera' will automaticly create topic named
                'camera/image/raw' and 'camera/image/compressed')
            queue_size : int (default=3)
                Topic queue size
        Return
        ------
            image_transport.Publisher
        """
        return Publisher(
            topic_uri  = ImageTransport.filter_uri(topic_uri),
            queue_size = queue_size
        )

    @staticmethod
    def filter_uri(uri:str):
        return uri.lstrip('/').rstrip('/')
