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
"""

# ==================================================================================================
#                                          I M P O R T S
# ==================================================================================================

import rospy
from typing import Callable

from . import TransportType, ImageType

# ==================================================================================================
#                                             C O D E
# ==================================================================================================

class Subscriber():

    def __init__(self, topic_uri : str, callback: Callable, queue_size : int = 3,
        transport_type_name : str = 'compressed', image_type : str = ImageType.BGR8) -> None:
        """
        Create a new subscriber

        Parameters
        ----------
            topic_uri : str
                Image topic name (eg: 'camera' will automaticly search for topic named
                'camera/image/raw_image' or 'camera/image/compressed')
            callback: Callable
                Function callback that will be call each time ImageTransport receive an image
            queue_size : int (default=3)
                Topic queue size
            transport_type_name : str (default='compressed')
                Transport type name that correspond to his uri end (eg:'compressed','image_raw',...)
            image_type : str (default=ImageType.BGR8)
                Image type, look at image_transport.Imagetype for more option ('bgr8','rgb8',...)
        Return
        ------
            image_transport.Subscriber
        """
        self._topic_uri     = topic_uri
        self._queue_size    = queue_size
        self._user_callback = callback
        self._image_type    = image_type
        self._transport     = TransportType.get(transport_type_name)

        rospy.Subscriber(
            name       = f'{topic_uri}/image/{transport_type_name}',
            data_class = self._transport.get_message_type(),
            callback   = self._callback,
            queue_size = queue_size
        )

    def _callback(self, message):
        """
        Function call each time a new message is receive, it decode the image
        according the to selected image transport and call the user callback.
        """
        image = self._transport.read_message(message, self._image_type)
        self._user_callback(image)
