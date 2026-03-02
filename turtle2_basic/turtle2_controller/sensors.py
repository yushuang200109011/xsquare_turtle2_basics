# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
from sensor_msgs.msg import Image
import rospy
import cv2
from cv_bridge import CvBridge
# msgs

class SensorBase:
    def __init__(self):
        pass
    
class Camera(SensorBase):
    def __init__(self):
        super().__init__()
        self.image1Data = None
        self.image2Data = None
        self.image3Data = None
        self.bridge = CvBridge()
        self.camera1_sub = rospy.Subscriber('/camera1/usb_cam1/image_raw', Image, self.camera1_callback)
        self.camera2_sub = rospy.Subscriber('/camera2/usb_cam2/image_raw', Image, self.camera2_callback)
        self.camera3_sub = rospy.Subscriber('/camera3/usb_cam3/image_raw', Image, self.camera3_callback)
        self.cam1_time = rospy.Time.now()
        self.cam2_time = rospy.Time.now()
        self.cam3_time = rospy.Time.now()

    def compress_image(self,image_np):
        success, encoded = cv2.imencode(".jpg", image_np)
        if not success:
            raise ValueError("Failed to compress image")
        return encoded.tobytes()

    def camera1_callback(self, data):
        if data is not None:
            self.image1Data = data
            self.cam1_time = rospy.Time.now()

    def camera2_callback(self, data):
        if data is not None:
            self.image2Data = data
            self.cam2_time = rospy.Time.now()
        
    def camera3_callback(self, data):
        if data is not None:
            self.image3Data = data
            self.cam3_time = rospy.Time.now()

    def get_cam1_data(self):
        cv_image = self.bridge.imgmsg_to_cv2(self.image1Data, "rgb8")
        return cv_image

    def get_cam2_data(self):
        cv_image = self.bridge.imgmsg_to_cv2(self.image2Data, "rgb8")
        return cv_image
    
    def get_cam3_data(self):
        cv_image = self.bridge.imgmsg_to_cv2(self.image3Data, "rgb8")
        return cv_image
    
    def check_cam1(self, timeout=0.5):
        rospy.sleep(timeout)
        elapsed_time = (rospy.Time.now() - self.cam1_time).to_sec()
        return elapsed_time < timeout

    def check_cam2(self, timeout=0.5):
        rospy.sleep(timeout)
        elapsed_time = (rospy.Time.now() - self.cam2_time).to_sec()
        return elapsed_time < timeout
    
    def check_cam3(self, timeout=0.5):
        rospy.sleep(timeout)
        elapsed_time = (rospy.Time.now() - self.cam3_time).to_sec()
        return elapsed_time < timeout
    



