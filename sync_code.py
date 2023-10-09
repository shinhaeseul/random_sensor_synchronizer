#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo

rospy.init_node('sync_test')

def gotimage(image, camerainfo):
    print("Image와 CameraInfo를 받았습니다.")
    

image_sub=message_filters.Subscriber("/camera/rgb/image_raw", Image)
camera_sub=message_filters.Subscriber("/camera/rgb/camera_info", CameraInfo)

ats=message_filters.ApproximateTimeSynchronizer([image_sub, camera_sub], queue_size=5, slop=0.1)
ats.registerCallback(gotimage)
rospy.spin()