#!/usr/bin/env python3

import rospy
import random
from sensor_msgs.msg import Imu, NavSatFix
from custom_msgs.msg import Encoder

def imu_callback(event):
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.angular_velocity.x = random.uniform(-1.0, 1.0)
    imu_msg.angular_velocity.y = random.uniform(-1.0, 1.0)
    imu_msg.angular_velocity.z = random.uniform(-1.0, 1.0)
    imu_msg.linear_acceleration.x = random.uniform(-1.0, 1.0)
    imu_msg.linear_acceleration.y = random.uniform(-1.0, 1.0)
    imu_msg.linear_acceleration.z = random.uniform(-1.0, 1.0)

    imu_pub.publish(imu_msg)

def gps_callback(event):
    gps_msg = NavSatFix()
    gps_msg.header.stamp = rospy.Time.now()
    gps_msg.latitude = random.uniform(-90.0, 90.0)
    gps_msg.longitude = random.uniform(-180.0, 180.0)
    gps_msg.altitude = random.uniform(0.0, 5000.0)

    gps_pub.publish(gps_msg)
    
def encoder_callback(event):
    encoder_msg = Encoder()
    encoder_msg.header.stamp = rospy.Time.now()
    encoder_msg.data = random.randint(0, 1000)
    encoder_pub.publish(encoder_msg)


if __name__ == '__main__':
    rospy.init_node('sensor_simulator', anonymous=True)
    imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
    gps_pub = rospy.Publisher('gps', NavSatFix, queue_size=10)
    encoder_pub = rospy.Publisher('encoder', Encoder, queue_size=10)

    imu_timer = rospy.Timer(rospy.Duration(0.1), imu_callback)  # 10Hz for IMU
    gps_timer = rospy.Timer(rospy.Duration(0.2), gps_callback)  # 5Hz for GPS
    encoder_timer = rospy.Timer(rospy.Duration(0.06667), encoder_callback)  # 15Hz for Encoder
    
    rospy.spin()