#!/usr/bin/env python

import rospy
import random
from sensor_msgs.msg import Imu, NavSatFix

def imu_gps_publisher():
    rospy.init_node('imu_gps_publisher', anonymous=True)
    imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
    gps_pub = rospy.Publisher('gps', NavSatFix, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        # Create IMU message with random values
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.angular_velocity.x = random.uniform(-1.0, 1.0)
        imu_msg.angular_velocity.y = random.uniform(-1.0, 1.0)
        imu_msg.angular_velocity.z = random.uniform(-1.0, 1.0)
        imu_msg.linear_acceleration.x = random.uniform(-1.0, 1.0)
        imu_msg.linear_acceleration.y = random.uniform(-1.0, 1.0)
        imu_msg.linear_acceleration.z = random.uniform(-1.0, 1.0)

        # Create GPS message with random values
        gps_msg = NavSatFix()
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.latitude = random.uniform(-90.0, 90.0)
        gps_msg.longitude = random.uniform(-180.0, 180.0)
        gps_msg.altitude = random.uniform(0.0, 5000.0)

        imu_pub.publish(imu_msg)
        gps_pub.publish(gps_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        imu_gps_publisher()
    except rospy.ROSInterruptException:
        pass