#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32
from pyproj import Proj, transform
from message_filters import ApproximateTimeSynchronizer, Subscriber

class OdometryPublisher:
    def __init__(self):
        rospy.init_node('odometry_publisher', anonymous=True)
        
        # Initialize publishers
        self.odom_pub = rospy.Publisher('odometry', Odometry, queue_size=10)

        # Initialize variables
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.kcity = Proj(init='epsg:5179')
        self.wgs84 = Proj(init='epsg:4326')
        
        # Initialize initial pose and velocity
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0

        # Create time synchronizer for IMU, GPS, and encoder data
        imu_sub = Subscriber('imu', Imu)
        gps_sub = Subscriber('gps', NavSatFix)
        encoder_sub = Subscriber('encoder', Int32)

        ts = ApproximateTimeSynchronizer([imu_sub, gps_sub, encoder_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.callback)

    def callback(self, imu_msg, gps_msg, encoder_msg):
        # Extract yaw angle from IMU orientation
        orientation = imu_msg.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.theta = euler_from_quaternion(quaternion)

        # Extract GPS position data
        self.x, self.y = transform(self.wgs84, self.kcity, gps_msg.longitude, gps_msg.latitude)

        # Convert encoder value to linear velocity (assuming a wheel diameter)
        wheel_diameter = 0.1  # Replace with your wheel diameter
        encoder_value = encoder_msg.data
        delta_encoder = encoder_value - self.prev_encoder_value

        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Calculate linear velocity
        self.vx = delta_encoder * (wheel_diameter / 2.0) / dt

        # Update previous encoder value and time
        self.prev_encoder_value = encoder_value
        self.last_time = current_time

    def publish_odometry(self):
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()

        # Compute angular velocity
        self.vtheta = (self.theta - self.prev_theta) / dt

        # Create and publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(*euler_from_quaternion([0, 0, self.theta]))

        # Set velocity
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.vtheta

        self.odom_pub.publish(odom_msg)

        # Update previous theta
        self.prev_theta = self.theta

    def run(self):
        self.prev_encoder_value = 0
        self.prev_theta = 0.0
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.publish_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        odometry_publisher = OdometryPublisher()
        odometry_publisher.run()
    except rospy.ROSInterruptException:
        pass
