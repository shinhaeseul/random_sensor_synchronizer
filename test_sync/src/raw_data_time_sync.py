#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Int32
from message_filters import ApproximateTimeSynchronizer, Subscriber
from custom_msgs.msg import CombinedMessage,Encoder  # 사용자 정의 메시지 추가

combined_pub = rospy.Publisher('combined_message', CombinedMessage, queue_size=10)


    
def callback(imu_msg, gps_msg, encoder_msg):
    print("combined")
    combined_msg = CombinedMessage()
    combined_msg.header.stamp=rospy.Time.now()
    combined_msg.imu_data = imu_msg
    combined_msg.gps_data = gps_msg
    combined_msg.encoder_data = encoder_msg

    combined_pub.publish(combined_msg)
    
if __name__ == '__main__':
    rospy.init_node('message_combiner', anonymous=True)

    imu_sub=Subscriber('imu', Imu)
    gps_sub=Subscriber('gps', NavSatFix)
    encoder_sub=Subscriber('encoder', Encoder)

    ts = ApproximateTimeSynchronizer([imu_sub, gps_sub, encoder_sub], queue_size=10, slop=1)
    ts.registerCallback(callback)

    rospy.spin()