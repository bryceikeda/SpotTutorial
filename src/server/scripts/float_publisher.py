#!/usr/bin/env python

import random
import rospy
from std_msgs.msg import Int32

TOPIC_NAME = 'inty'
NODE_NAME = 'int_publisher'


def loop_send_integer():
    pub = rospy.Publisher(TOPIC_NAME, Int32, queue_size=10)
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():

        rand_int = random.randint(0, 255)
        rospy.loginfo("Integer Sent")
        pub.publish(rand_int)
        rate.sleep()
        break


if __name__ == '__main__':
    try:
        loop_send_integer()
    except rospy.ROSInterruptException:
        pass
