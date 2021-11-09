#!/usr/bin/env python
# -*- coding: utf-8 -*-

import logging
import sys

import rospy
from sensor_msgs.msg import Imu

logger = logging.getLogger(__name__)
logger.addHandler(logging.StreamHandler(sys.stdout))

logger.setLevel(logging.DEBUG)


class RosImu:
    def __init__(self):
        rospy.init_node('MyTeleop')
        rospy.on_shutdown(self.on_shutdown)
        self.subscriber = rospy.Subscriber('/imu/data_raw', Imu, self.callback)

    def callback(self, data_raw):
        ang = [data_raw.angular_velocity.x, data_raw.angular_velocity.y, data_raw.angular_velocity.z]
        logger.debug(ang)

    def on_shutdown(self):
        print "shutdown!"
        self.subscriber.unregister()


if __name__ == '__main__':
    try:
        RosImu()
        while not rospy.is_shutdown():  # rospy.spin()と同じ
            rospy.sleep(0.2)

    except rospy.ROSInterruptException:
        pass
