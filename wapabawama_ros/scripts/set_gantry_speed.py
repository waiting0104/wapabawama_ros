#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2021 nvidia <nvidia@nvidia-desktop>
#
# Distributed under terms of the MIT license.

import sys
import rospy
import time
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32

def main(speed):
    pub = rospy.Publisher('/wapabawama_ros/gantry/set_speed', Float32, queue_size=10, latch=True)
    rospy.init_node('simple_set_gantry_speed', anonymous=True)
    while pub.get_num_connections() < 1:
        pass
    if not rospy.is_shutdown():
        p = Float32()
        p.data = eval(speed)
        pub.publish(p)

if __name__ == '__main__':
    try:
        main(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
