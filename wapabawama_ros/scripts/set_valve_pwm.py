#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2021 nvidia <nvidia@nvidia-desktop>
#
# Distributed under terms of the MIT license.

import sys
import rospy
from std_msgs.msg import Int8

def main(pwm):
    pub = rospy.Publisher('/wapabawama_ros/valve1/pwm', Int8, queue_size=10, latch=True)
    rospy.init_node('simple_set_valve_pwm', anonymous=True)
    while pub.get_num_connections() < 1:
        pass
    if not rospy.is_shutdown():
        
        p = Int8()
        p.data = eval(pwm)
        pub.publish(p)

if __name__ == '__main__':
    try:
        main(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
