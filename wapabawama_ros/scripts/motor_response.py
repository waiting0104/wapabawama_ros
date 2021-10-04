#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2021 nvidia <nvidia@nvidia-desktop>
#
# Distributed under terms of the MIT license.

import sys
import rospy
import json
import time
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32

ref = []
res = []

def myhook():
    global res, ref
    all_data = {
        'ref': ref,
        'res': res
    }
    with open('data.json', 'w') as f:
        json.dump(all_data, f)

    print "shutdown time!"

def callback1(data):
    global res
    data = {
        'time': time.time(),
        'value': data.data
    }
    res.append(data)

def callback2(data):
    global ref
    data = {
        'time': time.time(),
        'value': data.data
    }
    ref.append(data)
    print(data)

def main():
    # pos = 0.2
    # pub = rospy.Publisher('/la1/set_pose', Float32, queue_size=10, latch=True)

    rospy.Subscriber("/la1/pose", Float32, callback1)
    rospy.Subscriber("/la1/set_pose", Float32, callback2)
    rospy.init_node('motor_resp', anonymous=True, disable_signals=False)
    # while pub.get_num_connections() < 1:
    #     pass
    # if not rospy.is_shutdown():
    #     p = Float32()
    #     p.data = pos 
    #     pub.publish(p)
    rospy.on_shutdown(myhook)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
