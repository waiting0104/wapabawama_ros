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
from nav_msgs.msg import Path 

pos_datas = []

def callback_once(data):
    for pose in data.poses:
        pos_datas.append({
            'x': pose.pose.position.x,
            'y': pose.pose.position.y,
            'time': pose.header.stamp.to_sec()
        })

    with open('result.json', 'w') as fp:
        json.dump(pos_datas, fp)
    print("Done!")
    sub_once.unregister()
    rospy.signal_shutdown("Shutdown")

def listener(topic):

    rospy.init_node('dump_path', anonymous=True, disable_signals=False)

    global sub_once
    sub_once = rospy.Subscriber(topic, Path, callback_once)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
