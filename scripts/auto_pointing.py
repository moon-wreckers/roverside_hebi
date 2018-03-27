#!/usr/bin/env python

"""
auto_pointing.py
Automatically pointing the claw manipulator towards the companion rover ring.
"""

__version__     = "0.0.1"
__author__      = "David Qiu"
__email__       = "dq@cs.cmu.edu"
__website__     = "http://mrsdprojects.ri.cmu.edu/2017teami/"
__copyright__   = "Copyright (C) 2018, the Moon Wreckers. All rights reserved."

import rospy
from std_msgs.msg import String, Float32, UInt16
from sensor_msgs.msg import Imu, JointState, Joy
from geometry_msgs.msg import Vector3Stamped, Twist
from nav_msgs.msg import Odometry

import hebi
import numpy as np


def auto_pointing():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'auto_pointing' node so that multiple nodes can run
    # simultaneously.
    rospy.init_node('auto_pointing', anonymous=True)

    hebi_modules = [('arm', 'joint_0'), ('arm', 'joint_1')]

    # discover HEBI modules
    count_found = 0
    lookup = hebi.Lookup()
    sleep(2) # Give the Lookup process 2 seconds to discover modules
    print('Modules found on network:')
    for entry in lookup.entrylist:
        print('{0} | {1}'.format(entry.family, entry.name))
        if (entry.family, entry.name) in hebi_modules:
            count_found = count_found + 1

    if len(hebi_modules) != count_found:
        print('Some HEBI module(s) is missing (%d/%d)...' % (count_found, len(hebi_modules)))
        exit()

    # create a HEBI control group from a set of names
    hebi_group = lookup.get_group_from_names([m[0] for m in hebi_modules], [m[1] for m in hebi_modules])
    print('HEBI control group created...')
    print(hebi_group)

    # create TF listener
    tf_listener = tf.TransformListener()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            (T_ak1_ring, R_ak1_ring) = tf_listener.lookupTransform('/map', '/ak1_ring', rospy.Time(0))
            (T_ak2_claw, R_ak2_claw) = tf_listener.lookupTransform('/map', '/ak2_claw', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # TODO: controller code...

        rate.sleep()

    #rospy.spin() # use spin only in pure event handling programming model


if __name__ == '__main__':
    try:
        auto_pointing()
    except rospy.ROSInterruptException:
        pass
