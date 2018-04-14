#!/usr/bin/env python

"""
auto_pointing.py
Automatically pointing the claw manipulator towards the companion rover ring.
"""

__version__ = "0.0.1"
__author__ = "David Qiu"
__email__ = "dq@cs.cmu.edu"
__website__ = "http://mrsdprojects.ri.cmu.edu/2017teami/"
__copyright__ = "Copyright (C) 2018, the Moon Wreckers. All rights reserved."

import rospy
import tf
# from std_msgs.msg import String, Float32, UInt16
# from sensor_msgs.msg import Imu, JointState, Joy
# from geometry_msgs.msg import Vector3Stamped, Twist
# from nav_msgs.msg import Odometry

import hebi
import time
import numpy as np

VERT_MOTOR_OFFSET = .09  # cm
HORZ_MOTOR_OFFSET = .07  # cm
CLAW_ARM_LENGTH = .31  # cm


def auto_pointing():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'auto_pointing' node so that multiple nodes can run
    # simultaneously.
    rospy.init_node('auto_pointing', anonymous=True)

    hebi_modules = [('arm', 'joint_0'), ('arm', 'joint_1')]

    theta_bounds = [[-.7, 1.4], [-0.8, 0.8]]
    # motion_bound = 0.05

    # discover HEBI modules
    count_found = 0
    lookup = hebi.Lookup()
    time.sleep(2)  # Give the Lookup process 2 seconds to discover modules
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
    print('HEBI control group created (size=%d)' % (hebi_group.size))
    print(hebi_group)

    group_command = hebi.GroupCommand(hebi_group.size)

    # create TF listener
    tf_listener = tf.TransformListener()

    # initialize HEBI actuator position control parameters
    # update_alpha = 0.5
    theta = np.array([0.0, 0.0])

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # observe the current ring location
        try:
            (T_ak1_ring, R_ak1_ring) = tf_listener.lookupTransform(
                '/ak2_claw', '/ak1_ring', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        (x, y, z) = [T_ak1_ring[0], T_ak1_ring[1], T_ak1_ring[2]]

        d3 = z - VERT_MOTOR_OFFSET
        # rospy.logwarn("d3: " + str(d3))
        # rospy.logwarn("theta1: " + str(np.arctan2(d3, CLAW_ARM_LENGTH)))

        theta[1] = np.arctan2(d3, CLAW_ARM_LENGTH)
        phi = np.arctan2(HORZ_MOTOR_OFFSET, CLAW_ARM_LENGTH * np.cos(theta[1]))
        theta[0] = np.arctan2(y, x) + phi

        for i_dim in range(len(theta)):
            theta[i_dim] = max(theta_bounds[i_dim][0], min(theta_bounds[i_dim][1], theta[i_dim]))

    # send command to HEBI control group
        group_command.position = theta
        hebi_group.send_command(group_command)
        print('theta1 = %.4f, theta2 = %.4f, x_c = (%.2f, %.2f, %.2f)' % (
            180 * theta[0] / np.pi, 180 * theta[1] / np.pi, x, y, z))

        rate.sleep()

    # rospy.spin() # use spin only in pure event handling programming model


if __name__ == '__main__':
    try:
        auto_pointing()
    except rospy.ROSInterruptException:
        pass
