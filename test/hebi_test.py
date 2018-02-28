#!/usr/bin/env python

"""
hebi_test.py
Test the basic control for HEBI actuator.
"""

__version__     = "0.0.1"
__author__      = "David Qiu"
__email__       = "dq@cs.cmu.edu"
__website__     = "http://mrsdprojects.ri.cmu.edu/2017teami/"
__copyright__   = "Copyright (C) 2018, the Moon Wreckers. All rights reserved."


import hebi
from time import sleep
import numpy as np

import IPython


def main():
    # Discover modules
    lookup = hebi.Lookup()
    sleep(2) # Give the Lookup process 2 seconds to discover modules
    print('Modules found on network:')
    for entry in lookup.entrylist:
      print('{0} | {1}'.format(entry.family, entry.name))

    # Create a group from a set of names
    group = lookup.get_group_from_names(['Robot B'], ['wrist 1'])

    sleep(3)

    # Control
    pmax = 2.90 # rad
    pmin = -2.90 # rad
    nsteps = 200 # rad
    pstep = (pmax - pmin) / nsteps
    deltaT = 0.01 # sec
    group_command = hebi.GroupCommand(group.size)
    p = np.array([pmin + i * pstep for i in range(nsteps)])
    p_inv = p[[nsteps-1-i for i in range(nsteps)]]

    while True:
        for i in range(nsteps):
            group_command.position = [p[i]]
            group.send_command(group_command)
            print('ctrl_pos = %.4f' % (p[i]));
            sleep(deltaT)
        for i in range(nsteps):
            group_command.position = [p_inv[i]]
            group.send_command(group_command)
            print('ctrl_pos = %.4f' % (p_inv[i]));
            sleep(deltaT)


if __name__ == '__main__':
    main()
