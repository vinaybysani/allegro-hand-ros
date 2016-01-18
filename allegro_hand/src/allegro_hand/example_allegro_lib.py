#!/usr/bin/env python

import sys
import numpy as np
import rospy
from allegro_hand.liballegro import AllegroClient


def wave_fingers(allegro_client,
                 finger_indices=None,
                 num_seconds=10):

    hz = 4
    r = rospy.Rate(hz)

    # Choose which fingers to wave, by default do all of them.
    if not finger_indices:
        finger_indices = [0, 1, 2, 3]

    # The default pose is 1.0, except for the first joint of each finger (the
    # finger rotation along its axis) which is 0.0.
    position = np.ones(16)
    position[[0, 4, 8, 12]] = 0.0

    for t in range(hz * num_seconds):

        # Generate a sinusoidal signal between 0 and 1.5
        val = (np.sin(0.2 * t) + 1) * 0.75

        # Set all joints for the fingers we are controlling.
        for finger_idx in finger_indices:
            inds = range(4*finger_idx + 1, 4*finger_idx + 4)
            position[inds] = val
        # Command the joint position.
        allegro_client.command_joint_position(position)

        r.sleep()
        pass
    return


def run(args):
    print args
    rospy.init_node('example_allegro_lib', args)
    client = AllegroClient()
    rospy.sleep(0.5)  # Wait for connections.

    wave_fingers(client, finger_indices=[0, 1, 2], num_seconds=20)
    client.command_hand_configuration('ready')

    return

    pose[0:5] = [0.4] * 4
    client.command_joint_position(pose)
    rospy.sleep(1.0)

    return

    client.command_hand_configuration('home')
    return

    rospy.sleep(1.0)
    client.set_envelop_torque(0.1)
    client.command_hand_configuration('envelop')

    rospy.sleep(1.0)
    client.command_hand_configuration('ready')

    rospy.sleep(1.0)



if __name__ == '__main__':
    args = sys.argv[1:]
    run(args)
