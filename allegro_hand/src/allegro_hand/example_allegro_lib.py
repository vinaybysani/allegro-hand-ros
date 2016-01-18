#!/usr/bin/env python

import sys
import numpy as np
import rospy
from allegro_hand.liballegro import AllegroClient


def wave_fingers(allegro_client,
                 finger_indices=None,
                 num_seconds=10):
    """
    Wave one or more fingers in a sinusoidal pattern.
    :param allegro_client: The client.
    :param finger_indices: List of finger indices (between 0 and 3)
    :param num_seconds: Total time to spend doing this.
    """
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


def command_named_configurations(allegro_client, delay=2.0):
    """
    Go through a bunch of the named configurations, with a delay between them.

    :param allegro_client: The client
    :param delay: Time to sleep between configs.
    """

    # Run through these named configurations; notice there are multiple ways of
    # calling the same configuration.
    configs = ['ready', 'three finger grasp', 'three_finger_grasp',
               'index_pinch', 'gravity_compensation', 'ready']

    for config in configs:
        rospy.loginfo('Commanding configuration: {}'.format(config))
        allegro_client.command_hand_configuration(config)
        rospy.sleep(delay)
    return


def run(args):
    print args
    rospy.init_node('example_allegro_lib', args)
    client = AllegroClient()
    rospy.sleep(0.5)  # Wait for connections.

    client.command_hand_configuration('ready')

    wave_fingers(client, finger_indices=[0, 1], num_seconds=5)
    client.command_hand_configuration('ready')

    command_named_configurations(client)

    # Get the hand joint positions.
    joints = client.poll_joint_position(wait=False)
    rospy.loginfo('Hand configuration: {}'.format(joints))
    joints = client.poll_joint_position(wait=True)
    rospy.loginfo('Hand configuration: {}'.format(joints))

    return

if __name__ == '__main__':
    args = sys.argv[1:]
    run(args)
