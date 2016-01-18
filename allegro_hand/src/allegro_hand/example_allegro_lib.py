#!/usr/bin/env python

import rospy
from allegro_hand.liballegro import AllegroClient


def run():
    rospy.init_node('example_allegro_lib')
    client = AllegroClient()
    rospy.sleep(0.5)  # Wait for connections.

    client.command_hand_configuration('home')
    rospy.sleep(1.0)
    client.set_envelop_torque(0.1)
    client.command_hand_configuration('envelop')

    rospy.sleep(1.0)
    client.command_hand_configuration('ready')

if __name__ == '__main__':
    run()
