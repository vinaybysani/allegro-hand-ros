import sys
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import JointState


class AllegroClient(object):

    def __init__(self):

        # Topics (that can be remapped) for named graps
        # (ready/envelop/grasp/etc.) and joint commands (position and
        # velocity).
        grasp_cmd_topic = '/allegroHand/lib_cmd'
        joint_cmd_topic = '/allegroHand/joint_cmd'

        # Publishers for above topics.
        self.pub_grasp = rospy.Publisher(grasp_topic, String, queue_size=10)
        self.pub_joint = rospy.Publisher(joint_topic, JointState, queue_size=10)

        self._named_grasps_mappings = {
            'home': 'home',
            'ready': 'ready',
            'three_finger_grasp': 'grasp_3',
            'four_finger_grasp': 'grasp_4',
            'index_pinch': 'pinch_it',
            'middle_pinch': 'pinch_mt',
            'envelop': 'envelop',
            'off': 'off',
            'gravity_compensation': 'gravcomp'
            }

    def command_joint_pose(self, pose):
        # TODO check number of dimensions of pose data.

        msg = JointState()
        msg.position = pose
        self.pub_joint.publish(msg)
        rospy.loginfo('Published desired pose.')

    def command_hand_configuration(self, config):
        msg = None

        # Automatic conversion of string -> msg
        if grasp in self._named_grasps_mappings
            msg = String(self._named_grasps_mappings[grasp])
            rospy.loginfo('Commanding grasp: {}'.format(msg))
            self.pub_grasp.publish(msg)
            return True
        else:
            rospy.logwarn('Unable to parse desired grasp {}'.format(config))
            return False

    def set_envelop_torque(self, torque):

        # TODO check boundaries.
