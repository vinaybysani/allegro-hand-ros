import rospy

from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState


class AllegroClient(object):

    def __init__(self):

        # Topics (that can be remapped) for named graps
        # (ready/envelop/grasp/etc.), joint commands (position and
        # velocity), envelop torque.
        topic_grasp_command = '/allegroHand/lib_cmd'
        topic_joint_command = '/allegroHand/joint_cmd'
        evelop_torque_topic = '/allegroHand/joint_cmd'

        # Publishers for above topics.
        self.pub_grasp = rospy.Publisher(
                topic_grasp_command, String, queue_size=10)
        self.pub_joint = rospy.Publisher(
                topic_joint_command, JointState, queue_size=10)
        self.pub_envelop_torque = rospy.Publisher(
                evelop_torque_topic, Float32, queue_size=1)


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

    def command_hand_configuration(self, hand_config):
        # Only use known named grasps.
        if hand_config in self._named_grasps_mappings:
            # Look up conversion of string -> msg
            msg = String(self._named_grasps_mappings[hand_config])
            rospy.loginfo('Commanding grasp: {}'.format(msg))
            self.pub_grasp.publish(msg)
            return True
        else:
            rospy.logwarn('Unable to parse desired grasp {}'.format(hand_config))
            return False

    def set_envelop_torque(self, torque):

        # TODO check boundaries.
        pass

