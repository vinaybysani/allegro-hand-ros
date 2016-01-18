import rospy

from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState


class AllegroClient(object):

    def __init__(self):

        # Topics (that can be remapped) for named graps
        # (ready/envelop/grasp/etc.), joint commands (position and
        # velocity), envelop torque.
        topic_grasp_command = '/allegroHand_0/lib_cmd'
        topic_joint_command = '/allegroHand_0/joint_cmd'
        evelop_torque_topic = '/allegroHand_0/envelop_torque'

        # TODO Figure out a way to remap topics automatically: rosparam?

        # Publishers for above topics.
        self.pub_grasp = rospy.Publisher(
                topic_grasp_command, String, queue_size=10)
        self.pub_joint = rospy.Publisher(
                topic_joint_command, JointState, queue_size=10)
        self.pub_envelop_torque = rospy.Publisher(
                evelop_torque_topic, Float32, queue_size=1)

        rospy.loginfo('Publishers started. {}'.format(topic_grasp_command))

        # "Named" grasps are those provided by the bhand library. These can be
        # commanded directly and the hand will execute them. The keys are more
        # human-friendly names, the values are the expected names from the
        # allegro controller side. Multiple strings mapping to the same value
        # are allowed.
        self._named_grasps_mappings = {
            'home': 'home',
            'ready': 'ready',
            'three_finger_grasp': 'grasp_3',
            'three finger grasp': 'grasp_3',
            'four_finger_grasp': 'grasp_4',
            'four finger grasp': 'grasp_4',
            'index_pinch': 'pinch_it',
            'index pinch': 'pinch_it',
            'middle_pinch': 'pinch_mt',
            'middle pinch': 'pinch_mt',
            'envelop': 'envelop',
            'off': 'off',
            'gravity_compensation': 'gravcomp',
            'gravity compensation': 'gravcomp',
            'gravity': 'gravcomp'
            }

    def command_joint_pose(self, pose):
        # TODO check number of dimensions of pose data.

        msg = JointState()
        msg.position = pose
        self.pub_joint.publish(msg)
        rospy.loginfo('Published desired pose.')

    def command_hand_configuration(self, hand_config):
        """
        Command a named hand configuration (e.g., pinch_index, envelop,
        gravity_compensation).

        The internal hand configuration names are defined in the
        AllegroNodeGrasp controller file. More human-friendly names are used
        by defining them as 'shortcuts' in the _named_grasps_mapping variable.
        Multiple strings can map to the same commanded configuration.

        :param hand_config: A human-friendly string of the desired
        configuration.
        :return: True if the grasp was known and commanded, false otherwise.
        """

        # Only use known named grasps.
        if hand_config in self._named_grasps_mappings:
            # Look up conversion of string -> msg
            msg = String(self._named_grasps_mappings[hand_config])
            rospy.loginfo('Commanding grasp: {}'.format(msg))
            self.pub_grasp.publish(msg)
            return True
        else:
            rospy.logwarn('Unable to command unknown grasp {}'.format(
                    hand_config))
            return False

    def list_hand_configurations(self):
        return self._named_grasps_mappings.keys()

    def set_envelop_torque(self, torque):

        rospy.logwarn('Not implemented!')

        # TODO check boundaries.
        pass

