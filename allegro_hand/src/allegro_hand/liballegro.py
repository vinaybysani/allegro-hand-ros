import rospy

from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState


class AllegroClient(object):

    def __init__(self, num_joints=16):

        # Topics (that can be remapped) for named graps
        # (ready/envelop/grasp/etc.), joint commands (position and
        # velocity), envelop torque.
        topic_grasp_command = '/allegroHand_0/lib_cmd'
        topic_joint_command = '/allegroHand_0/joint_cmd'
        topic_joint_state = '/allegroHand_0/joint_states'
        topic_envelop_torque = '/allegroHand_0/envelop_torque'

        # TODO Figure out a way to remap topics automatically: rosparam?

        # Publishers for above topics.
        self.pub_grasp = rospy.Publisher(
            topic_grasp_command, String, queue_size=10)
        self.pub_joint = rospy.Publisher(
            topic_joint_command, JointState, queue_size=10)
        self.pub_envelop_torque = rospy.Publisher(
            topic_envelop_torque, Float32, queue_size=1)
        rospy.Subscriber(topic_joint_state, JointState,
                         self._joint_state_callback)
        self._joint_state = None

        self._num_joints = num_joints

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

    def _joint_state_callback(self, data):
        self._joint_state = data

    def command_joint_position(self, desired_pose):
        """
        Command a specific desired hand pose.

        The desired pose must be the correct dimensionality (self._num_joints).
        Only the pose is commanded, and **no bound-checking happens here**:
        any commanded pose must be valid or Bad Things May Happen. (Generally,
        values between 0.0 and 1.5 are fine, but use this at your own risk.)

        :param desired_pose: The desired joint configurations.
        :return: True if pose is published, False otherwise.
        """

        # Check that the desired pose can have len() applied to it, and that
        # the number of dimensions is the same as the number of hand joints.
        if (not hasattr(desired_pose, '__len__') or
                len(desired_pose) != self._num_joints):
            rospy.logwarn('Desired pose must be a {}-d array: got {}.'
                          .format(self._num_joints, desired_pose))
            return False

        msg = JointState()  # Create and publish
        try:
            msg.position = desired_pose
            self.pub_joint.publish(msg)
            rospy.loginfo('Published desired pose.')
            return True
        except rospy.exceptions.ROSSerializationException:
            rospy.logwarn('Incorrect type for desired pose: {}.'.format(
                desired_pose))
            return False

    def poll_joint_position(self, wait=False):
        """
        Get the current joint positions of the hand.

        :param wait: If true, waits for a 'fresh' state reading.
        :return: Joint positions, or None if none have been received.
        """
        if wait:
            self._joint_state = None  # Wait for the next state reading.
            while not self._joint_state:
                rospy.sleep(0.001)

        if self._joint_state:
            return self._joint_state.position
        else:
            return None

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
        """
        Command a specific envelop grasping torque.

        This only applies for the envelop named hand command. You can set the
        envelop torque before or after commanding the envelop grasp.

        :param torque: Desired torque, between 0 and 1. Values outside this
        range are clamped.
        :return: True.
        """

        torque = max(0.0, min(1.0, torque))  # Clamp within [0, 1]
        msg = Float32(torque)
        self.pub_envelop_torque.publish(msg)
        return True
