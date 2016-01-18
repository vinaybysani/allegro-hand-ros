import unittest
from allegro_hand.liballegro import AllegroClient


class MockPublisher(object):
    def __init__(self):
        self._pub_count = 0

    def publish(self, args):
        self._pub_count += 1
        self._last_published = args
    pass


class TestAllegro(unittest.TestCase):

    def setUp(self):
        self.client = AllegroClient()
        self.client.pub_grasp = MockPublisher()
        self.client.pub_joint = MockPublisher()
        self.client.pub_envelop_torque = MockPublisher()

    def test_instantiate(self):
        self.assertIsNotNone(self.client)
        self.assertEqual(0, self.client.pub_grasp._pub_count)
        self.assertEqual(0, self.client.pub_joint._pub_count)
        self.assertEqual(0, self.client.pub_envelop_torque._pub_count)

    def test_send_hand_configuration(self):
        ret = self.client.command_hand_configuration('envelop')
        self.assertEqual(True, ret)
        self.assertEqual(1, self.client.pub_grasp._pub_count)

    def test_send_invalid_hand_config(self):
        ret = self.client.command_hand_configuration('garbage')
        self.assertEqual(False, ret)
        self.assertEqual(0, self.client.pub_grasp._pub_count)

    def test_list_hand_configs(self):
        ret = self.client.list_hand_configurations()
        self.assertTrue(ret)  # Have something.
        self.assertIn('three_finger_grasp', ret)  # Have the human-readable one.

    def test_command_envelop_torque(self):
        ret = self.client.set_envelop_torque(0.5)
        self.assertTrue(ret)
        self.assertEqual(1, self.client.pub_envelop_torque._pub_count)
        published_value = self.client.pub_envelop_torque._last_published.data
        self.assertEqual(0.5, published_value)

    def test_command_large_envelop_torque(self):
        ret = self.client.set_envelop_torque(1.5)
        self.assertTrue(ret)
        self.assertEqual(1, self.client.pub_envelop_torque._pub_count)
        published_value = self.client.pub_envelop_torque._last_published.data
        self.assertEqual(1.0, published_value)

    def test_command_small_envelop_torque(self):
        ret = self.client.set_envelop_torque(-0.5)
        self.assertTrue(ret)
        self.assertEqual(1, self.client.pub_envelop_torque._pub_count)
        published_value = self.client.pub_envelop_torque._last_published.data
        self.assertEqual(0.0, published_value)
