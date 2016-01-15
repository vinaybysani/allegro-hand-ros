import unittest
from allegro_hand.liballegro import AllegroClient

class TestAllegro(unittest.TestCase):
    def test_instantiate(self):
        client = AllegroClient()
        self.assertIsNotNone(client)
