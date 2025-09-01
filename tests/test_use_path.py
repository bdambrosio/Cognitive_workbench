import unittest
import random
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Import Agent from map.py
from src.map import Agent


class MockPatch:
    def __init__(self, x, y, has_path=False):
        self.x = x
        self.y = y
        self.has_path = has_path


class MockWorld:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.patches = [[MockPatch(x, y, False) for y in range(height)] for x in range(width)]

    def register_agent(self, agent):
        # No-op for unit tests
        pass

    def unregister_agent(self, agent):
        # No-op for unit tests
        pass


class UsePathTests(unittest.TestCase):
    def setUp(self):
        random.seed(1337)

    def test_near_not_on_moves_to_across_when_available(self):
        # Grid 5x5; path at (2,2) and across at (3,3)
        world = MockWorld(5, 5)
        world.patches[2][2].has_path = True
        world.patches[3][3].has_path = True
        agent = Agent(1, 1, world, 'Tester')

        ok = agent.use_path('Path22')
        self.assertTrue(ok)
        self.assertEqual((agent.x, agent.y), (3, 3))
        self.assertTrue(world.patches[agent.x][agent.y].has_path)

    def test_no_across_steps_onto_path(self):
        # Only the path patch exists; no across candidates
        world = MockWorld(5, 5)
        world.patches[2][2].has_path = True
        agent = Agent(1, 1, world, 'Tester')

        ok = agent.use_path('Path22')
        self.assertTrue(ok)
        self.assertEqual((agent.x, agent.y), (2, 2))

    def test_on_path_continues_in_last_direction(self):
        world = MockWorld(5, 5)
        # Create a short straight path to the east
        world.patches[2][2].has_path = True
        world.patches[3][2].has_path = True
        agent = Agent(2, 2, world, 'Tester')
        # Seed a previous traversal direction to the east
        agent._last_path_dir = (1, 0)

        ok = agent.use_path('Path22')
        self.assertTrue(ok)
        self.assertEqual((agent.x, agent.y), (3, 2))
        self.assertTrue(world.patches[agent.x][agent.y].has_path)

    def test_bounds_no_across_steps_onto_path(self):
        world = MockWorld(3, 3)
        # Path at border (0,1); agent at (-) across would be OOB
        world.patches[0][1].has_path = True
        agent = Agent(1, 1, world, 'Tester')

        ok = agent.use_path('Path01')
        self.assertTrue(ok)
        self.assertEqual((agent.x, agent.y), (0, 1))


if __name__ == '__main__':
    unittest.main()


