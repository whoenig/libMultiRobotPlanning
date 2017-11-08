import unittest
import subprocess
import yaml

class TestAstarEpsilon(unittest.TestCase):

  def runAstarEpsilon(self, start, goal, mapFile, w):
    subprocess.run(
      ["./a_star_epsilon",
       "--startX", str(start[0]),
       "--startY", str(start[1]),
       "--goalX", str(goal[0]),
       "--goalY", str(goal[1]),
       "-m", mapFile,
       "-w", str(w),
       "-o", "output.yaml"],
       check=True)
    with open("output.yaml") as output_file:
      return yaml.load(output_file)

  def test_startAndGoalIdentical(self):
    r = self.runAstarEpsilon([0, 0], [0, 0], "../test/map_3x3.txt", 1.0)
    self.assertTrue(len(r["schedule"]["agent1"]) == 1)

  def test_goalOnObstacle(self):
    r = self.runAstarEpsilon([0, 0], [1, 1], "../test/map_3x3.txt", 1.0)
    self.assertTrue(r is None)

  def test_startOnObstacle(self):
    r = self.runAstarEpsilon([1, 1], [0, 0], "../test/map_3x3.txt", 1.0)
    self.assertTrue(r is None)

  def test_startAndGoalOnObstacle(self):
    r = self.runAstarEpsilon([1, 1], [2, 2], "../test/map_3x3.txt", 1.0)
    self.assertTrue(r is None)

  def test_validSimple(self):
    r = self.runAstarEpsilon([0, 0], [2, 1], "../test/map_3x3.txt", 1.0)
    self.assertTrue(len(r["schedule"]["agent1"]) == 4)

if __name__ == '__main__':
    unittest.main()
