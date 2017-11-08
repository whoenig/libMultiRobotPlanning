import unittest
import subprocess
import yaml

class TestAstar(unittest.TestCase):

  def runAstar(self, start, goal, mapFile):
    subprocess.run(
      ["./a_star",
       "--startX", str(start[0]),
       "--startY", str(start[1]),
       "--goalX", str(goal[0]),
       "--goalY", str(goal[1]),
       "-m", mapFile,
       "-o", "output.yaml"],
       check=True)
    with open("output.yaml") as output_file:
      return yaml.load(output_file)

  def test_startAndGoalIdentical(self):
    r = self.runAstar([0, 0], [0, 0], "../test/map_3x3.txt")
    self.assertTrue(len(r["schedule"]["agent1"]) == 1)

  def test_goalOnObstacle(self):
    r = self.runAstar([0, 0], [1, 1], "../test/map_3x3.txt")
    self.assertTrue(r is None)

  def test_startOnObstacle(self):
    r = self.runAstar([1, 1], [0, 0], "../test/map_3x3.txt")
    self.assertTrue(r is None)

  def test_startAndGoalOnObstacle(self):
    r = self.runAstar([1, 1], [2, 2], "../test/map_3x3.txt")
    self.assertTrue(r is None)

  def test_validSimple(self):
    r = self.runAstar([0, 0], [2, 1], "../test/map_3x3.txt")
    self.assertTrue(len(r["schedule"]["agent1"]) == 4)

if __name__ == '__main__':
    unittest.main()
