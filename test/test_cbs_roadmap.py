import unittest
import subprocess
import yaml
import os

class TestCBS(unittest.TestCase):

  def runCBS(self, inputFile, createVideo=False, timeout=None, additionalArgs=[]):
    subprocess.run(
      ["./cbs_roadmap",
       "-i", inputFile,
       "-o", "output.yaml"] + additionalArgs,
       check=True,
       timeout=timeout)
    if createVideo:
      subprocess.run(
        ["python3", "../example/visualize.py",
         inputFile,
         "output.yaml",
         "--video", os.path.splitext(os.path.basename(inputFile))[0] + "_cbs.mp4"],
        check=True)
    with open("output.yaml") as output_file:
      return yaml.safe_load(output_file)

  def test_mapfSimple1(self):
    r = self.runCBS("../test/mapf_simple1_roadmap.yaml")
    self.assertTrue(r["statistics"]["cost"] == 8)

  def test_mapfCircle(self):
    r = self.runCBS("../test/mapf_circle_roadmap.yaml")
    self.assertTrue(r["statistics"]["cost"] == 4)

  def test_atGoal(self):
    r = self.runCBS("../test/mapf_atGoal_roadmap.yaml")
    self.assertTrue(r["statistics"]["cost"] == 0)

  def test_someAtGoal(self):
    # This case is impossible, since two agents have the same goal location
    # Our implementation doesn't check that; hence a timeout is expected.
    self.assertRaises(subprocess.TimeoutExpired, self.runCBS, "../test/mapf_someAtGoal_roadmap.yaml", timeout=0.5)

  def test_someAtGoal_disappearingAgents(self):
    r = self.runCBS("../test/mapf_someAtGoal_roadmap.yaml", additionalArgs=["--disappear-at-goal"])
    self.assertTrue(r["statistics"]["cost"] == 1)

if __name__ == '__main__':
    unittest.main()
