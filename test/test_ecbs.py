import unittest
import subprocess
import yaml
import os

class TestECBS(unittest.TestCase):

  def runECBS(self, inputFile, w, createVideo=False, timeout=None, additionalArgs=[]):
    subprocess.run(
      ["./ecbs",
       "-i", inputFile,
       "-o", "output.yaml",
       "-w", str(w),
       ] + additionalArgs,
       check=True,
       timeout=timeout)
    if createVideo:
      subprocess.run(
        ["python3", "../example/visualize.py",
         inputFile,
         "output.yaml",
         "--video", os.path.splitext(os.path.basename(inputFile))[0] + "_ecbs.mp4"],
        check=True)
    with open("output.yaml") as output_file:
      return yaml.safe_load(output_file)

  def test_mapfSimple1(self):
    r = self.runECBS("../test/mapf_simple1.yaml", 1.0)
    self.assertTrue(r["statistics"]["cost"] == 8)

  def test_mapfCircle(self):
    r = self.runECBS("../test/mapf_circle.yaml", 1.0)
    self.assertTrue(r["statistics"]["cost"] == 4)

  def test_atGoal(self):
    r = self.runECBS("../test/mapf_atGoal.yaml", 1.0)
    self.assertTrue(r["statistics"]["cost"] == 0)

  def test_someAtGoal(self):
    # This case is impossible, since two agents have the same goal location
    # Our implementation doesn't check that; hence a timeout is expected.
    self.assertRaises(subprocess.TimeoutExpired, self.runECBS, "../test/mapf_someAtGoal.yaml", 1.0, timeout=0.5)

  def test_someAtGoal_disappearingAgents(self):
    r = self.runECBS("../test/mapf_someAtGoal.yaml", 1.0, additionalArgs=["--disappear-at-goal"])
    self.assertTrue(r["statistics"]["cost"] == 1)

  def test_issue28_disappearingAgents(self):
    r = self.runECBS("../test/issue28.yaml", 1.0, additionalArgs=["--disappear-at-goal"])
    self.assertTrue(r["statistics"]["cost"] == 8)

  def test_issue30(self):
    r = self.runECBS("../test/issue30.yaml", 1.5)
    # no need to check anything other than successful process termination

if __name__ == '__main__':
    unittest.main()
