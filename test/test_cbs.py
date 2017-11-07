import unittest
import subprocess
import yaml
import os

class TestCBS(unittest.TestCase):

  def runCBS(self, inputFile, createVideo=False):
    subprocess.run(
      ["./cbs",
       "-i", inputFile,
       "-o", "output.yaml"],
       check=True)
    if createVideo:
      subprocess.run(
        ["python3", "../example/visualize.py",
         inputFile,
         "output.yaml",
         "--video", os.path.splitext(os.path.basename(inputFile))[0] + "_cbs.mp4"],
        check=True)
    with open("output.yaml") as output_file:
      return yaml.load(output_file)

  def test_mapfSimple1(self):
    r = self.runCBS("../test/mapf_simple1.yaml")
    self.assertTrue(r["statistics"]["cost"] == 8)

  def test_mapfCircle(self):
    r = self.runCBS("../test/mapf_circle.yaml")
    self.assertTrue(r["statistics"]["cost"] == 4)

  def test_atGoal(self):
    r = self.runCBS("../test/mapf_atGoal.yaml")
    self.assertTrue(r["statistics"]["cost"] == 0)


if __name__ == '__main__':
    unittest.main()
