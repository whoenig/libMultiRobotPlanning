import unittest
import subprocess
import yaml
import os

class TestECBS(unittest.TestCase):

  def runECBS(self, inputFile, w, createVideo=False):
    subprocess.run(
      ["./ecbs",
       "-i", inputFile,
       "-o", "output.yaml",
       "-w", str(w)],
       check=True)
    if createVideo:
      subprocess.run(
        ["python3", "../example/visualize.py",
         inputFile,
         "output.yaml",
         "--video", os.path.splitext(os.path.basename(inputFile))[0] + "_ecbs.mp4"],
        check=True)
    with open("output.yaml") as output_file:
      return yaml.load(output_file)

  def test_mapfSimple1(self):
    r = self.runECBS("../test/mapf_simple1.yaml", 1.0)
    self.assertTrue(r["statistics"]["cost"] == 8)

  def test_mapfCircle(self):
    r = self.runECBS("../test/mapf_circle.yaml", 1.0)
    self.assertTrue(r["statistics"]["cost"] == 4)

  def test_atGoal(self):
    r = self.runECBS("../test/mapf_atGoal.yaml", 1.0)
    self.assertTrue(r["statistics"]["cost"] == 0)


if __name__ == '__main__':
    unittest.main()
