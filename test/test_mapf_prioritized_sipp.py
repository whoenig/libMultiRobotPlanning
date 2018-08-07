import unittest
import subprocess
import yaml
import os

class TestMAPFPrioritizedSIPP(unittest.TestCase):

  def exec(self, inputFile, createVideo=False):
    subprocess.run(
      ["./mapf_prioritized_sipp",
       "-i", inputFile,
       "-o", "output.yaml"],
       check=True)
    if createVideo:
      subprocess.run(
        ["python3", "../example/visualize.py",
         inputFile,
         "output.yaml",
         "--video", os.path.splitext(os.path.basename(inputFile))[0] + "_mapf_prioritized_sipp.mp4"],
        check=True)
    with open("output.yaml") as output_file:
      return yaml.load(output_file)

  def test_mapfSimple1(self):
    r = self.exec("../test/mapf_simple1.yaml")
    self.assertEqual(r["statistics"]["cost"], 8)

  def test_mapfSimple1b(self):
    r = self.exec("../test/mapf_simple1b.yaml")
    self.assertEqual(r["statistics"]["cost"], 2)
    self.assertEqual(len(r["schedule"]["agent1"]), 0)
    self.assertEqual(len(r["schedule"]["agent0"]), 3)

  def test_mapfCircle(self):
    r = self.exec("../test/mapf_circle.yaml")
    self.assertEqual(r["statistics"]["cost"], 4)

  def test_atGoal(self):
    r = self.exec("../test/mapf_atGoal.yaml")
    self.assertEqual(r["statistics"]["cost"], 0)

  def test_swap2(self):
    r = self.exec("../test/mapf_swap2.yaml")
    self.assertEqual(r["statistics"]["cost"], 12)

  def test_swap4(self):
    r = self.exec("../test/mapf_swap4.yaml")
    self.assertEqual(r["statistics"]["cost"], 28)

  def test_someAtGoal(self):
    r = self.exec("../test/mapf_someAtGoal.yaml")
    self.assertEqual(r["statistics"]["cost"], 0)

if __name__ == '__main__':
    unittest.main()
