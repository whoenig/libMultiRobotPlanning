import unittest
import subprocess
import yaml
import os

class TestECBSTA(unittest.TestCase):

  def runECBSTA(self, inputFile, w, createVideo=False):
    subprocess.run(
      ["./ecbs_ta",
       "-i", inputFile,
       "-w", str(w),
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

  def test_mapfSimple1_TA1(self):
    r = self.runECBSTA("../test/mapfta_simple1_a1.yaml", 1.0)
    self.assertTrue(r["statistics"]["cost"] == 6)

  def test_mapfSimple1_TA2(self):
    r = self.runECBSTA("../test/mapfta_simple1_a2.yaml", 1.0)
    self.assertEqual(r["statistics"]["cost"], 6)
    self.assertTrue(r["schedule"]["agent0"][-1] == {'x': 4, 'y': 0, 't': 4})
    self.assertTrue(r["schedule"]["agent1"][-1]["x"] == 2)
    self.assertTrue(r["schedule"]["agent1"][-1]["y"] == 1)

  def test_mapfSimple1_TA3(self):
    r = self.runECBSTA("../test/mapfta_simple1_a3.yaml", 1.0)
    self.assertEqual(r["statistics"]["cost"], 5)
    self.assertTrue(r["schedule"]["agent0"][-1] == {'x': 3, 'y': 0, 't': 3})


if __name__ == '__main__':
    unittest.main()
