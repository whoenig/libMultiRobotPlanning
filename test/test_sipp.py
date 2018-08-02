import unittest
import subprocess
import yaml

class TestSIPP(unittest.TestCase):

  def runSIPP(self, inputFile):
    subprocess.run(
      ["./sipp",
       "-i", inputFile,
       "-o", "output.yaml"],
       check=True)
    with open("output.yaml") as output_file:
      return yaml.load(output_file)

  def test_1(self):
    r = self.runSIPP("../test/sipp_1.yaml")
    self.assertEqual(len(r["schedule"]["agent1"]), 6)
    self.assertEqual(r["schedule"]["agent1"][-1]["x"], 2)
    self.assertEqual(r["schedule"]["agent1"][-1]["y"], 3)
    self.assertEqual(r["schedule"]["agent1"][-1]["t"], 9)



if __name__ == '__main__':
    unittest.main()
