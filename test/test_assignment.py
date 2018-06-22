import unittest
import subprocess
import yaml

class TestAssignment(unittest.TestCase):

  def runAssignment(self, mapping):
    with open("input.txt", "w") as mapping_file:
      for pair, value in mapping.items():
        mapping_file.write(pair[0] + "->" + pair[1] + ":" + str(value) + "\n")
    subprocess.run(
      ["./assignment",
       "-i", "input.txt",
       "-o", "output.yaml"],
       check=True)
    with open("output.yaml") as output_file:
      return yaml.load(output_file)

  def test_empty(self):
    mapping = dict()
    r = self.runAssignment(mapping)
    self.assertTrue(r["cost"] == 0)

  def test_1by2(self):
    mapping = dict()
    mapping[("a0", "t0")] = 2
    mapping[("a0", "t1")] = 1
    r = self.runAssignment(mapping)
    self.assertTrue(r["cost"] == 1)
    self.assertTrue(r["assignment"]["a0"] == "t1")

  def test_2by1(self):
    mapping = dict()
    mapping[("a0", "t0")] = 2
    mapping[("a1", "t0")] = 1
    r = self.runAssignment(mapping)
    self.assertTrue(r["cost"] == 1)
    self.assertTrue(r["assignment"]["a1"] == "t0")

  def test_4by4(self):
    mapping = dict()
    mapping[("a0", "t0")] = 90
    mapping[("a0", "t1")] = 76
    mapping[("a0", "t2")] = 75
    mapping[("a0", "t3")] = 80
    mapping[("a1", "t0")] = 35
    mapping[("a1", "t1")] = 85
    mapping[("a1", "t2")] = 55
    mapping[("a1", "t3")] = 65
    mapping[("a2", "t0")] = 125
    mapping[("a2", "t1")] = 95
    mapping[("a2", "t2")] = 90
    mapping[("a2", "t3")] = 105
    mapping[("a3", "t0")] = 45
    mapping[("a3", "t1")] = 110
    mapping[("a3", "t2")] = 95
    mapping[("a3", "t3")] = 115
    r = self.runAssignment(mapping)
    self.assertTrue(r["cost"] == 275)
    self.assertTrue(r["assignment"]["a0"] == "t3")
    self.assertTrue(r["assignment"]["a1"] == "t2")
    self.assertTrue(r["assignment"]["a2"] == "t1")
    self.assertTrue(r["assignment"]["a3"] == "t0")

if __name__ == '__main__':
    unittest.main()
