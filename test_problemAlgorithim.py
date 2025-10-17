import unittest
from problemAlgorithim import get_flow_data


class TestProblemAlgorithm(unittest.TestCase):
    def test_zero_flights(self):
        a, b, r = [], [], []
        data = get_flow_data(a, b, r)
        self.assertEqual(data["min_planes"], 0)
        self.assertEqual(data["max_flow"], 0)
        self.assertEqual(data["planes"], [])

    def test_no_transitions(self):
        a = [0, 10]
        b = [5, 15]
        r = [[0, 100], [100, 0]]
        data = get_flow_data(a, b, r)
        self.assertEqual(data["max_flow"], 0)
        self.assertEqual(data["min_planes"], 2)

    def test_full_chain(self):
        a = [0, 10, 20]
        b = [5, 15, 25]
        r = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        data = get_flow_data(a, b, r)
        self.assertEqual(data["max_flow"], 2)
        self.assertEqual(data["min_planes"], 1)
        self.assertEqual(len(data["planes"]), 1)


if __name__ == "__main__":
    unittest.main()
