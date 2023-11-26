import unittest
from row_nav import get_rate, read_pc_from_file, reduce_pc


class TestCases(unittest.TestCase):
    def __init__(self, methodName: str = "runTest") -> None:
        super().__init__(methodName)
        self.pc = read_pc_from_file("1.npz")
        self.pc2 = read_pc_from_file("4.npz")
        self.reduced = reduce_pc(self.pc)
        self.reduced2 = reduce_pc(self.pc2)

    def testReadFile(self):
        """
        Test case to determine that pointclouds are read in correctly and have
        expected dimensions.
        """

        self.assertGreater(self.pc.shape[0], 0, "PointCloud 1 is empty")
        self.assertGreater(self.pc2.shape[0], 0, "PointCloud 2 is empty")
        self.assertEqual(
            self.pc.shape[1],
            3,
            f"PointCloud 1 has incorrect dimension, expected 3, got {self.pc.shape[1]}",
        )
        self.assertEqual(
            self.pc2.shape[1],
            3,
            f"PointCloud 2 has incorrect dimension, expected 3, got {self.pc2.shape[1]}",
        )

    def testReducePC(self):
        """
        Test case to confirm that pointclouds are reduced in size, and not the same size
        or larger than the raw pointclouds.
        """

        self.assertLess(
            self.reduced.shape[0],
            self.pc.shape[0],
            "Reduced PointCloud 1 is not smaller than PointCloud 1",
        )

        self.assertLess(
            self.reduced2.shape[0],
            self.pc2.shape[0],
            "Reduced PointCloud 2 is not smaller than PointCloud 2",
        )

    def testGetRate(self):
        """
        Test case to determine that the angular rate is correctly determined based on
        the provided pointclouds.
        """

        rate = get_rate(self.reduced)
        rate2 = get_rate(self.reduced2)

        self.assertEqual(rate, 0, f"Rate 1 should be 0, but is {rate} instead")
        self.assertLess(
            rate2, 0, f"Rate 2 should be less than 0, but is {rate2} instead"
        )


if __name__ == "__main__":
    unittest.main()
