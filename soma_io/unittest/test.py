import unittest
from soma_io.soma_io import *


class TestSomaIO(unittest.TestCase):

    def test_remove_nan(self):
        pc_in = pcl.load("example.pcd")
        pc_out = pcl.PointCloud()
        FileIO.remove_nan(pc_in, pc_out)

    def test_pcd_to_octree_using_remove_nan(self):
        pc_in = pcl.load("example.pcd")
        pc_out = pcl.PointCloud()
        FileIO.remove_nan(pc_in, pc_out)
        octree = octomap.OcTree(0.01)
        FileIO.pcd_to_octree(pc_out, octree)

    def test_pcd_to_octree_without_remove_nan(self):
        pc_in = pcl.load("example.pcd")
        octree = octomap.OcTree(0.01)
        FileIO.pcd_to_octree(pc_in, octree)


if __name__ == "__main__":
    unittest.main()
