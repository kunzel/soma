#!/usr/bin/env python
import sys
from soma_io.soma_io import FileIO
import octomap


if __name__ == '__main__':
    if len(sys.argv) is 2:
        print "search pcd file under given path: ", str(sys.argv[1])
        ROMBUS_DB = str(sys.argv[1])
    else:
        ROMBUS_DB = "/Volumes/60G/strands_data_backup/20150505/patrol_run_10/"
        print "Path not given, use default path instead: "
        print "======>", ROMBUS_DB

    pcd_list = FileIO.scan_file(ROMBUS_DB, ".pcd")
    print "Found ", len(pcd_list), "target files"
    pcd_list.sort()
    for pcd in pcd_list:
        pc2 = FileIO.load_pcd(pcd)
        # convert pcd to octree
        print "Start converting " + pcd

        octree = octomap.OcTree(0.01)
        FileIO.pcd_to_octree(pc2, octree)
        # save octree to bt format
        for i in range(len(pcd)):
            x = len(pcd) - (i + 1)
            if pcd[x] == '.':
                bt_name = pcd[:x] + ".bt"
                FileIO.save_oct(bt_name, octree)
                break
