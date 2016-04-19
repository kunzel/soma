#!/usr/bin/env python
import rospy
import argparse
import sys
from soma_io.soma_io import FileIO
from soma_io.soma_io import Importer
from soma_io.state import World

from sensor_msgs.msg import PointCloud2
from soma_io import octree
from octomap_msgs.msg import Octomap
import struct

"""
Import all the data to mongodb, works for G4S dataset.
"""


if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog='datasets_importer.py')
    parser.add_argument("-m", metavar='mode', help="'local' for saved dataset,'online' for ros topics ")
    parser.add_argument("-p", metavar='path', help="when use 'local' mode, need specify the path to dataset")
    parser.add_argument("-t", metavar='topic', help="ros sensor_msgs.PointCloud2 type topic")

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    if args.m == 'local':
        rospy.init_node("local_data_importer", anonymous=True)

        # load local dataset info index
        if args.p is None:
            print "error: need specify path parameter"
            exit()
        ROMBUS_DB = args.p
        object_classes = FileIO.scan_objects(ROMBUS_DB)
        rooms = FileIO.scan_xml(ROMBUS_DB, "room.xml")

        # init collection and start inserting data
        world = World()
        rooms.sort(key=lambda x: x[-1])
        for i, r in enumerate(rooms):
            print "-" * 20
            print "[%d/%d]" % (i, len(rooms)), r[0], "\t\t", r[-1]
            print "\n" * 5
            FileIO.parse_room(world, r[0], r[2], object_classes)

        print "Done!"
        exit()
    elif args.m == 'online':
        if args.t is None:
            print "error: need topic name for online mode"
            exit()
        #rospy.init_node("online_data_importer", anonymous=True)
        #rospy.Subscriber(args.t, PointCloud2, FileIO.online_mode)
        #rospy.spin()
        imp = Importer(args.t)
        imp.init_subscriber('/narrow_stereo_link', 'head_pan_link', '/odom_combined')

    else:
         print("Unknown options detected, use -h for usage info")
