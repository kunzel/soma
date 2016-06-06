#!/usr/bin/env python


import rospy
import argparse

from soma_map_manager.soma_map import SOMAMapManager


if __name__=="__main__":

    # TODO: add list command

    parser = argparse.ArgumentParser(description='Check and publish the given map name')

    parser.add_argument('--mapname', default=None, help='map name')

    args = parser.parse_args((rospy.myargv()[1:]))

    #print args.mapname

    rospy.init_node("soma_map_manager")
    rospy.loginfo("Running SOMA map manager")

    SOMAMapManager(args.mapname)
