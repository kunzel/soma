#!/usr/bin/env python

import rospy
import argparse

from soma_roi_manager.soma_roi import SOMAROIManager

if __name__=="__main__":

    # TODO: add list command

    parser = argparse.ArgumentParser(prog='soma_roi.py')
    #parser.add_argument("map", nargs=1, help='Path of the used 2D map')
    #parser.add_argument("map_name",nargs=1, help='Name of the used 2D map')
    parser.add_argument("conf", nargs=1, help='Name of the object configuration')
    parser.add_argument('-t', metavar='config-file')

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("soma_roi")
    rospy.loginfo("Running SOMA ROI (conf: %s, types: %s)", args.conf[0], args.t)
    SOMAROIManager(args.conf[0],args.t)
