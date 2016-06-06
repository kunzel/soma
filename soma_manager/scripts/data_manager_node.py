#!/usr/bin/env python

import rospy
import argparse
import sys

from soma_manager.data_manager import SOMADataManager

if __name__=="__main__":

    parser = argparse.ArgumentParser(prog='data_manager_node.py')
    #parser.add_argument("map", nargs=1, help='Path of the used 2D map')
    #parser.add_argument("map_name",nargs=1, help='Name of the used 2D map')
    parser.add_argument("db_name", nargs='?', help='Name of the database')
    parser.add_argument('collection_name', nargs='?', help='Name of the collection')

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("soma_data_manager")
    if args.db_name is not None:
        if args.collection_name is not None:
           rospy.loginfo("Running SOMA data manager (dbname: %s, collection_name: %s)", args.db_name, args.collection_name)
           SOMADataManager(args.db_name,args.collection_name)
        else:
            rospy.loginfo("Running SOMA data manager (dbname: %s, collection_name: soma2)", args.db_name)
            SOMADataManager(args.db_name)
    else:
        rospy.loginfo("Running SOMA data manager (dbname: soma2data, collection_name: soma2)")
        SOMADataManager()
