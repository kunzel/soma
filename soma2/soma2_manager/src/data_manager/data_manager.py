#!/usr/bin/env python

import roslib; roslib.load_manifest("soma2_manager")
import rospy
from rospkg import RosPack
import json
import argparse
import random
import copy
import sys

from threading import Timer
from mongodb_store.message_store import MessageStoreProxy
from soma2_msgs.msg import SOMA2ROIObject

from soma2_manager.srv import *


from soma2_map_manager.srv import *

class SOMA2DataManager():

    def __init__(self, db_name, collection_name="soma2"):

       # self.soma_map_name = soma_map_name
        self._db_name = db_name
        self._collection_name = collection_name

        self._message_store = MessageStoreProxy(database=db_name, collection=collection_name)

        s = rospy.Service('soma2/insert_objects', SOMA2InsertObjs, self.handle_insert_request)

        rospy.spin()

    def handle_insert_request(self,req):

        for obj in req.objects:
            try:
                _id = self._message_store.insert(obj)
            except:
                return SOMA2InsertObjsResponse(False)

        return SOMA2InsertObjsResponse(True)





if __name__=="__main__":

    parser = argparse.ArgumentParser(prog='data_manager.py')
#parser.add_argument("map", nargs=1, help='Path of the used 2D map')
#parser.add_argument("map_name",nargs=1, help='Name of the used 2D map')
    parser.add_argument("db_name", nargs=1, help='Name of the database')
    parser.add_argument('collection_name', nargs=1, help='Name of the collection')

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("soma2_data_manager")
    rospy.loginfo("Running SOMA2 data manager (dbname: %s, collection_name: %s)", args.db_name[0], args.collection_name[0])
    SOMA2DataManager(args.db_name[0],args.collection_name[0])
