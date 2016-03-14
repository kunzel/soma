#!/usr/bin/env python

import roslib; roslib.load_manifest("soma2_manager")
import rospy
from rospkg import RosPack
import json
import argparse
import random
import copy
import sys
import datetime
import time
import math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from threading import Timer
from mongodb_store.message_store import MessageStoreProxy
from soma2_msgs.msg import SOMA2ROIObject

from soma2_manager.srv import *


from soma2_map_manager.srv import *

class SOMA2DataManager():

    def __init__(self, db_name="soma2data", collection_name="soma2"):

       # self.soma_map_name = soma_map_name
        self._db_name = db_name
        self._collection_name = collection_name

        self._message_store = MessageStoreProxy(database=db_name, collection=collection_name)

        s = rospy.Service('soma2/insert_objects', SOMA2InsertObjs, self.handle_insert_request)

        rospy.spin()

    def handle_insert_request(self,req):

        for obj in req.objects:

          if(obj.logtimestamp == 0):
            obj.logtimestamp = rospy.Time.now().secs

          d = datetime.datetime.utcfromtimestamp(obj.logtimestamp)
          obj.loghour = d.hour
          obj.logminute = d.minute
          obj.logday = d.isoweekday()
          obj.logtimeminutes = obj.loghour*60 + obj.logminute

          if (obj.header.frame_id == ""):
              obj.header.frame_id = "/map"

          res = self.coords_to_lnglat(obj.pose.position.x,obj.pose.position.y)

          geopose = Pose()

          geopose.position.x = res[0]
          geopose.position.y = res[1]

          obj.geoposearray.poses.append(geopose)

          try:
                _id = self._message_store.insert(obj)
          except:
                return SOMA2InsertObjsResponse(False)

        return SOMA2InsertObjsResponse(True)

    def coords_to_lnglat(self, x, y):
        earth_radius = 6371000.0 # in meters
        lng = 90 - math.degrees(math.acos(float(x) / earth_radius))
        lat = 90 - math.degrees(math.acos(float(y) / earth_radius))
        return [lng , lat]




if __name__=="__main__":

    parser = argparse.ArgumentParser(prog='data_manager.py')
#parser.add_argument("map", nargs=1, help='Path of the used 2D map')
#parser.add_argument("map_name",nargs=1, help='Name of the used 2D map')
    parser.add_argument("db_name", nargs='?', help='Name of the database')
    parser.add_argument('collection_name', nargs='?', help='Name of the collection')

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("soma2_data_manager")
    if args.db_name is not None:
        rospy.loginfo("Running SOMA2 data manager (dbname: %s, collection_name: %s)", args.db_name[0], args.collection_name[0])
        SOMA2DataManager(args.db_name[0],args.collection_name[0])
    else:
        rospy.loginfo("Running SOMA2 data manager (dbname: soma2data, collection_name: soma2)")
        SOMA2DataManager()
