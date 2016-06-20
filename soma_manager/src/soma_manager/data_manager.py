#!/usr/bin/env python

import roslib; roslib.load_manifest("soma_manager")
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
from soma2_msgs.msg import SOMA2ROIObject, SOMA2Object
from soma_manager.srv import *
from soma_map_manager.srv import *
from std_msgs.msg import String


# Soma2 Data Manager For storing and deleting data
class SOMADataManager():

    def __init__(self, db_name="soma2data", collection_name="soma2"):

       # self.soma_map_name = soma_map_name
        self._db_name = db_name
        self._collection_name = collection_name

        # Get the map information from soma2 map_manager
        resp = self._init_map()
        #print resp
        self.soma_map_name = resp.map_name
        self.map_unique_id = resp.map_unique_id
        print "Map name: ",self.soma_map_name," Unique ID ",self.map_unique_id

        # Initialize the mongodb proxy
        self._message_store = MessageStoreProxy(database=db_name, collection=collection_name)

        # Object insertion service
        inss = rospy.Service('soma2/insert_objects', SOMA2InsertObjs, self.handle_insert_request)

        # Object deletion service
        dels = rospy.Service('soma2/delete_objects',SOMA2DeleteObjs, self.handle_delete_request)

        upts = rospy.Service('soma2/update_object',SOMA2UpdateObject,self.handle_update_request)

        rospy.spin()

    # Listens the map information from soma2 map_manager
    def _init_map(self):
        print "Waiting for the map info from soma_map_manager"
        try:
            rospy.wait_for_service('soma2/map_info')
            print "Map info received..."
        except:
           # print("No 'static_map' service")
            return None
        try:
           map_info = rospy.ServiceProxy('soma2/map_info',MapInfo)
           resp1 = map_info(0)
           return resp1
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e
           return None

    # Handles the soma2 objects to be inserted
    def handle_insert_request(self,req):
        _ids = list()
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

          if(obj.cloud.header.frame_id == ""):
              obj.cloud.header.frame_id = "/map"

          obj.map_name = self.soma_map_name
          obj.map_unique_id = self.map_unique_id

          # SOMA2 Objects are represented as a 3D point in the world so this could be set here as point
          obj.geotype = "Point"


          res = self.coords_to_lnglat(obj.pose.position.x,obj.pose.position.y)

          geopose = Pose()

          geopose.position.x = res[0]
          geopose.position.y = res[1]

          obj.geoposearray.poses.append(geopose)

          try:
                _id = self._message_store.insert(obj)
                st = String
                st.data = _id
                _ids.append(st)

          except:
                return SOMA2InsertObjsResponse(False,_ids)

        return SOMA2InsertObjsResponse(True,_ids)


    # Handles the delete request of soma2 objs
    def handle_delete_request(self,req):

        for oid in req.ids:
            res = self._message_store.query(SOMA2Object._type,message_query={"id": oid})
            #print len(res)
            for o,om in res:
                try:
                    self._message_store.delete(str(om['_id']))
                except:
                      return SOMA2DeleteObjsResponse(False)

        return SOMA2DeleteObjsResponse(True)

    # Handles the soma2 objects to be inserted
    def handle_update_request(self,req):

        obj = req.object


        if(obj.logtimestamp == 0):
            obj.logtimestamp = rospy.Time.now().secs

        d = datetime.datetime.utcfromtimestamp(obj.logtimestamp)
        obj.loghour = d.hour
        obj.logminute = d.minute
        obj.logday = d.isoweekday()
        obj.logtimeminutes = obj.loghour*60 + obj.logminute

        if (obj.header.frame_id == ""):
            obj.header.frame_id = "/map"

        if(obj.cloud.header.frame_id == ""):
            obj.cloud.header.frame_id = "/map"

        obj.map_name = self.soma_map_name
        obj.map_unique_id = self.map_unique_id

        # SOMA2 Objects are represented as a 3D point in the world so this could be set here as point
        obj.geotype = "Point"


        res = self.coords_to_lnglat(obj.pose.position.x,obj.pose.position.y)

        geopose = Pose()

        geopose.position.x = res[0]
        geopose.position.y = res[1]

        obj.geoposearray.poses.append(geopose)

        try:
            self._message_store.update_id(req.db_id, obj)
        except:
            return SOMA2UpdateObjectResponse(False)

        return SOMA2UpdateObjectResponse(True)

    def coords_to_lnglat(self, x, y):
        earth_radius = 6371000.0 # in meters
        lng = 90 - math.degrees(math.acos(float(x) / earth_radius))
        lat = 90 - math.degrees(math.acos(float(y) / earth_radius))
        return [lng , lat]


