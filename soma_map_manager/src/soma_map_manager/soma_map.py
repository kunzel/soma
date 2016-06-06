#!/usr/bin/env python

import roslib; roslib.load_manifest("soma_map_manager")
import rospy
from rospkg import RosPack
import json
import argparse
import random
import copy
import sys

from threading import Timer

from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

from soma2_msgs.msg import SOMA2ROIObject
from soma2_msgs.msg import SOMA2OccupancyMap
from bson.objectid import ObjectId

from soma_map_manager.srv import *

class SOMAMapManager():

    def __init__(self, mapname=None):

       # self.soma_map_name = soma_map_name
        self.map_unique_id = -1
        self.soma2map = SOMA2OccupancyMap()

        self.pub = rospy.Publisher('soma2/map', OccupancyGrid, queue_size=1, latch=True)

        self._map_store=MessageStoreProxy(database="maps", collection="soma2")

        if(self._init_map(mapname)):
            s = rospy.Service('soma2/map_info', MapInfo, self.handle_map_info)
            self.run_node()
        print "Quitting..."

        #rospy.spin()
    def handle_map_info(self,req):
        return MapInfoResponse(self.soma2map.mapname,str(self.map_unique_id))

    def _init_map(self,mapname):

        if(mapname != None):

            res = self._map_store.query(SOMA2OccupancyMap._type, message_query={"mapname":mapname})
            if res:
                print "Map is found in DB. Publishing... "
                self.soma2map,meta = res[0]
                #print meta
                self.soma2map.map.data = self.rldecode(self.soma2map.rle_values,self.soma2map.rle_counts)
                self.map_unique_id = meta['_id']
                return True

        #check for the map with map name
        res = self._map_store.query(SOMA2OccupancyMap._type, message_query={})

        #if map is already stored just get the unique identifier
        if res:
            print "These maps are found in DB, enter the number to choose: "
            count  = 1
            for amap,meta in res:
                print count," ",amap.mapname
                count +=1
            while 1:
                var = raw_input("Please enter your choice (0 to load new map via an active map_server node, -99 to quit): ")
                print "you entered", var
                if(var.isdigit() or var.startswith('-') and var[1:].isdigit()):
                    if int(var) == -99:
                        return False
                    if(int(var) <= len(res)):
                        if(int(var) == 0):
                            return self.listen_and_store_map()
                        self.soma2map,meta = res[(int(var)-1)]
                              #print meta
                        self.soma2map.map.data = self.rldecode(self.soma2map.rle_values,self.soma2map.rle_counts)
                        self.map_unique_id = meta['_id']
                        return True
            #print self.selected_map.map.data
            #print selected_map.map.data

            #pub.publish(selected_map.map)

        else:
             print "No maps are found in DB. Listening map_server for a map..."
             if(self.listen_and_store_map()):
                return True
             return False

    def listen_and_store_map(self):
         occmap = SOMA2OccupancyMap()
         map = self._get_occupancy_map()
         if map:
            var = raw_input("Map is received. Please enter a name for storing it in the database: ")
            print "you entered", var
            occmap.map = map.map
            occmap.mapname = var
            tempmap = copy.deepcopy(occmap)
            #print map.map.data[0]
            values,counts = self.rlencode(tempmap.map.data)
            #print values
            #print counts
            occmap.rle_values = values
            #print sum(counts)
            occmap.rle_counts = counts
            #print len(occmap.map.data)
            occmap.map.data = []
            _id = self._map_store.insert(occmap)
            self.soma2map = tempmap
            self.map_unique_id = _id
            return True
         else:
            print "map_server is not working. Please check map_server node and ensure that it is working"
            return False

    def run_node(self):
           rate = rospy.Rate(1) # 1hz
           print "map is now being published on soma2/map topic"
           self.pub.publish(self.soma2map.map)
           while not rospy.is_shutdown():
                rate.sleep()
    def rlencode(self,src):
        result = []
        counts = []
        values = []
        if src:
            current = src[0]
            #print current
            counter = 0
            for e in src:
                #print e
                if e == current:
                    counter += 1
                else:
                    counts.append(counter)
                    values.append(current)
                    result.append((counter, current))
                    current = e
                    counter = 1
            counts.append(counter)
            values.append(current)
            result.append((counter, current))
        #print result
        return values,counts


    def rldecode(self,values,counts):
        q = []
        count = 0
        for avalue in values:
            for i in range(counts[count]):
                q.append(avalue)
            count +=1

        return q



    def _get_occupancy_map(self):
        try:
            rospy.wait_for_service('/static_map')
        except:
            print("No 'static_map' service")
            return None
        try:
           static_map = rospy.ServiceProxy('/static_map',GetMap)
           resp1 = static_map()
           return resp1
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e
           return None





if __name__=="__main__":

    # TODO: add list command

    parser = argparse.ArgumentParser(description='Check and publish the given map name')

    parser.add_argument('--mapname', default=None, help='map name')

    args = parser.parse_args((rospy.myargv()[1:]))

    #print args.mapname

    rospy.init_node("soma_map_manager")
    rospy.loginfo("Running SOMA map manager")

    SOMAMapManager(args.mapname)
