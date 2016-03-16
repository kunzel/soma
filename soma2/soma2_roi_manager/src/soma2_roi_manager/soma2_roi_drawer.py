#!/usr/bin/env python

import roslib; roslib.load_manifest("soma2_roi_manager")
import rospy
from rospkg import RosPack
import json
import argparse
import random
import copy
import sys

from threading import Timer

from mongodb_store.message_store import MessageStoreProxy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import PoseArray

from soma2_roi_manager.srv import *

from soma2_msgs.msg import SOMA2ROIObject
from soma2_msgs.msg import SOMA2OccupancyMap
from bson.objectid import ObjectId

def trapezoidal_shaped_func(a, b, c, d, x):
    min_val = min(min((x - a)/(b - a), float(1.0)), (d - x)/(d - c))
    return max(min_val, float(0.0))


def r_func(x):
    a = -0.125
    b =  0.125
    c =  0.375
    d =  0.625

    x = 1.0 - x

    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value

def g_func(x):
    a =  0.125
    b =  0.375
    c =  0.625
    d =  0.875

    x = 1.0 - x

    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value


def b_func(x):
    a =  0.375
    b =  0.625
    c =  0.875
    d =  1.125

    x = 1.0 - x

    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value




class SOMA2ROIDrawer():

    def __init__(self):

        #self.soma_map = soma_map
        #self.soma_map_name = soma_map_name

            # default file
        rp = RosPack()


       # self._interactive = True

        self._msg_store=MessageStoreProxy(database="soma2data",collection="soma2_roi")

        s = rospy.Service('soma2/draw_roi', DrawROI, self.handle_draw_roi)

       # Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
        self.markerpub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=1)

        rospy.spin()

    def handle_draw_roi(self,req):
       self._delete_markers()
       if(req.roi_id >=0):
        return DrawROIResponse(self.load_objects(req.map_name,req.roi_id))
       return True

    def _update_poly(self, feedback):
        return
    def _delete_markers(self):
        marker = Marker()
        marker.action = 3
        marker.header.frame_id = "map"
        markerarray = MarkerArray()
        markerarray.markers.append(marker)
        self.markerpub.publish(markerarray)



    def _retrieve_objects(self, map_name, roi_id):

        objs = self._msg_store.query(SOMA2ROIObject._type, message_query={"map_name": map_name,
                                                                      "roi_id": roi_id})
        #print objs
        max_id = 0
        max_roi_id = 0
        for o,om in objs:
            if int(o.id) > max_id:
                max_id = int(o.id)
            if int(o.roi_id) > max_roi_id:
                max_roi_id = int(o.roi_id)
        self._soma_id = max_id
        self._soma_roi_id = max_roi_id

        return objs

    def load_objects(self, map_name, roi_id):

        # this is the array for roi ids
        self._soma_obj_roi_ids = dict()

        markerarray = MarkerArray()

        #get objects from db
        objs = self._retrieve_objects(map_name,roi_id)

        # if collection is empty insert initial object
        if not objs:
            return False

        # otherwise, load all object from collection
        for o,om  in objs:
            count = 1
            for pose in o.posearray.poses:
                self.load_object(o.id, o.roi_id, o.type, pose,count,markerarray)
                count +=1
               # print count
        self.draw_roi(roi_id,o.posearray.poses,markerarray,count)
       # print len(markerarray.markers)
        self.markerpub.publish(markerarray)
        return True

    def draw_all_roi(self):

        for key  in self._soma_obj_roi_ids:
            self.draw_roi(key)

    def undraw_all_roi(self):

        for key  in self._soma_obj_roi_ids:
            self.undraw_roi(key)

    def draw_roi(self, roi,poses,markerarray,ccstart):
        roicp = roi

        p = poses
        cc = ccstart
      #  print "t is ",t," p is ", p
        for pose in p:
           # print "This is the pose: ", pose
            int_marker = self.create_roi_marker(roi, pose, p,cc)
            markerarray.markers.append(int_marker)
            cc = cc+1

    def undraw_roi(self, roi):
        self._server.erase("ROI-" + roi)
        self._server.applyChanges()

    def load_object(self, soma_id, roi, soma_type, pose,markerno, markerarray):

       # print self._soma_obj_markers[str(soma_id)]
       # print str(soma_id)
        int_marker = self.create_object_marker(soma_id, roi, soma_type, pose, markerno)

        markerarray.markers.append(int_marker)
        #self.markerpub.publish(int_marker)

        #print self._soma_obj_markers[str(soma_id)].keys()



#soma_type = Office, Kitchen, etc, Pose is position
    def add_object(self, soma_type, pose, roi_id=None):
        # todo: add to mongodb

        #create a SOMA2ROI Object
        soma_obj = SOMA2ROIObject()

       # print roi_id

        # a new roi
        if roi_id == None:

            #soma_id is an id for the soma object like 1,2,3,4. It updates itself from the db if there are existing objects
            soma_id = self._next_id()

            #soma_roi_id is acutally the roi number. Is it 1,2,3,4? Multiple soma objects can have the same roi id
            soma_roi_id = self._next_roi_id()

            roi_id = soma_roi_id
           # print soma_roi_id

            soma_obj.id = str(soma_id)
            soma_obj.roi_id = str(soma_roi_id)
            soma_obj.map_name = str(self.soma_map_name)
            soma_obj.map_unique_id = str(self.map_unique_id)
            soma_obj.config = str(self.soma_conf)
            soma_obj.type = soma_type
            soma_obj.posearray.poses.append(pose)
            soma_obj.frame = 'map'
            self._soma_obj_roi_ids[str(soma_roi_id)] = list()
            self._soma_obj_markers[soma_obj.id] = dict()
            #_id = self._msg_store.update_id
            _id = self._msg_store.insert(soma_obj)
            self._soma_obj_ids[soma_obj.id] = _id
            self._soma_obj_roi_ids[soma_obj.roi_id].append(soma_obj.id)
            self._soma_obj_type[soma_obj.id] = soma_type
            self._soma_obj_roi[soma_obj.id] = roi_id
            self._soma_obj_msg[soma_obj.id] = soma_obj
            self._soma_obj_pose[soma_obj.id] = soma_obj.posearray.poses

        else:
            # Get the roi id
            soma_roi_id = roi_id
            #print roi_id," ",self.soma_map," ",self.soma_conf," ",self._soma_obj_ids['1']

            #call the object with that id
            res = self._msg_store.query(SOMA2ROIObject._type,message_query={'id':str(roi_id)})

            #iterate through the objects. Normally there should be only 1 object returned
            for o,om in res:
               # print o," hi ",om
                soma_obj = o
              #  print "Soma Object: ", soma_obj
            if soma_obj:
                soma_obj.posearray.poses.append(pose)

                self._soma_obj_pose[soma_obj.id] = soma_obj.posearray.poses

                self.insert_geo_json(soma_obj.roi_id,soma_obj)

                #print soma_obj
                _id = self._soma_obj_ids[soma_obj.id]
                _newid =  self._msg_store.update_id(_id,soma_obj)

                soma_id = soma_obj.id

                self._soma_obj_msg[soma_obj.id] = soma_obj

        #_id = self._msg_store.update_id

        #self._soma_obj_ids[soma_obj.id] = _id
        #self._soma_obj_msg[soma_obj.id] = soma_obj
        #self._soma_obj_roi_ids[soma_obj.roi_id].append(soma_obj.id)
        #self._soma_obj_roi[soma_obj.id].append(soma_obj.roi_id)



        #for pose in soma_obj.posearray.poses:
        self.load_object(str(soma_id), soma_obj.roi_id, soma_type, pose)



    def create_object_marker(self, soma_obj, roi, soma_type, pose,markerno):
        # create an interactive marker for our server
        marker = Marker()
        marker.header.frame_id = "map"
        #int_marker.name = soma_obj+'_'+str(markerno)
       #int_marker.description = soma_type + ' (' + roi +'_'+str(markerno)+  ')'
        marker.pose = pose
        marker.id = markerno;
       # print marker.pose
        marker.pose.position.z = 0.01


        #marker = Marker()
        marker.type = Marker.SPHERE
        marker.action = 0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.pose.position.z = (marker.scale.z / 2)

        random.seed(soma_type)
        val = random.random()
        marker.color.r = r_func(val)
        marker.color.g = g_func(val)
        marker.color.b = b_func(val)
        marker.color.a = 1.0
        #marker.pose = pose

        return marker

# This part draws the line strips between the points
    def create_roi_marker(self, roi, pose, points, count):
        #print "POINTS: " + str(points)
        #points are all the points belong to that roi, pose is one of the points
        marker = Marker()

       # print "Marker name: ", int_marker.name

        marker.pose = pose
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.1
        marker.id= count

        random.seed(str(count))
        val = random.random()
        marker.color.r = r_func(val)
        marker.color.g = g_func(val)
        marker.color.b = b_func(val)
        marker.color.a = 1.0

        marker.points = []
        for point in points:
            p = Point()
            pose = point#self._soma_obj_pose[point]

            p.x = pose.position.x - marker.pose.position.x
            p.y = pose.position.y - marker.pose.position.y
            marker.points.append(p)

        p = Point()
        pose = points[0]
        p.x = pose.position.x - marker.pose.position.x
        p.y = pose.position.y - marker.pose.position.y
        marker.points.append(p)

        return marker





if __name__=="__main__":

    # TODO: add list command


    rospy.init_node("soma2roidrawer")
    rospy.loginfo("Running SOMA2 ROI Drawer")
    SOMA2ROIDrawer()
