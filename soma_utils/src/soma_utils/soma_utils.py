#!/usr/bin/env python

import roslib; roslib.load_manifest("soma_utils")
import rospy
from rospkg import RosPack
import json
import argparse
import random
import copy

from tabulate import tabulate

from threading import Timer

from ros_datacentre.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose

from soma_msgs.msg import SOMAObject
from soma_msgs.msg import SOMAROIObject

from bson.objectid import ObjectId


class ROI():

    def __init__(self, soma_roi):
        pass

    def add(self,soma_roi):
        pass

    def get_nodes(self):
        pass


    
class SOMAUtils():

    def __init__(self, obj_map, obj_conf, roi_map, roi_conf):

        self.obj_map = obj_map
        self.obj_conf = obj_conf
        self.roi_map = roi_map
        self.roi_conf = roi_conf
        self._soma_roi = dict()
        self._obj_msg_store=MessageStoreProxy(collection="soma")
        self._roi_msg_store=MessageStoreProxy(collection="soma_roi")
        self._retrieve_objects()
        self._retrieve_rois()



    def _retrieve_objects(self):
        self._objs = self._obj_msg_store.query(SOMAObject._type,
                                               message_query={"map": self.obj_map,
                                                              "config": self.obj_conf})
    def _retrieve_rois(self):        
        self._rois = self._roi_msg_store.query(SOMAROIObject._type,
                                               message_query={"map": self.roi_map,
                                                              "config": self.roi_conf})
        for roi, _meta in self._rois:
            if not roi.roi_id in self._soma_roi:
                self._soma_roi[roi.roi_id] = list()
                self._soma_roi[roi.roi_id].append(roi)
            else:
                self._soma_roi[roi.roi_id].append(roi)

    def _get_polygon(self,roi):

        polygon = []
        nodes = self._soma_roi[roi]
        for n in nodes:
            polygon.append([n.pose.position.x,n.pose.position.y])
        return polygon

    def _get_objects(self):
        return self._objs

    def _objects(self, polygon, objects):

        obj_ids = []
        for obj, _meta in objects:
            point = [obj.pose.position.x, obj.pose.position.y]
            if self._inside(point,polygon):
                obj_ids.append(obj.id)

        return obj_ids

    def area(self,roi):
        polygon = self._get_polygon(roi)
        return self._area(polygon)

    def center(self, roi):
        polygon = self._get_polygon(roi)
        return self._center(polygon) 

    def objects(self, roi):
        polygon = self._get_polygon(roi)
        objects = self._get_objects()
        return self._objects(polygon, objects) 

    def list_roi(self):

        table = []

        for k, v in self._soma_roi.iteritems():
            table.append([ k , self._soma_roi[str(k)][0].type , self.area(k), str(self.center(k)), str(self.objects(k))])

        print tabulate(table, headers=['ID', 'Type', 'Area (m^2)', 'Center', 'Object IDs'],tablefmt='rst')
        print "Total %i" %  len(self._soma_roi)

    #########################################################################################    
    ### Begin STEM project ##################################################################
    #########################################################################################    
    
    def _area(self, polygon):
        # a polygon is formatted as follows: [[-3.2, 6.6], [-1.1, 8.4], [0.1, 7.3]]
        area = 0.0

        # Put your code here!
        
        return area

    def _center(self, polygon):
        # a polygon is formatted as follows: [[-3.2, 6.6], [-1.1, 8.4], [0.1, 7.3]]
        center = [0.0, 0.0]

        # Put your code here!
        
        return center
    
    def _inside(self, point, polygon):
        # a point is formatted as follows: [-1.2, 2.6]
        # a polygon is formatted as follows: [[-3.2, 6.6], [-1.1, 8.4], [0.1, 7.3]]

        # Put your code here!
        
        # return True
        return False
    
    #########################################################################################    
    ### End STEM project ####################################################################
    #########################################################################################            

if __name__=="__main__":

    parser = argparse.ArgumentParser(prog='soma_utils.py')
    parser.add_argument("cmd", nargs=1, choices=['ls'], help='Name of the command')
    parser.add_argument("roi_map", nargs=1, help='Name of the used ROI map')
    parser.add_argument("roi_conf", nargs=1, help='Name of the ROI configuration')
    parser.add_argument("--obj_map", nargs=1,default=None, help='Name of the used object map')
    parser.add_argument("--obj_conf", nargs=1, default=None, help='Name of the object configuration')
    parser.add_argument("-r", "--roi", nargs=1, default=0, help='ID of a ROI')

    args = parser.parse_args()
    
    rospy.init_node("soma")

    if args.obj_map == None:
        args.obj_map = args.roi_map

    if args.obj_conf == None:
        args.obj_conf = args.roi_conf
    
    rospy.loginfo("Running SOMA utils (obj_map: %s, obj_conf: %s, obj_map: %s, obj_conf: %s)", args.obj_map[0], args.obj_conf[0], args.roi_map[0], args.roi_conf[0])
    soma = SOMAUtils(args.obj_map[0], args.obj_conf[0], args.roi_map[0], args.roi_conf[0])
    soma.list_roi()
    


