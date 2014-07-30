#!/usr/bin/env python

import roslib; roslib.load_manifest("soma_utils")
import sys
import rospy
from rospkg import RosPack
import json
import argparse
import random
import copy

from tabulate import tabulate

from threading import Timer

from ros_datacentre.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Polygon

from visualization_msgs.msg import Marker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import *

from soma_msgs.msg import SOMAObject
from soma_msgs.msg import SOMAROIObject

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

    
    
class SOMAUtils():

    def __init__(self, obj_map, obj_conf, roi_map, roi_conf, config_file=None):

        self._soma_obj_roi_ids = dict()
        self._soma_obj_type = dict()
        self._soma_obj_pose = dict()

        
        self.obj_map = obj_map
        self.obj_conf = obj_conf
        self.roi_map = roi_map
        self.roi_conf = roi_conf
        self._soma_roi = dict()
        self._obj_msg_store=MessageStoreProxy(collection="soma")
        self._roi_msg_store=MessageStoreProxy(collection="soma_roi")
        self._retrieve_objects()
        self._retrieve_rois()

        self._server = InteractiveMarkerServer("soma_vis")
        
        if config_file:
            self._config_file = config_file
        else:
            # default file
            rp = RosPack()
            path = rp.get_path('soma_objects') + '/config/'
            filename = 'default.json'
            self._config_file=path+filename

        self._init_types()

    def _init_types(self):
        # read from config in soma_objects 
        
        with open(self._config_file) as config_file:
            config = json.load(config_file)

            self.mesh = dict()
            self.marker = dict()
            if '2D' in config:
                for k, v in config['2D'].iteritems():
                    self.mesh[k] = v
                    self.marker[k] = '2D'

            if '3D' in config:
                for k, v in config['3D'].iteritems():
                    self.mesh[k] = v
                    self.marker[k] = '3D'

    def _update_cb(self,feedback):
        return
                    
    def load_object(self, soma_id, soma_type, pose):

        int_marker = self.create_object_marker(soma_id, soma_type, pose)
        self._server.insert(int_marker, self._update_cb)
        self._server.applyChanges()

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

            if not roi.roi_id in self._soma_obj_roi_ids:
                self._soma_obj_roi_ids[roi.roi_id] = list()
                self._soma_obj_roi_ids[roi.roi_id].append(roi.id)
            else:
                self._soma_obj_roi_ids[roi.roi_id].append(roi.id)
            self._soma_obj_type[roi.id] = roi.type
            self._soma_obj_pose[roi.id] = roi.pose

    def _get_polygon(self,roi):

        polygon = Polygon()

        if roi not in self._soma_roi:
            rospy.logwarn("Invalid ROI ID")
            return polygon

        nodes = self._soma_roi[roi]
        for n in nodes:
            p = Point()
            p.x = n.pose.position.x
            p.y = n.pose.position.y
            polygon.points.append(p)
            
        return polygon

    def _get_objects(self):
        return self._objs

    def _objects(self, polygon, objects):

        obj_ids = []
        for obj, _meta in objects:
            point = Point()
            point.x = obj.pose.position.x
            point.y = obj.pose.position.y
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

    def draw_roi(self, roi):

        if roi not in self._soma_obj_roi_ids:
            rospy.logwarn("Invalid ROI ID")
            return
        
        v = self._soma_obj_roi_ids[roi]
        t = self._soma_obj_type[v[0]]
        p = self._soma_obj_pose[v[0]]
        int_marker = self.create_roi_marker(roi, t, p, v)
        
        self._server.insert(int_marker, self._update_poly)
        self._server.applyChanges()

    def _update_poly(self,feedback):
        return
    
    def show (self, roi):
        print "Show: " + roi

        self.draw_roi(roi)

        polygon = self._get_polygon(roi)    
        for obj, meta in self._objs:
            point = Point()
            point.x = obj.pose.position.x
            point.y = obj.pose.position.y
            if self._inside(point,polygon):
                self.load_object(obj.id, obj.type, obj.pose)
    
    def list(self):

        table = []

        for k, v in self._soma_roi.iteritems():
            table.append([ k , self._soma_roi[str(k)][0].type , self.area(k), str(self.center(k)), str(self.objects(k))])

        print tabulate(table, headers=['ID', 'Type', 'Area (m^2)', 'Center', 'Object IDs'],tablefmt='rst')
        print "Total %i" %  len(self._soma_roi)


    def create_object_marker(self, soma_obj, soma_type, pose):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/map"
        int_marker.name = soma_obj
        int_marker.description = "id" + soma_obj
        int_marker.pose = pose
        
        mesh_marker = Marker()
        mesh_marker.type = Marker.MESH_RESOURCE
        mesh_marker.scale.x = 1
        mesh_marker.scale.y = 1
        mesh_marker.scale.z = 1

        random.seed(soma_type)
        val = random.random()
        mesh_marker.color.r = r_func(val)
        mesh_marker.color.g = g_func(val)
        mesh_marker.color.b = b_func(val)
        mesh_marker.color.a = 1.0
        #mesh_marker.pose = pose
        mesh_marker.mesh_resource = self.mesh[soma_type]

        control = InteractiveMarkerControl()
        control.markers.append(mesh_marker)
        int_marker.controls.append(control)
        
        return int_marker

    def create_roi_marker(self, roi, soma_type, pose, points):
        #print "POINTS: " + str(points)
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/map"
        int_marker.name = "ROI-" + roi
        int_marker.description = roi
        int_marker.pose = pose
        
        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.1
        
        random.seed(soma_type)
        val = random.random()
        marker.color.r = r_func(val)
        marker.color.g = g_func(val)
        marker.color.b = b_func(val)
        marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.markers.append( marker )

        int_marker.controls.append(control)
        
        marker.points = []
        for point in points:
            p = Point()
            pose = self._soma_obj_pose[point]
            p.x = pose.position.x - int_marker.pose.position.x  
            p.y = pose.position.y - int_marker.pose.position.y
            marker.points.append(p)

        p = Point()
        pose = self._soma_obj_pose[points[0]]
        p.x = pose.position.x - int_marker.pose.position.x  
        p.y = pose.position.y - int_marker.pose.position.y
        marker.points.append(p)

        return int_marker
        
    #########################################################################################    
    ### Begin STEM project ##################################################################
    #########################################################################################    
    
    def _area(self, polygon):
        area = 0.0

        # Put your code here!
        
        return area

    def _center(self, polygon):
        center = [0.0, 0.0]

        # Put your code here!
        
        return center
    
    def _inside(self, point, polygon):
        if not polygon:
            return False
        
        # Put your code here!
        
        # return True
        return False
    
    #########################################################################################    
    ### End STEM project ####################################################################
    #########################################################################################            

    
if __name__=="__main__":

    parser = argparse.ArgumentParser(prog='soma_utils.py')
    parser.add_argument("cmd", nargs=1, choices=['list', 'show'], help='Name of the command')
    parser.add_argument("roi_map", nargs=1, help='Name of the used ROI map')
    parser.add_argument("roi_conf", nargs=1, help='Name of the ROI configuration')
    parser.add_argument("--obj_map", nargs=1,default=None, help='Name of the used object map')
    parser.add_argument("--obj_conf", nargs=1, default=None, help='Name of the object configuration')
    parser.add_argument("-r", "--roi", nargs=1, default='0', help='ID of a ROI')

    args = parser.parse_args()
    
    rospy.init_node("soma")

    if args.obj_map == None:
        args.obj_map = args.roi_map

    if args.obj_conf == None:
        args.obj_conf = args.roi_conf
    
    rospy.loginfo("Running SOMA utils (cmd: %s, obj_map: %s, obj_conf: %s, obj_map: %s, obj_conf: %s)", args.cmd[0], args.obj_map[0], args.obj_conf[0], args.roi_map[0], args.roi_conf[0])
    soma = SOMAUtils(args.obj_map[0], args.obj_conf[0], args.roi_map[0], args.roi_conf[0])

    
    if args.cmd[0] == 'list':
        soma.list()
    elif args.cmd[0] == 'show':
        roi =  str(args.roi[0])
        soma.show(roi)
        rospy.spin()
        


