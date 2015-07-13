#!/usr/bin/env python

import roslib; roslib.load_manifest("soma_roi_manager")
import rospy
from rospkg import RosPack
import json
import argparse
import random
import copy
import sys

from threading import Timer

from mongodb_store.message_store import MessageStoreProxy
from soma_geospatial_store.geospatial_store import GeoSpatialStoreProxy
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Polygon

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

class SOMAROIQuery():

    def __init__(self, soma_map, soma_conf):
        self.soma_map = soma_map
        self.soma_conf = soma_conf
        self._msg_store=MessageStoreProxy(collection="soma_roi")

    def get_polygon(self, roi_id):
        objs = self._msg_store.query(SOMAROIObject._type, message_query={"map": self.soma_map,
                                                                         "config": self.soma_conf,
                                                                         "roi_id": roi_id})
        ids = []
        poses = []
        for o,om in objs:
            ids.append(o.id)
            poses.append(o.pose)

        sorted_poses = [_pose for (_id,_pose) in sorted(zip(ids, poses))]
        poly = Polygon()
        poly.points = []
        for p in sorted_poses:
            point = Point()
            point.x = p.position.x 
            point.y = p.position.y 
            poly.points.append(point)
        return poly

    def get_rois(self, roi_type=None):
        """
        Returns a set of roi IDs of the given type. If type not specified,
        returns all rois in this map/configuration.
        """
        if roi_type is not None:
            objs = self._msg_store.query(SOMAROIObject._type,
                                         message_query={"map": self.soma_map,
                                                        "config": self.soma_conf,
                                                        "type": roi_type})
        else:
            objs = self._msg_store.query(SOMAROIObject._type,
                                         message_query={"map": self.soma_map,
                                                        "config": self.soma_conf} )
        #TODO: here it would be nice to be able to use mongodb distinct function
        rois=set()
        for o in objs:
            rois.add(o[0].roi_id)
        return rois
            

class SOMAROIManager():

    def __init__(self, soma_map, soma_conf, config_file=None):

        self.soma_map = soma_map
        self.soma_conf = soma_conf
        if config_file:
            self._config_file = config_file
        else:
            # default file
            rp = RosPack()
            path = rp.get_path('soma_roi_manager') + '/config/'
            filename = 'default.json'
            self._config_file=path+filename
        self._soma_obj_ids = dict()
        self._soma_obj_msg = dict()
        self._soma_obj_roi_ids = dict()
        self._soma_obj_roi = dict()
        self._soma_obj_type = dict()
        self._soma_obj_pose = dict()

        self._interactive = True

        self._msg_store=MessageStoreProxy(collection="soma_roi")

        self._gs_store=GeoSpatialStoreProxy(db="geospatial_store", collection="soma")
        
        self._server = InteractiveMarkerServer("soma_roi")

        self._init_types()

        self._init_menu()
        
        self.load_objects()

        self.update_objects()

        rospy.spin()


    def _init_types(self):
        # read from config in soma_objects 
        
        with open(self._config_file) as config_file:
            config = json.load(config_file)

            self.mesh = dict()
            for k, v in config['roi'].iteritems():
                self.mesh[k] = v

    def _init_menu(self):

        self.menu_handler = MenuHandler()

        add_point_entry = self.menu_handler.insert( "Add Point", callback=self._add_point_cb)
        del_point_entry = self.menu_handler.insert( "Delete Point", callback=self._del_point_cb)


        add_entry = self.menu_handler.insert( "Add ROI" )

        self.menu_item = dict()
        for k in sorted(self.mesh.keys()):
            entry =  self.menu_handler.insert(k, parent=add_entry, callback=self._add_cb)
            self.menu_item[entry] = k
            
        del_entry =  self.menu_handler.insert( "Delete ROI", callback=self._del_cb)

        
        enable_entry = self.menu_handler.insert( "Movement control", callback=self._enable_cb )
        self.menu_handler.setCheckState( enable_entry, MenuHandler.CHECKED )

    def _add_cb(self, feedback):
        rospy.loginfo("Add ROI: %s", self.menu_item[feedback.menu_entry_id])
        self.add_object(self.menu_item[feedback.menu_entry_id], feedback.pose)

    def _del_cb(self, feedback):
        rospy.loginfo("Delete ROI: %s", feedback.marker_name)
        roi = self._soma_obj_roi[feedback.marker_name]
        for r in self._soma_obj_roi_ids[roi]:
            self.delete_object(r)
        self.undraw_roi(roi)

    def _add_point_cb(self, feedback):
        rospy.loginfo("Add point: %s", feedback.marker_name)
        roi = self._soma_obj_roi[feedback.marker_name]
        t   = self._soma_obj_type[feedback.marker_name]
        self.add_object(t, feedback.pose, roi)
        #self.draw_roi(roi)

    def _del_point_cb(self, feedback):
        rospy.loginfo("Delete point: %s", feedback.marker_name)
        self.delete_object(feedback.marker_name)
        roi = self._soma_obj_roi[feedback.marker_name]
        self.draw_roi(roi)

    def _update_poly(self, feedback):
        return
    
    def _update_cb(self, feedback):
        p = feedback.pose.position
        #print "Marker " + feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y)
        self._soma_obj_pose[feedback.marker_name] = feedback.pose

        roi = self._soma_obj_roi[feedback.marker_name]
        self.draw_roi(roi)

        if hasattr(self, "vp_timer_"+feedback.marker_name):
            getattr(self, "vp_timer_"+feedback.marker_name).cancel()        
        setattr(self, "vp_timer_"+feedback.marker_name,
                Timer(3, self.update_object, [feedback]))
        getattr(self, "vp_timer_"+feedback.marker_name).start()        
        
    def _enable_cb(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )

        if state == MenuHandler.CHECKED:
            self.menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            self._interactive = False
        else:
            self.menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            self._interactive = True

        self.menu_handler.reApply( self._server )

        self.load_objects()
        
        self._server.applyChanges()
        
    def _next_id(self):
        self._soma_id += 1
        return self._soma_id

    def _next_roi_id(self):
        self._soma_roi_id += 1
        return self._soma_roi_id

    def _retrieve_objects(self):

        objs = self._msg_store.query(SOMAROIObject._type, message_query={"map": self.soma_map,
                                                                      "config": self.soma_conf})

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
    
    def load_objects(self):


        self._soma_obj_roi_ids = dict()
        
        objs = self._retrieve_objects()

        # if collection is empty insert initial object
        if not objs:
            pose = Pose()
            self.add_object('Office', pose)
            return

        # otherwise, load all object from collection
        for o, om  in objs:
            self._soma_obj_ids[o.id] = om['_id']
            self._soma_obj_msg[o.id] = o
            if o.roi_id in self._soma_obj_roi_ids:
                self._soma_obj_roi_ids[o.roi_id].append(o.id)
            else:
                self._soma_obj_roi_ids[o.roi_id] = list()
                self._soma_obj_roi_ids[o.roi_id].append(o.id)
                
            self._soma_obj_roi[o.id] = o.roi_id
            self._soma_obj_type[o.id] = o.type
            self._soma_obj_pose[o.id] = o.pose
            
            self.load_object(o.id, o.roi_id, o.type, o.pose)

        self.draw_all_roi()

    def draw_all_roi(self):

        for key  in self._soma_obj_roi_ids:
            self.draw_roi(key)

    def undraw_all_roi(self):

        for key  in self._soma_obj_roi_ids:
            self.undraw_roi(key)

    def draw_roi(self, roi):
        
        v = self._soma_obj_roi_ids[roi]
        t = self._soma_obj_type[v[0]]
        p = self._soma_obj_pose[v[0]]
        int_marker = self.create_roi_marker(roi, t, p, v)
        
        self._server.erase("ROI-" + roi)
        self._server.applyChanges()
        self._server.insert(int_marker, self._update_poly)
        self._server.applyChanges()

    def undraw_roi(self, roi):
        self._server.erase("ROI-" + roi)
        self._server.applyChanges()

    def load_object(self, soma_id, roi, soma_type, pose):

        int_marker = self.create_object_marker(soma_id, roi, soma_type, pose)

        self._server.insert(int_marker, self._update_cb)

        self.menu_handler.apply( self._server, soma_id )

        self._server.applyChanges()

    def add_object(self, soma_type, pose, roi_id=None):
        # todo: add to mongodb
        
        soma_id = self._next_id()

        if roi_id == None:
            soma_roi_id = self._next_roi_id()
            self._soma_obj_roi_ids[str(soma_roi_id)] = list()
        else:
            soma_roi_id = roi_id

        soma_obj = SOMAROIObject()
        soma_obj.id = str(soma_id)
        soma_obj.roi_id = str(soma_roi_id)
        soma_obj.map = str(self.soma_map)
        soma_obj.config = str(self.soma_conf)
        soma_obj.type = soma_type
        soma_obj.pose = pose
        soma_obj.frame = 'map'

        _id = self._msg_store.insert(soma_obj)
        
        self._soma_obj_ids[soma_obj.id] = _id
        self._soma_obj_msg[soma_obj.id] = soma_obj
        self._soma_obj_roi_ids[soma_obj.roi_id].append(soma_obj.id)
        self._soma_obj_roi[soma_obj.id] = soma_obj.roi_id
        self._soma_obj_type[soma_obj.id] = soma_type
        self._soma_obj_pose[soma_obj.id] = pose

        self.load_object(str(soma_id), soma_obj.roi_id, soma_type, pose)

        # geospatial store
        # delete old message
        res = self._gs_store.find_one({'soma_roi_id': soma_roi_id,
                                       'soma_map': self.soma_map,
                                       'soma_config': self.soma_conf})
        if res:
            _gs_id = res['_id']
            self._gs_store.remove(_gs_id)
            #rospy.loginfo("GS Store: deleted roi")           

        # add new object to geospatial store
        geo_json = self.geo_json_from_soma_obj(soma_obj)
        if geo_json:
            try:
                self._gs_store.insert(geo_json)
                rospy.loginfo("GS Store: updated roi (%s %s)" % (soma_obj.type, soma_obj.roi_id) )
            except:
                rospy.logerr("The polygon of %s %s is malformed (self-intersecting) => Please update geometry." % (soma_type, soma_roi_id))



    def geo_json_from_soma_obj(self, soma_obj):

        geo_json = {}
        geo_json['soma_roi_id'] = soma_obj.roi_id
        geo_json['soma_map'] = soma_obj.map
        geo_json['soma_config'] = soma_obj.config
        geo_json['type'] = soma_obj.type


        if len(self._soma_obj_roi_ids[soma_obj.roi_id]) < 3:
            rospy.logerr("GS Store: %s %s, less then 3 points => Add more points or delete ROI." % (soma_obj.type, soma_obj.roi_id) )
            return None
        coordinates = []    
        for obj_id in self._soma_obj_roi_ids[soma_obj.roi_id]:
            p = self._soma_obj_pose[obj_id]
            coordinates.append(self._gs_store.coords_to_lnglat(p.position.x, p.position.y))

        p = self._soma_obj_pose[self._soma_obj_roi_ids[soma_obj.roi_id][0]]
        coordinates.append(self._gs_store.coords_to_lnglat(p.position.x, p.position.y))
            
        geo_json['loc'] = {'type': 'Polygon',
                           'coordinates': [coordinates]}
        return geo_json

    def delete_object(self, soma_id):
        # todo: delete from mongodb

        _id = self._soma_obj_ids[str(soma_id)]
        self._msg_store.delete(str(_id))
        
        self._server.erase(soma_id)
        self._server.applyChanges()

        roi = self._soma_obj_roi[str(soma_id)]
        nodes = self._soma_obj_roi_ids[roi]
        new_nodes = []
        for n in nodes:
            if n != str(soma_id):
                new_nodes.append(n)
        self._soma_obj_roi_ids[roi] = new_nodes

        # geospatial store
        # delete old message
        old_msg = self._soma_obj_msg[soma_id]
        res = self._gs_store.find_one({'soma_roi_id': roi,
                                       'soma_map': self.soma_map,
                                       'soma_config': self.soma_conf})
        if res:
            _gs_id = res['_id']
            self._gs_store.remove(_gs_id)
            #rospy.loginfo("GS Store: deleted roi")

        # add new object to geospatial store
        geo_json = self.geo_json_from_soma_obj(old_msg)
        if geo_json:
            try:
                self._gs_store.insert(geo_json)
                rospy.loginfo("GS Store: update roi (%s %s)" % (old_msg.type, old_msg.roi_id) )
            except:
                rospy.logerr("The polygon of %s %s is malformed (self-intersecting) => Please update geometry." % (old_msg.type, old_msg.roi_id))
    

    def update_objects(self):
        rospy.loginfo("Update all objects (incl. geospatial store)")

        for k in self._soma_obj_ids.keys():
            _id = self._soma_obj_ids[k]
            msg = self._soma_obj_msg[k]

            new_msg = copy.deepcopy(msg)

            # no change in pose!!!
            
            self._msg_store.update_id(_id, new_msg)

            # geospatial store
            # delete old message
            res = self._gs_store.find_one({'soma_roi_id': new_msg.roi_id,
                                           'soma_map': self.soma_map,
                                           'soma_config': self.soma_conf})
            if res:
                _gs_id = res['_id']
                self._gs_store.remove(_gs_id)
                #rospy.loginfo("GS Store: deleted roi")            

            # add new object to geospatial store
            geo_json = self.geo_json_from_soma_obj(new_msg)
            if geo_json:
                try:
                    self._gs_store.insert(geo_json)
                except:
                    rospy.logerr("The polygon of %s %s is malformed (self-intersecting) => Please update geometry." % (new_msg.type, new_msg.roi_id))

        rospy.loginfo("Done")
            
                
    def update_object(self, feedback):
        rospy.loginfo("Updated marker: %s", feedback.marker_name)

        _id = self._soma_obj_ids[feedback.marker_name]
        msg = self._soma_obj_msg[feedback.marker_name]

        new_msg = copy.deepcopy(msg)
        new_msg.pose = feedback.pose

        self._soma_obj_pose[feedback.marker_name] = feedback.pose
        
        self._msg_store.update_id(_id, new_msg)

        # geospatial store
        # delete old message
        res = self._gs_store.find_one({'soma_roi_id': new_msg.roi_id,
                                       'soma_map': self.soma_map,
                                       'soma_config': self.soma_conf})
        if res:
            _gs_id = res['_id']
            self._gs_store.remove(_gs_id)
            #rospy.loginfo("GS Store: deleted roi")            

        # add new object to geospatial store
        geo_json = self.geo_json_from_soma_obj(new_msg)
        if geo_json:
            try:
                self._gs_store.insert(geo_json)
                rospy.loginfo("GS Store: updated roi (%s %s)" % (new_msg.type, new_msg.roi_id) )
            except:
                rospy.logerr("The polygon of %s %s is malformed (self-intersecting) => Please update geometry." % (new_msg.type, new_msg.roi_id))  

    def create_object_marker(self, soma_obj, roi, soma_type, pose):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = soma_obj
        int_marker.description = soma_type + ' (' + roi +  ')'
        int_marker.pose = pose
        int_marker.pose.position.z = 0.01
        
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        int_marker.pose.position.z = (marker.scale.z / 2)
        
        random.seed(soma_type)
        val = random.random()
        marker.color.r = r_func(val)
        marker.color.g = g_func(val)
        marker.color.b = b_func(val)
        marker.color.a = 1.0
        #marker.pose = pose
        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE

        if self._interactive:
            int_marker.controls.append(copy.deepcopy(control))
            # add the control to the interactive marker
            int_marker.controls.append(control);

        # add menu control
        menu_control = InteractiveMarkerControl()

        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True
        
        menu_control.markers.append( marker) #makeBox(int_marker) )
        int_marker.controls.append(menu_control)

        return int_marker

    def create_roi_marker(self, roi, soma_type, pose, points):
        #print "POINTS: " + str(points)
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
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
        control.always_visible = True
        control.markers.append( marker )

        int_marker.controls.append(control )
        
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


    
        

if __name__=="__main__":

    # TODO: add list command
    
    parser = argparse.ArgumentParser(prog='soma_roi.py')
    parser.add_argument("map", nargs=1, help='Name of the used 2D map')
    parser.add_argument("conf", nargs=1, help='Name of the object configuration')
    parser.add_argument('-t', metavar='config-file')
                        
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    
    rospy.init_node("soma")
    rospy.loginfo("Running SOMA (map: %s, conf: %s, types: %s)", args.map[0], args.conf[0], args.t)
    SOMAROIManager(args.map[0], args.conf[0],args.t)
    


