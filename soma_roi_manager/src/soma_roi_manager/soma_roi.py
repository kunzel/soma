#!/usr/bin/env python

import roslib; roslib.load_manifest("soma_roi_manager")
import rospy
from rospkg import RosPack
import json
import argparse
import random
import copy
import sys
import math

from threading import Timer

from mongodb_store.message_store import MessageStoreProxy
from soma_geospatial_store.geospatial_store import GeoSpatialStoreProxy
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

from soma_map_manager.srv import *

from soma2_msgs.msg import SOMA2ROIObject
from soma2_msgs.msg import SOMA2OccupancyMap
from bson.objectid import ObjectId

def trapezoidal_shaped_func(a, b, c, d, x):
    min_val = min(min((x - a)/(b - a), float(1.0)), (d - x)/(d - c))
    return max(min_val, float(0.0))

def coords_to_lnglat(x, y):
        earth_radius = 6371000.0 # in meters
        lng = 90 - math.degrees(math.acos(float(x) / earth_radius))
        lat = 90 - math.degrees(math.acos(float(y) / earth_radius))
        return [lng , lat]


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

class SOMAROIManager():

    def __init__(self, soma_conf, config_file=None):

        #self.soma_map = soma_map
        #self.soma_map_name = soma_map_name
        self.map_unique_id = -1
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
        self._soma_obj_markers = dict()

        self._interactive = True

        self._msg_store=MessageStoreProxy(database="soma2data", collection="soma2_roi")

        self._server = InteractiveMarkerServer("soma2_roi")

        self._init_types()


        #print resp

        # Get the SOMA2 map name and unique id
        resp = self._init_map()
        self.soma_map_name = resp.map_name
        self.map_unique_id = resp.map_unique_id

        print "Map name: ",self.soma_map_name," Unique ID: ",self.map_unique_id

        self._init_menu()

        self.load_objects()

        rospy.spin()

    def _init_map(self):
        print "Waiting for the map info from soma2_map_manager"
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

    def _init_types(self):
        # read from config in soma_objects

        with open(self._config_file) as config_file:
            config = json.load(config_file)

            self.mesh = dict()
            for k, v in config['roi'].iteritems():
                self.mesh[k] = v

    ## Initialize the right-click menu
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

    # Add a roi callback
    def _add_cb(self, feedback):
        rospy.loginfo("Add ROI: %s", self.menu_item[feedback.menu_entry_id])
        pose = feedback.pose
        pose.position.x = pose.position.x+0.5
        pose.position.y = pose.position.y+0.5
        self.add_object(self.menu_item[feedback.menu_entry_id], pose)

    # Delete a roi callback
    def _del_cb(self, feedback):
        rospy.loginfo("Delete ROI: %s", feedback.marker_name)
        roi_and_count = feedback.marker_name.split('_')
        roi = self._soma_obj_roi[roi_and_count[0]]
        rospy.loginfo("ROI Info %s",roi)
        self.delete_object(str(roi),feedback.marker_name,True)

    # Add a point
    def _add_point_cb(self, feedback):

        #This is the object that we are pressing (feedback) so
        #that we can get the marker name etc..
        rospy.loginfo("Add point: %s", feedback.marker_name)
        roi_and_count = feedback.marker_name.split('_')
        #This is the roi that we are adding the point to
        roi = self._soma_obj_roi[roi_and_count[0]]
        #print "ROI is ", roi
        # This is the type of the roi (Office, Library, etc..)
        t   = self._soma_obj_type[roi_and_count[0]]

        # Get the pose and create the new object a little away
        pose = feedback.pose
        pose.position.x = pose.position.x+0.5
        pose.position.y = pose.position.y+0.5
        ######################################################

        # Add object
        self.add_object(t, pose, roi)

        # Draw the ROI
        self.draw_roi(roi)

    def _del_point_cb(self, feedback):

        rospy.loginfo("Delete point: %s", feedback.marker_name)

        roi_and_count = feedback.marker_name.split('_')

        # Get the poses of the markers that belong roi
        poses = self._soma_obj_pose[roi_and_count[0]]

        # Find out which marker wants to be deleted
        markerindex = roi_and_count[1]
        roi = roi_and_count[0]

        marker = self._soma_obj_markers[roi][markerindex]
        keys = self._soma_obj_markers[roi].keys();


        #this was the last marker of this roi so we should delete the roi
        if(len(keys)==1):
            self.delete_object(roi,feedback.marker_name,True)
            return

        # We only want to delete particular marker
        del self._soma_obj_markers[roi][markerindex]

        # We delete the obj pose array to reinitialize
        del self._soma_obj_pose[roi]
        self._soma_obj_pose[roi] = list()

        #print self._soma_obj_markers[str(roi_and_count[0])]
        # Reinitializing the pose array
        for key,amarker in self._soma_obj_markers[roi].iteritems():
            self._soma_obj_pose[roi].append(amarker.pose)



        # Using the del statement
        self.delete_object(roi,feedback.marker_name,False)


        #self.delete_object(feedback.marker_name)
        #roi = self._soma_obj_roi[feedback.marker_name]
        self.draw_roi(roi)

    def _update_poly(self, feedback):
        return

    def _update_cb(self, feedback):
        p = feedback.pose.position
       # print "Feedback is ",feedback
        #print "Marker " + feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y)
        #self._soma_obj_pose[feedback.marker_name] = feedback.pose
        roi_and_count = feedback.marker_name.split('_')

        del self._soma_obj_pose[roi_and_count[0]]
        self._soma_obj_pose[roi_and_count[0]] = list()

        roi = roi_and_count[0]
        markerindex = roi_and_count[1]

        self._soma_obj_markers[roi][markerindex].pose = feedback.pose

        #print self._soma_obj_markers[str(roi_and_count[0])]

        for key,amarker in self._soma_obj_markers[roi].iteritems():
            self._soma_obj_pose[roi].append(amarker.pose)


        #roi = self._soma_obj_roi[roi_and_count[0]]
        #print "ROI is",roi
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

    #retrieve the objects from DB
    def _retrieve_objects(self):

        objs = self._msg_store.query(SOMA2ROIObject._type, message_query={"map_name": self.soma_map_name})
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

    def load_objects(self):

        # this is the array for roi ids
        self._soma_obj_roi_ids = dict()

        #get objects from db
        objs = self._retrieve_objects()

        # if collection is empty insert initial object
        if not objs:
            pose = Pose()
            self.add_object('Office', pose)
            return

        # otherwise, load all object from collection
        for o,om  in objs:
##            print om
##            print o.roi_id
##            print self._soma_obj_roi_ids
            self._soma_obj_ids[o.id] = om['_id']
            self._soma_obj_msg[o.id] = o
            if o.roi_id in self._soma_obj_roi_ids:
                self._soma_obj_roi_ids[o.roi_id].append(o.id)
            else:
                self._soma_obj_roi_ids[o.roi_id] = list()
                self._soma_obj_roi_ids[o.roi_id].append(o.id)

            self._soma_obj_roi[o.id] = o.roi_id
            self._soma_obj_type[o.id] = o.type
            self._soma_obj_pose[o.id] = o.posearray.poses
            self._soma_obj_markers[o.id] = dict()

            for pose in o.posearray.poses:
                self.load_object(o.id, o.roi_id, o.type, pose)

        self.draw_all_roi()

    def draw_all_roi(self):

        for key  in self._soma_obj_roi_ids:
            self.draw_roi(key)

    def undraw_all_roi(self):

        for key  in self._soma_obj_roi_ids:
            self.undraw_roi(key)

    def draw_roi(self, roi):
    #    print"ROI IDS ",self._soma_obj_roi_ids
        roicp = roi
        v = self._soma_obj_roi_ids[str(roicp)]
    #    print "V is ",v
        t = self._soma_obj_type[v[0]]
        p = self._soma_obj_pose[v[0]]
        cc = 0
      #  print "t is ",t," p is ", p
        for pose in p:
           # print "This is the pose: ", pose
            int_marker = self.create_roi_marker(roi, t, pose, p,cc)
            self._server.erase("ROI-" + str(roicp))
            self._server.applyChanges()
            self._server.insert(int_marker, self._update_poly)
            self._server.applyChanges()
            cc = cc+1

    def undraw_roi(self, roi):
        self._server.erase("ROI-" + roi)
        self._server.applyChanges()

    def load_object(self, soma_id, roi, soma_type, pose):

        #default marker count value
        markerno = 1

        #get the max key value
        if self._soma_obj_markers[str(soma_id)].keys():
            maxkey = max(self._soma_obj_markers[str(soma_id)].keys(), key=int)
            markerno = int(maxkey)+1;
    #    print "marker no ",markerno

       # print self._soma_obj_markers[str(soma_id)]
       # print str(soma_id)
        int_marker = self.create_object_marker(soma_id, roi, soma_type, pose, markerno)

        self._soma_obj_markers[str(soma_id)][str(markerno)] = int_marker

        #call the update_cb when marker moves
        self._server.insert(int_marker, self._update_cb)

        name = soma_id+'_'+str(markerno)

        self.menu_handler.apply( self._server, name )

        self._server.applyChanges()

        #print self._soma_obj_markers[str(soma_id)].keys()



#soma_type = Office, Kitchen, etc, Pose is position
    def add_object(self, soma_type, pose, roi_id=None):
        # todo: add to mongodb

        #create a SOMA2ROI Object
        soma_obj = SOMA2ROIObject()

        print roi_id

        # a new roi
        if roi_id == None:

            #soma_id is an id for the soma object like 1,2,3,4. It updates itself from the db if there are existing objects
            soma_id = self._next_id()

            #soma_roi_id is acutally the roi number. Is it 1,2,3,4? Multiple soma objects can have the same roi id
            soma_roi_id = self._next_roi_id()

            roi_id = soma_roi_id
            print soma_roi_id

            soma_obj.id = str(soma_id)
            soma_obj.roi_id = str(soma_roi_id)
            soma_obj.map_name = str(self.soma_map_name)
            soma_obj.map_unique_id = str(self.map_unique_id)
            soma_obj.config = str(self.soma_conf)
            soma_obj.type = soma_type
            soma_obj.posearray.poses.append(pose)
            soma_obj.header.frame_id = '/map'
            soma_obj.header.stamp = rospy.get_rostime()
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
                #soma_obj.posearray.poses.append(pose)

                self._soma_obj_pose[soma_obj.id].append(pose)# = soma_obj.posearray.poses
                soma_obj.posearray.poses = self._soma_obj_pose[soma_obj.id]

                self.insert_geo_json(soma_obj.roi_id,soma_obj)

                #print soma_obj.geoposearray

                #print soma_obj
                _id = self._soma_obj_ids[soma_obj.id]

                try:
                    _newid =  self._msg_store.update_id(_id,soma_obj)
                    rospy.loginfo("ROI Store: updated roi (%s %s)" % (soma_obj.type, soma_obj.roi_id) )
                except:
                    soma_obj.geotype = ''
                    soma_obj.geoposearray = []
                    rospy.logerr("The polygon of %s %s is malformed (self-intersecting) => Please update geometry." % (soma_obj.type, soma_obj.roi_id))


                soma_id = soma_obj.id

                self._soma_obj_msg[soma_obj.id] = soma_obj


        self.load_object(str(soma_id), soma_obj.roi_id, soma_type, pose)



    def insert_geo_json(self, soma_roi_id,soma_obj):

        geo_json = self.geo_json_from_soma_obj(soma_obj)
        if geo_json:
            soma_obj.geotype = geo_json['loc']['type']
            soma_obj.geoposearray = geo_json['loc']['coordinates']
        else:
            soma_obj.geotype = ''
            soma_obj.geoposearray = PoseArray()

    def geo_json_from_soma_obj(self, soma_obj):

        geo_json = {}
        geo_json['soma_roi_id'] = soma_obj.roi_id
        geo_json['soma_map_name'] = soma_obj.map_name
        geo_json['soma_config'] = soma_obj.config
        geo_json['type'] = soma_obj.type


        if len(self._soma_obj_pose[soma_obj.roi_id]) < 3:
            rospy.logerr("GS Store: %s %s, less then 3 points => Add more points or delete ROI." % (soma_obj.type, soma_obj.roi_id) )
            return None
        coordinates = PoseArray()
        for pose in self._soma_obj_pose[soma_obj.roi_id]:
            p = copy.deepcopy(pose)
            res = coords_to_lnglat(p.position.x, p.position.y)
            p.position.x = res[0]
            p.position.y = res[1]
            coordinates.poses.append(p)

        p = copy.deepcopy(self._soma_obj_pose[soma_obj.roi_id][0])

        res = coords_to_lnglat(p.position.x, p.position.y)
        p.position.x = res[0]
        p.position.y = res[1]
        coordinates.poses.append(p)

        #print coordinates


        geo_json['loc'] = {'type': 'Polygon',
                           'coordinates': coordinates}
        return geo_json

    def delete_object(self, soma_id, marker_name, should_delete_roi):
        # todo: delete from mongodb

        _id = self._soma_obj_ids[soma_id]
        msg = self._soma_obj_msg[soma_id]

        if(should_delete_roi):
            try:
                self._msg_store.delete(str(_id))
            except:
                rospy.logerr("Error deleting ROI %s." % (str(soma_id)) )
                return

            del self._soma_obj_ids[soma_id]
            del self._soma_obj_msg[soma_id]
            markers = self._soma_obj_markers[soma_id]
            for key,amarker in markers.iteritems():
                self._server.erase(amarker.name)
                self._server.applyChanges()
            self.undraw_roi(soma_id)
            del self._soma_obj_markers[soma_id]
            del self._soma_obj_pose[soma_id]
            del self._soma_obj_roi_ids[soma_id]
            return



        new_msg = copy.deepcopy(msg)

        new_msg.posearray.poses = self._soma_obj_pose[str(soma_id)]
        self.insert_geo_json(soma_id,new_msg)


        try:
            self._msg_store.update_id(_id,new_msg)
        except:
            rospy.logerr("Error deleting Marker %s." % (marker_name) )
            return

    # self._msg_store.delete(str(_id))

        self._server.erase(marker_name)
        self._server.applyChanges()


    def update_object(self, feedback):
        rospy.loginfo("Updated marker: %s", feedback.marker_name)
        roi_and_count = feedback.marker_name.split('_')
        _id = self._soma_obj_ids[roi_and_count[0]]
        msg = self._soma_obj_msg[roi_and_count[0]]

        new_msg = copy.deepcopy(msg)

##        for amarker in self._soma_obj_markers[roi_and_count[0]]:
##            if amarker.description:
##                print "Hello"

        roi = roi_and_count[0]
        #new_msg.posearray.poses[markerindex] = feedback.pose
        #new_msg.posearray.poses.append(feedback.pose)

        new_msg.posearray.poses = self._soma_obj_pose[roi]

        #self._soma_obj_pose[roi_and_count[0]] = new_msg.posearray.poses;

        self.insert_geo_json(roi,new_msg)

        try:
            self._msg_store.update_id(_id, new_msg)
            rospy.loginfo("ROI %s updated successfully" %(roi))
        except:
            rospy.logerr("Error updating ROI %s" %(roi))



    def create_object_marker(self, soma_obj, roi, soma_type, pose,markerno):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = soma_obj+'_'+str(markerno)
        int_marker.description = soma_type + ' (' + roi +'_'+str(markerno)+  ')'
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

    # This part draws the line strips between the points
    def create_roi_marker(self, roi, soma_type, pose, points, count):
        #print "POINTS: " + str(points)
        #points are all the points belong to that roi, pose is one of the points
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = "ROI-" + str(roi)
       # print "Marker name: ", int_marker.name
        int_marker.description = str(roi)
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
            pose = point#self._soma_obj_pose[point]

            p.x = pose.position.x - int_marker.pose.position.x
            p.y = pose.position.y - int_marker.pose.position.y
            marker.points.append(p)

        p = Point()
        pose = points[0]
        p.x = pose.position.x - int_marker.pose.position.x
        p.y = pose.position.y - int_marker.pose.position.y
        marker.points.append(p)

        return int_marker





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
