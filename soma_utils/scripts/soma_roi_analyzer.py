#!/usr/bin/env python
import roslib; roslib.load_manifest("soma_utils")
import rospy
from rospkg import RosPack

from sensor_msgs.msg import Image

from soma_manager.srv import SOMA2QueryObjs, SOMA2QueryObjsRequest
from soma_io.observation import Observation
from soma_io.state import World, Object

from mongodb_store.message_store import MessageStoreProxy
from robblog.msg import RobblogEntry
import robblog.utils

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


import sys
import argparse
import json
import time
from datetime import datetime as dt

class SOMAROIAnalyzer():

    def __init__(self, config_file=None, blog=None):    

        if blog:
            self.blog_collection = blog
        else:
            self.blog_collection = 'soma_blog'
        
        soma_srv_name = '/soma2/query_db'
        rospy.loginfo("Waiting for SOMA query service...")
        rospy.wait_for_service(soma_srv_name)
        rospy.loginfo("Done")        
        self.soma_srv = rospy.ServiceProxy(soma_srv_name, SOMA2QueryObjs)

        if config_file:
            self._config_file = config_file
        else:
            # default file
            rp = RosPack()
            path = rp.get_path('soma_utils') + '/config/'
            filename = 'default.json'
            self._config_file=path+filename

        rospy.loginfo("Use KB at: %s", self._config_file)
        self._init_kb()

        self.pub = rospy.Publisher('/soma_report', Image, queue_size=1)
        

    def _init_kb(self):
        # read KB from config 
        with open(self._config_file) as config_file:
            config = json.load(config_file)
            self.kb = config

    def get_objects(self, roi_id, start, end):

        try:
            req = SOMA2QueryObjsRequest()
            req.query_type = 0 
            req.useroi = True
            req.roi_id = str(roi_id)
            req.usedates = True
            req.lowerdate = int(start) * 1000 # MongoDB requires time in miliseconds
            req.upperdate = int(end) * 1000 # MongoDB requires time in miliseconds

            rospy.loginfo("Requesting objects")
            res = self.soma_srv(req)
            rospy.loginfo("Received objects: %s", len(res.objects))
            
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)

        return res

    def get_roi_name(self, roi_id):
        if roi_id not in self.kb:
            rospy.logerr("ROI is not kmown: %s", roi_id)
            return "UNKNOWN-ROI"

        name = "UNNAMED-ROI"
        if "name" in self.kb[roi_id]:
            name = sra.kb[roi_id]["name"]
        return name
            
    def analyze(self, roi_id, res):

        pos_objs = []
        neg_objs = []
        
        pos_res = []
        neg_res = []

        # COUNT POS/NEG OBJECTS
        if roi_id not in self.kb:
            rospy.logerr("ROI is not kmown: %s", roi_id)
            return pos_res, neg_res
        

        if "pos_objects" in self.kb[roi_id]: 
            pos_objs = self.kb[roi_id]["pos_objects"]

        if "neg_objects" in self.kb[roi_id]: 
            neg_objs = self.kb[roi_id]["neg_objects"]

        for idx, o in enumerate(res.objects):
            obj = res.objects[idx]
            if obj.type == "person":
                continue
            
            if obj.type in pos_objs or (pos_objs == [] and obj.type not in neg_objs):
                pos_res.append(obj)
            
            elif obj.type in neg_objs or (neg_objs == [] and obj.type not in pos_objs):
                neg_res.append(obj)

        return pos_res, neg_res

        
    def gen_blog_entry(self, roi_id, start, end, pos_objs, neg_objs):

        if int(end) > 0:
            enddate = str(self.enddate)
        else:
            enddate = str(dt.fromtimestamp(self.end_))

        body = '### OBJECT REPORT\n\n'
        body += '- **Region:** ' + self.name + '\n\n'
        body += '- **Startime:** ' + str(self.startdate) + '\n\n'
        body += '- **Endtime:** '  + enddate  + '\n\n'
        body += '- **Summary**: <font color="green">ALLOWED ITEMS (' + str(len(pos_objs)) + ')</font>, <font color="red">NOT-ALLOWED ITEMS (' + str(len(neg_objs)) + ')</font>\n\n'


        # # Create some blog entries
        msg_store = MessageStoreProxy(collection=self.blog_collection)
        robblog_path = roslib.packages.get_pkg_dir('soma_utils') 

        world_model = World(server_host='localhost',server_port=62345)

        print "POS_OBJS:", len(pos_objs)
        for idx, obj in enumerate(pos_objs):        
            try:
                o = world_model.get_object(obj.id)
                print idx, obj.id, obj.type
                observations = o._observations
                obs = observations[0]
            except:
                rospy.logerr("Object not in world model: %s, %s", idx, obj.id)

            # CHECK that objservation is within timeframe
            if start < int(obs.stamp) and int(obs.stamp) < self.end_:

                rgb_mask = obs.get_message("rgb_mask")
                bridge = CvBridge()
                im = bridge.imgmsg_to_cv2(rgb_mask, desired_encoding="bgr8")
                imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
                ret,thresh = cv2.threshold(imgray,127,255,0)
                contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

                full_scene = obs.get_message("/head_xtion/rgb/image_rect_color")
                bridge = CvBridge()
                cv_image = bridge.imgmsg_to_cv2(full_scene, desired_encoding="bgr8")

                cv2.drawContours(cv_image,contours,-1,(0,255,0),3)
                full_scene_contour = bridge.cv2_to_imgmsg(cv_image)

                #success = cv2.imwrite(obj.id+'.jpeg',cv_image)
                img_id = msg_store.insert(full_scene_contour)
                body += '<font color="green">ALLOWED ITEM (' + str(idx+1) + '/'+ str(len(pos_objs)) + '):</font> ' + obj.type + '\n\n![My helpful screenshot](ObjectID(%s))\n\n' % img_id
                
            else:
                rospy.logerr("Ignore old observation for object: %s", obj.id)

        print "NEG_OBJS:", len(neg_objs)
        for idx, obj in enumerate(neg_objs):        
            try:
                o = world_model.get_object(obj.id)
                print idx, obj.id, obj.type
                observations = o._observations
                obs = observations[0]
            except:
                rospy.logerr("Object not in world model: %s, %s", idx, obj.id)

            # CHECK that objservation is within timeframe
            if start < int(obs.stamp) and int(obs.stamp) < self.end_:

                rgb_mask = obs.get_message("rgb_mask")
                bridge = CvBridge()
                im = bridge.imgmsg_to_cv2(rgb_mask, desired_encoding="bgr8")
                imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
                ret,thresh = cv2.threshold(imgray,127,255,0)
                contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

                full_scene = obs.get_message("/head_xtion/rgb/image_rect_color")
                bridge = CvBridge()
                cv_image = bridge.imgmsg_to_cv2(full_scene, desired_encoding="bgr8")

                cv2.drawContours(cv_image,contours,-1,(0,255,0),3)
                full_scene_contour = bridge.cv2_to_imgmsg(cv_image)

                #success = cv2.imwrite(obj.id+'.jpeg',cv_image)
                img_id = msg_store.insert(full_scene_contour)
                body += '<font color="red">NOT-ALLOWED ITEM (' + str(idx+1) + '/'+ str(len(neg_objs)) + '):</font> ' + obj.type + '\n\n![My helpful screenshot](ObjectID(%s))\n\n' % img_id
                
            else:
                rospy.logerr("Ignore old observation for object: %s", obj.id)

            e = RobblogEntry(title=self.name + " (" + enddate + ")", body= body )
            msg_store.insert(e)


    def gen_cmdline_report(self, roi_id, start, end, pos_objs, neg_objs):

        # GENERATE REPORT 
        print
        print 80 * "="
        print "Region:", self.name
        print "Start :", self.startdate
        if int(end) > 0:
            print "End   :", self.enddate
        else:
            print "End   :", dt.fromtimestamp(self.end_), "(now)"
            print
        print "POSITIVE objects:", len(pos_objs)
        for idx, obj in enumerate(pos_objs):
                print idx, obj.type 
        print "NEGATIVE objects:", len(neg_objs)
        for idx, obj in enumerate(neg_objs):
                print idx, obj.type
        print 80 * "="
        print



if __name__ == '__main__':
    rospy.init_node("soma_roi_analyzer")

    parser = argparse.ArgumentParser(prog='soma_roi_analyzer.py')
    parser.add_argument("roi_id", nargs=1, help='ROI ID')
    parser.add_argument("start_time", nargs=1, help='Start time (unix timestamp)')
    parser.add_argument("end_time", nargs=1, help='End time (unix timestamp)')
    parser.add_argument('-kb', metavar='<ROI-object-KB>')
    parser.add_argument('-blog', metavar='<blog-store>')

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    # INPUT ROI ID, TIME INTERVAL (start, end) 
    roi_id = args.roi_id[0]
    start = int(args.start_time[0])
    end =   int(args.end_time[0])

    # INIT ANALYSZER AND LOAD ROI-OBJ KB
    sra = SOMAROIAnalyzer(args.kb, args.blog)
    sra.end_= end
    
    if  start < 0 or end < 0:
        rospy.logerr("Start and end times must be positive (unix timestamps)")
        sys.exit()

    if end <= start:
        rospy.logwarn("end < start: set end to NOW")
        end = 0
        sra.end_  = int(time.time())

    
    # RETRIEVE ALL OBJECTS IN ROI IN TIME INTERVAL
    objs = sra.get_objects(roi_id, start, end)
   
    # CHECK WHETHER THERE ARE ANY ALLOWED OR NOT ALLOWED OBJECTS IN THE ROI
    pos_objs, neg_objs = sra.analyze(roi_id, objs)

    sra.name = sra.get_roi_name(roi_id) 
    sra.startdate = dt.fromtimestamp(int(start))
    sra.enddate = dt.fromtimestamp(int(end))
        
    if args.blog != None:
        sra.gen_blog_entry(roi_id, start, end, pos_objs, neg_objs)
    else:
        sra.gen_cmdline_report(roi_id, start, end, pos_objs, neg_objs)

        
