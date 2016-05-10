#!/usr/bin/env python
import roslib; roslib.load_manifest("soma_utils")
import rospy
from rospkg import RosPack

from soma_manager.srv import SOMA2QueryObjs, SOMA2QueryObjsRequest

import sys
import argparse
import json
import time
from datetime import datetime as dt

class SOMAROIAnalyzer():

    def __init__(self, config_file=None):    
        
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
            path = rp.get_path('viper_ros') + '/config/'
            filename = 'default.json'
            self._config_file=path+filename

        rospy.loginfo("Use KB at: %s", self._config_file)
        self._init_kb()


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
            rospy.logerr("ROI is not kmown: %s", len(roi_id))
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
            rospy.logerr("ROI is not kmown: %s", len(roi_id))
            return pos_res, neg_res
        

        if "pos_objects" in self.kb[roi_id]: 
            pos_objs = self.kb[roi_id]["pos_objects"]

        if "neg_objects" in self.kb[roi_id]: 
            neg_objs = self.kb[roi_id]["neg_objects"]

        for idx, o in enumerate(res.objects):
            obj = res.objects[idx].type
            if obj in pos_objs or (pos_objs == [] and obj not in neg_objs):
                pos_res.append(obj)
            
            elif obj in neg_objs or (neg_objs == [] and obj not in pos_objs):
                neg_res.append(obj)

        return pos_res, neg_res


if __name__ == '__main__':
    rospy.init_node("soma_roi_analyzer")

    parser = argparse.ArgumentParser(prog='soma_roi_analyzer.py')
    parser.add_argument("roi_id", nargs=1, help='ROI ID')
    parser.add_argument("start_time", nargs=1, help='Start time (unix timestamp)')
    parser.add_argument("end_time", nargs=1, help='End time (unix timestamp)')
    parser.add_argument('-kb', metavar='ROI-object KB')

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    # INPUT ROI ID, TIME INTERVAL (start, end) 
    roi_id = args.roi_id[0]
    start = int(args.start_time[0])
    end =   int(args.end_time[0])

    if  start < 0 or end < 0:
        rospy.logerr("Start and end times must be positive (unix timestamps)")
        sys.exit()

    if end <= start:
        rospy.logwarn("end < start: set end to NOW")
        end = 0

    # INIT ANALYSZER AND LOAD ROI-OBJ KB
    sra = SOMAROIAnalyzer(args.kb)
    
    # RETRIEVE ALL OBJECTS IN ROI IN TIME INTERVAL
    objs = sra.get_objects(roi_id, start, end)
   
    # CHECK WHETHER THERE ARE ANY ALLOWED OR NOT ALLOWED OBJECTS IN THE ROI
    pos_objs, neg_objs = sra.analyze(roi_id, objs)
    
    name = sra.get_roi_name(roi_id) 
    startdate = dt.fromtimestamp(int(start))
    enddate = dt.fromtimestamp(int(end))
    
    # GENERATE REPORT (TODO: generate blog entry or whatever)
    print
    print 80 * "="
    print "Region:", name
    print "Start :", startdate
    if int(end) > 0:
        print "End   :", enddate
    else:
        
        print "End   :", dt.fromtimestamp(int(time.time())), "(now)"
    print
    print "POSITIVE objects:", len(pos_objs) , pos_objs
    print "NEGATIVE objects: ", len(neg_objs) , neg_objs
    print 80 * "="
    print
