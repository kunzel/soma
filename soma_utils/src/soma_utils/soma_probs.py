#!/usr/bin/env python

import roslib; roslib.load_manifest("soma_utils")
import sys
import rospy
from rospkg import RosPack
import json
import argparse
import random
import copy

from  soma_utils import SOMAUtils

from tabulate import tabulate

from threading import Timer

from ros_datacentre.message_store import MessageStoreProxy

from geometry_msgs.msg import Point

from soma_msgs.msg import SOMAObject
from soma_msgs.msg import SOMAROIObject
    
    
class SOMAProbs():

    def __init__(self, config_file=None):

        self._map = dict()
        self._obj = dict()
        self._roi = dict()

        self._roi_cnt = dict()
        self._obj_cnt = dict()

        self._num_of_objs = 0
        self._num_of_rois = 0

        self._soma_utils = dict()
        
        self._obj_msg_store=MessageStoreProxy(collection="soma")
        self._roi_msg_store=MessageStoreProxy(collection="soma_roi")

        if config_file:
            self._config_file = config_file
        else:
            # default file
            rp = RosPack()
            path = rp.get_path('soma_utils') + '/config/'
            filename = 'maps.json'
            self._config_file=path+filename
        self._init_maps()

    def _init_maps(self):
        # read from config in soma_objects 
        
        with open(self._config_file) as config_file:
            config = json.load(config_file)
            
            for k, v in config.iteritems():
                self._map[k] = v
                obj_meta = self._retrieve_objects(self._map[k]['obj'])
                if obj_meta: 
                    self._obj[k] = obj_meta
                map_roi = self._retrieve_rois(self._map[k]['roi'])
                if map_roi: 
                    self._roi[k] = map_roi

                soma_util = SOMAUtils(self._map[k]['obj'][0],self._map[k]['obj'][1],
                                      self._map[k]['roi'][0],self._map[k]['roi'][1])

                self._soma_utils[k] = soma_util

        
        for k, obj_lst in self._obj.iteritems():
            for o in obj_lst:
                if o[0].type not in self._obj_cnt:
                    self._obj_cnt[o[0].type] = 1
                else: 
                    self._obj_cnt[o[0].type] += 1
                
                    
        for k, map_roi in self._roi.iteritems():
            for roi_id, roi  in map_roi.iteritems():
                if roi[0].type not in self._roi_cnt:
                    self._roi_cnt[roi[0].type] = 1
                else:
                    self._roi_cnt[roi[0].type] += 1
                    
        for k, v in self._obj_cnt.iteritems():
            self._num_of_objs += v
            
        for k, v in self._roi_cnt.iteritems():
            self._num_of_rois += v
                
    def _retrieve_objects(self, map_config):
        return self._obj_msg_store.query(SOMAObject._type,
                                  message_query={"map": map_config[0],
                                                 "config": map_config[1]})
                                         
    def _retrieve_rois(self, map_config):
        rois = self._roi_msg_store.query(SOMAROIObject._type,
                                         message_query={"map": map_config[0],
                                                        "config": map_config[1]})    

        map_roi = dict()
        for roi, _meta in rois:
            if not roi.roi_id in map_roi:
                map_roi[roi.roi_id] = list()
                map_roi[roi.roi_id].append(roi)
            else:
                map_roi[roi.roi_id].append(roi)

        return map_roi


    def get_rois(self, roiT):

        rois = dict()
        
        for map_key, map_roi in self._roi.iteritems():
            for roi_id in map_roi:
                if map_roi[roi_id][0].type == roiT:
                    if map_key not in rois:
                        rois[map_key] = list()
                        rois[map_key].append(roi_id)
                    else:
                        rois[map_key].append(roi_id)

                    #print map_key, roi_id, len(self._soma_utils[map_key]._get_polygon(roi_id).points)

        return rois

    def cnt_all_obj_inside_roi(self, roiT):

        rois = self.get_rois(roiT)

        objT_cnt = dict()
        
        for k, v in self._obj.iteritems():
            for obj in self._obj[k]:


                if k in rois:
                    for roi_id in rois[k]:
                        poly = self._soma_utils[k]._get_polygon(roi_id)

                        point = Point()
                        point.x = obj[0].pose.position.x
                        point.y = obj[0].pose.position.y

                        if self._soma_utils[k]._inside(point, poly):
                        #print k, obj[0].id, obj[0].type
                            if obj[0].type not in objT_cnt:
                                objT_cnt[obj[0].type] = 1
                            else:
                                objT_cnt[obj[0].type] += 1
        return objT_cnt


    
    def p_obj(self, obj):
        return float(self._obj_cnt[obj]) / float(self._num_of_objs)

    def p_roi(self, roi):
        return float(self._roi_cnt[roi])/float(self._num_of_rois)
    
    def p_obj_given_roi(self, obj, roi):
        obj_cnt = self.cnt_all_obj_inside_roi(roi)

        _lambda = 0.5
        
        denominator = 0.0
        
        if obj in obj_cnt:
            numerator = obj_cnt[obj] + _lambda
        else:
            numerator = _lambda

        for t in obj_cnt:
            denominator += obj_cnt[t]

        denominator += (len(self._obj_cnt) *  _lambda )
        
        return float(numerator) / float(denominator)

    def p_roi_given_obj(self, roi, obj):
        # Put your code here!

        num = self.p_obj_given_roi(obj, roi) * self.p_roi(roi)
        den = 0.0 
        for r in self._roi_cnt:
            den +=  self.p_obj_given_roi(obj, r) * self.p_roi(r)

        prob = num / den

        if prob > 1:
            print self.p_obj_given_roi(obj, roi), self.p_roi(roi), self.p_obj(obj)
            print 'ERROR!!!!!!!!!!!!!!!!!!!!!!!', roi, obj
        return prob

    def p_obj_and_roi(self, obj, roi):
        # Joint probability distribution
        # Put your code here! 
        return self.p_roi_given_obj(roi,obj) * self.p_obj(obj) 
    
    def list(self):

        table = []
        obj_table = []
        roi_table = []

        for roi  in self._roi_cnt:
            roi_table.append([roi , self.p_roi(roi)])

        total_por = 0
        total_pro = 0
        total_por_joint = 0
        for obj in self._obj_cnt:
            obj_table.append([obj , self.p_obj(obj)])
            for roi  in self._roi_cnt:
                total_por += self.p_obj_given_roi(obj,roi)
                total_pro += self.p_roi_given_obj(roi, obj)
                total_por_joint += self.p_obj_and_roi(obj,roi)
                
                table.append([obj ,
                              roi,
                              self.p_obj_given_roi(obj,roi),
                              self.p_roi_given_obj(roi,obj),
                              self.p_obj_and_roi(obj,roi)])

        print
        print tabulate(obj_table, headers=['Object', 'P(O)'],tablefmt='rst')
        print
        print tabulate(roi_table, headers=['ROI', 'P(R)'],tablefmt='rst')
        print
        print tabulate(table, headers=['Object', 'ROI', 'P(O|R)', 'P(R|O)', 'P(O,R)'],tablefmt='rst')

        print "Total: ", total_por/len(self._roi_cnt), total_pro/len(self._obj_cnt), total_por_joint
        #print 'Office', self.cnt_all_obj_inside_roi('Office')
        #print 'Library', self.cnt_all_obj_inside_roi('Library')
        #print 'Resource room', self.cnt_all_obj_inside_roi('Resource room')
        
if __name__=="__main__":

    parser = argparse.ArgumentParser(prog='soma_probs.py')
    parser.add_argument("cmd", nargs=1, choices=['list'], help='Name of the command')

    args = parser.parse_args()
    
    rospy.init_node("soma_prob")

    rospy.loginfo("Running SOMA probs (cmd: %s)", args.cmd[0])
    soma = SOMAProbs()
    
    if args.cmd[0] == 'list':
        soma.list()
        


