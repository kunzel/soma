#!/usr/bin/env python

import math
import json
import argparse
import rospy
from rospkg import RosPack
from soma_pcl_segmentation.srv import *
from octomap_msgs.msg import Octomap

from semantic_segmentation.srv import LabelIntegratedPointCloud, LabelIntegratedPointCloudRequest, LabelIntegratedPointCloudResponse

from semantic_map_publisher.srv import ObservationOctomapServiceRequest, ObservationOctomapService, ObservationServiceRequest, ObservationService
from sensor_msgs.msg import PointCloud2

class SOMAPCLSegmentationServer():

    def __init__(self, kb_file=None):

        self.octomaps = dict()
        self.pointclouds = dict()
        self.labels = dict()
        self.octomap_keys = dict()
        
        if kb_file:
            self._kb_file = kb_file
        else:
            # default file
            rp = RosPack()
            path = rp.get_path('soma_pcl_segmentation') + '/data/'
            filename = 'object_kb.json'
            self._kb_file=path+filename

        self._init_object_kb()
            
        self._waypoint_service = rospy.Service('soma_probability_at_waypoint', GetProbabilityAtWaypoint, self.get_probability_at_waypoint)
        
        self._view_service    = rospy.Service('soma_probability_at_view', GetProbabilityAtView, self.get_probability_at_view)
        
        rospy.spin()

    def _init_object_kb(self):
        self.obj_types = dict()
        self.obj_labels = dict()
        self.obj_probability = dict()
        self.obj_cost = dict()
        with open(self._kb_file) as kb_file:
            kb = json.load(kb_file)
            for k, v in kb.iteritems():
                self.obj_types[k] = v['type']
                self.obj_labels[k] = v['labels']
                self.obj_probability[k] = v['probability']
                self.obj_cost[k] = v['cost']

    def _init_fake_labels(self, waypoints):
        self.labels = dict()
        for i in range(len(waypoints)):
            import random
            p = random.random()
            p_rest = (1-p) / 5
            self.labels[waypoints[i]] = {"wall" : p, 
                                         "chair/sofa": p_rest, 
                                         "prop":  p_rest,
                                         "table":  p_rest,
                                         "floor": p_rest,
                                         "door":  p_rest}
            print waypoints[i], self.labels[waypoints[i]]
        
    def _get_pointcloud(self, waypoint):
        pointcloud = PointCloud2()
        rospy.loginfo("Waiting for pointcloud service")
        service_name = '/semantic_map_publisher/SemanticMapPublisher/ObservationService'
        rospy.wait_for_service(service_name)
        rospy.loginfo("Done")
        try:
            service = rospy.ServiceProxy(service_name, ObservationService)
            req = ObservationServiceRequest()
            req.waypoint_id = waypoint
            req.resolution = 0.05
            rospy.loginfo("Requesting pointcloud for waypoint: %s", waypoint)
            res = service(req)
            pointcloud = res.cloud
            rospy.loginfo("Received pointcloud: size:%s", len(pointcloud.data))
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
        return pointcloud

        
    def _get_octomap(self, waypoint):
        octomap = Octomap()
        rospy.loginfo("Waiting for octomap service")
        service_name = '/semantic_map_publisher/SemanticMapPublisher/ObservationOctomapService'
        rospy.wait_for_service(service_name)
        rospy.loginfo("Done")
        try:
            service = rospy.ServiceProxy(service_name, ObservationOctomapService)
            req = ObservationOctomapServiceRequest()
            req.waypoint_id = waypoint
            req.resolution = 0.05
            rospy.loginfo("Requesting octomap for waypoint: %s", waypoint)
            res = service(req)
            octomap = res.octomap
            rospy.loginfo("Received octomap: size:%s resolution:%s", len(octomap.data), octomap.resolution)

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
        return octomap


    def _get_labels(self, waypoints):

        rospy.loginfo("Waiting for labelling service")
        service_name = '/semantic_segmentation_node/label_integrated_cloud'
        rospy.wait_for_service(service_name)
        rospy.loginfo("Done")
        service = rospy.ServiceProxy(service_name, LabelIntegratedPointCloud)

        self.label_names = dict()
        self.labels = dict() # store p(l|d)
        self.label_probs = dict()
        self.label_freq = dict()
        self.points = dict()
        
        for waypoint in waypoints:
            try:
                self.labels[waypoint] = dict()
                    
                req = LabelIntegratedPointCloudRequest()
                req.integrated_cloud = self.pointclouds[waypoint]
             
                rospy.loginfo("Requesting labelling for waypoint: %s", waypoint)
                res = service(req)

                rospy.loginfo("Received labels. names:%s", res.index_to_label_name)
                rospy.loginfo("Received labels. freq:%s",  res.label_frequencies)
                rospy.loginfo("Received labels. probs:%s",  len(res.label_probabilities))
                rospy.loginfo("Received labels. points:%s pointsxlabels:%s",  len(res.points), len(res.points)*len(res.index_to_label_name))

                self.label_names[waypoint] = res.index_to_label_name
                self.label_probs[waypoint] = res.label_probabilities
                self.label_freq[waypoint] =  res.label_frequencies
                self.points[waypoint] = res.points
                
                for i in range(len(res.index_to_label_name)):
                    print res.index_to_label_name[i], res.label_frequencies[i]
                    self.labels[waypoint][res.index_to_label_name[i]] = res.label_frequencies[i]


            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s"%e)
            
    def _points_to_keys(self, points, octomap):

        rospy.loginfo("Waiting for octomap points-keys mapping service")
        service_name = '/pcl_octomap_mapper_service/pcl_octomap_mapper'
        rospy.wait_for_service(service_name)
        rospy.loginfo("Done")
        try:
            service = rospy.ServiceProxy(service_name, GetPCLOctomapMapping)
            req = GetPCLOctomapMappingRequest()
            req.octomap = octomap
            req.points = points
            rospy.loginfo("Requesting mapping for points. size:%s", len(points))
            res = service(req)
            keys = res.keys
            rospy.loginfo("Received keys: size:%s", len(keys))

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
        return keys



    def waypoint_probability(self, waypoint, obj):
        # P(d|q) = P(q|d)P(d)/P(q)
        # P(q) is the same for all documents
        # The prior probability of a document P(d) is often treated as uniform across all d
        # and so it can also be ignored
        # results ranked by simply P(q|d)
        p_label_at_waypoint = self.labels[waypoint]
        num = 1.0
        for label in self.obj_labels[obj]:
            num *= math.pow(p_label_at_waypoint[label], self.obj_labels[obj][label])

        den = 0.0
        for w in self.labels.keys():
            p_label_at_waypoint = self.labels[w]
            p = 1.0
            for label in self.obj_labels[obj]:
                p *= math.pow(p_label_at_waypoint[label], self.obj_labels[obj][label])
            den += p

        p_labels = num / den # waypoint_probability
        p_success = self.obj_probability[obj]
        p_scaled = p_labels * p_success

        rospy.loginfo("wp:%s obj:%s p_labels:%s" % (waypoint, obj, p_labels))
        rospy.loginfo("wp:%s obj:%s p_success:%s" % (waypoint, obj, p_success))
        rospy.loginfo("wp:%s obj:%s p_scaled:%s" % (waypoint, obj, p_scaled))
        
        return p_scaled

        

    def view_probability(self, waypoint, obj, keys, values):

         # self.label_names[waypoint] = res.index_to_label_name
         #        self.label_probs[waypoint] = res.label_probabilities
         #        self.points[waypoint] = res.points

        
        probs = dict()
        
        for k in range(len(keys)):
            for i  in range(len(self.octomap_keys[waypoint])):
                if keys[k] == self.octomap_keys[waypoint][i]:
                    for j in range(len(self.label_names[waypoint])):
                        if self.label_names[waypoint][j] not in probs:
                            probs[self.label_names[waypoint][j]] = 0.0
                        probs[self.label_names[waypoint][j]] += self.label_probs[waypoint][i*len(self.label_names[waypoint]) + j]
        #print self.label_names[waypoint]
        #print self.label_freq[waypoint]

        # normalize
        total_sum = sum(probs.values())
        for p in probs:
            probs[p] = probs[p]/total_sum
        #print probs

        p_label_at_view = probs
        num = 1.0
        for label in self.obj_labels[obj]:
            num *= math.pow(p_label_at_view[label], self.obj_labels[obj][label])

        # denominator is not calculated as it is the same for all views
        # view probabilities are normalized in the planning step 
        return num
        
    def get_probability_at_waypoint(self, req):
        rospy.loginfo("Received request: %s", req)
        for waypoint in req.waypoints:
            if waypoint not in self.pointclouds: 
                cloud = self._get_pointcloud(waypoint)
                self.pointclouds[waypoint] = cloud

        # call alex's service
        self._get_labels(req.waypoints)
        #self._init_fake_labels(req.waypoints)
        
        res = GetProbabilityAtWaypointResponse()
        res.probability = []
        res.cost = []

        for waypoint in req.waypoints:
            for obj in req.objects:
                p = self.waypoint_probability(waypoint,obj)
                res.probability.append(p)
                res.cost.append(self.obj_cost[obj])
        rospy.loginfo("Sent response: %s", res)
        return res

    def get_probability_at_view(self, req):
        rospy.loginfo("Received request: %s", req)
        waypoint = req.waypoint
        if waypoint not in self.pointclouds: 
            cloud = self._get_pointcloud(waypoint)
            self.pointclouds[waypoint] = cloud
        if waypoint not in self.octomaps: 
            octomap = self._get_octomap(waypoint)
            self.octomaps[waypoint] = octomap

        # call alex's service
        self._get_labels([waypoint])
        #self._init_fake_labels([req.waypoint])

        if waypoint not in self.octomap_keys:
            self.octomap_keys[waypoint] = self._points_to_keys(self.points[waypoint], self.octomaps[waypoint])

        p = 1.0 # compute joint probability for all objects
        for obj in req.objects:
            p *= self.view_probability(waypoint,obj,self.octomap_keys[waypoint],req.values)

        res = GetProbabilityAtViewResponse()
        res.probability = p
        rospy.loginfo("Sent response: %s", res)
        return res

if __name__ == "__main__":
 
    parser = argparse.ArgumentParser(prog='soma_pcl_segmentation_server.py')
    parser.add_argument('-kb', metavar='config-file')
                        
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    
    rospy.init_node('soma_pcl_segmentation_server')
    rospy.loginfo("Running soma_pcl_segmentation_server KB: %s)", args.kb)
    SOMAPCLSegmentationServer(args.kb)
    #rospy.spin()
