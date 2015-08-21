#!/usr/bin/env python
import os
import sys
import json
import rospy
import math
import scipy
from semantic_segmentation.srv import *
import numpy as np
import pylab as pl
import matplotlib.pyplot as plt
from unidecode import unidecode
#from aklearn.cluster import KMeans
from scipy.cluster.vq import kmeans,vq
from matplotlib.mlab import PCA
from scipy.optimize import fmin_bfgs, fmin_l_bfgs_b
from mpl_toolkits.mplot3d import Axes3D
from scipy.cluster.hierarchy import dendrogram, linkage
from scipy.spatial import distance
import random

class my_client:

    def __init__(self):
        self.label_names=dict()
        self.label_prob=dict()
        self.label_frq=dict()
        self.points=dict()
        self.aver_frq=dict()
        self.part_feature=dict()
        self.whole_feature=[]
        self.expand_feature=np.array([])
        self.aver_feature=[]
        self.new_waypoints=dict()

        waypoints=['WayPoint13','WayPoint14','WayPoint15','WayPoint16','WayPoint19','WayPoint22','WayPoint23','WayPoint26','WayPoint27','WayPoint3','WayPoint30','WayPoint31',
'WayPoint34','WayPoint4','WayPoint42','WayPoint44','WayPoint5','WayPoint6']
        for i in range(len(waypoints)):
            self.label_names[waypoints[i]]=dict()
            self.label_prob[waypoints[i]]=dict()
            self.label_frq[waypoints[i]]=dict()
            self.points[waypoints[i]]=dict()


      
#get the information of instances
    def label_client(self,request):
        rospy.loginfo("waiting for service")
        rospy.wait_for_service('/semantic_segmentation_integrate_node/label_integrated_cloud')
        rospy.loginfo("start right now")
        X=[]
        Y=dict()
        feature_n=11
        Z=[]
        new_waypoints=dict()
        for waypoint, instances in request.items():
            Y[waypoint]=[]
            for instance in instances:
                    point_information=rospy.ServiceProxy('/semantic_segmentation_integrate_node/label_integrated_cloud_plus',LabelIntegratedPointInstanceCloud)
                    req=LabelIntegratedPointInstanceCloudRequest()
	            req.waypoint_id = waypoint
                    req.instance_number=instance
                    label_info=point_information(req)

                    rospy.loginfo("processing :%s during %s", waypoint, instance)
                    self.label_names[waypoint][instance] = label_info.index_to_label_name
                    self.label_prob[waypoint] [instance]= label_info.label_probabilities
                    self.label_frq[waypoint][instance] =  label_info.label_frequencies
#generate matrix of position of points 
                    position=[]
                    for i in range(len(label_info.points)):
                            position=position+[label_info.points[i].x, label_info.points[i].y, label_info.points[i].z]
                    position=np.array(position).reshape(-1,3)
                    self.points[waypoint][instance]=position.tolist()
#save information of points
                    prob=open(waypoint+str(instance)+"prob_large.json","w")
                    json.dump(self.label_prob[waypoint][instance],prob,indent=4)
                    points=open(waypoint+str(instance)+"points_large.json","w")
                    json.dump(self.points[waypoint][instance],points,indent=4)
#feature matrix for every waypoint
                    a=list(self.label_frq[waypoint][instance])
                    X=X+a
                    Y[waypoint]=Y[waypoint]+a
            self.part_feature[waypoint]=np.array(Y[waypoint]).reshape(-1,len(label_info.index_to_label_name)).tolist()
#condense information from instance to waypoints:   average & its feature matrix
            full_info=self.label_frq[waypoint]
            temp=np.zeros([len(full_info.items()[0][1])])
            for instance, freq in full_info.items():
                temp=np.add(np.array(freq),temp)
            self.aver_frq[waypoint]=temp/np.sum(temp)
            self.aver_feature=self.aver_feature+list(self.aver_frq[waypoint])
#cluster inside waypoint & its feature matrix
            if (np.amax(self.distance(waypoint)) > 0.2) & (len(instances)>4) :
               k=self.select_k(waypoint)
               _,centroids,_=self.cluster_part(waypoint,k)
               Z=Z+sum(centroids.tolist(),[])
               self.new_waypoints[waypoint]=range(len(centroids))
            else:
               Z=Z+list(self.aver_frq[waypoint])
               self.new_waypoints[waypoint]=[0]
        Z=np.array(Z).reshape(-1,11)
        self.expand_feature=Z
#save all kinds of feature in one dict
        self.aver_feature=np.array(self.aver_feature).reshape(-1,feature_n)
        self.whole_feature=np.array(X).reshape(-1,feature_n)
        myfeature=dict()
        myfeature={
          "whole":  self.whole_feature.tolist(),
          "aver":   self.aver_feature.tolist(),
          "part":   self.part_feature,
          "expand": self.expand_feature.tolist()
        }
        feature=open("feature_large.json","w")
        json.dump(myfeature,feature,indent=4)
        feature.close()
        label_names=open("label_names_large.json","w")
        json.dump(self.label_names,label_names,indent=4)
        new_waypoints=open("new_waypoints_large.json","w")
        json.dump(self.new_waypoints,new_waypoints,indent=4)
        freq=open("freq_large.json","w")
        json.dump(self.label_frq,freq,indent=4)

#distance
    def distance(self,req,draw=0):
        waypoint=req
        X=np.array(self.part_feature[waypoint])
        dis=np.zeros([len(X),len(X)])
        for i in range(len(X)):
            for j in np.arange(i+1,len(X),1):
                d1=np.array(X[i,:])
                d2=np.array(X[j,:])
                dis[i,j]=np.sum(np.multiply(d1,np.log(np.divide(d1,d2))))
        #plot the distance
        if draw:
           fig, ax = plt.subplots()
           ax.imshow(dis,cmap=plt.cm.gray, interpolation='nearest')
           ax.set_title('distance between instance')
           ax.spines['left'].set_position(('outward', 10))
           ax.spines['bottom'].set_position(('outward', 10))
           ax.spines['right'].set_visible(False)
           ax.spines['top'].set_visible(False)
           ax.yaxis.set_ticks_position('left')
           ax.xaxis.set_ticks_position('bottom')
           plt.show()
        return dis
      
#cluster  
    def cluster_part(self,waypoint,k):
        X=np.array(self.part_feature[waypoint])
        centroids,_=kmeans(X,k)
        idx,_=vq(X,centroids)
        i=0
        for instance in self.label_names[waypoint].keys():
            i=i+1
        return idx,centroids,waypoint

#select best k
    def compute_bic(self,idx,centroids):
        feature=np.array(self.part_feature)
        index=np.array(idx)
        N,d=feature.shape
        m=len(centroids)
        n=[0]*m
        for i in range(m):
            n[i]=np.sum(index==i)
        cl_var=(1.0 / (N - m) / d) * sum([sum(distance.cdist(feature[np.where(index == i)], [centroids[i]], 'euclidean')**2) for i in range(m)])
        const_term = 0.5 * m * np.log(N) * (d+1)
        bic=np.sum([n[i] * np.log(n[i]) -
               n[i] * np.log(N) -
             ((n[i] * d) / 2) * np.log(2*np.pi*cl_var) -
             ((n[i] - 1) * d/ 2) for i in range(m)]) - const_term
        return bic
    def select_k(self,req):
        bic=[0]*10
        for k in range(10):
            idx, centroids=self.cluster_part(req,(k+1))
            bic[k]=self.compute_bic(idx,centroids)
            if (k>2) & (bic[k]<bic[k-1]):
               break
        return k        


if __name__=="__main__":
    request=dict()
    waypoints=['WayPoint13','WayPoint14','WayPoint15','WayPoint16','WayPoint19','WayPoint22','WayPoint23','WayPoint26','WayPoint27','WayPoint3','WayPoint30','WayPoint31',
'WayPoint34','WayPoint4','WayPoint42','WayPoint44','WayPoint5','WayPoint6']
    instances=[3, 2, 15, 38, 6, 1, 32, 7, 37, 34, 2, 40, 3, 7, 33, 32, 7, 61]
    for i in range(len(waypoints)):
        request[waypoints[i]]=range(instances[i])
    rospy.init_node('soma_pcl_segmentation_server')
    print 'creating'
    example=my_client()
    example.label_client(request)
