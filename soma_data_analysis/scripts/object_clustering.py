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

class identify_objects:

     def __init__(self):
         self.prob=json.load(open("prob_large.json","r"))
         self.points=json.load(open("points_large.json","r"))
         self.label_names=json.load(open("label_names_large.json","r"))
         self.label=dict()
         self.cluster=dict()
         self.objects=dict()
         self.position_matrix=dict()

     def asign_label(self,waypoint,instance):
         prob=np.array(self.prob[waypoint][instance]).reshape(-1,11)
         points=self.points[waypoint][instance]
         self.label[waypoint]=dict()
         temp=[]
         indexs=prob.argmax(axis=1).tolist()
         for index in indexs:
             temp=temp+[self.label_names[index]]
         self.label[waypoint][instance]=temp

#select best k
     def compute_bic(self,idx,centroids,feature):
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
     def select_k(self,waypoint,instance,objects):
         bic=[0]*6
         for k in range(6):
            idx, centroids,feature=self.sub_cluster_points(waypoint,instance,objects,(k+1))
            bic[k]=self.compute_bic(idx,centroids,feature)
         real_k=bic.index(max(bic))
         return k 
 
     def sub_cluster_points(self,waypoint,instance,objects,k):
         for i in range(len(self.label[waypoint][instance])):
             if self.label[waypoint][instance][i]==objects:
                temp=self.points[waypoint][instance][i]+temp
         self.position_matrix[objects]=np.array(temp).reshape(-1,3)
         centroids,_=kmeans(X,k)
         idx,_=vq(X,centroids)
         return idx, centroids, self.position_matrix[objects]     
     def cluster_points(self,waypoint,instsance,objects):
         k=select_k(waypoint,instance,objects)
         idx,centroids,feature=sub_cluster_points(waypoint,instance,objects,k)
         return idx,centroids,feature

     def bounding_box(self,waypoint,instance,objects):
         idx,centroids,feature=cluster_points(waypoint,instance,objects)
         self.objects[objects]=dict()
         for i in range(len(centroids)):
             part_feature=feature[np.where(idx==i)]
             maxi=mini=centre=[0]*3
             for axis in range(3):
                 maxi[axis]=max(part_feature[:,axis])
                 mini[axis]=min(part_feature[:,axis])          
             self.objects[objects][i]=np.array([maxi,mini])

