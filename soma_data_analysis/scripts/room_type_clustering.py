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

def cluster_whole(req,k,display=0):
        feature=json.load(open("data/feature_large.json","r"))
        X=np.array(feature['whole'])
        #K-means clustering using scipy
        centroids,_=kmeans(X,k)
        idx,_=vq(X,centroids)
        i=0
        if display :
           for waypoint, instances in req.items():
            for instance in instances:
                print waypoint, instance, idx[i]
                i=i+1
        return idx,centroids

def cluster_aver(req,k,display=0):
        feature=json.load(open("data/feature_large.json","r"))
        X=np.array(feature['aver'])
        centroids,_=kmeans(X,k)
        idx,_=vq(X,centroids)
        i=0
        if display :
           for waypoint in req.keys():
            print waypoint, idx[i]
            i=i+1
        return idx,centroids
   
def cluster_expand(k,display=0):
        feature=json.load(open("data/feature_large.json","r"))
        X=np.array(feature['expand'])
        new=json.load(open("data/new_waypoints_large.json","r"))
        centroids,_=kmeans(X,k)
        idx,_=vq(X,centroids)
        j=0
        if display :
           for waypoint, sub in new.items():
            for i in sub: 
                print waypoint, i, idx[j]
                j=j+1
        return idx,centroids

#select best k
def compute_bic(idx,centroids,method):
        feature1=json.load(open("data/feature_large.json","r"))
        if method =="whole":
           feature=np.array(feature1['whole'])
        elif method =="aver":
           feature=np.array(feature1['aver'])
        elif method =="expand":
           feature=np.array(feature1['expand'])
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
def select_k(req,method):
        bic=[0]*10
        for k in range(10):
            if method == 'whole' :
               idx, centroids=cluster_whole(req,(k+1))
            elif method == 'aver' :
               idx, centroids=cluster_aver(req,(k+1))
            elif method == 'expand' :
               idx, centroids= cluster_expand(k+1)
            bic[k]=compute_bic(idx,centroids,method)
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
    mission=json.loads(raw_input('operation'))
    k=select_k(request,mission)
    print 'best choice of k',k
    if mission == 'whole' :
        cluster_whole(request,k,display=1)
    elif mission == 'aver' :
        cluster_aver(request,k,display=1)
    elif mission == 'expand' :
        cluster_expand(k,display=1)

