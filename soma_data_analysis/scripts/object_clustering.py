#!/usr/bin/env python
from __future__ import division
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
from scipy.cluster.vq import kmeans,vq
from matplotlib.mlab import PCA
from scipy.optimize import fmin_bfgs, fmin_l_bfgs_b
from mpl_toolkits.mplot3d import Axes3D
from scipy.cluster.hierarchy import dendrogram, linkage
from scipy.spatial import distance
import random
from scipy.spatial import ConvexHull
#from scipy.stats import multivariate_normal
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point


class identify_objects:

     def __init__(self,waypoint,instances):
         self.prob=dict()
         self.points=dict()
         self.label=dict()
         label_names=json.load(open("label_names_large.json","r"))
         self.label_names=label_names[waypoint]
         self.objects=dict()
         self.position_matrix=dict()
         self.tag=dict()
         self.predict=dict()
         for instance in instances:
             print 'for', instance
             self.objects[instance]=dict()
             self.tag[instance]=dict()
             self.points[instance]=json.load(open(waypoint+str(instance)+"points_large.json","r"))
             self.prob[instance]=json.load(open(waypoint+str(instance)+"prob_large.json","r"))
             self.asign_label(waypoint,instance)
             self.position_matrix[instance]=dict()
             for objects in self.label_names[unicode(instance)]:
                 self.tag[instance][objects]=dict()
                 temp=[]
                 print 'setting',objects
                 for i in range(len(self.label[instance])):
                     if self.label[instance][i]==objects:
                         temp=self.points[instance][i]+temp
                 self.position_matrix[instance][objects]=np.array(temp).reshape(-1,3)
           
# for every point, asign the label with the max probability
     def asign_label(self,waypoint,instance):
         prob=np.array(self.prob[instance]).reshape(-1,11)
         temp=[]
         indexs=prob.argmax(axis=1).tolist()
         i=0
         for num in prob.max(1):
             if num<0.5:
                indexs[i]=11
         for index in indexs:
             if index<11:
                temp=temp+[unidecode(self.label_names[unicode(instance)][index])]
             else:
                temp=temp+["unsure"]
         self.label[instance]=temp

#clustering the point given a kind of label
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
             ((n[i] - 1) * d/ 2) for i
 in range(m)]) - const_term
         return bic
     def select_k(self,waypoint,instance,objects):
         bic=[0]*30
         for k in range(30):
            idx, centroids,feature=self.sub_cluster_points(waypoint,instance,objects,(k+1))
            bic[k]=self.compute_bic(idx,centroids,feature)
            if (k>2) & (bic[k]<bic[k-1]):
               break
         return k 
 
#cluster
     def sub_cluster_points(self,waypoint,instance,objects,k):
         X=self.position_matrix[instance][objects]
         centroids,_=kmeans(X,k)
         idx,_=vq(X,centroids)
         return idx, centroids, self.position_matrix[instance][objects]     
     def cluster_points(self,waypoint,instance,objects):
         _,_,feature=self.sub_cluster_points(waypoint,instance,objects,1)
         if len(feature)<50:
            print 'no such object in this room'
            return 0,0,[]
         else :
            k=self.select_k(waypoint,instance,objects)
            #k=math.sqrt(len(feature)/2)
            idx,centroids,feature=self.sub_cluster_points(waypoint,instance,objects,k)
            return idx,centroids,feature

#form the bounding boxes given a kind of label
     def set_box(self,waypoint,instance,objects):
         idx,centroids,feature=self.cluster_points(waypoint,instance,objects)
         self.objects[instance][objects]=dict()
         if feature != [] :
            for i in range(len(centroids)):
             part_feature=np.array(feature[np.where(idx==i)])
             #print len(part_feature)
             if len(part_feature)<50:
                #print 'ignore this cluster'
                self.objects[instance][objects][i]=[]
                self.tag[instance][objects][i]=-1
             else:
                maxi=[0]*3
                mini=[0]*3
                aver=[0]*3
                max_index=np.argmax(part_feature,axis=0)
                min_index=np.argmin(part_feature,axis=0)
                a=part_feature[max_index[1]]
                b=part_feature[min_index[0]]
                c=part_feature[min_index[1]]
                for axis in range(3):
                    maxi[axis]=np.percentile(part_feature[:,axis],80)
                    mini[axis]=np.percentile(part_feature[:,axis],20) 
                    aver[axis]=np.percentile(part_feature[:,axis],50)  
                self.objects[instance][objects][i]=[maxi,mini,aver,a,b,c]
                self.tag[instance][objects][i]=[]

#draw bounding boxes given a kind of label
     def draw(self,instance,objects):
         topic='visualization_marker_array'
         publisher=rospy.Publisher(topic,MarkerArray,queue_size=10)
         rospy.init_node('register',anonymous=True)
         markerArray=MarkerArray()
         i=0
         while not i>=len(self.objects[instance][objects].keys()):
                 print i 
                 data=self.objects[instance][objects][i]
                 if data==[]:
                    print 'ignore'
                 else:
                    maxi=data[0]

                    mini=data[1]
                    ave=data[2]
                    marker = Marker()
                    marker.header.frame_id = "/map"
                    marker.ns="bounding_box"
                    marker.type = marker.CUBE
                    marker.action = marker.ADD
                    marker.scale.x = maxi[0]-mini[0]
                    marker.scale.y = maxi[1]-mini[1]
                    marker.scale.z = maxi[2]-mini[2]
                    marker.color.a = 1.0
                    #marker.color.r = i/10
                    marker.pose.orientation.w = 1.0
                    marker.pose.position.x = ave[0]
                    marker.pose.position.y = ave[1]
                    marker.pose.position.z = ave[2]
                    markerArray.markers.append(marker)
                    id_num=0
                    for m in markerArray.markers:
                        m.id=id_num
                        id_num+=1
                 i=i+1

         while not rospy.is_shutdown():
                 publisher.publish(markerArray)
                 rospy.sleep(0.01)

#draw actual cloud given a kind of label
     def actaul_clouds(self,waypoint,instance,objects):
         global ID2
         position=self.position_matrix[instance][objects]
         points=[]
         for i in range(len(position)):
            point=Point()
            point.x=position[i][0]
            point.y=position[i][1]
            point.z=position[i][2]
            points=points+[point]
         topic='visualization_marker'
         publisher=rospy.Publisher(topic,Marker)
         rospy.init_node('actual',anonymous=True)
         while not rospy.is_shutdown():
                 marker = Marker()
                 marker.header.frame_id = "/map"
                 marker.ns="actual"
                 marker.id=1
                 marker.type = marker.POINTS
                 marker.action = marker.ADD
                 marker.scale.x = 0.04
                 marker.scale.y = 0.04
                 marker.scale.z = 0.04
                 marker.color.a = 1.0
                 marker.color.r=1.0
                 marker.color.g=1.0
                 marker.points=points
                 publisher.publish(marker)

#general operation to generate the bounding boxes and display them on rviz given the label        
     def bounding_box(self,waypoint,instance,objects):
                 self.set_box(waypoint,instance,objects)
                 print 'finish generating'
                 try:
                     self.draw(instance,objects)
                 except rospy.ROSInterruptException:
                     pass

#judge the spatial relationship between a point and a line in 2D
     def vector(self,start,end):
         b=np.array(end)
         a=np.array(start)
         return b-a
     def unit(self,vec):
         return vec/sum(vec**2)              
     def pnt2line(self,pnt, start, end):
        inner=1
        line_vec = self.vector(start, end)
        pnt_vec = self.vector(start, pnt)
        line_len = distance.euclidean(start,end)
        line_unitvec = self.unit(line_vec)
        pnt_vec_scaled = pnt_vec*1.0/line_len
        t = sum(line_unitvec*pnt_vec_scaled)    
        if t < 0.0:
           inner=0
        elif t > 1.0:
           inner=0
        return inner
#define the 'above' relationship between tv and table        
     def above(self,table,tv):
        tv[2][2]=0
        table[3][2]=0
        table[4][2]=0
        table[5][2]=0
        if tv[1][2]+0.1<table[0][2]:
            above=0
        elif self.pnt2line(tv[2],table[3],table[4])==0:
            above=0
        elif self.pnt2line(tv[2],table[4],table[5])==0:
            above=0
        else:
            above=1  
        return above 
#compute the possibility of a monitor being above a desk      
     def monitor_spatial(self,waypoint,instances):
        print 'start counting'
        total=0.0
        count=0.0
        for instance in instances:
            print 'counting instsance',instance
            self.set_box(waypoint,instance,'table')
            self.set_box(waypoint,instance,'monitor/tv')
            for index, tv in self.objects[instance]['monitor/tv'].items():
                if tv==[]:
                   i=0
                else :
                   for index, table in self.objects[instance]['table'].items():
                      if table != []:
                          if self.above(table,tv):
                              count=count+1
                   total=total+1
        p=count/total
        print p
        return p
# define the 'near' relationship between chair and table        
     def near(self,table,chair):
        if distance.euclidean(table[2],chair[3])<1:
            near=1
        else :
            near=0
        return near  
#compute the possibility of a chair being near a table 
     def chair_spatial(self,waypoint,instances):
        print 'start counting'
        total=0.0
        count=0.0
        for instance in instances:
            print 'counting instance', instance
            self.set_box(waypoint,instance,'table')
            self.set_box(waypoint,instance,'chair/sofa')
            for index, chair in self.objects[instance]['chair/sofa'].items():
                if chair==[]:
                   i=0
                else:
                   for index, table in self.objects[instance]['table'].items():
                       if self.near(table,chair):
                           count=count+1
                   total=total+1
        p=count/total
        print p 
        return p

#compute the average position and size of one object given a list of instances       
     def overview(self,waypoint,instances,objects):
         current=0
         data=[]
         self.predict[objects]=dict()
         for instance in instances:
             self.set_box(waypoint,instance,objects)
         for i in range(2):
             instance=instances[i]
             print 'first instance',i
             for k in self.objects[instance][objects].keys():
                 print 'object',k
                 if self.tag[instance][objects][k]==[]:
                     data=[self.objects[instance][objects][k]]
                     for j in range(i+1,len(instances)):
                         print 'last instance',j
                         instance_plus=instances[j]
                         index=self.search(waypoint,instance,instance_plus,objects,k)
                         if index < 100:
                                 self.tag[instance_plus][objects][index]=current
                                 data=data+[self.objects[instance_plus][objects][index]]
                     if len(data)>len(instances)-3:
                         data=np.array(data)
                         self.predict[objects][current]=np.mean(data,axis=0)
                     current = current + 1
                     print 'a new object'

# visualize the prediction                        
     def preview(self,waypoint,instances):
        topic='visualization_marker_array'
        publisher=rospy.Publisher(topic,MarkerArray,queue_size=10)
        rospy.init_node('register',anonymous=True)
        markerArray=MarkerArray()
        color=0
        for objects in ["chair/sofa","monitor/tv","table"]:
            self.overview(waypoint,instances,objects)
            for index, data in self.predict[objects].items():
                   maxi=data[0]
                   mini=data[1]
                   ave=data[2]
                   marker = Marker()
                   marker.header.frame_id = "/map"
                   marker.ns="predict_box"
                   marker.type = marker.CUBE
                   marker.action = marker.ADD
                   marker.scale.x = maxi[0]-mini[0]
                   marker.scale.y = maxi[1]-mini[1]
                   marker.scale.z = maxi[2]-mini[2]
                   marker.color.a = 1.0
                   marker.color.r = color
                   marker.color.g = 1-color
                   marker.pose.orientation.w = 1.0
                   marker.pose.position.x = ave[0]
                   marker.pose.position.y = ave[1]
                   marker.pose.position.z = ave[2]
                   markerArray.markers.append(marker)
                   id_num=0
                   for m in markerArray.markers:
                       m.id=id_num
                       id_num+=1
            color=color+1/2
        print 'start drawing'
        while not rospy.is_shutdown():
                publisher.publish(markerArray)
                rospy.sleep(0.01)              

#search the objects in instance2 which is considered as the same object as objects[i] in instance1
     def search(self,waypoint,instance1,instance2,objects,i): 
         data1=self.objects[instance1][objects][i]
         record=2
         index=100
         for j in self.objects[instance2][objects].keys():
             data2=self.objects[instance2][objects][j]
             if data2 != []:
                if distance.euclidean(data1[2],data2[2])< min([1.5,record]):
                   index=j
                   record=distance.euclidean(data1[2],data2[2])
         return index

#generate the distribution of the movement of a single object            
     def single_object_over_time(self,waypoint,instances,objects,i,draw_region=0,draw_distribution=0):
         centre=[]
         for instance in instances:
             self.set_box(waypoint,instance,objects)
             print 'finish sorting in', instance
             temp=self.search(waypoint,instances[0],instance,objects,i)
             print temp
             if temp==100 :
                print 'no such object in', instance
             else:
                pos=self.objects[instance][objects][temp][2]
                centre=centre+pos
         centre=np.array(centre).reshape(-1,3)
         mean=np.mean(centre,axis=0)
         cov=np.cov(centre,rowvar=0)
         print mean,cov
         centre=centre[:,[0,1]]
         chull=ConvexHull(centre)
         if draw_region:
             points=[]
             for line in chull.simplices:
                for index in line:
                    point=Point()
                    point.x=centre[index][0]
                    point.y=centre[index][1]
                    point.z=0
                    points=points+[point]
             topic='visualization_marker'
             publisher=rospy.Publisher(topic,Marker)
             rospy.init_node('actual',anonymous=True)
             while not rospy.is_shutdown():
                     marker = Marker()
                     marker.header.frame_id = "/map"
                     marker.ns="movable_region"
                     marker.id=1
                     marker.type = marker.LINE_LIST
                     marker.action = marker.ADD
                     marker.scale.x = 0.1
                     marker.color.a = 1.0
                     marker.color.r =1.0
                     marker.points = points
                     publisher.publish(marker)

         if draw_distribution:
             points=[]
             colors=[]
             var = multivariate_normal(mean=mean, cov=cov)
             for r in range(100):
                 for theta in range(36):
                     point=Point()
                     point.x=r/100*cos(2*pi/theta)
                     point.y=r/100*sin(2*pi/theta)
                     point.z=mean[2]
                     color=ColorRGBA()
                     color.a=1.0
                     color.r=var.pdf([r/100*math.cos(2*pi/theta),r/100*math.sin(2*pi/theta),0])
                     color.g=1-var.pdf([r/100*math.cos(2*pi/theta),r/100*math.sin(2*pi/theta),0])
                     points=points+[point]
             topic='visualization_marker'
             publisher=rospy.Publisher(topic,Marker)
             rospy.init_node('actual',anonymous=True)
             while not rospy.is_shutdown():
                     marker = Marker()
                     marker.header.frame_id = "/map"
                     marker.ns="distribution"
                     marker.id=1
                     marker.type = marker.POINTS
                     marker.action = marker.ADD
                     marker.colors=colors
                     marker.points=points
                     publisher.publish(marker)    
         return mean,cov

if __name__ == "__main__":
   label_type=["prop","wall","cabinet","ceiling","chair/sofa", "window", "floor","monitor/tv","person","shelf", "table"]
   waypoint=unidecode(json.loads(raw_input('request waypoint')))
   mission=unidecode(json.loads(raw_input('operation')))
   instances=json.loads(raw_input('request instances'))
   objects=identify_objects(waypoint,instances)
   if mission == 'box':
      num=json.loads(raw_input('request label')) 
      objects.bounding_box(waypoint,instances[0],label_type[num])
   elif mission == 'cloud':
      num=json.loads(raw_input('request label'))
      objects.actaul_clouds(waypoint,instances[0],label_type[num])
   elif mission == 'single':
      num=json.loads(raw_input('request label'))
      obj=json.loads(raw_input('specific object'))
      mean,cov=objects.single_object_over_time(waypoint,instances,label_type[num],obj,draw_region=1)
   elif mission == 'tv_spatial':
      objects.monitor_spatial(waypoint,instances)
   elif mission == 'chair_spatial':
      objects.chair_spatial(waypoint,instances)
   elif mission == 'predict':
      objects.preview(waypoint,instances)
   else :
      print 'wrong order' 
   #waypoint="WayPoint42"
   #instances=[0,1,2,3,4]
   #req=[u"WayPoint15",[0,1,2,3,4]]
   #objects=identify_objects(unidecode(req[0]),req[1])
   #objects.preview(unidecode(req[0]),req[1])
   #print 'finish setting'
   #mean,cov=objects.single_object_over_time(unidecode(req[0]),req[1],label_type[4],0,draw=0)
   #objects.bounding_box(unidecode(req[0]),req[1][0],label_type[10]) 
   #objects.actaul_clouds(unidecode(req[0]),req[1][0],label_type[4])
   #objects.monitor_spatial(unidecode(req[0]),req[1])
   #objects.chair_spatial(unidecode(req[0]),req[1])
