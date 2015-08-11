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
        self.labels=dict()
        #self.range=dict()
        self.aver_frq=dict()
        self.part_feature=dict()
        self.whole_feature=[]
        self.aver_feature=[]
        self.new_waypoints=dict()
        self.expand_feature=np.array([])

        waypoints=['WayPoint13','WayPoint14','WayPoint15','WayPoint16','WayPoint19','WayPoint22','WayPoint23','WayPoint26','WayPoint27','WayPoint3','WayPoint30','WayPoint31',
'WayPoint34','WayPoint4','WayPoint42','WayPoint44','WayPoint5','WayPoint6']
        #waypoints=['WayPoint1','WayPoint12','WayPoint16','WayPoint19','WayPoint20','WayPoint21','WayPoint22','WayPoint23','WayPoint4','WayPoint5','WayPoint7']

        #available_range=[2,2,1,3,1,1]
        for i in range(len(waypoints)):
            self.label_names[waypoints[i]]=dict()
            self.label_prob[waypoints[i]]=dict()
            self.label_frq[waypoints[i]]=dict()
            self.points[waypoints[i]]=dict()
            self.labels[waypoints[i]]=dict()
            #self.range[waypoints[i]]=available_range[i]
      

    def label_client(self,request):
        rospy.loginfo("waiting for service")
        rospy.wait_for_service('/semantic_segmentation_plus_node/label_integrated_cloud')
        rospy.loginfo("start right now")
        X=[]
        Y=dict()
        feature_n=11
        Z=[]
        new_waypoints=dict()
        for waypoint, instances in request.items():
            Y[waypoint]=[]
            for instance in instances:
                #if instance > self.range[waypoint]-1:
                   #sys.exit('index out of range')
                #else:
                    point_information=rospy.ServiceProxy('/semantic_segmentation_plus_node/label_integrated_cloud',LabelIntegratedPointInstanceCloud)
                    req=LabelIntegratedPointInstanceCloudRequest()
	            req.waypoint_id = waypoint
                    req.instance_number=instance
                    label_info=point_information(req)

                    rospy.loginfo("processing :%s during %s", waypoint, instance)
                    self.label_names[waypoint][instance] = label_info.index_to_label_name
                    self.label_prob[waypoint] [instance]= label_info.label_probabilities
                    self.label_frq[waypoint][instance] =  label_info.label_frequencies
                    position=[]
                    for i in range(len(label_info.points)):
                            position=[label_info.points[i].x, label_info.points[i].y, label_info.points[i].z]+position
                    position=np.array(position).reshape(-1,3)
                    self.points[waypoint][instance]=position.tolist()
                    self.labels[waypoint][instance]=dict()
                    for i in range(len(label_info.index_to_label_name)):
                        self.labels[waypoint][instance][label_info.index_to_label_name[i]] = label_info.label_frequencies[i]
                    prob=open(waypoint+str(instance)+"prob_large.json","w")
                    json.dump(self.label_prob[waypoint][instance],prob,indent=4)
                    points=open(waypoint+str(instance)+"points_large.json","w")
                    json.dump(self.points[waypoint][instance],points,indent=4)
        #feature matrix for whole & part
                    #a=list(self.label_frq[waypoint][instance])
                    #X=X+a
                    #Y[waypoint]=Y[waypoint]+a
            #self.part_feature[waypoint]=np.array(Y[waypoint]).reshape(-1,len(label_info.index_to_label_name)).tolist()
        #condense information from instance to waypoints(cifiw) average & its feature matrix
            #full_info=self.label_frq[waypoint]
            #temp=np.zeros([len(full_info.items()[0][1])])
            #for instance, freq in full_info.items():
                #temp=np.add(np.array(freq),temp)
            #self.aver_frq[waypoint]=temp/np.sum(temp)
            #self.aver_feature=self.aver_feature+list(self.aver_frq[waypoint])
#cifiw cluster & its feature matrix
            #if np.amax(self.distance(waypoint)) > 0.2:
               #_,centroids,_=self.cluster_part(waypoint)
               #print centroids
               #print sum(centroids.tolist(),[])
               #Z=Z+sum(centroids.tolist(),[])
               #self.new_waypoints[waypoint]=range(len(centroids))
            #else:
               #Z=Z+list(self.aver_frq[waypoint])
               #self.new_waypoints[waypoint]=[0]
        #Z=np.array(Z).reshape(-1,11)
        #self.expand_feature=Z

            #if (np.amax(self.distance(waypoint)) > 0.2) & (len(instances)>4) :
              # k=self.select_k(waypoint)
               #_,centroids,_=self.cluster_part(waypoint,k)
               #Z=Z+sum(centroids.tolist(),[])
               #self.new_waypoints[waypoint]=range(len(centroids))
            #else:
               #Z=Z+list(self.aver_frq[waypoint])
               #self.new_waypoints[waypoint]=[0]
        #self.aver_feature=np.array(self.aver_feature).reshape(-1,feature_n)
        #self.whole_feature=np.array(X).reshape(-1,feature_n)
        #Z=np.array(Z).reshape(-1,11)
        #self.expand_feature=Z
        #myfeature=dict()
        #myfeature={
          #"whole":  self.whole_feature.tolist(),
          #"aver":   self.aver_feature.tolist(),
          #"part":   self.part_feature,
          #"expand": self.expand_feature.tolist()
        #}
        #feature=open("feature_large.json","w")
        #json.dump(myfeature,feature,indent=4)
        #feature.close()
        label_names=open("label_names_large.json","w")
        json.dump(self.label_names,label_names,indent=4)
        #new_waypoints=open("new_waypoints_large.json","w")
        #json.dump(self.new_waypoints,new_waypoints,indent=4)
        #freq=open("freq_large.json","w")
        #json.dump(self.label_frq,freq,indent=4)

#distance
    def distance(self,req,draw=0):
        feature=json.load(open("feature.json","r"))
        #req=json.loads(raw_input('which kind of distance'))
        if req=="whole":
           X=np.array(feature['whole'])

        elif req=="aver":
           X=np.array(feature['aver'])
        elif req=="expand":
           X=np.array(feature['expand'])
        else:
           waypoint=req
           X=np.array(feature['part'][waypoint])
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
    def cluster_part_old(self,waypoint):
        #cifiw clustering
        #waypoint=json.loads(raw_input('which waypoint to cluster'))
        feature=json.load(open("feature_large.json","r"))
        X=np.array(feature['part'][waypoint])
        centroids,_=kmeans(X,3)
        idx,_=vq(X,centroids)
        i=0
        for instance in self.label_names[waypoint].keys():
            print instance, idx[i]
            i=i+1
        return idx,centroids,waypoint
   
    def cluster_part(self,waypoint,k):
        #cifiw clustering
        #waypoint=json.loads(raw_input('which waypoint to cluster'))
        #feature=json.load(open("feature.json","r"))
        X=np.array(self.part_feature[waypoint])
        centroids,_=kmeans(X,k)
        idx,_=vq(X,centroids)
        i=0
        for instance in self.label_names[waypoint].keys():
            i=i+1
        return idx,centroids,waypoint

    def cluster_whole(self,req,k):
        feature=json.load(open("feature_large.json","r"))
        X=np.array(feature['whole'])
        #K-means clustering using scipy
        centroids,_=kmeans(X,k)
        idx,_=vq(X,centroids)
        i=0
        for waypoint, instances in req.items():
            for instance in instances:
                print waypoint, instance, idx[i]
                i=i+1
        return idx,centroids

    def cluster_aver(self,req,k):
        feature=json.load(open("feature.json","r"))
        X=np.array(feature['aver'])
        centroids,_=kmeans(X,k)
        idx,_=vq(X,centroids)
        i=0
        for waypoint in req.keys():
            print waypoint, idx[i]
            i=i+1
        return idx,centroids
   
    def cluster_expand(self):
        feature=json.load(open("feature_large.json","r"))
        X=np.array(feature['expand'])
        new=json.load(open("new_waypoints_large.json","r"))
        centroids,_=kmeans(X,5)
        idx,_=vq(X,centroids)
        j=0
        for waypoint, sub in new.items():
            for i in sub: 
                print waypoint, i, idx[j]
                j=j+1
        return new, idx

    def h_cluster(self,req):
        link=linkage(self.distance(req),'single')
        print link
        plt.figure(101)
        plt.subplot(1,2,1)
        plt.title("ascending")
        dendrogram(link,color_threshold=1,truncate_mode='lastp')
        plt.show()

#plot 3d
    def find_points(self,command,position,prob):
        points_label=np.array(prob.argmax(axis=0)) 
        return position[np.where(points_label==command)]   
    def plot_label(self,req):
        if req=='large':
           position=json.load(open("points_large.json","r"))
           prob=json.load(open("prob_large.json","r"))
        else :
           position=json.load(open("points.json","r"))
           prob=json.load(open("prob.json","r"))
        command=json.loads(raw_input('which kind of label'))
        plot_position=self.find_points(command,position,prob)
        fig=plt.figure()
        ax=Axes3D(fig)
        ax.scatter(plot_position[:,0],plot_position[:,1],plot_position[:,2])
        ax.set_title('position of label'+command)
        plt.show()


#select best k
    def compute_bic(self,idx,centroids):
        feature1=json.load(open("feature.json","r"))
        feature=np.array(feature1['aver'])
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
        bic=[0]*6
        for k in range(6):
            idx, centroids=self.cluster_aver(req,(k+1))
            bic[k]=self.compute_bic(idx,centroids)
        real_k=bic.index(max(bic))
        return k        

#regression
    def data_decorate(self,test_instance,label,data=1):
        train=[]
        train_label=[]
        train_multilabel=dict()
        test=[]
        test_label=[]
        if data==1:
           frq=json.load(open("freq.json","r"))
        else : 
           frq=json.load(open("freq_large.json","r"))
        for waypoint in label.keys():
            test_index=test_instance[waypoint]
            full_info=frq[waypoint]
            for instance, freq in full_info.items():
                if int(unidecode(instance)) in test_index:
                   test=test+freq
                   test_label=test_label+[label[waypoint]]
                else:
                   train=train+freq
                   train_label=train_label+[label[waypoint]]
        train=np.array(train).reshape(-1,11)
        test=np.array(test).reshape(-1,11)
        train_label=np.array(train_label)
        for i in range(5):
            train_multilabel[i]=np.array([0]*len(train_label))
            train_multilabel[i][np.where(train_label==i)]=1
        return train, train_multilabel, test, test_label
    def logit(self,x,beta):
        return((1/(1.0 + np.exp(-np.dot(x, beta)))))
    def cost(self,beta,x,y):
        l=[0]*len(x)
        for i in range(len(x)):
            l[i]=self.logit(x[i],beta)
        l=np.array(l)
        y=np.array(y)
        s=y*np.log(l) + (1-y)*(np.log(1-l))
        return -(np.sum(y*np.log(l) + (1-y)*(np.log(1-l))))
    def log_reg(self,train,test,label):
        x=train
        (n,d)=x.shape
        y=label
        beta = scipy.optimize.fmin_bfgs(self.cost, 
                                  x0 = np.array([0]*d) ,
                                  args = (x, y), gtol = 1e-3)
        predict=[0]*len(test)
        for i in range(len(test)):
            predict[i]=self.logit(test[i],beta)
        return (beta,predict)
    def multi_log_reg(self,test_instance1,test_instance2,label1,label2):
        beta=dict()
        train_multilabel=dict()
        pre=[]
        _,_,test,test_label=self.data_decorate(test_instance1,label1,data=2)
        random,rl,train, train_label=self.data_decorate(test_instance2,label2)
        train_label=np.array(train_label)
        for i in range(2):
            train_multilabel[i]=np.array([0]*len(train_label))
            train_multilabel[i][np.where(train_label==i)]=1
        for i in train_multilabel.keys():

            beta[i],temp=self.log_reg(train,test,train_multilabel[i])
            pre=pre+temp
        pre=np.array(pre).reshape(-1,len(test))
        predict=np.array(pre.argmax(axis=0))
        rn=len(predict)-np.count_nonzero(predict-np.array(test_label))
        accuracy=float(rn)/float(len(predict))
        #accuracy=np.sum(predict==np.array(test_label))/len(predict)
        #random_label=[0]*len(test_label)
        #for i in range(len(test_label)):
             # random_label[i]=random.sample(range(3),1)
        print predict, test_label,accuracy#, random_label
        return predict

#kmeans self_defined
    def kmeans(self,req, k):
        feature=json.load(open("feature.json","r"))
        #req=json.loads(raw_input('which kind of distance'))
        if req=="whole":
           data=np.array(feature['whole'])

        elif req=="aver":
           data=np.array(feature['aver'])
        elif req=="expand":
           data=np.array(feature['expand'])
        else:
           waypoint=req
           data=np.array(feature['part'][waypoint])

        centroids = []

        centroids = self.randomize_centroids(data, centroids, k)  

        old_centroids = [[] for i in range(k)] 

        iterations = 0
        while not (self.has_converged(centroids, old_centroids, iterations)):
              iterations += 1

              clusters = [[] for i in range(k)]

        # assign data points to clusters
              clusters = self.euclidean_dist(data, centroids, clusters)

        # recalculate centroids
              index = 0
              for cluster in clusters:
                  old_centroids[index] = centroids[index]
                  centroids[index] = np.mean(cluster, axis=0).tolist()
                  index += 1
        #print("The total number of data instances is: " + str(len(data)))
        #print("The total number of iterations necessary is: " + str(iterations))
        #print("The means of each cluster are: " + str(centroids))
        #print("The clusters are as follows:")
        #for cluster in clusters:
            #print("Cluster with a size of " + str(len(cluster)) + " starts here:")
            #print(np.array(cluster).tolist())
            #print("Cluster ends here.")
        return
    def euclidean_dist(self,data, centroids, clusters):
        for instance in data:  
        # Find which centroid is the closest
        # to the given data point.
            mu_index = min([(i[0], self.pdist(instance,centroids[i[0]])) \
                            for i in enumerate(centroids)], key=lambda t:t[1])[0]
            try:
                clusters[mu_index].append(instance)
            except KeyError:
                clusters[mu_index] = [instance]
        # If any cluster is empty then assign one point
        # from data set randomly so as to not have empty
        # clusters and 0 means.        
        for cluster in clusters:
            if not cluster:
               cluster.append(data[np.random.randint(0, len(data), size=1)].flatten().tolist())
        return clusters
    def pdist(self,x,y):
        d1=np.array(x)
        d2=np.array(y)
        dis=np.sum(np.multiply(d1,np.log(np.divide(d1,d2))))
        return dis
    def randomize_centroids(self,data, centroids, k):
        for cluster in range(0, k):
            centroids.append(data[np.random.randint(0, len(data), size=1)].flatten().tolist())
        return centroids  
    def has_converged(self,centroids, old_centroids, iterations):
        MAX_ITERATIONS = 1000
        if iterations > MAX_ITERATIONS:
           return True
        return old_centroids == centroids


if __name__=="__main__":
    request=dict()
    test_instance1=dict()
    test_instance2=dict()
    concrete_label1=dict()
    concrete_label2=dict()
    general_label=dict()
    #waypoints=['WayPoint1','WayPoint12','WayPoint16','WayPoint19','WayPoint20','WayPoint21','WayPoint22','WayPoint23','WayPoint4','WayPoint5','WayPoint7']
    #instances=[16, 13, 17, 17, 4, 1, 23, 1, 11, 23, 23]
    #test_number2=#[0,0,3,7,1,0,6,1,7,7,0,8,0,1,6,6,1,12]
    #con_waypointlabel2=#[2,2,2,1,2,2,1,2,1,2,3,2,1,4,3,0,1,3,3]
    #general_waypointlabel=[0,0,1,1,0,0,1,1,0,2,0]
    waypoints=['WayPoint13','WayPoint14','WayPoint15','WayPoint16','WayPoint19','WayPoint22','WayPoint23','WayPoint26','WayPoint27','WayPoint3','WayPoint30','WayPoint31',
'WayPoint34','WayPoint4','WayPoint42','WayPoint44','WayPoint5','WayPoint6']
    instances=[3, 2, 15, 38, 6, 1, 32, 7, 37, 34, 2, 40, 3, 7, 33, 32, 7, 61]
    #test_number2=[0,0,3,7,1,0,6,1,7,7,0,8,0,1,6,6,1,12]
    #con_waypointlabel2=[2,2,2,1,2,2,1,2,1,2,3,2,1,4,3,0,1,3,3]

#test of general
    #waypoints1=['WayPoint16','WayPoint19','WayPoint22','WayPoint23','WayPoint5']
    #instances1=[ 17, 17, 23, 1, 23]
    #test_number1=
    #con_waypointlabel1=[0,0,1,1,2]
    #general_waypointlabel=[0,0,1,1,0,0,1,1,0,2,0]
    #waypoints2=['WayPoint13','WayPoint14','WayPoint15','WayPoint19','WayPoint26','WayPoint30',
#'WayPoint34','WayPoint42']
    #instances2=[3, 2, 15, 6, 7, 11, 3, 33]
    #test_number2=[0,0,3,1,1,0,8,0,1,6]
    #con_waypointlabel2=[0,0,0,0,0,0,1,2]
    for i in range(len(waypoints)):
        request[waypoints[i]]=range(instances[i])
    #for i in range(len(waypoints1)):
        #test_instance1[waypoints1[i]]=range(instances1[i])
        #concrete_label1[waypoints1[i]]=con_waypointlabel1[i]
    #for i in range(len(waypoints2)):
        #test_instance2[waypoints2[i]]=range(instances2[i])
        #concrete_label2[waypoints2[i]]=con_waypointlabel2[i]
        #general_label[waypoints[i]]=general_waypointlabel[i]
        #print x
        #request[x]=range(json.loads(raw_input('instances')))
    #x=json.loads(raw_input('input waypoint'))
    #while not x== 0:
        #x=unidecode(x)# randomize initial centroids

        #request[x]=range(json.loads(raw_input('instances of this waypoint'))) 
        #x=json.loads(raw_input('input waypoint'))
    #request[waypoints[6]]=range(6)
    rospy.init_node('soma_pcl_segmentation_server')
    print 'creating'
    example=my_client()
    #example.kmeans('whole',3)
    #regression=example.multi_log_reg(test_instance2,test_instance1,concrete_label2,concrete_label1)
    example.label_client(request)
#whole
    #k=example.select_k(request)
    #print 'best choice of k'
    #print k
    #example.cluster_whole(request,k)
    #example.distance("whole",draw=1)
#aver
    #k=example.select_k(request)
    #print k
    #example.cluster_aver(request,k)
    #example.distance("aver",draw=1)
#expand
    #example.cluster_expand()
    #example.distance("expand",draw=1)

    
    #req=json.loads(raw_input('type of h cluster'))
    #example.h_cluster(req)
