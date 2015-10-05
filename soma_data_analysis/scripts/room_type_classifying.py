#!/usr/bin/env python
import os
import sys
import json
import rospy

import scipy
import numpy as np
from unidecode import unidecode
from scipy.optimize import fmin_bfgs, fmin_l_bfgs_b


class my_client:

    def __init__(self):
        self.label_names=dict()

#regression
    def data_decorate(self,test_instance,label):
        train=[]
        train_label=[]
        train_multilabel=dict()
        test=[]
        test_label=[]
        frq=json.load(open("data/freq_large.json","r"))
        for waypoint in label.keys():
            if waypoint in test_instance.keys():
                test_index=test_instance[waypoint]
            else :
                test_index=[]
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
        print len(train)
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
    def multi_log_reg(self,test_instance,label,mission):
        beta=dict()
        train_multilabel=dict()
        pre=[]
        train,train_multilabel,test,test_label=self.data_decorate(test_instance,label)
        for i in train_multilabel.keys():
            beta[i],temp=self.log_reg(train,test,train_multilabel[i])
            pre=pre+temp
        pre=np.array(pre).reshape(-1,len(test))
        predict=np.array(pre.argmax(axis=0))
        rn=len(predict)-np.count_nonzero(predict-np.array(test_label))
        accuracy=float(rn)/float(len(predict))
        print predict
        print test_label
        if mission=='test':
            print accuracy
        return predict


if __name__=="__main__":
    test_instance=dict()
    concrete_label=dict()
    waypoints=['WayPoint13','WayPoint14','WayPoint15','WayPoint16','WayPoint19','WayPoint22','WayPoint23','WayPoint26','WayPoint27','WayPoint3','WayPoint30','WayPoint31',
'WayPoint34','WayPoint4','WayPoint42','WayPoint44','WayPoint5','WayPoint6']
    test=[0,0,3,7,1,0,6,1,7,7,0,8,0,1,6,6,1,12]
    con_waypointlabel=[2,2,2,1,2,2,1,2,1,2,3,2,1,4,3,0,1,3,3]
    test_number=dict()
    mission=unidecode(json.loads(raw_input('operation')))
    if mission=='predict':
        req=json.loads(raw_input('request waypoints'))
        for waypoint in req:
            instances=json.loads(raw_input('request instances'))
            test_number[waypoint]=instances
    elif mission=='test':
         req=waypoints
         i=0
         for waypoint in req:
             test_number[waypoint]=range(test[i])
             i=i+1
    for i in range(len(waypoints)):
        if waypoints[i] in req :
           test_instance[waypoints[i]]=test_number[waypoints[i]]
        concrete_label[waypoints[i]]=con_waypointlabel[i]
    print 'creating'
    example=my_client()
    regression=example.multi_log_reg(test_instance,concrete_label,mission)


