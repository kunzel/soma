#!/usr/bin/env python
import roslib; roslib.load_manifest("soma_trajectory")
import rospy
import pymongo
import json
import argparse
import math
from datetime import datetime


from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from human_trajectory.msg import Trajectory

from soma_geospatial_store.geospatial_store import GeoSpatialStoreProxy 

class TrajectoryCls:

    def __init__(self, uuid):

        self.uuid = uuid
        self.pose = []
        self.secs = []
        self.nsecs = []
        self.vel = []
        self.max_vel = 0.0
        self.length = 0.0
        
    def append_pose(self, pose, secs, nsecs):
        self.pose.append(pose)
        self.secs.append(secs)
        self.nsecs.append(nsecs)

    def sort_pose(self):
        if len(self.pose) > 1:
            self.pose, self.secs, self.nsecs = self.__quick_sort(self.pose,
                    self.secs, self.nsecs)

    def __quick_sort(self, pose, secs, nsecs):
        less_pose = []; equal_pose = []; greater_pose = []
        less_secs = []; equal_secs = []; greater_secs = []
        less_nsecs = []; equal_nsecs = []; greater_nsecs = []

        if len(secs) > 1:
            pivot = secs[0]
            for i,sec in enumerate(secs):
                if sec < pivot:
                    less_secs.append(sec)
                    less_pose.append(pose[i])
                    less_nsecs.append(nsecs[i])
                if sec == pivot:
                    equal_secs.append(sec)
                    equal_pose.append(pose[i])
                    equal_nsecs.append(nsecs[i])
                if sec > pivot:
                    greater_secs.append(sec)
                    greater_pose.append(pose[i])
                    greater_nsecs.append(nsecs[i])

            less_pose,less_secs,less_nsecs = self.__quick_sort(less_pose, less_secs, less_nsecs) 
            greater_pose,greater_secs,greater_nsecs = \
                self.__quick_sort(greater_pose, greater_secs, greater_nsecs)
            equal_pose,equal_secs,equal_nsecs = \
                self.__quick_sort(equal_pose,equal_nsecs,equal_secs)

            return less_pose + equal_pose + greater_pose, less_secs + \
                equal_secs + greater_secs, less_nsecs + equal_nsecs + greater_nsecs
        else:
            return pose, secs, nsecs

    def calc_stats(self):

        length = 0.0
        if len(self.pose) < 2:
            return length
        self.vel.append(0.0) 
        for i in range(1,len(self.pose)):
            j = i - 1

            distance =  math.sqrt( math.pow(self.pose[i]['position']['x'] - self.pose[j]['position']['x'], 2) +
                                   math.pow(self.pose[i]['position']['y'] - self.pose[j]['position']['y'], 2))

            vel = distance / ((self.secs[i] - self.secs[j]) + (self.nsecs[i] - self.nsecs[j]) / math.pow(10,9))

            length += distance
            if vel > self.max_vel:
                self.max_vel = vel
            self.vel.append(vel) 
            
        self.length = length
        
    def to_JSON(self):
        return json.dumps(self, default=lambda o: o.__dict__, sort_keys=True, indent=4)

    def to_ros_msg(self):

        msg = Trajectory()
        msg.uuid = self.uuid
        msg.trajectory = []
        for i in range(len(self.pose)):
            
            ps = PoseStamped()
            ps.header.stamp.secs = self.secs[i]
            ps.header.stamp.nsecs = self.nsecs[i]            
            ps.pose = Pose()
            ps.pose.position.x = self.pose[i]['position']['x']
            ps.pose.position.y = self.pose[i]['position']['y']
            msg.trajectory.append(ps)
        msg.start_time = self.secs[0]
        msg.end_time = self.secs[-1] 
        
        return msg


def retrieve_trajectories():

    host = rospy.get_param('mongodb_host', 'localhost')
    port = rospy.get_param('mongodb_port', '62345')
    _client=pymongo.MongoClient(host,port)
    
    logs = _client.message_store.people_perception.find() #.sort({'header.stamp': 1})

    _traj = {}
    
    for log in logs:
        for i, uuid in enumerate(log['uuids']):
            if uuid not in _traj:
                t = TrajectoryCls(uuid)
                t.append_pose(log['people'][i],log['header']['stamp']['secs'],log['header']['stamp']['nsecs'])
                _traj[uuid] = t
            else:
                t = _traj[uuid]
                t.append_pose(log['people'][i],log['header']['stamp']['secs'],log['header']['stamp']['nsecs'])

    return _traj


def geojson_from_trajectory(msg):
    geojson = {}
    
    # trajectory UUID
    geojson['uuid'] = msg.uuid
    
    # space
    loc = {}
    loc['type'] = 'LineString'
    loc['coordinates'] = []
    for ps in msg.trajectory:
        xy_coord = [ps.pose.position.x, ps.pose.position.y]
        loc['coordinates'].append(xy_coord)
        geojson['loc'] = loc 

    # time
    geojson['start'] = msg.start_time
    geojson['end'] =   msg.end_time
    
    start_dt = datetime.fromtimestamp(msg.start_time)
    geojson['start_weekday'] = start_dt.weekday()
    geojson['start_hour'] = start_dt.hour
    geojson['start_minute'] = start_dt.minute
    
    end_dt = datetime.fromtimestamp(msg.end_time)
    geojson['end_weekday'] = end_dt.weekday()
    geojson['end_hour'] = end_dt.hour
    geojson['end_minute'] = end_dt.minute
    return geojson


if __name__=="__main__":

    rospy.init_node("trajectory_importer")    
    rospy.loginfo("Running trajectory_importer")

    gs = GeoSpatialStoreProxy('geospatial_store','soma')

    trajectories = retrieve_trajectories()
    
    count = 0
    for k, v in trajectories.iteritems():
        v.sort_pose()
        v.calc_stats()
        print k, len(v.pose), v.length, v.max_vel
        count += 1
        msg = v.to_ros_msg()
        
        geo_json = geojson_from_trajectory(msg)

        # in case trajectory is already in there => replace
        uuid = geo_json['uuid']
        gs.remove(uuid)
        gs.insert(geo_json)
        print "GEO_JSON inserted"

    # res = gs.find({'start_hour': { '$gt': 16, '$lt': 18}, # query
    #                'end_hour': { '$gt': 16, '$lt': 18}},
    #               {'uuid'}) # projection
    
    # print "GEO_JSON Objects found: " + str(res.count())
    # for obj in res:
    #     print obj
        
    rospy.spin()
