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

from mongodb_store.message_store import MessageStoreProxy
from soma_geospatial_store.geospatial_store import GeoSpatialStoreProxy 

from human_trajectory.trajectory import Trajectory


class TrajectoryImporter(object):

    def __init__(self):
        self._traj = dict()
        self.start_secs = -1
        rospy.loginfo("Connecting to mongodb...")

        self.gs = GeoSpatialStoreProxy('geospatial_store','soma')
        
        self._client = pymongo.MongoClient(rospy.get_param("mongodb_host"),
                                           rospy.get_param("mongodb_port"))
        self._store_client = MessageStoreProxy(collection="people_trajectories")

        self._retrieve_logs()
        self._validating_trajectories()
        rospy.loginfo("Data is ready...")

    def get_poses_persecond(self):
        average_poses = 0
        for uuid in self._traj:
            traj = self._traj[uuid]
            inner_counter = 1
            outer_counter = 1
            prev_sec = traj.secs[0]
            for i, sec in enumerate(traj.secs[1:]):
                if prev_sec == sec:
                    inner_counter += 1
                else:
                    prev_sec = sec
                    outer_counter += 1
            average_poses += round(inner_counter/outer_counter)
        return round(average_poses/len(self._traj))

    def _validating_trajectories(self):
        rospy.loginfo("Sorting data...")
        untraj = []
        mframe = self.get_poses_persecond() * 5
        for uuid in self._traj:
            self._traj[uuid].sort_pose()
            self._traj[uuid].calc_length()

            if self._traj[uuid].length < 0.1 and uuid not in untraj:
                untraj.append(uuid)
            if len(self._traj[uuid].pose) < mframe and uuid not in untraj:
                untraj.append(uuid)

        rospy.loginfo("Validating data...")
        for i, uuid in enumerate(untraj):
            del self._traj[uuid]

    def _retrieve_logs(self):
        rospy.loginfo("Retrieving data from mongodb...")
        logs = self._client.message_store.people_perception.find()

        for log in logs:
            for i, uuid in enumerate(log['uuids']):
                if uuid not in self._traj:
                    t = Trajectory(uuid)
                    t.append_pose(log['people'][i],
                                  log['header']['stamp']['secs'],
                                  log['header']['stamp']['nsecs'],
                                  log['robot'])
                    self._traj[uuid] = t
                else:
                    t = self._traj[uuid]
                    t.append_pose(log['people'][i],
                                  log['header']['stamp']['secs'],
                                  log['header']['stamp']['nsecs'],
                                  log['robot'])
                if self.start_secs == -1 or \
                        log['header']['stamp']['secs'] < self.start_secs:
                    self.start_secs = log['header']['stamp']['secs']


    def store_all(self):
        for uuid, t in self._traj.iteritems():
            msg = t.get_trajectory_message()
            geo_json = self.geojson_from_trajectory(msg)
            # in case trajectory is already in there => replace
            _uuid = geo_json['uuid']
            self.gs.remove(_uuid)
            self.gs.insert(geo_json)
            meta = dict()
            meta["map"] = 'library'
            self._store_client.insert(msg, meta)
            #rospy.loginfo("GEO_JSON stored for uuid: %s" % _uuid)



    def geojson_from_trajectory(self, msg):
        geojson = {}
    
        # trajectory UUID
        geojson['uuid'] = msg.uuid
        
        # space
        loc = {}
        loc['type'] = 'LineString'
        loc['coordinates'] = []
        for ps in msg.trajectory:
            xy_coord = self.gs.coords_to_lnglat(ps.pose.position.x, ps.pose.position.y)
            loc['coordinates'].append(xy_coord)
            geojson['loc'] = loc 

        # time
        geojson['start'] = msg.start_time.to_sec()
        geojson['end'] =   msg.end_time.to_sec() 
    
        start_dt = datetime.fromtimestamp(msg.start_time.to_sec())
        geojson['start_weekday'] = start_dt.weekday()
        geojson['start_hour'] = start_dt.hour
        geojson['start_minute'] = start_dt.minute
        
        end_dt = datetime.fromtimestamp(msg.end_time.to_sec())
        geojson['end_weekday'] = end_dt.weekday()
        geojson['end_hour'] = end_dt.hour
        geojson['end_minute'] = end_dt.minute
        return geojson


if __name__=="__main__":

    rospy.init_node("trajectory_importer")    
    rospy.loginfo("Running trajectory_importer")
    ti = TrajectoryImporter()

    rospy.loginfo("Importing trajectories...")
    ti.store_all()
    rospy.loginfo("Finished import")

    # res = gs.find({'start_hour': { '$gt': 16, '$lt': 18}, # query
    #                'end_hour': { '$gt': 16, '$lt': 18}},
    #               {'uuid'}) # projection
    
    # print "GEO_JSON Objects found: " + str(res.count())
    # for obj in res:
    #     print obj
        
    rospy.spin()
