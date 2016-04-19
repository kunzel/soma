#!/usr/bin/env python

import roslib; roslib.load_manifest("soma_trajectory")
import sys
import rospy
import pymongo
from datetime import datetime

from mongodb_store.message_store import MessageStoreProxy
from soma_geospatial_store.geospatial_store import GeoSpatialStoreProxy

from human_trajectory.trajectories import OfflineTrajectories
from human_trajectory.msg import Trajectories


class TrajectoryImporter(object):

    def __init__(self, online):
        self._traj = dict()
        rospy.loginfo("Connecting to mongodb...")

        self.gs = GeoSpatialStoreProxy('geospatial_store', 'soma')

        if not online:
            # self._client = pymongo.MongoClient(rospy.get_param("mongodb_host"),
            #                                    rospy.get_param("mongodb_port"))
            # self._store_client = MessageStoreProxy(
            #     collection="people_trajectory"
            # )
            self._traj = dict()
            trajs = OfflineTrajectories()
            for uuid, traj in trajs.traj.iteritems():
                self._traj[uuid] = traj.get_trajectory_message()
        else:
            rospy.loginfo("Subscribing to /human_trajectories/trajectories...")
            rospy.Subscriber(
                "/human_trajectories/trajectories/complete", Trajectories,
                self.traj_callback, None, queue_size=10
            )

    def store_all(self):
        for uuid, t in self._traj.items():
            # msg = t.get_trajectory_message()
            # geo_json = self.geojson_from_trajectory(msg)
            geo_json = self.geojson_from_trajectory(t)
            # in case trajectory is already in there => replace
            _uuid = geo_json['uuid']
            self.gs.remove(_uuid)
            rospy.loginfo("Storing %s data to geospatial_store", uuid)
            self.gs.insert(geo_json)
            # meta = dict()
            # meta["map"] = map
            # rospy.loginfo("Storing %s in geo
            # self._store_client.insert(msg, meta)
            del self._traj[uuid]

        rospy.sleep(0.5)

    def traj_callback(self, msg):
        self._traj.update({
            traj.uuid: traj for i, traj in enumerate(msg.trajectories)
        })

    def geojson_from_trajectory(self, msg):
        geojson = {}

        # trajectory UUID
        geojson['uuid'] = msg.uuid

        # space
        loc = {}
        loc['type'] = 'LineString'
        loc['coordinates'] = []
        for ps in msg.trajectory:
            xy_coord = self.gs.coords_to_lnglat(
                ps.pose.position.x, ps.pose.position.y
            )
            loc['coordinates'].append(xy_coord)
            geojson['loc'] = loc

        # time
        geojson['start'] = msg.start_time.to_sec()
        geojson['end'] = msg.end_time.to_sec()

        start_dt = datetime.fromtimestamp(msg.start_time.to_sec())
        geojson['start_weekday'] = start_dt.weekday()
        geojson['start_hour'] = start_dt.hour
        geojson['start_minute'] = start_dt.minute

        end_dt = datetime.fromtimestamp(msg.end_time.to_sec())
        geojson['end_weekday'] = end_dt.weekday()
        geojson['end_hour'] = end_dt.hour
        geojson['end_minute'] = end_dt.minute
        return geojson


if __name__ == "__main__":

    rospy.init_node("trajectory_importer")
    rospy.loginfo("Running trajectory_importer")
    if len(sys.argv) < 2:
        rospy.logerr("usage: trajectory online/offline[1/0]")
        sys.exit(2)

    ti = TrajectoryImporter(int(sys.argv[1]))
    if not int(sys.argv[1]):
        rospy.loginfo("Importing trajectories...")
        ti.store_all()
        rospy.loginfo("Finished import")
    else:
        rospy.loginfo("trajectory importer is running online...")
        while not rospy.is_shutdown():
            ti.store_all()

    # res = gs.find({'start_hour': { '$gt': 16, '$lt': 18}, # query
    #                'end_hour': { '$gt': 16, '$lt': 18}},
    #               {'uuid'}) # projection

    # print "GEO_JSON Objects found: " + str(res.count())
    # for obj in res:
    #     print obj

    rospy.spin()
