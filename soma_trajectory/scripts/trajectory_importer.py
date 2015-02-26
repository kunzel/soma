#!/usr/bin/env python
import roslib; roslib.load_manifest("soma_trajectory")
import rospy
import pymongo
from datetime import datetime

from mongodb_store.message_store import MessageStoreProxy
from soma_geospatial_store.geospatial_store import GeoSpatialStoreProxy

from human_trajectory.trajectories import OfflineTrajectories


class TrajectoryImporter(object):

    def __init__(self):
        self._traj = dict()
        self.start_secs = -1
        rospy.loginfo("Connecting to mongodb...")

        self.gs = GeoSpatialStoreProxy('geospatial_store', 'soma')

        self._client = pymongo.MongoClient(rospy.get_param("mongodb_host"),
                                           rospy.get_param("mongodb_port"))
        self._store_client = MessageStoreProxy(collection="people_trajectories")

        self._traj = OfflineTrajectories()

        rospy.loginfo("Data is ready...")

    def store_all(self):
        for uuid, t in self._traj.traj.iteritems():
            msg = t.get_trajectory_message()
            geo_json = self.geojson_from_trajectory(msg)
            # in case trajectory is already in there => replace
            _uuid = geo_json['uuid']
            self.gs.remove(_uuid)
            self.gs.insert(geo_json)
            meta = dict()
            meta["map"] = 'library'
            self._store_client.insert(msg, meta)
            # rospy.loginfo("GEO_JSON stored for uuid: %s" % _uuid)

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
