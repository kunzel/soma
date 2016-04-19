import rospy
import os
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import genpy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import datetime
import math
import xmltodict
import identification
import observation as ob
import geometry
import msg_io
import octomap
import octree
import struct
import soma_math
import pcl
from octomap_msgs.msg import Octomap
import sensor_msgs.point_cloud2 as pypc2
import tf


def get_number(s):
    """ return the number (int) in a string. uggly assumptions made """
    s = [c for c in s if c in "0123456789"]
    return "".join(s)


class Importer(object):
    def __init__(self, pc2_topic):
        rospy.init_node('importer_node')
        # need specify the point cloud topic to run the importer
        self.pc2_topic = pc2_topic
        self.tf_listener = tf.TransformListener()
        self.frame_robot= self.frame_sensor= None
        self.frame_global = None

    def init_subscriber(self, frame_1, frame_2, frame_3):
        """
        init ROS topics to subscribe, also init the transform frame here.
        :param frame_1: robot_frame
        :param frame_2: point cloud sensor_frame
        :param frame_3: global_frame
        :return:
        """
        self.frame_robot = frame_1
        self.frame_sensor = frame_2
        self.frame_global = frame_3
        rospy.Subscriber(self.pc2_topic, PointCloud2, self.online_mode)
        rospy.spin()

    def online_mode(self, pc2_data):
        """
        The online mode should subscribe a topic which publishes the observation data periodically.
        These observation data are considered to store in the database, so it shouldn't subscribe
        to a topic that publishes data constantly.
        :param pc2_data: the point cloud data to be stored, PointCloud2 type.
        :return:
        """
        try:
            # Transformation from sensor to robot
            trans1, rot1 = self.tf_listener.lookupTransform(self.frame_robot, self.frame_sensor,
                                                            rospy.Time(pc2_data.header.stamp.secs,
                                                                       pc2_data.header.stamp.nsecs))
            # Transformation from robot to global frame
            trans2, rot2 = self.tf_listener.lookupTransform(self.frame_global, self.frame_robot,
                                                            rospy.Time(pc2_data.header.stamp.secs,
                                                                       pc2_data.header.stamp.nsecs))
        except tf.ConnectivityException:
            print "***connection problem occurs..."
            return
        except tf.LookupException:
            print "***encounter problem when look up transformaion in tf tree..."
            return
        except tf.ExtrapolationException:
            print "***extrapolation problem..."
            return

        # reconstruct a single tf message for the two frames
        pose1 = pose2 = PoseStamped()
        pose1.pose.position.x = trans1[0]
        pose1.pose.position.y = trans1[1]
        pose1.pose.position.z = trans1[2]
        pose2.pose.position.x = trans2[0]
        pose2.pose.position.y = trans2[1]
        pose2.pose.position.z = trans2[2]

        pose1.pose.orientation.x = rot1[0]
        pose1.pose.orientation.y = rot1[1]
        pose1.pose.orientation.z = rot1[2]
        pose1.pose.orientation.w = rot1[3]
        pose2.pose.orientation.x = rot2[0]
        pose2.pose.orientation.y = rot2[1]
        pose2.pose.orientation.z = rot2[2]
        pose2.pose.orientation.w = rot2[3]

        pose1.header = pc2_data.header
        pose2.header = pc2_data.header

        p1 = geometry.Pose.from_ros_msg(pose1)
        p2 = geometry.Pose.from_ros_msg(pose2)
        # TODO:check the transformaiton
        pose = geometry.Pose.from_homog(np.dot(p2.as_homog_matrix(),p1.as_homog_matrix()))
        t = pose.to_ros_tf()
        t.header.stamp.secs = pc2_data.header.stamp.secs
        t.header.stamp.nsecs = pc2_data.header.stamp.nsecs
        transform_store = ob.TransformationStore.create_from_transforms([t])
        # pointcloud data to octomap but with same header(for storage purpose)
        print "convert point cloud to octomap msg"
        oct_obj = octree.SOMAOctree()
        FileIO.pc2_to_octree(pc2_data, oct_obj.octree)
        oct_data = oct_obj.octree.writeBinary()
        # process octree stream
        info, data = oct_data.rsplit('\n', 1)
        oct_msg = Octomap()
        oct_msg.id = 'OcTree'
        oct_msg.binary = True  # True for bt file (compact binary version)
        oct_msg.resolution = 0.01
        # transform data to int8
        print "unpack octree data to msg"
        oct_msg.data = struct.unpack(str(len(data))+'b', data[0:len(data)])
        oct_msg.header.stamp.secs = pc2_data.header.stamp.secs
        oct_msg.header.stamp.nsecs = pc2_data.header.stamp.nsecs
        oct_msg.header.frame_id = '/octomap_topic/frame_id'

        # store tf,pointcloud data,store octomap data (pickled)
        print "Creating observations..."
        cloud_observation = ob.Observation.make_observation_from_messages([
            ("/tf", transform_store.pickle_to_msg()),
            ("/head_xtion/depth_registered/points", pc2_data),
            ("/octree_topic", msg_io.pickle_msg_data(oct_msg))])
        print "tf & point cloud & octomap importing Done."
        print "\n"*2

        pass

class FileIO(object):

    def __init__(self):
        # ros point cloud
        # TODO: ros pointcloud2 also can be processed?
        # self.pc2 = PointCloud2()
        pass

    @staticmethod
    def load_pcd(path):
        # TODO: load ply file, for the soma mesh
        """
        load point cloud form pcd file
        @param path: the whole path to the file
        @type  path: string
        @return: the point cloud
        @rtype: pcl::PointCloud2
        """
        pc2 = pcl.load(path)
        return pc2

    @staticmethod
    def save_pcd(self, point_cloud, path, _format=False):
        # TODO: can load ply file, for the soma mesh
        # TODO: should accept multi point_cloud format, currently can only use pcl PointCloud2
        pcl.save(point_cloud, path, _format)

    @staticmethod
    def remove_nan(cloud_in, cloud_out):
        """
        remove the nan point in not dense point cloud
        :param cloud_in: input not dense point cloud
        :param cloud_out: output dense point cloud
        :return:
        """
        # If the clouds are not the same, prepare the output
        if cloud_in != cloud_out:
            # attributes not writeable
            # cloud_out.sensor_origin = cloud_in.sensor_origin
            # cloud_out.sensor_orientation = cloud_in.sensor_orientation
            cloud_out.resize(cloud_in.size)

        # if the data is dense, we don't need to check fro nan
        if cloud_in.is_dense is True:
            # TODO: pycharm remind that its local variable?
            cloud_out = cloud_in
            return
        else:
            list_cloud =[]
            for point in cloud_in:
                if not math.isnan(point[0]) and not math.isnan(point[1]) and not math.isnan(point[2]):
                    list_cloud.append(point)

        ndarray_cloud = np.asarray(list_cloud, dtype=np.float32)
        cloud_out.from_array(ndarray_cloud)
        return

    @staticmethod
    def pc2_to_octree(point_cloud, octree, resolution=0.01, point=Point(0.0, 0.0, 0.0)):
        """
        convert ROS sensor_msgs.PointCloud2 msg to Octomap msg
        :param point_cloud:
        :param octree:
        :param resolution:
        :param point:
        :return:
        """
        if isinstance(point_cloud, PointCloud2):
            # sensor_msgs.PointCloud2 to array
            p = []
            for points in pypc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True):
                p.append([points[0], points[1], points[2]])
            cloud_array = np.asarray(p, dtype=np.float64)
        sensor_origin = np.array([point.x, point.y, point.z])
        octree.insertPointCloud(cloud_array, sensor_origin)

    @staticmethod
    def pcd_to_octree(point_cloud, octree, resolution=0.01, point=Point(0.0, 0.0, 0.0)):
        """
        :param point_cloud: point cloud to translate
        :type  point_cloud: pcl PointCloud2
        :param resolution: define octree resolution
        :type  resolution: float unit meter
        :param point:   sensor position, also can get form pcl::pointcloud type 'pc.sensor_origin'
        :type  ros geometry_msgs Point
        :return:  octomap octree
        """
        # Remove nan points, if the cloud is already a dense cloud jump this step
        result = []
        if not point_cloud.is_dense == True:
            # cloud_array = np.asarray(point_cloud)#already implemented in python-pcl api
            cloud_array = point_cloud.to_array()
            for mem in cloud_array:
                if not math.isnan(mem[0]):
                    result.append(mem)
            # pcl default is float32, octree need double
            print "remove nan points", "[origin]:", point_cloud.size, "[now]:", len(result)
            cloud_array = np.asarray(result, dtype=np.float64)

        # Generate Octree
        #tree = octomap.OcTree(resolution)
        if not point_cloud.is_dense == True:
            ndarray_cloud = cloud_array
        else:
            ndarray_cloud = np.asarray(point_cloud, dtype=np.float64)

        # usually the point should be the position of camera
        sensor_origin = np.array([point.x, point.y, point.z])
        octree.insertPointCloud(ndarray_cloud, sensor_origin)
        return

    @staticmethod
    def scan_file(input_path, suffix):
        # a base scan method return a file list with specified suffix
        file_list = []
        for find_path, folders, files in os.walk(input_path):
            for a_file in files:
                if a_file.endswith(suffix):
                    whole_file_path = os.path.join(find_path, a_file)
                    file_list.append(whole_file_path)

        file_list.sort()
        return file_list

    @staticmethod
    def scan_objects(input_path):
        """
        Go through all the folder under specified path to find txt file and extract the string append to a new file
        :return: a dictionary contains all the objects detected in the room
        """
        object_list = []
        object_dic = {}

        for find_path, folders, files in os.walk(input_path):
            for txt in files:
                part = txt.split('_')
                if len(part) == 4 and part[3].endswith(".txt"):
                    whole_file_path = os.path.join(find_path, txt)
                    with open(whole_file_path, "r") as txt_reader:
                        object_name = txt_reader.read()
                        object_name = object_name.strip()

                        # check for duplicate member
                        if object_name not in object_list:
                            object_list.append(object_name)

                            #add to object dictionary
                            if object_name[-1].isdigit():
                                for x in range(2, len(object_name)):
                                    if not object_name[-x].isdigit():
                                        object_dic[object_name] = object_name[:-x+1]
                                        break
                            else:
                                object_dic[object_name] = object_name
        return object_dic

    @staticmethod
    def scan_xml(input_path, file_name):
        """
        scan through all the folders find the specified file name(*.xml),
        :param file_name: file_name is room.xml in the KTH dataset and G4S dataset
        :return: a list contains more concrete description of the dataset(divide dataset by room number)
        """
        rooms=[]
        # os.path.walk is deprecated and removed in python3, use os.walk() instead
        for absolute_path, folders, files in os.walk(input_path):
            if file_name in files:
                with open(os.path.join(absolute_path, file_name), "r") as f_handle:
                    room = xmltodict.parse(f_handle)
                room_name = room['SemanticRoom']["RoomStringId"]
                rooms.append((absolute_path,
                              room_name,
                              files,
                              datetime.datetime.strptime(room['SemanticRoom']['RoomLogStartTime'],
                                                         "%Y-%b-%d %H:%M:%S.%f")))
        return rooms

    @staticmethod
    def get_xml_pose(xml_file, file_name):
        """
        get the pose of specified pcd file
        :param xml_file:
        :param file_name:
        :return:
        """
        with open(xml_file, 'r') as xml:
            room = xmltodict.parse(xml)

        transform = None
        for cloud_info in room['SemanticRoom']['RoomIntermediateClouds']['RoomIntermediateCloud']:
            if str(cloud_info['@filename']) == file_name:
                transform = cloud_info["RoomIntermediateCloudTransform"]
                transform = PoseStamped(
                    Header(0,
                           genpy.Time(int(transform['Stamp']['sec']),
                                      int(transform['Stamp']['nsec'])),
                           transform["FrameId"]),
                    Pose(Point(float(transform['Transform']['Translation']['x']),
                               (transform['Transform']['Translation']['y']),
                               float(transform['Transform']['Translation']['z'])),
                         Quaternion(float(transform['Transform']['Rotation']['x']),
                                    float(transform['Transform']['Rotation']['y']),
                                    float(transform['Transform']['Rotation']['z']),
                                    float(transform['Transform']['Rotation']['w']))))
        if transform is None:
            raise Exception("can't find the given file name in xml, name:", file_name)
        return transform

    @staticmethod
    def parse_room(world, dirname, files, class_lookup, pickle_octomap_data=True):
        files.sort()
        if "room.xml" in files:
            with open(os.path.join(dirname, "room.xml"), "r") as f:
                room = xmltodict.parse(f)
            room_name = room['SemanticRoom']["RoomStringId"]

            # Create room object in world_state if it is the first time to store
            """
            room object should contain all the observation in this room
            """
            if not world.does_object_exist(room_name):
                room_object = world.create_object(room_name)
            else:
                room_object = world.get_object(room_name)

            # Add the intermediate clouds as observations
            observations = {}
            for cloud_info in room['SemanticRoom']["RoomIntermediateClouds"]["RoomIntermediateCloud"]:
                print cloud_info["@filename"]
                print "Calculating cloud frame..."
                """
                read form xml get info about a point cloud
                transform: Ros Header, Pose are included
                transform_reg: Header, Pose are included
                """
                time_stamp = rospy.Time(int(cloud_info["RoomIntermediateCloudTransform"]['Stamp']['sec']),
                                        int(cloud_info["RoomIntermediateCloudTransform"]['Stamp']['nsec']))
                transform = cloud_info["RoomIntermediateCloudTransform"]
                transform = PoseStamped(Header(0,
                                               genpy.Time(int(transform['Stamp']['sec']),
                                                          int(transform['Stamp']['nsec'])),
                                               transform["FrameId"]),
                                        Pose(Point(float(transform['Transform']['Translation']['x']),
                                                   float(transform['Transform']['Translation']['y']),
                                                   float(transform['Transform']['Translation']['z'])),
                                             Quaternion(float(transform['Transform']['Rotation']['x']),
                                                        float(transform['Transform']['Rotation']['y']),
                                                        float(transform['Transform']['Rotation']['z']),
                                                        float(transform['Transform']['Rotation']['w']))))
                transform_reg = cloud_info["RoomIntermediateCloudTransformRegistered"]
                transform_reg = PoseStamped(Header(0,
                                                   genpy.Time(int(transform_reg['@Stamp_sec']),
                                                              int(transform_reg['@Stamp_nsec'])),
                                                   transform_reg["@FrameId"]),
                                            Pose(Point(float(transform_reg['@Trans_x']),
                                                       float(transform_reg['@Trans_y']),
                                                       float(transform_reg['@Trans_z'])),
                                                 Quaternion(float(transform_reg['@Rot_x']),
                                                            float(transform_reg['@Rot_y']),
                                                            float(transform_reg['@Rot_z']),
                                                            float(transform_reg['@Rot_w']))))
                """
                pose are extracted form above generated two ros msg
                the final 'pose' is generated by multiplying original pose and transformation matrix
                """
                p1 = geometry.Pose.from_ros_msg(transform)
                p2 = geometry.Pose.from_ros_msg(transform_reg)
                pose = geometry.Pose.from_homog(np.dot(p2.as_homog_matrix(),
                                                                   p1.as_homog_matrix()))
                print "ok."

                """
                load point cloud from pcd file and add ros header for cloud.
                load octomap from bt file, same name as point cloud file.
                """
                print "Reading cloud PCD..."
                pcd_name = os.path.join(dirname, cloud_info['@filename'])
                cloud = msg_io.read_pcd(pcd_name, get_tf=False)
                print "done."

                # load octree data
                print "Reading octomap BT..."
                octree_name = pcd_name[:-3] + "bt"
                octo_msg = msg_io.read_bt(octree_name)
                print "done."

                """
                generate a tf message 't'
                define a Header for point cloud data
                """
                t = pose.to_ros_tf()
                t.header.stamp.secs = time_stamp.secs
                t.header.stamp.nsecs = time_stamp.nsecs
                # init head for octomap and point cloud
                octo_msg.header.stamp.secs = cloud.header.stamp.secs = time_stamp.secs
                octo_msg.header.stamp.nsecs = cloud.header.stamp.nsecs = time_stamp.nsecs

                """
                Prepare for sensor_msgs.PointCloud2 observation
                """
                print "Creating cloud observations..."
                transform_store = ob.TransformationStore.create_from_transforms([t])

                """
                Here tf and point cloud is stored in 'message_store' database,
                observation is stored in 'world_state', it contains the index_id of the above two data.
                e.g. in which database the topic is stored? which collection? what's the topic type? the identification?
                """
                # insert zlib pickled tf message, point cloud message to database
                cloud_observation = ob.Observation.make_observation_from_messages(
                    [("/tf", transform_store.pickle_to_msg()),
                     ("/head_xtion/depth_registered/points", cloud)])

                cloud_observation.stamp = time_stamp.to_time()
                observations[get_number(cloud_info["@filename"])] = cloud_observation

                # insert zlib pickled octomap message
                if pickle_octomap_data is True:
                    octo_observation = ob.Observation.make_observation_from_messages(
                        [("/octree_topic", msg_io.pickle_msg_data(octo_msg))])
                else:
                    octo_observation = ob.Observation.make_observation_from_messages(
                     [("/octree_topic", msg_io.pickle_msg(octo_msg))])
                print "ok."

                # add the new observation to this room_observation index in 'Object' collection
                print "Adding room cloud observation to index..."
                room_object.add_observation(cloud_observation)
                print "ok"

                """
                create new collection to store extra Octree info
                """
                print "prepare for the transformed binding box information..."
                # load octomap data
                octr = octree.SOMAOctree()
                if octr.load_tree(octree_name):
                    bbx_info = octr.bbx_info
                else:
                    raise Exception('failed when loading octomap')

                xml_file = os.path.join(dirname, "room.xml")
                file_name = cloud_info["@filename"]
                gl_pose = FileIO.get_xml_pose(xml_file, file_name)
                rot_mat = soma_math.quaternion_to_matrix(gl_pose.pose.orientation)

                max_p = np.dot(rot_mat, [[bbx_info['max'][0]], [bbx_info['max'][1]], [bbx_info['max'][2]]])
                max_p[0] += float(gl_pose.pose.position.x)
                max_p[1] += float(gl_pose.pose.position.y)
                max_p[2] += float(gl_pose.pose.position.z)

                min_p = np.dot(rot_mat, [[bbx_info['min'][0]], [bbx_info['min'][1]], [bbx_info['min'][2]]])
                min_p[0] += float(gl_pose.pose.position.x)
                min_p[1] += float(gl_pose.pose.position.y)
                min_p[2] += float(gl_pose.pose.position.z)

                bbx_info_list = []
                bbx_info_list.append(list(bbx_info['max']))
                bbx_info_list.append(list(bbx_info['min']))

                for i in xrange(0, 3):
                    bbx_info_list[0][i] = float(max_p[i])
                    bbx_info_list[1][i] = float(min_p[i])

                print "insert binding box information"
                print id(room_object)
                #print room_object.pose
                room_object.add_bbx(bbx_info_list)

            # What objects are "live" in this room at this point in time
            """
            live object should get from the tf, as we are load data from file 'live_object' is blank
            """
            live_objects = map(lambda x: x.name, world.get_children(room_name, {'_life_end': None,}))


            """
            find labeled object add it to 'Object' collection. If the object is the same
            object we added before, add info to sub-member '_observations' and '_poses'
            the data of a object contains
            """
            # What objects are labeled in this room
            for fl in files:
                f = fl.split("_")
                if len(f) == 4 and f[3].endswith(".pcd") and f[2] == "label":
                    obs_id = get_number(f[1])
                    # read the label file
                    with open(os.path.join(dirname, fl[:-3] + "txt"), "r") as label_file:
                        object_name = label_file.read()
                        object_name = object_name.strip()

                    print "Got labled object '{}'".format(object_name)
                    print object_name

                    if object_name in live_objects:
                        live_objects.remove(object_name)

                    if not world.does_object_exist(object_name):
                        object = world.create_object(object_name)
                        object._parent = room_object.name
                        object.add_identification("TRUTH",
                                                  identification.ObjectIdentification(
                                                      {class_lookup[object_name]: 1,},
                                                      {object_name: 1,}))
                    else:
                        object = world.get_object(object_name)

                    """
                    copy observation info we generated before and to this object observation
                    this object also contains index info that can be retrieved
                    """
                    observation = ob.Observation.copy(observations[obs_id])
                    src_cloud = observation.get_message("/head_xtion/depth_registered/points")
                    stamp = src_cloud.header.stamp
                    camera_pose = ob.TransformationStore.unpickle(
                        observation.get_message("/tf").msg)._transformations[0]
                    time = rospy.Time(stamp.secs, stamp.nsecs).to_time()

                    #when loading data form file time equals __life_start
                    if time < object._life_start:
                        object._life_start = time

                    # Find the pose of the object in the map frame...
                    # t = world_state.geometry.Pose.from_ros_msg(camera_pose).as_homog_matrix()
                    """
                    read the point cloud and calculate the object center
                    """
                    cloud = msg_io.read_pcd(os.path.join(dirname, fl), get_tf=False)
                    cloud.header = src_cloud.header
                    cent = np.array([0.0, 0.0, 0.0, 0.0])
                    cnt = 0
                    for pt in pc2.read_points(cloud):
                        cent += np.array(pt)
                        cnt += 1
                    cent /= cnt
                    print cent
                    pose = geometry.Pose()

                    pose.position.x, pose.position.y, pose.position.z, turd = cent
                    object.add_pose(pose)
                    observation.add_message(cloud, "object_cloud")
                    object.add_observation(observation)

            # Any object not observed this time in the room should be marked dead
            for obj in live_objects:
                print "Did not see ", obj, " this time..."
                world.get_object(obj).cut(time)


