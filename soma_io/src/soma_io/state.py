import rospy
import copy
import numpy as np

from mongo import MongoDocument, MongoTransformable, MongoConnection
from geometry import Pose
from identification import ObjectIdentification
from observation import Observation,  MessageStoreObject
from exceptions import StateException
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import PointStamped, Point


class Object(MongoDocument):
    def __init__(self, mongo=None):
        super(Object, self).__init__()
        if mongo is not None:
            # this will create the document live..
            self._connect(mongo)
        self._children = []
        self._parent = None
        self._observations = None

        self._life_start = rospy.Time.now().to_time()
        self._life_end = None

        self.identifications = {}
        self.identification = ObjectIdentification()

        self._msg_store_objects =  [] # a list of object IDs (strings)

        self._observations =  [] # a list of observation objects

        self._poses = []
        self._point_cloud = None # will be a MessageStoreObject or None
        self._octree_cloud = None
        self._bounding_box = []

        self._spans = [] # for storing life spans as tuples (start,end)

    # TODO: properies for all, remove MongoDocument?

    @property
    def pose(self):
        if len(self._poses) < 1:
            raise StateException("NOPOSE")
        return copy.deepcopy(self._poses[-1])

    @property
    def position(self):
        if len(self._poses) < 1:
            raise StateException("NOPOSE")
        return copy.deepcopy(self._poses[-1].position)

    @property
    def quaternion(self):
        if len(self._poses) < 1:
            raise StateException("NOPOSE")
        return copy.deepcopy(self._poses[-1].quaternion)

    @property
    def pose_homog_transform(self):
        if len(self._poses) < 1:
            raise StateException("NOPOSE")
        return self._poses[-1].as_homog_matrix

    def cut(self, stamp=None):
        """
        Marks the end of the life of this object, adding an entry for its life
        span.
        """
        if self._life_end is not None:
            return
        if stamp is not None:
            self._life_end = stamp
        else:
            self._life_end = rospy.Time.now().to_time()
        self._spans.append((self._life_start, self._life_end))
        self._spans = self._spans

    def cut_all_children(self):
        world =  World()
        children = world.get_children(self.name, {'_life_end': None,})
        for i in children:
            i.cut()

    def set_life_start(self, start):
        self._life_start = start
        self._life_end = None

    @property
    def name(self):
        return self.key

    @name.setter
    def name(self, name):
        # TODO: check doesn't already exist
        self.key = name

    def add_identification(self, classifier_id, identification):
        if not self.identifications.has_key(classifier_id):
            self.identifications[classifier_id] = []
        # TODO check time consistency
        self.identifications[classifier_id].append(identification)
        self.identifications = self.identifications #force mongo update
        # TODO: don't duplicate? include classifier_id?
        self.identification = identification

    def add_pose(self, pose):
        # TODO check time consistency
        p = copy.deepcopy(pose)
        self._poses.append(p) #[str(p)] = p
        self._poses = self._poses #force mongo update

    def add_bbx(self, bbx):
        bb = copy.deepcopy(bbx)
        print bb
        self._bounding_box.append(bb)
        self._bounding_box = self._bounding_box

    def add_observation(self, observation):
        assert isinstance(observation,  Observation)
        self._observations.append(observation)
        self._observations =  self._observations

    def add_msg_store(self, message):
        assert isinstance(message, MessageStoreObject)
        self._msg_store_objects.append(message)
        self._msg_store_objects = self._msg_store_objects

    def get_identification(self, classifier_id=None):
        if classifier_id is None:
            return self.identification
        return self.identifications[classifier_id][-1]

    #FIXME: get_object try to get the '_parent' attribute, but in unittest its null, and get_object method search the key value.

    ##the value of _parent need to be decided
    def get_parent(self):
        world =  World()
        ##FIX the error in unittest that can't find the object
        ##self._parent = self.key

        parent = world.get_object(self._parent)
        return parent

    ##usually, when remove a object we need find the parent a call remove child
    def remove_child(self, child_name):
        try:
            self._children.remove(child_name)
        except:
            raise Exception("Trying to remove child from object, but"
                            "parent object is not parent of this child.")

    def get_children_names(self):
        return copy.copy(self._children)

    def add_child(self, child_object):
        #self._children.append(child_object.get_name)
        # have to recreate to catch in setattr
        assert child_object._parent is None # otherwise dual parentage is ok?
        child_object._parent = self.name
        self._children+=[child_object.name]

    def get_message_store_messages(self, typ=None):
        msgs = []
        proxy = MessageStoreProxy()
        for msg in self._msg_store_objects:
            if typ != msg.typ and typ is not None:
                continue
            proxy.database =  msg.database
            proxy.collection =  msg.collection
            msgs.append(proxy.query_id(msg.obj_id, msg.typ)[0])
        return msgs

    @classmethod
    def _mongo_encode(cls, class_object):
        doc = {}
        doc.update(class_object.__dict__)
        try:
            doc.pop("_MongoDocument__mongo")
            doc.pop("_MongoDocument__connected")
        except KeyError:
            print "Warning: no no no"
        doc["__pyobject_class_type"] = class_object.get_pyoboject_class_string()
        doc = copy.deepcopy(doc)
        return doc


class World(object):
    """
    Provide basic manipulation of database,
    such as check existence,
    create object and insert to a collection,
    delete object in a collection
    and query options
    """
    def __init__(self, database_name='world_state', server_host=None,
                 server_port=None):
        self._mongo = MongoConnection(database_name, server_host, server_port)

    #TODO: This can't handle the situation when two same object exist in two different room
    def does_object_exist(self, object_name):
        result = self._mongo.database.Objects.find(
            {"__pyobject_class_type": Object.get_pyoboject_class_string(),
             'key': object_name,})

        return result.count() == 1

    ##find object in 'Object' collection
    def get_object(self, object_name):
        result = self._mongo.database.Objects.find(
            {"__pyobject_class_type": Object.get_pyoboject_class_string(),
             'key': object_name,})

        if result.count() != 1:
            raise Exception("get_object failed to find object '%s' in database."%object_name)
        ##return the first match
        found = result[0]
        found._connect(self._mongo)
        return found

    ##create Objects collection
    def create_object(self, object_name=None):
        new_object = Object()
        rospy.loginfo("Connecting to mongo: ")
        if object_name is not None:
            new_object.name = object_name
        new_id = self._mongo.database.Objects.insert(Object._mongo_encode(new_object))
        new_object._id = new_id
        new_object._connect(self._mongo)
        return new_object

    ##remove object from 'Object' collection and save it to 'ObjectWasteland' collection
    def remove_object(self, obj):
        if not isinstance(obj, Object):
            obj = self.get_object(obj)
            ##FIXME: the indent is weird
        new_id = self._mongo.database.ObjectWasteland.save(Object._mongo_encode(obj))
        # Remove the object from its parent
        obj.get_parent().remove_child(obj.name)
        self._mongo.database.Objects.remove({
            "__pyobject_class_type": Object.get_pyoboject_class_string(),
             'key': obj.name, })


    def get_objects_of_type(self, ob_type, min_confidence=None):
        if min_confidence is None:
            result = self._mongo.database.Objects.find(
                {"__pyobject_class_type": Object.get_pyoboject_class_string(),
                 'identification.class_type': ob_type,})
        else:
            raise NotImplementedError("TODO: implement confidence check.")

        objs = []
        for r in result:
            r._connect(self._mongo)
            objs.append(r)
        return objs

    def get_root_objects(self):
        """
        return all objects that have no parent
        """
        return self.get_children(None)

    def get_children(self, parent, condition=None):
        """ Get the actual children objects given the condition """
        q = {'_parent': parent,
             "__pyobject_class_type": Object.get_pyoboject_class_string(),
              }
        q.update(condition)
        result = self._mongo.database.Objects.find(q)
        objs = []
        for r in result:
            r._connect(self._mongo)
            objs.append(r)
        return objs

    def query_normal(self, options, collection='Objects'):
        """
        Execute the normal query commands
        :param options: should be a string, then convert to dic in this function
        :param collection: collection name, default is Objects
        :return:
        """
        key = []
        val = []
        if isinstance(options, str):
            commands = options.split(',')
        for com in commands:
            tmp_key, tmp_val = com.split(':')
            key.append(tmp_key)
            val.append(tmp_val)
        commands = dict(zip(key, val))

        return self._mongo.database[collection].find(commands)

    def query_room(self, room_id):
        """
        Query how many observations in a room
        :param room_id: the name of the room where robot made observations, if not given will retrieve all the observations
        :return: False when no result found, else return a pymongo cursor
        """
        result = self._mongo.database.Objects.find({"key": room_id})
        if result.count() == 0:
            return False
        elif result.count() > 1:
            print "Error:Multi room with same name found, should be only one room!"
            return False
        else:
            obs_list = result[0]._observations
            return obs_list

    def query_object(self, obj_id, room_id=None):
        """
        Query a object in all the observations,
        this can return multi result when different contains same object
        :param obj_id: object name
        :param room_id: the name of the room where robot made observations, if not given will retrieve all the observations
        :return: False when no result found, else return a cursor object(pymongo cursor list)
        """
        if room_id is None:
            return self._mongo.database.Objects.find({"key": obj_id})
        else:
            r = self._mongo.database.Objects.find({"key": obj_id, "_parent": room_id})
            if r.count() == 0:
                return False
            else:
                return r[0]

    def query_point(self, point, room_id=None):
        """
        Query a point and return a list of observations that cover the point
        :param point: a three axis coordinate of a point
        :param room_id: the name of the room where robot made observations, if not given will retrieve all the observations
        :return: False when no result found, else return a observation list (pymongo cursor list)
        """
        if room_id is not None:
            cursor = self._mongo.database.Objects.find({"key": room_id})
        else:
            cursor = self._mongo.database.Objects.find({})
        if cursor.count() == 0:
            return False
        elif cursor.count() > 1:
            print "Error:Multi room with same name found, should be only one room!"

        q_point = Point()
        if isinstance(point, PointStamped):
            q_point.x = point.point.x
            q_point.y = point.point.y
            q_point.z = point.point.z
        elif isinstance(point, Point):
            q_point.x = point.x
            q_point.y = point.y
            q_point.z = point.z
        else:
            raise Exception("Unknow Point Type.")

        obs_list = []
        reflection = 0
        for bbx in cursor[0]._bounding_box:
            if bbx[0][0] > q_point.x and bbx[0][1] > q_point.y and bbx[0][2] > q_point.z \
                    and bbx[1][0] < q_point.x and bbx[1][1] < q_point.y and bbx[1][2] < q_point.z:
                obs_list.append(cursor[0]._observations[reflection])
            reflection += 1

        if len(obs_list) == 0:
            print "No observations covers the given point, end task."
            return False
        return obs_list
