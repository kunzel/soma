from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Transform
from octomap_msgs.msg import Octomap
from mongodb_store.message_store import MessageStoreProxy
from mongodb_store_msgs.msg import SerialisedMessage
from soma_msgs.msg import PickledOctomap


import sensor_msgs.point_cloud2 as pc2
import cPickle as pickle
import zlib
import struct
import os


def datatype_to_size_type(datatype):
    """
    Takes a UINT8 datatype field from a PointFields and returns the size in
    bytes and a char for its type ('I': int, 'U': unsigned, 'F': float)
    """
    # might be nicer to look it up in message definition but quicker just to
    # do this.
    if datatype in [2, 3, 4]:
        t = 'U'
    elif datatype in [1, 3, 5]:
        t = 'I'
    elif datatype in [7, 8]:
        t = 'F'
    else:
        raise Exception("Unknown datatype in PointField")

    if datatype < 3:
        s = 1
    elif datatype < 5:
        s = 2
    elif datatype < 8:
        s = 4
    elif datatype < 9:
        s = 8
    else:
        raise Exception("Unknown datatype in PointField")

    return s, t


def size_type_to_datatype(size, type):
    """
    Given a .pcd size/type pair, return a sensor_msgs/PointField datatype
    """
    if type == "F":
        if size == 4:
            return 7
        if size == 8:
            return 8
    if type == "I":
        if size == 1:
            return 1
        if size == 2:
            return 3
        if size == 4:
            return 5
    if type == "U":
        if size == 1:
            return 2
        if size == 2:
            return 4
        if size == 4:
            return 6
    raise Exception("Unknown size/type pair in .pcd")


def write_pcd(filename,  pointcloud, overwrite=False, viewpoint=None,
              mode='binary'):
    """
    Writes a sensor_msgs::PointCloud2 to a .pcd file.
    :param filename - the pcd file to write
    :param pointcloud - sensor_msgs::PointCloud2 to write to a file
    :param overwrite - if True, allow overwriting existing files
    :param viewpoint - the camera viewpoint, (x,y,z,qw,qx,qy,qz)
    :param mode - the writing mode: 'ascii' for human readable, 'binary' for
                  a straight dump of the binary data, 'binary_stripped'
                  to strip out data padding before writing (saves space but it slow)
    """
    assert isinstance(pointcloud, PointCloud2)
    if mode not in ['ascii', 'binary', 'binary_stripped']:
        raise Exception("Mode must be 'binary' or 'ascii'")
    if not overwrite and os.path.isfile(filename):
        raise Exception("File exists.")
    try:
        with open(filename, "w") as f:
            f.write("VERSION .7\n")
            _size =  {}
            _type = {}
            _count = {}
            _offsets = {}
            _fields = []
            _size['_'] = 1
            _count['_'] = 1
            _type['_'] = 'U'
            offset = 0
            for field in pointcloud.fields:
                if field.offset != offset:
                    # some padding
                    _fields.extend(['_']*(field.offset - offset))
                isinstance(field, PointField)
                _size[field.name], _type[field.name] = datatype_to_size_type(field.datatype)
                _count[field.name] = field.count
                _offsets[field.name] = field.offset
                _fields.append(field.name)
                offset = field.offset + _size[field.name] * _count[field.name]
            if pointcloud.point_step != offset:
                _fields.extend(['_']*(pointcloud.point_step - offset))


            if mode != 'binary':
                #remove padding fields
                while True:
                    try:
                        _fields.remove('_')
                    except:
                        break

            #_fields = _count.keys()
            _fields_str =  reduce(lambda a, b: a +  ' ' + b,
                                  map(lambda x: "{%s}" % x,
                                      _fields))            

            f.write("FIELDS ")
            f.write(reduce(lambda a, b: a + ' ' + b,
                           _fields))
            f.write("\n")
            f.write("SIZE ")
            f.write(_fields_str.format(**_size))
            f.write("\n")
            f.write("TYPE ")
            f.write(_fields_str.format(**_type))
            f.write("\n")
            f.write("COUNT ")
            f.write(_fields_str.format(**_count))

            f.write("\n")
            f.write("WIDTH %s" % pointcloud.width)
            f.write("\n")
            f.write("HEIGHT %s" % pointcloud.height)
            f.write("\n")

            if viewpoint is None:
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            else:
                try:
                    assert len(viewpoint) == 7
                except:
                    raise Exception("viewpoint argument must be  tuple "
                                    "(x y z qx qy qz qw)")
                f.write("VIEWPOINT {} {} {} {} {} {} {}\n".format(*viewpoint))

            f.write("POINTS %d\n" % (pointcloud.width * pointcloud.height))
            if mode == "binary":
                #TODO: check for row padding.
                f.write("DATA binary\n")
                f.write(bytearray(pointcloud.data))
            elif mode ==  "binary_stripped":
                f.write("DATA binary\n")
                if pointcloud.point_step == sum([v[0]*v[1] for v in zip(_size.values(),
                                                                        _count.values())]): #danger, assumes ordering
                    # ok to just blast it all out; TODO: this assumes row step has no padding
                    f.write(bytearray(pointcloud.data))
                else:
                    # strip all the data padding
                    _field_step = {}
                    for field in _fields:
                        _field_step[field] = _size[field] * _count[field]
                    out =  bytearray(sum(_field_step.values())*pointcloud.width*pointcloud.height)
                    b = 0
                    for v in range(pointcloud.height):
                        offset = pointcloud.row_step * v
                        for u in range(pointcloud.width):
                            for field in _fields:
                                out[b:b+_field_step[field]] = pointcloud.data[offset+_offsets[field]:
                                                                              offset+_offsets[field]+_field_step[field]]
                                b += _field_step[field]
                            offset += pointcloud.point_step
                    f.write(out)
            else:
                f.write("DATA ascii\n")
                for p in pc2.read_points(pointcloud,  _fields):
                    for i, field in enumerate(_fields):
                        f.write("%f " % p[i])
                    f.write("\n")

    except IOError,  e:
        raise Exception("Can't write to %s: %s" %  (filename, e.message))


def read_pcd(filename, cloud_header=None, get_tf=True):
    if not os.path.isfile(filename):
        raise Exception("[read_pcd] File does not exist.")
    string_array =  lambda x: x.split()
    float_array  =  lambda x: [float(j) for j in x.split()]
    int_array  =  lambda x: [int(j) for j in x.split()]
    word =  lambda x: x.strip()
    headers =  [("VERSION", float),
               ("FIELDS", string_array),
               ("SIZE", int_array),
               ("TYPE", string_array),
               ("COUNT", int_array),
               ("WIDTH", int),
               ("HEIGHT", int),
               ("VIEWPOINT", float_array),
               ("POINTS", int),
               ("DATA", word)]
    header = {}
    with open(filename, "r") as pcdfile:
        while len(headers) > 0:
            line = pcdfile.readline()
            if line == "":
                raise Exception("[read_pcd] EOF reached while looking for headers.")
            f, v = line.split(" ", 1)
            if f.startswith("#"):
                continue
            if f not in zip(*headers)[0]:
                raise Exception("[read_pcd] Field '{}' not known or duplicate.".format(f))
            func =  headers[zip(*headers)[0].index(f)][1]
            header[f] = func(v)
            headers.remove((f, func))
        data = pcdfile.read()
    # Check the number of points
    if header["VERSION"] != 0.7:
        raise Exception("[read_pcd] only PCD version 0.7 is understood.")
    if header["DATA"] != "binary":
        raise Exception("[read_pcd] Only binary .pcd files are readable.")
    if header["WIDTH"] * header["HEIGHT"] != header["POINTS"]:
        raise Exception("[read_pcd] POINTS count does not equal WIDTH*HEIGHT")
    
    cloud = PointCloud2()
    cloud.point_step = sum([size * count
                            for size, count in zip(header["SIZE"], header["COUNT"])])
    cloud.height = header["HEIGHT"]
    cloud.width = header["WIDTH"]
    cloud.row_step = cloud.width * cloud.point_step
    cloud.is_bigendian = False
    if cloud.row_step * cloud.height > len(data):
        raise Exception("[read_pcd] Data size mismatch.")
    offset = 0
    for field, size, type, count in zip(header["FIELDS"],
                                        header["SIZE"],
                                        header["TYPE"],
                                        header["COUNT"]):
        
        if field != "_":
            pass
        pf =  PointField()
        pf.count = count
        pf.offset = offset
        pf.name = field
        pf.datatype = size_type_to_datatype(size, type)
        cloud.fields.append(pf)
        offset += size * count
        
    cloud.data = data[0:cloud.row_step * cloud.height]
    if cloud_header is not None:
        cloud.header = header
    else:
        cloud.header.frame_id = "/pcd_cloud"
        
    if get_tf:
        tf = Transform()
        tf.translation.x, tf.translation.y, tf.translation.z =  header["VIEWPOINT"][0:3]
        tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z =  header["VIEWPOINT"][3:]
        
        return cloud, tf
    
    return cloud


def write_bt():
    pass


def read_bt(filename, oct_header=None, get_tf=True):
    """
    Transfrom bt file to octomap_msgs::Octomap message
    :param filename: path to bt file
    :param oct_header:
    :param get_tf:
    :return: the Octomap message
    """
    if not os.path.isfile(filename):
        raise Exception("[read_pcd] File does not exist.")

    # Only the resolution & id & binary & data info is used in Octomap.msg
    headers = [
               ("size", int),
               ("res", float)]

    header = {}
    with open(filename, "r") as btfile:
        # Skip the '#' comment
        # miss the line of id but all the id are the same we can add it manually
        while btfile.readline()[0] is "#":
            continue

        # start form the the line 5 (size defined here)
        while len(headers) > 0:
            line = btfile.readline()
            if line == "":
                raise Exception("[read_bt] EOF reached while looking for headers.")
            f, v = line.split(" ", 1)
            if f not in zip(*headers)[0]:
                raise Exception("[read_bt] Field '{}' not known or duplicate.".format(f))
            func = headers[zip(*headers)[0].index(f)][1]
            header[f] = func(v)
            headers.remove((f, func))

        # Skip the data line
        line = btfile.readline()
        # read the left data
        data = btfile.read()

    # Create octomap message
    oct_msg = Octomap()
    oct_msg.id = 'OcTree'
    oct_msg.binary = True  # True for bt file (compact binary version)
    oct_msg.resolution = header["res"]
    # transform data to int8
    oct_msg.data = struct.unpack(str(len(data))+'b', data[0:len(data)])

    print "octree data list size is "+str(len(oct_msg.data))

    if oct_header is not None:
        if isinstance(oct_header, oct_msg.header):
            oct_msg.header = oct_header
        else:
            raise Exception("Unknown Header type " + type(oct_header))
    else:
        oct_msg.header.frame_id = "/from_octomap_file"

    return oct_msg


def pickle_msg(input_msg):
    if isinstance(input_msg, Octomap):
        s = SerialisedMessage()
        s.msg = zlib.compress(pickle.dumps(input_msg, protocol=pickle.HIGHEST_PROTOCOL))
        s.type = 'pickled_octomap_msg'
        return s
    else:
        raise Exception('unknown msg type')


def pickle_msg_data(input_msg):
    """
    Only want to pickle the data field thus other info can be eassily access from robomongo
    pickle the data field in ros message
    :param input_msg: ros message
    :return: only data field pickled message
    """

    if isinstance(input_msg, Octomap):
        # Convert to PickledOctomap.msg (only change the data field)
        pickled_oct = PickledOctomap()

        pickled_oct.binary = input_msg.binary
        pickled_oct.header = input_msg.header
        pickled_oct.id = input_msg.id
        pickled_oct.resolution = input_msg.resolution
        pickled_oct.pickled_data = zlib.compress(pickle.dumps(input_msg.data, protocol=pickle.HIGHEST_PROTOCOL))

        return pickled_oct
    else:
        raise Exception("input message object illegal, only support Octomap.msg and PointCloud2.msg")


def unpickle(pickle_string):
    return pickle.loads(zlib.decompress(pickle_string))


class SOMAMsg(object):
    """
    manipulate the message processing, such as:
    create message form file (support *.pcd *.bt files),
    pickle and unpickle message when storing to DB or loading from DB,
    """
    def __init__(self, msg_obj):
        # Load Msg
        if msg_obj is not Octomap or PointCloud2:
            raise Exception('unsupported msg type')
        if msg_obj is Octomap:
            self.octomap = msg_obj
        if msg_obj is PointCloud2:
            pass

    def pickle(self):
        return zlib.compress(pickle.dumps(self.octomap.data, protocol=pickle.HIGHEST_PROTOCOL))

    def pickle_to_msg(self):
        s = SerialisedMessage()
        s.msg = pickle(self)
        s.type = "zlibed_pickled_data"
        return s




