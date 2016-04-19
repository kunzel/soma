#!/usr/bin/env python
import rospy
import argparse
import sys
from soma_io.state import World
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import PointStamped, Point

"""
A Query tool that can handle the query task of MessageStore message and normal DB data.
usage example:
```
#Query 'room' Object instance
$world_tool.py -q key:WayPoint42
#Multi query commands
$world_tool.py -q key:bin2,_parent:WayPoint42
#Query how much observations in a room that covers the point
$world_tool.py -r WayPoint42 -p 22,24,1
#Query how much observations in a room that record in database
$world_tool.py -r WayPoint42
```
"""

if __name__ == '__main__':
    # Prepare parameters
    parser = argparse.ArgumentParser(prog='world_tool.py')
    parser.add_argument("-dbpath", metavar='database address and port number',
                        help="format: database_address:port_number, default: localhost:62345")
    parser.add_argument("-q", metavar='query content',
                        help="query commands: field1:field_content1,field2:field_content2...")
    parser.add_argument("-r", metavar='room string',
                        help="query options to specify a room")
    parser.add_argument("-p", metavar='a query point to check which observations covers this point',
                        help="example: -p 1.0,2.0,3.0 the order is x,y,z")
    parser.add_argument("-obj", metavar='find the given object name in database',
                        help="example: -obj bin1")

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    # Setup Connection
    rospy.init_node("world_tools")
    if args.dbpath is None:
        print "No database specified, trying to connection to default database..."
        # handle world_state database
        world = World()
        msg_store = MessageStoreProxy(database='message_store', collection='ws_observations')
    else:
        addr, port = args.d.split(':')
        # use default database name 'world_state', but change the database address
        world = World(server_host=addr, server_port=port)
        msg_store = MessageStoreProxy(database='message_store', collection='ws_observations')

    # Query
    if args.q is not None:
        print "Receive normal query task: " + args.q
        result = world.query_normal(args.q)
        print "Got " + str(result.count()) + "  results"

    # Query how much observations covered a given Point
    if args.p is not None:
        if args.r is None:
            room_str = "not specified"
        else:
            room_str = args.r

        _x, _y, _z = args.p.split(',')
        query_p = Point()
        query_p.x = _x = float(_x)
        query_p.y = _y = float(_y)
        query_p.z = _z = float(_z)
        print "Checking coverd observations in " + room_str + " room "
        print "Point: (" + str(query_p.x) + "," + str(query_p.y) + "," + str(query_p.z) + ")"
        result = world.query_point(query_p, room_str)

        # As a room is unique in the database, so result can only be 0 or 1
        if result is False:
            print "No result found!"
            exit()
        print "Got " + str(len(result)) + " observations that match the condition"

    # Query how much observations has been done in the given room id
    if args.p is None and args.q is None and args.r is not None:
        print "searching observasions amount in a room..."
        result = world.query_room(args.r)
        print "Got " + str(len(result)) + " Observations in room " + args.r
