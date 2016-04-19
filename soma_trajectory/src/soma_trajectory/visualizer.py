#!/usr/bin/env python
import roslib
roslib.load_manifest("soma_trajectory")
import rospy
import random
import math
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import ColorRGBA

from human_trajectory.msg import Trajectories, Trajectory

class TrajectoryVisualizer():
        
    def __init__(self, topic):
        self._server = InteractiveMarkerServer(topic)
        
    def trapezoidal_shaped_func(self, a, b, c, d, x):
        min_val = min(min((x - a)/(b - a), float(1.0)), (d - x)/(d - c))
        return max(min_val, float(0.0))

    def r_func(self, x):
        a = -0.125
        b =  0.125
        c =  0.375
        d =  0.625
        x = 1.0 - x
        value = self.trapezoidal_shaped_func(a,b,c,d,x)
        return value

    def g_func(self,x):
        a =  0.125
        b =  0.375
        c =  0.625
        d =  0.875
        x = 1.0 - x
        value = self.trapezoidal_shaped_func(a,b,c,d,x)
        return value

    def b_func(self,x):
        a =  0.375
        b =  0.625
        c =  0.875
        d =  1.125
        x = 1.0 - x
        value = self.trapezoidal_shaped_func(a,b,c,d,x)
        return value

    def visualize_trajectories(self, msg):
        for t in msg.trajectories:
            self.visualize_trajectory(t)


    def clear(self):
        self._server.clear()
        self._server.applyChanges()
        
    def _update_cb(self,feedback):
        return

    def visualize_trajectory(self, traj):
        int_marker = self.create_trajectory_marker(traj)
        self._server.insert(int_marker, self._update_cb)
        self._server.applyChanges()

    def delete_trajectory(self, traj):
        self._server.erase(traj.uuid)
        self._server.applyChanges()

    def create_trajectory_marker(self, traj):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/map"
        int_marker.name = traj.uuid
        #int_marker.description = traj.uuid
        pose = Pose()
        pose.position.x = traj.trajectory[0].pose.position.x
        pose.position.y = traj.trajectory[0].pose.position.y
        int_marker.pose = pose

        line_marker = Marker()
        line_marker.type = Marker.LINE_STRIP
        line_marker.scale.x = 0.1

        random.seed(traj.uuid)
        val = random.random()
        line_marker.color.r = self.r_func(val)
        line_marker.color.g = self.g_func(val)
        line_marker.color.b = self.b_func(val)
        line_marker.color.a = 1.0

        line_marker.points = []

        for pt in traj.trajectory:
            x = pt.pose.position.x
            y = pt.pose.position.y
            p = Point()
            p.x = x - int_marker.pose.position.x  
            p.y = y - int_marker.pose.position.y
            line_marker.points.append(p)

        # line_marker.colors = []
        # for i, vel in enumerate(traj.vel):
        #     if i % MOD == 0:
        #         color = ColorRGBA()
        #         val = vel / traj.max_vel
        #         color.r = r_func(val)
        #         color.g = g_func(val)
        #         color.b = b_func(val)
        #         color.a = 1.0
        #         line_marker.colors.append(color)
        
                
        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.markers.append(line_marker) 
        int_marker.controls.append(control)
        
        return int_marker
