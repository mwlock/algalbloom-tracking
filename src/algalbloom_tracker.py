#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8

# Behavior tree imports
import py_trees as pt
# import py_trees_ros as ptr

# Python imports
import time
import math
import numpy as np

# Ros imports
import rospy
import os

# Smarc imports
from vehicle import Vehicle
from auv_config import AUVConfig
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Float64

# globally defined values
# import bb_enums
# import imc_enums
import common_globals

update_period = 1

# Init node
rospy.init_node("algalbloom_tracker")

# Return true if we have an updated state
# def got_state_data():
#     return None not in (depth, lat, lon, x, y)


# Subscribers


class algalbloom_tracker_node(object):

    # Callbacks
    def depth__cb(self,fb):
        self.depth = fb.data
    def x__cb(self,fb):
        self.x = fb.data
    def y__cb(self,fb):
        self.y = fb.data
    def lat_lon__cb(self,fb):
        self.lat = fb.latitude
        self.long = fb.longitude


    def __init__(self):
        # Set initial values
        self.depth = None
        self.lat = None
        self.lon = None
        self.x = None
        self.y = None
        # Subscribe to relevant topics
        self.depth_sub = rospy.Subscriber('/sam/dr/depth', Float64, self.depth__cb, queue_size=2)
        self.depth_sub = rospy.Subscriber('/sam/dr/x', Float64, self.x__cb, queue_size=2)
        self.depth_sub = rospy.Subscriber('/sam/dr/y', Float64, self.y__cb, queue_size=2)
        self.depth_sub = rospy.Subscriber('/sam/dr/lat_lon', GeoPoint, self.lat_lon__cb, queue_size=2)

    def run_node(self):

        rate = rospy.Rate(1/update_period)
        while not rospy.is_shutdown():

            print("Depth: {}".format(self.depth))

            rate.sleep()

# Main runtime function
if __name__ == '__main__':
    tracking = algalbloom_tracker_node()
    tracking.run_node()
        