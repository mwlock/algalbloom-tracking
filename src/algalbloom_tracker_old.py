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

# TRY USING VEHICLE SETUP AT SOME POINT
# =========================================================================================
# first construct a vehicle that will hold and sub to most things
# rospy.loginfo("Setting up vehicle")
# config = AUVConfig()
# package_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), os.path.pardir)
# launch_path = os.path.join(package_path, 'launch', 'algalbloom_tracker.launch')
# try:
#     config.generate_launch_file(launch_path)
# except Exception as e:
#     print("Did not generate the launch file, will continue: \n{}".format(e))
# # config.read_rosparams()
# vehicle = Vehicle(config)
# tf_listener = vehicle.setup_tf_listener(timeout_secs=common_globals.SETUP_TIMEOUT)
# if tf_listener is None:
#     rospy.logerr("TF Listener could not be setup! Exiting!")
#     exit()
# =========================================================================================

depth = None
lat = None
lon = None
x = None
y = None

# Return true if we have an updated state
def got_state_data():
    return None not in (depth, lat, lon, x, y)

# Subscriber callbacks
def depth__cb(fb):
    global depth
    depth = fb.data
def x__cb(fb):
    global x
def y__cb(fb):
    global y
def lat_lon__cb(fb):
    global lat
    global lon
    lat = fb.latitude
    long = fb.longitude

# Subscribers
depth_sub = rospy.Subscriber('/sam/dr/depth', Float64, depth__cb, queue_size=2)
depth_sub = rospy.Subscriber('/sam/dr/x', Float64, x__cb, queue_size=2)
depth_sub = rospy.Subscriber('/sam/dr/y', Float64, y__cb, queue_size=2)
depth_sub = rospy.Subscriber('/sam/dr/lat_lon', GeoPoint, lat_lon__cb, queue_size=2)

def algalbloom_tracker_node():

    # GLobal variables
    global depth

    # Set update rate
    rate = rospy.Rate(1/update_period)

    depth = None
    lat = None
    long = None
    x = None
    y = None

    
    # Behavior
    while not rospy.is_shutdown():

        # Tick the vehicle
        # =========================================================================================
        # vehicle.tick(tf_listener)
        # print(vehicle)
        # =========================================================================================
        print(depth)


        # Sleep
        rate.sleep()


# Main runtime function
if __name__ == '__main__':
    algalbloom_tracker_node()
        