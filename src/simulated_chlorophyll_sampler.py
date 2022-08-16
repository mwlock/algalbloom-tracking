#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8


import os

import scipy.io
import numpy as np
from scipy.interpolate import RegularGridInterpolator
import signal

import rospy
from std_msgs.msg import Float64, Header, Bool, Empty, Header
from geographic_msgs.msg import GeoPoint, GeoPointStamped
from smarc_msgs.msg import ChlorophyllSample,GotoWaypoint,AlgaeFrontGradient

# Saving data
from utils import Utils

# Graphing
import matplotlib.pyplot as plt
fig,ax = plt.subplots()

# Constants
GRADIENT_TOPIC = '/sam/algae_tracking/gradient'
VITUAL_POSITION_TOPIC = '/sam/algae_tracking/vp'
LIVE_WP_BASE_TOPIC = 'sam/smarc_bt/live_wp/'
WAPOINT_TOPIC=LIVE_WP_BASE_TOPIC+'wp'

# Publishers
from publishers.lat_lon_offset import publish_offset

import math

def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin  
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

# Return GeoGrid
class GeoGrid:
    def __init__(self, data, lon, lat, time, t_idx, include_time=False):
        self.data = data
        self.lon = lon
        self.lat = lat
        self.time = time
        self.t_idx = t_idx

        if include_time is False:
            self.field = RegularGridInterpolator((self.lon, self.lat), self.data[:,:,self.t_idx])
        else:
            self.field = RegularGridInterpolator((self.lon, self.lat, self.time), self.data)

    def is_within_limits(self, x, include_time=False):
        if include_time is False:
            if (self.lon[0] <= x[0] <= self.lon[-1]) and (self.lat[0] <= x[1] <= self.lat[-1]):
                return True
        else:
            if (self.lon[0] <= x[0] <= self.lon[-1]) and (self.lat[0] <= x[1] <= self.lat[-1]) and (self.time[0] <= x[2] <= self.time[-1]):
                return True

# Read matlab data
def read_mat_data(timestamp,include_time=False,scale_factor=1,lat_shift=0,lon_shift = 0):

    # Get datapath
    base_path = rospy.get_param('~data_file_base_path')

    # Read mat files
    chl = scipy.io.loadmat(base_path+'/chl.mat')['chl']
    lat = scipy.io.loadmat(base_path+'/lat.mat')['lat']
    lon = scipy.io.loadmat(base_path+'/lon.mat')['lon']
    time = scipy.io.loadmat(base_path+'/time.mat')['time']

    # Reshape
    lat = np.reshape(lat,[-1,])
    lon = np.reshape(lon,[-1,])
    chl = np.swapaxes(chl,0,2)
    time = np.reshape(time,[-1,])    

    # Scale data        
    lat = ((lat - lat[0])*scale_factor)+lat[0]
    lon = ((lon - lon[0])*scale_factor)+lon[0]

    # Shift the data
    lat = lat + lat_shift
    lon = lon + lon_shift

    # Logging
    rospy.loginfo('Scale factor : {}'.format(scale_factor))
    rospy.loginfo("Dimensions of lat {} - {}".format(lat[0],lat[-1]))
    rospy.loginfo("Dimensions of lon {} - {}".format(lon[0],lon[-1]))

    t_idx = np.argmin(np.abs(timestamp - time))

    return GeoGrid(chl, lon, lat, time, t_idx, include_time=include_time)

class chlorophyll_sampler_node(object):

    def lat_lon__cb(self,fb):

        # Determine the offset of the GPS
        if not self.init and self.offset_gps:
            self.gps_lat_offset = fb.latitude - self.lat_centre
            self.gps_lon_offset = fb.longitude - self.lon_centre

        # Publish offset (for the purpose of plottting in plot_live_grid)
        publish_offset(lat=self.gps_lat_offset,lon=self.gps_lon_offset,pub=self.lat_lon_offset_publisher)

        # Offset position
        self.lat = fb.latitude # - self.gps_lat_offset 
        self.lon = fb.longitude # - self.gps_lon_offset

        # Check offset correct set
        if not self.init:

            # Determine offsets
            lat_error = (fb.latitude - self.gps_lat_offset) - self.lat_centre
            long_Error = (fb.longitude - self.gps_lon_offset) - self.lon_centre
            rospy.loginfo("Offset error : {}, {}".format(lat_error,long_Error))
            rospy.loginfo("Offset lat : {}".format(self.gps_lat_offset))
            rospy.loginfo("Offset lon : {}".format(self.gps_lon_offset))

            # Offset the data
            self.grid = read_mat_data(self.timestamp, include_time=self.include_time,scale_factor=self.scale_factor,lat_shift=self.gps_lat_offset,lon_shift=self.gps_lon_offset)

            # Set origin of rotation
            self.origin_lat = fb.latitude
            self.origin_lon = fb.longitude

        # Rotate data       
        # origin = (self.origin_lon,self.origin_lat)
        # point = (self.lon, self.lat)
        # angle = math.radians(self.data_rotate_angle)
        # self.lon, self.lat = rotate(origin, point, -angle)

        self.init = True

    def __init__(self):
        """ Init the sampler"""

        # Parameters
        self.update_period = rospy.get_param('~sampling_time')

        # Determine if data needs to be scaled
        self.scale_factor =  float(1)/float(rospy.get_param('~data_downs_scale_factor'))
        self.delta_ref = rospy.get_param('~delta_ref')

        # Init values     
        self.init = False
        self.counter = 0

        # Real position
        self.lat = None
        self.lon = None

        self.origin_lat = None
        self.origin_lon = None
        self.data_rotate_angle = rospy.get_param('~data_rotate_angle')

        # WGS84 grid (lookup-table for sampling)
        self.include_time = False
        self.timestamp = 1618610399
        self.grid = read_mat_data(self.timestamp, include_time=self.include_time,scale_factor=self.scale_factor)

        # Check if data should be offset
        self.gps_lat_offset = 0
        self.gps_lon_offset = 0
        self.lat_centre =  0
        self.lon_centre =  0
        self.offset_gps = rospy.get_param('~offset_gps')
        if self.offset_gps:
            self.lat_centre =  rospy.get_param('~starting_lat')
            self.lon_centre =  rospy.get_param('~starting_lon')

        # Publishers and subscribers
        self.dr_sub = rospy.Subscriber('/sam/dr/lat_lon', GeoPoint, self.lat_lon__cb, queue_size=2)
        self.chlorophyll_publisher = rospy.Publisher('/sam/algae_tracking/chlorophyll_sampling', ChlorophyllSample, queue_size=1)
        self.lat_lon_offset_publisher = rospy.Publisher('/sam/algae_tracking/lat_lon_offset', GeoPointStamped, queue_size=2)

        # Plotting
        self.grid_plotted = False

    def publish_sample(self):
        """ Publish Chlorophyll Sample"""

        # Get current position
        current_position = [self.lon,self.lat]

        # Do nothing if current lat/long not set
        if None in current_position:
            rospy.logwarn("Cannot take sample, current lat/lon is None : [{},{}]".format(self.lat,self.lon))
            return

        # Get sample
        try:
            std = 1e-3 # standard deviation of measurement
            val = self.grid.field(current_position) + np.random.normal(0, std)
        except Exception as e:
            rospy.logwarn("Unable to attain sample at : {} {}".format(self.lat,self.lon))
            return

        # Publish sample message
        sample = ChlorophyllSample()
        sample.header = Header()
        sample.header.stamp = rospy.Time.now()
        sample.lat = self.lat
        sample.lon = self.lon
        sample.sample = val

        # Publish message
        rospy.loginfo('Publishing sample : {} at {},{}'.format(sample.sample,sample.lat,sample.lon))
        self.chlorophyll_publisher.publish(sample)

    def run_node(self):
        """ Start sampling """

        # Sampling rate
        rate = rospy.Rate(float(1)/self.update_period)

        while not rospy.is_shutdown():

            self.counter +=1

            self.publish_sample()
            rate.sleep()

    def close_node(self,signum, frame):
        """
        Kill node and save data
        """
        
        rospy.logwarn("Closing node")
        out_path = rospy.get_param('~output_data_path')

        # Get all relevant ros params
        self.all_params = rospy.get_param_names()
        self.tracking_params = [a for a in self.all_params if "algalbloom_tracker" in a]
        self.sampler_params = [a for a in self.all_params if "simulated_chlorophyll_sampler" in a]

        # track_params
        track_params= {}
        for key in self.tracking_params:
            track_params[key] = rospy.get_param(key)

        # sample_params
        sample_params = {}
        for key in self.sampler_params:
            sample_params[key] = rospy.get_param(key)

        try :
            Utils.save_mission(out_path=out_path,grid=self.grid,meas_per=self.update_period,sample_params=sample_params,track_params=track_params)
            rospy.logwarn("Data saved!")
        except Exception as e:
            rospy.logwarn(e)
            rospy.logwarn("Failed to save data")

        exit(1)

if __name__ == '__main__':

    rospy.init_node("simulated_chlorophyll_sampler")
    sampler = chlorophyll_sampler_node()

    # Attach exit handler
    signal.signal(signal.SIGINT, sampler.close_node)

    sampler.run_node()
        