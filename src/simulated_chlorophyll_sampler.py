#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8


import os

import scipy.io
import numpy as np
from scipy.interpolate import RegularGridInterpolator

import rospy
from std_msgs.msg import Float64, Header, Bool, Empty, Header
from geographic_msgs.msg import GeoPoint
from smarc_msgs.msg import ChlorophyllSample

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

class chlorophyll_sampler_node(object):

    # Read matlab data
    def read_mat_data(self,timestamp,include_time=False):

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

        rospy.loginfo('Scale factor : {}'.format(self.scale_factor))

        # Scale data        
        lat = ((lat - lat[0])*self.scale_factor)+lat[0]
        lon = ((lon - lon[0])*self.scale_factor)+lon[0]

        t_idx = np.argmin(np.abs(timestamp - time))

        return GeoGrid(chl, lon, lat, time, t_idx, include_time=include_time)

    def lat_lon__cb(self,fb):

        # Determine the offset of the GPS
        if not self.init and self.offset_gps:
            self.gps_lat_offset = fb.latitude - self.lat_centre
            self.gps_lon_offset = fb.longitude - self.lon_centre

        self.lat = fb.latitude + self.lat_centre
        self.lon = fb.longitude + self.lon_centre
        self.init = True

    def __init__(self):
        """ Init the sampler"""

        # Init values
        self.update_period = 3
        self.lat = None
        self.lon = None
        self.init = False

        # Determine if data needs to be scaled
        self.scale_factor =  float(1)/rospy.get_param('~data_downs_scale_factor') 

        # WGS84 grid (lookup-table for sampling)
        self.include_time = False
        self.timestamp = 1618610399
        self.grid = self.read_mat_data(self.timestamp, include_time=self.include_time)

        # Check if data should be offset
        self.gps_lat_offset = 0
        self.gps_lon_offset = 0
        self.lat_centre =  0
        self.lon_centre =  0
        self.offset_gps = rospy.get_param('~offset_gps')
        if self.offset_gps:
            self.lat_centre =  rospy.get_param('~starting_lat')
            self.lon_centre =  rospy.get_param('~starting_lon')

        # Subscribe to SAM posiiton
        self.depth_sub = rospy.Subscriber('/sam/dr/lat_lon', GeoPoint, self.lat_lon__cb, queue_size=2)
        
        # Create the publisher
        self.chlorophyll_publisher = rospy.Publisher('/sam/algae_tracking/chlorophyll_sampling', ChlorophyllSample, queue_size=1)

    def publish_sample(self):
        """ Publish Chlorophyll Sample"""

        # Get current position
        current_position = [self.lon,self.lat]

        # Do nothing if current lat/long not set
        if None in current_position:
            rospy.logwarn("Cannot take sample, current lat/lon is None")
            return

        # Get sample
        std = 1e-3 # standard deviation of measurement
        val = self.grid.field(current_position) + np.random.normal(0, std)

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
            self.publish_sample()
            rate.sleep()

if __name__ == '__main__':

    rospy.init_node("simulated_chlorophyll_sampler")
    sampler = chlorophyll_sampler_node()
    sampler.run_node()
        