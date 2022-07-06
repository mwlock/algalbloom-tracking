#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8

# Python imports
import time
import math
import numpy as np
import scipy.io
from scipy.interpolate import RegularGridInterpolator
import sklearn.gaussian_process as gp
from scipy.spatial.distance import cdist
from scipy.spatial import distance
import signal

# Ros imports
import rospy
import os

# Smarc imports
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped, Point
from smarc_msgs.msg import GotoWaypoint, LatLonOdometry
from std_msgs.msg import Float64, Header, Bool, Empty
from smarc_msgs.srv import LatLonToUTM
from smarc_msgs.srv import UTMToLatLon
from smarc_msgs.msg import GotoWaypointActionResult,ChlorophyllSample
import geographic_msgs.msg

# Sampler import
from simulated_chlorophyll_sampler import GeoGrid
from simulated_chlorophyll_sampler import read_mat_data

import matplotlib.pyplot as plt
fig,ax = plt.subplots()

from positions import RelativePosition
from ControllerParameters import ControllerParameters
from ControllerState import ControllerState
from utils import Utils

class GPEstimator:
    def __init__(self, kernel, s, range_m, params=None, earth_radius=6369345):
        if not (kernel == 'RQ' or kernel == 'MAT'):
            raise ValueError("Invalid kernel. Choices are RQ or MAT.")

        if params is not None:
            if kernel == 'RQ':
                self.__kernel = gp.kernels.ConstantKernel(params[0])*gp.kernels.RationalQuadratic(length_scale=params[2], alpha=params[3])

            elif kernel == 'MAT':
                self.__kernel = gp.kernels.ConstantKernel(params[0])*gp.kernels.Matern(length_scale=params[1:])

        else:
            if kernel == 'RQ':
                self.__kernel = gp.kernels.ConstantKernel(91.2025)*gp.kernels.RationalQuadratic(length_scale=0.00503, alpha=0.0717)
            elif kernel == 'MAT':
                self.__kernel = gp.kernels.ConstantKernel(44.29588721)*gp.kernels.Matern(length_scale=[0.54654887, 0.26656638])

        self.__kernel_name = kernel

        self.s = s
        self.__model = gp.GaussianProcessRegressor(kernel=self.__kernel, optimizer=None, alpha=self.s**2)

        # Estimation range where to predict values
        self.range_deg = range_m / (np.radians(1.0) * earth_radius)


    """
    Gaussian Process Regression - Gradient analytical estimation

    Parameters
    ----------
    X:self.trajectory coordinates array
    y: self.measurements on X coordinates
    dist_metric: distance metric used to calculate distances
    """
    def est_grad(self, X, y, dist_metric='euclidean'):
        self.__model.fit(X[:-1], y[:-1])
        x = np.atleast_2d(X[-1])

        params = self.__kernel.get_params()

        if self.__kernel_name == 'RQ':
            sigma = params["k1__constant_value"]
            length_scale = params["k2__length_scale"]
            alpha = params["k2__alpha"]

            dists = cdist(x, X[:-1], metric=dist_metric)
            x_dist = nonabs_1D_dist(x[:,0], X[:-1,0])
            y_dist = nonabs_1D_dist(x[:,1], X[:-1,1])

            common_term = 1 + dists ** 2 / (2 * alpha * length_scale ** 2)
            common_term = common_term ** (-alpha-1)
            common_term = -sigma /  (length_scale ** 2) * common_term

            dx = x_dist * common_term
            dy = y_dist * common_term

        elif self.__kernel_name == 'MAT':
            sigma = params["k1__constant_value"]
            length_scale = params["k2__length_scale"]

            dists = cdist(x/length_scale, X[:-1]/length_scale, metric=dist_metric)

            dists = dists * np.sqrt(3)

            x_dist = nonabs_1D_dist(x[:,0], X[:-1,0]) / (length_scale[0]**2)
            y_dist = nonabs_1D_dist(x[:,1], X[:-1,1]) / (length_scale[1]**2)

            common_term = -3 * sigma * np.exp(-dists)

            dx = x_dist * common_term
            dy = y_dist * common_term

        return np.matmul(dx,self.__model.alpha_) , np.matmul(dy,self.__model.alpha_) 


def nonabs_1D_dist(x, X):
    res = np.zeros((x.shape[0], X.shape[0]))

    for i in range(res.shape[0]):
        for j in range(res.shape[1]):
            res[i, j] = x[i] - X[j]

    return res

class algalbloom_tracker_node(object):

    # Subscriber callbacks
    def depth__cb(self,fb):
        self.depth = fb.data

    def x__cb(self,fb):
        self.x = fb.data

    def y__cb(self,fb):
        self.y = fb.data

    def lat_lon__cb(self,fb):
        """ update virtual position of the robot using dead reckoning"""

        # Offset position
        if not self.inited and self.offset_gps:
            self.gps_lat_offset = fb.latitude - self.lat_centre
            self.gps_lon_offset = fb.longitude - self.lon_centre

        fb.latitude = fb.latitude - self.gps_lat_offset 
        fb.longitude = fb.longitude - self.gps_lon_offset

        # Get position
        self.controller_state.absolute_position.lat = fb.latitude
        self.controller_state.absolute_position.lon = fb.longitude
        # rospy.loginfo(self.controller_state)

        # Set virtual postion (initalisation of vp)
        if not self.inited:           
            self.controller_state.virtual_position.lat = fb.latitude
            self.controller_state.virtual_position.lon = fb.longitude

        # Calculate displacement (in m)
        dx,dy = Utils.displacement(current_position=self.controller_state.absolute_position,virtual_position=self.controller_state.virtual_position)
        self.controller_state.relative_postion.x = dx
        self.controller_state.relative_postion.y = dy

        # Update ref if mission not started
        if not self.inited:
            
            self.update_ref()  
            self.inited = True 


    def waypoint_reached__cb(self,fb):
        """ Waypoint reached
        
        Logic checking the threshold for proximity to the waypoint is handled by the line following action"""

        # Determine if waypoint has been reached
        if fb.status.text == "WP Reached":

            rospy.loginfo("Waypoint reached signal received")

            # Check distance to waypoint
            x,y = Utils.displacement(self.controller_state.absolute_position,self.controller_state.waypoint_position)
            dist = np.linalg.norm(np.array([x,y]))
            rospy.loginfo("Distance to the waypoint : {}".format(dist))
            if dist < 5:
                self.waypoints_cleared = True

            # TODO - Check if the waypoint reached was not already reached? I.e. repetitve signal from bt

            # Check that previous waypoints were reached
            if not self.waypoints_cleared:
                return 

            self.controller_state.n_waypoints +=1

            # Switch direction of the zig zag
            if self.controller_state.n_waypoints  >= 2:

                rospy.loginfo("Switching direction")
                self.controller_state.n_waypoints = 0
                self.update_direction()

            # Update position on the track
            self.update_virtual_position()

            # Send new waypoint
            self.update_ref()

            self.waypoints_cleared = False


    def chlorophyl__cb(self,fb):
        """ Callback when a sensor reading is received 
        
        The sensor reading should be appended to the list of sensor readings, along with the associated
        lat lon position where the reading was taken. """

        # read values (the sensor is responsible for providing the Geo stamp i.e. lat lon co-ordinates)
        position = np.array([[fb.lat,fb.lon]])
        sample = fb.sample
        self.last_sample = fb.header.stamp

        # add to list of measurements
        self.samples = np.append(self.samples,sample)    
        self.samples_positions = np.append(self.samples_positions, position,axis=0)

        # Check if front has been reached
        if not self.front_crossed:
            if self.samples[-1] >= 0.95*self.args['delta_ref']:
                rospy.loginfo("FRONT HAS BEEN REACHED")
                self.front_crossed = True


        # logging stuff :)
        rospy.loginfo('Received sample : {} at {},{} (#{})'.format(fb.sample,fb.lat,fb.lon,len(self.samples)))

    # Return true if pose remains uninitialized
    def pose_is_none(self):
        return None in [self.depth,self.lat,self.lon,self.x,self.y]

    def init_tracker(self):
        """ Initialise controller and such """

        # Relative position
        self.controller_state.relative_postion.x = 0
        self.controller_state.relative_postion.y = 0

        # Init virtual position (init on first message from dead reckoning)
        self.controller_state.absolute_position.lat = 0
        self.controller_state.absolute_position.lon = 0

        # GPS offset
        self.gps_lat_offset = 0
        self.gps_lon_offset = 0
        self.lat_centre =  0
        self.lon_centre =  0
        if self.offset_gps:
            self.lat_centre =  rospy.get_param('~starting_lat')
            self.lon_centre =  rospy.get_param('~starting_lon')

        # Init controller state
        self.controller_state.n_waypoints = 0
        self.controller_state.speed = self.args['initial_speed']
        self.controller_state.direction = self.args['initial_heading']  # (radians)

        # Init controller params
        self.controller_params.angle = self.args['zig_zag_angle']
        self.controller_params.distance = self.args['horizontal_distance']
        self.controller_params.following_gain = self.args['following_gain']
        self.controller_params.seeking_gain = self.args['seeking_gain']

        # Setup estimator
        self.est = GPEstimator(kernel=self.kernel, s=self.std, range_m=self.range, params=self.params)

        # Subscribe to topics
        self.depth_sub = rospy.Subscriber(self.latlong_topic, GeoPoint, self.lat_lon__cb, queue_size=2)        
        self.chlorophyll_sub = rospy.Subscriber(self.chlorophyll_topic, ChlorophyllSample, self.chlorophyl__cb, queue_size=2)      
        self.goal_reached_sub = rospy.Subscriber(self.got_to_waypoint_result, GotoWaypointActionResult, self.waypoint_reached__cb, queue_size=2)

        rospy.loginfo("Subscribed to {}".format(self.latlong_topic))
        rospy.loginfo("Subscribed to {}".format(self.chlorophyll_topic))
        rospy.loginfo("Subscribed to {}".format(self.got_to_waypoint_result))

    # Init object
    def __init__(self):

        # Arguments
        self.args = {}
        self.args['initial_speed'] = initial_speed = rospy.get_param('~initial_speed')      # inital speed 
        self.args['initial_heading']  = rospy.get_param('~initial_heading')                 # initial heading (degrees)
        self.args['delta_ref']  = rospy.get_param('~delta_ref')                             # target chlorophyll value
        self.args['following_gain']  = rospy.get_param('~following_gain')
        self.args['seeking_gain']  = rospy.get_param('~seeking_gain')
        self.args['zig_zag_angle']  = rospy.get_param('~zig_zag_angle')                     # zig zag angle (degrees)
        self.args['horizontal_distance']  = rospy.get_param('~horizontal_distance')         # horizontal_distance (m)
        self.args['show_matplot_lib'] = rospy.get_param('~show_matplot_lib') 
        self.args['estimation_trigger_val'] = rospy.get_param('~estimation_trigger_val')     # number of samples before estimation
        self.args['scale_factor'] = float(1)/float(rospy.get_param('~data_downs_scale_factor')) 
        self.offset_gps = rospy.get_param('~offset_gps')

        # Move these elsewhere (TODO)
        # Gaussian Process Regression
        self.kernel = "MAT"
        self.std = 1e-3
        self.range = 200
        self.params = [44.29588721, 0.54654887, 0.26656638]
        self.time_step = 1
        self.meas_per = int(10 / self.time_step) # measurement period

        # Move these elsewhere (TODO)
        # Algorithm settings
        self.n_iter = int(3e5) # 3e5
        self.n_meas = 125 # 125
        self.estimation_trigger_val = (self.n_meas-1) * self.meas_per
        self.grad_filter_len = 2 # 2
        self.meas_filter_len = 3 # 3
        self.alpha = 0.95 # Gradient update factor, 0.95


        # plot first to avoid errors
        if self.args['show_matplot_lib']:
            self.timestamp = 1618610399
            self.include_time = False
            self.grid = read_mat_data(self.timestamp, include_time=self.include_time,scale_factor=self.args['scale_factor'])
            ax.set_aspect('equal')
            xx, yy = np.meshgrid(self.grid.lon, self.grid.lat, indexing='ij')
            p = plt.pcolormesh(xx, yy, self.grid.data[:,:,self.grid.t_idx], cmap='viridis', shading='auto', vmin=0, vmax=10)
            cax = fig.add_axes([ax.get_position().x1+0.01,ax.get_position().y0,0.02,ax.get_position().height])
            cp = fig.colorbar(p, cax=cax)
            cp.set_label("Chl a density [mm/mm3]")
            ax.contour(xx, yy, self.grid.data[:,:,self.grid.t_idx], levels=[self.args['delta_ref']])
            plt.pause(0.0001)
            time.sleep(5) # allow some time for the plot to start
            

        # Chlorophyl samples
        self.samples = np.array([])
        self.samples_positions =  np.empty((0,2), dtype=float)
        self.last_sample = rospy.Time.now()

        # Gradient
        self.gradients = np.empty((0,2), dtype=float)

        # Controller 
        self.controller_state = ControllerState()
        self.controller_params = ControllerParameters()

        self.inited = False
        self.waypoints_cleared = True
        self.front_crossed = False

        # Subscriber topics
        self.chlorophyll_topic = '/sam/algae_tracking/chlorophyll_sampling'
        self.latlong_topic = '/sam/dr/lat_lon'
        self.got_to_waypoint_result = '/sam/ctrl/goto_waypoint/result'

        self.live_wp_base_topic = 'sam/smarc_bt/live_wp/'
        self.wapoint_topic=self.live_wp_base_topic+'wp'
        self.wapoint_enable_topic=self.live_wp_base_topic+'enable'

        # Waypoint enable publisher
        self.enable_waypoint_pub = rospy.Publisher(self.wapoint_enable_topic, Bool, queue_size=1)
        self.enable_waypoint_following = Bool()
        self.enable_waypoint_following.data = True

        # Waypoint following publisher
        self.waypoint_topic_type = GotoWaypoint
        self.waypoint_pub = rospy.Publisher(self.wapoint_topic, self.waypoint_topic_type,queue_size=5)

        # Latlong to UTM service
        # wait for the latlon_to_utm service to exist
        service_exists = False
        rospy.loginfo("Waiting for lat_lon_to_utm services")
        self.LATLONTOUTM_SERVICE = '/sam/dr/lat_lon_to_utm'
        while not service_exists:
            time.sleep(1)
            try:
                rospy.wait_for_service(self.LATLONTOUTM_SERVICE, timeout=1)
                service_exists = True
                break
            except:
                pass    
        rospy.loginfo("Aquired services")

        # Init
        self.init_flag = False

        rospy.loginfo("Node init complete.")

    # Convert latlon to UTM
    def latlon_to_utm(self,lat,lon,z,in_degrees=False):

        try:
            rospy.wait_for_service(self.LATLONTOUTM_SERVICE, timeout=1)
        except:
            rospy.logwarn(str(self.LATLONTOUTM_SERVICE)+" service not found!")
            return (None, None)
        try:
            latlontoutm_service = rospy.ServiceProxy(self.LATLONTOUTM_SERVICE,
                                                     LatLonToUTM)
            gp = GeoPoint()
            if in_degrees:
                gp.latitude = lat
                gp.longitude = lon
            else:
                gp.latitude = lat
                gp.longitude = lon
            gp.altitude = z
            utm_res = latlontoutm_service(gp)

            return (utm_res.utm_point.x, utm_res.utm_point.y)
        except rospy.service.ServiceException:
            rospy.logerr_throttle_identical(5, "LatLon to UTM service failed! namespace:{}".format(self.LATLONTOUTM_SERVICE))
            return (None, None)

    # Publish waypoint to SAM
    def publishWaypoint(self,lat,lon,depth):

        # Make sure lat/lon offset is taken care of
        lat += self.gps_lat_offset 
        lon += self.gps_lon_offset 

        self.lat_lon_point = geographic_msgs.msg.GeoPoint()
        self.lat_lon_point.latitude = lat
        self.lat_lon_point.longitude = lon        

        x, y = self.latlon_to_utm(lat=lat,lon=lon,z=depth)

        z_control_modes = [GotoWaypoint.Z_CONTROL_DEPTH]
        speed_control_mode = [GotoWaypoint.SPEED_CONTROL_RPM,GotoWaypoint.SPEED_CONTROL_SPEED]

        msg = GotoWaypoint()
        msg.travel_depth = -1
        msg.goal_tolerance = 2
        msg.lat = lat
        msg.lon = lon
        msg.z_control_mode = z_control_modes[0]
        #msg.travel_rpm = 1000
        msg.speed_control_mode = speed_control_mode[1]
        # msg.travel_speed = 5.0
        msg.travel_speed = 1
        msg.pose.header.frame_id = 'utm'
        msg.pose.header.stamp = rospy.Time(0)
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y

        self.enable_waypoint_pub.publish(self.enable_waypoint_following)
        self.waypoint_pub.publish(msg)

        rospy.loginfo('Published waypoint : {},{}'.format(lat,lon))
        rospy.loginfo(msg)

        # Store waypoint
        self.controller_state.waypoint_position.lat = msg.lat
        self.controller_state.waypoint_position.lon = msg.lon

        # Plot calculated waypoint
        if self.args['show_matplot_lib']:
            rospy.loginfo('plotting waypoint')
            lat -= self.gps_lat_offset 
            lon -= self.gps_lon_offset 
            ax.plot(lon,lat,'m.', linewidth=1)

    def dispatch_waypoint(self):
        pass       
    
    def run_node(self):

        init_coords = [20.87, 61.492]

        update_period = self.time_step
        rate = rospy.Rate(1/update_period)
        # while True:
        while not rospy.is_shutdown():

            self.dispatch_waypoint()

            # Plot position
            if self.args['show_matplot_lib'] and self.inited:
                if not 0 in [self.controller_state.absolute_position.lon,self.controller_state.absolute_position.lat]:                    
                    ax.plot(self.controller_state.absolute_position.lon,self.controller_state.absolute_position.lat,'r.', linewidth=1)
                    plt.pause(0.0001)

            rate.sleep()

    def update_ref(self):
        """ Update referece """

        rospy.loginfo("Updating reference")

        # Determine if y displacement should be positive or negative
        sign = 2 * (self.controller_state.n_waypoints % 2) - 1

        # Determine if we have reached the front
        i = len(self.samples)
        front_crossed = self.has_crossed_the_front()
        rospy.loginfo("Crossed the front : {}".format(front_crossed))
        rospy.loginfo("Samples taken : {}/{}".format(i,self.args['estimation_trigger_val']))
        if len(self.samples)>0:
            rospy.loginfo("Latest sample : {}/{}".format(self.samples[-1],0.95*self.args['delta_ref']))

        distance = self.controller_params.distance if front_crossed else 0
        along_track_displacement = distance / math.tan(math.radians(self.controller_params.angle)) if front_crossed else self.controller_params.distance

        # Determine the next waypoint
        next_wp = RelativePosition(x=along_track_displacement,y=sign*distance)
        rospy.loginfo("Next waypoint is {} m, {} m relative to current position".format(next_wp.x,next_wp.y))

        # Bearing should always be 45%?
        bearing, range = Utils.toPolar(next_wp.x,next_wp.y)

        # Add current direction to bearing
        bearing += self.controller_state.direction
        rospy.loginfo("Next waypoint is has bearing and range : {} {}".format(bearing,range))
        bearing = math.radians(bearing)

        # calculate change from current position
        dx = range*math.cos(bearing)
        dy = range*math.sin(bearing)

        # calculate displacement for waypoint
        lat, lon = Utils.displace(current_position=self.controller_state.virtual_position,dx=dx,dy=dy)
        self.publishWaypoint(lat=lat,lon=lon,depth=0)

    def get_track_position(self,origin,use_relative_position=True):
        """ Return distance along the track """

        if use_relative_position:
            bearing, range = Utils.toPolar(self.controller_state.relative_postion.x,self.controller_state.relative_postion.y)
            bearing =- self.controller_state.direction

            x = range * math.cos(bearing)
            return x

        return None

    def update_virtual_position(self):
        """ Update virtual position """

        origin = RelativePosition()

        # Get distance along the track
        x = self.get_track_position(origin=origin)
        rospy.loginfo("Along track position : {}".format(x))

        lat,lon = Utils.displace(current_position=self.controller_state.virtual_position,dx=x*math.cos(self.controller_state.direction),dy=x*math.sin(self.controller_state.direction))

        self.controller_state.virtual_position.lat = lat
        self.controller_state.virtual_position.lon = lon
        rospy.loginfo("New virtual position : {},{}".format(lat,lon))

        # Plot new virtual position
        if self.args['show_matplot_lib']:
            rospy.loginfo('plotting waypoint')
            ax.plot(lon,lat,'y.', linewidth=1)
    
    def reset_virtual_position(self):
        """ Reset virtual position """
        pass

    def has_crossed_the_front(self):
        """ Logic for determining if the front has been crossed"""
        i = len(self.samples)
        return (i >= self.args['estimation_trigger_val'] and self.front_crossed)

    def update_direction(self):
        """ Update the direction of the track """

        rospy.loginfo("Updating bearing direction")

        # Determine if we have reached the front
        i = len(self.samples)
        front_crossed = self.has_crossed_the_front()
        rospy.loginfo("Crossed the front : {}".format(front_crossed))
        rospy.loginfo("Samples taken : {}/{}".format(i,self.args['estimation_trigger_val']))
        if len(self.samples)>0:
            rospy.loginfo("Latest sample : {}/{}".format(self.samples[-1],0.95*self.args['delta_ref']))

        # Carry on moving in straight line if the front has not been crossed
        if not front_crossed:
            # Do not change direction
            return

        # Estimate direction of the front
        grad = self.estimate_gradient()
        grad_heading = math.atan2(grad[1],grad[1]) # rad
        rospy.loginfo("Estimated gradient : {} ({} degrees)".format(grad,math.degrees(grad_heading)))

        # Perform control
        self.controller_state.direction = self.perform_control(grad=grad)

    def estimate_gradient(self):
        """ Estimate gradient """

        # TODO (Add windowed filtering on samples to smoothen out?)

        rospy.loginfo("SAMPLE POSITIONS : {}".format(self.samples_positions[-1]))

        # Estimate the gradient
        grad = np.array(self.est.est_grad(self.samples_positions[-self.n_meas:], \
                                                            self.samples[-self.n_meas:])).squeeze()

        self.gradients = np.append(self.gradients,[grad],axis=0)

        # Normalise gradient (unit vector)
        self.gradients[-1] = self.gradients[-1] / np.linalg.norm(self.gradients[-1])

        # Apply decaying factor to gradient (not sure if this will work)
        if len(self.gradients)>1:
            self.gradients[-1] = self.gradients[-2] * self.alpha + self.gradients[-1] * (1-self.alpha)

        return self.gradients[-1]

    def perform_control(self,grad):
        """ Calculate the control action 
        
        returns the new heading"""

        # Create vector orthogonal to gradient
        epsi = np.array([-grad[1],grad[0]])

        error = self.args['delta_ref'] - self.samples[-1]
        u_seek = self.controller_params.seeking_gain * error * grad
        u_follow = self.controller_params.following_gain * epsi
        u = u_seek + u_follow

        rospy.loginfo(("error : {}".format(error)))
        rospy.loginfo(("seek control : {}".format(u_seek)))
        rospy.loginfo(("follow control : {}".format(u_follow)))

        heading = math.atan2(u[1],u[0])
        rospy.loginfo(("heading : {} (degrees)".format(math.degrees(heading))))

        return heading

    def close_node(self,signum, frame):
        rospy.logwarn("Closing node")
        rospy.logwarn("Attempting to end waypoint following")

        try :
            self.enable_waypoint_following.data = False
            self.enable_waypoint_pub.publish(self.enable_waypoint_following)
            rospy.logwarn("Waypoint following successfully disabled")
        except Exception as e:
            rospy.logwarn("Failed to disabled Waypoint following")

        exit(1)

if __name__ == '__main__':

    rospy.init_node("algalbloom_tracker")
    tracking = algalbloom_tracker_node()
    tracking.init_tracker()

    # Attach exit handler
    signal.signal(signal.SIGINT, tracking.close_node)

    # run the node
    tracking.run_node()