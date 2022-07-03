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

import matplotlib.pyplot as plt
fig,ax = plt.subplots()

from AbsolutePosition import AbsolutePosition
from RelativePosition import RelativePosition
from ControllerState import ControllerState
from ControllerParameters import ControllerParameters

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


def grad_moving_average_2D(grad, idx, n, weights):
    val_x = grad[idx - n+1:idx + 1, 0]
    val_y = grad[idx - n+1:idx + 1, 1]

    x_ma = np.average(val_x, weights=weights)
    y_ma = np.average(val_y, weights=weights)

    return np.array([x_ma, y_ma])


# Vehicle dynamics
class Dynamics:
    def __init__(self, alpha_seek, alpha_follow, delta_ref, speed):
        self.alpha_seek = alpha_seek
        self.alpha_follow = alpha_follow
        self.delta_ref = delta_ref
        self.speed = speed


    def __call__(self, delta, grad, include_time=False):
        self.u_x = self.alpha_seek*(self.delta_ref - delta)*grad[0] \
                            - self.alpha_follow*grad[1]
        self.u_y = self.alpha_seek*(self.delta_ref - delta)*grad[1] \
                            + self.alpha_follow*grad[0]

        u = np.array([self.u_x, self.u_y])
        if include_time is False:
            return u * self.speed / np.linalg.norm(u)
        else:
            # Normalization still to be tested in TV conditions
            u_norm = u * self.speed / np.linalg.norm(u)
            return np.array([u_norm[0], u_norm[1], 1])

class algalbloom_tracker_node(object):

    # Subscriber callbacks
    def depth__cb(self,fb):
        self.depth = fb.data

    def x__cb(self,fb):
        self.x = fb.data

    def y__cb(self,fb):
        self.y = fb.data

    def lat_lon__cb(self,fb):
        self.lat = fb.latitude
        self.lon = fb.longitude

    def waypoint_reached__cb(self,fb):
        if fb.status.text == "WP Reached":
            self.following_waypoint = False
            rospy.loginfo("Waypoint reached")

    def chlorophyl__cb(self,fb):

        rospy.loginfo("Received sample : {}".format(fb))

        # read values
        lat = fb.lat
        lon = fb.lon
        sample = fb.sample

        # add to list of measurements
        self.samples = np.append(self.samples,sample)
        rospy.loginfo(sample)
        

    # Return true if pose remains uninitialized
    def pose_is_none(self):
        return None in [self.depth,self.lat,self.lon,self.x,self.y]

    def init_tracker(self):
        """ Initialise controller and such """

        # Relative position
        self.controller_state.relative_postion.x = 0
        self.controller_state.relative_postion.y = 0

        # Init virtual position (init on first message from dead reckoning)
        self.controller_state.virtual_position.lat = 0
        self.controller_state.virtual_position.lon = 0

        # Init controller state
        self.controller_state.n_waypoints = 0
        self.controller_state.speed = self.args['initial_speed']
        self.controller_state.direction = self.args['initial_heading']  # (degrees)

    # Init object
    def __init__(self):
        
        # Arguments
        self.args = {}
        self.args['initial_speed'] = initial_speed = rospy.get_param('~initial_speed')      # inital speed 
        self.args['initial_heading']  = rospy.get_param('~initial_heading')                 # initial heading (degrees)
        self.args['target_value']  = rospy.get_param('~target_value')                       # target chlorophyll value
        self.args['following_gain']  = rospy.get_param('~following_gain')
        self.args['seeking_gain']  = rospy.get_param('~seeking_gain')
        self.args['zig_zag_angle']  = rospy.get_param('~zig_zag_angle')                     # zig zag angle (degrees)
        self.args['horizontal_distance']  = rospy.get_param('~horizontal_distance')         # horizontal_distance (m)

        # Chlorophyl samples
        self.samples = np.array([])
        self.last_sample = rospy.Time.now()

        # Controller 
        self.controller_state = ControllerState()
        self.controller_params = ControllerParameters()

        self.inited = False
        self.waypoints_cleared = True
        self.front_crossed = False

        # Subscriber topics
        chlorophyll_topic = '/sam/algae_tracking/chlorophyll_sampling'

        # Subscriber setup

        # self.depth_sub = rospy.Subscriber('/sam/dr/depth', Float64, self.depth__cb, queue_size=2)
        # self.depth_sub = rospy.Subscriber('/sam/dr/x', Float64, self.x__cb, queue_size=2)
        # self.depth_sub = rospy.Subscriber('/sam/dr/y', Float64, self.y__cb, queue_size=2)
        # self.depth_sub = rospy.Subscriber('/sam/dr/lat_lon', GeoPoint, self.lat_lon__cb, queue_size=2)

        self.chlorophyll_sub = rospy.Subscriber(chlorophyll_topic, ChlorophyllSample, self.lat_lon__cb, queue_size=2)
        rospy.loginfo("Subscribed to {}".format(chlorophyll_topic))

        # self.goal_reached_sub = rospy.Subscriber('/sam/ctrl/goto_waypoint/result', GotoWaypointActionResult, self.waypoint_reached__cb, queue_size=2)

        # Waypoint enable publisher
        self.enable_waypoint_pub = rospy.Publisher('/sam/algae_farm/enable', Bool, queue_size=1)
        self.enable_waypoint_following = Bool()
        self.enable_waypoint_following.data = True

        # Waypoint following publisher
        self.waypoint_topic = '/sam/algae_farm/wp'
        self.waypoint_topic_type = GotoWaypoint
        self.waypoint_pub = rospy.Publisher(self.waypoint_topic, self.waypoint_topic_type,queue_size=5)

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

        # Setup dynamics
        self.alpha_seek = 30
        self.alpha_follow = 1
        self.delta_ref = 7.45
        self.speed = 0.00004497 # 5m/s
        self.dynamics = Dynamics(self.alpha_seek, self.alpha_follow, self.delta_ref, self.speed)

        # Gaussian Process Regression
        self.kernel = "MAT"
        self.std = 1e-3
        self.range = 200
        self.params = [44.29588721, 0.54654887, 0.26656638]
        self.time_step = 1
        self.meas_per = int(10 / self.time_step) # measurement period
        self.est = GPEstimator(kernel=self.kernel, s=self.std, range_m=self.range, params=self.params)

        # Algorithm settings
        self.n_iter = int(3e5) # 3e5
        self.n_meas = 125 # 125
        self.estimation_trigger_val = (self.n_meas-1) * self.meas_per
        self.grad_filter_len = 2 # 2
        self.meas_filter_len = 3 # 3
        self.alpha = 0.95 # Gradient update factor, 0.95

        # Meas filter
        self.weights_meas = None

        # Init
        self.init_flag = False
        self.following_waypoint = False

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
        msg.travel_speed = 5.0
        msg.pose.header.frame_id = 'utm'
        msg.pose.header.stamp = rospy.Time(0)
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y

        self.enable_waypoint_pub.publish(self.enable_waypoint_following)
        self.waypoint_pub.publish(msg)
        rospy.loginfo('Published waypoint')

    def tick_control(self,x0, step, dynamics, grid, estimator, init_heading, meas_per, include_time=False, filter=False):
        """ Perform the control law """

        if self.pose_is_none():
            rospy.loginfo('State is none')
            return

        # Plot current position
        if self.init_flag:
            ax.plot(self.lon,self.lat,'g.', linewidth=1)
            plt.pause(0.0001)

        if self.following_waypoint:
            rospy.loginfo('Following waypoint')
            return

        rospy.loginfo("Ticking control law")

        current_position = [self.lon,self.lat]

        # ==============================================================================================================
        # Initialisation
        # ==============================================================================================================
        if not self.init_flag:

            rospy.loginfo("Initialising")

            # Prepare gradient filter
            if filter is not False:
                if filter == 'MA':
                    self.weights = None

                elif filter == 'WMA':
                    self.weights = np.zeros(self.grad_filter_len)
                    for i in range(self.grad_filter_len):
                        self.weights[i] = (i+1) / self.grad_filter_len

                else:
                    raise ValueError("Unrecognized filter.")

            if include_time:
                x0 = [x0[0], x0[1], grid.time[grid.t_idx]]
                self.init_heading = np.array([init_heading[0] - x0[0], init_heading[1] - x0[1], 1])

            else:
                self.init_heading = np.array([init_heading[0] - x0[0], init_heading[1] - x0[1]])

            self.traj = np.zeros((self.n_iter, len(x0)))
            self.traj[0] = x0

            self.true_traj = np.zeros((self.n_iter, len(x0)))
            self.true_traj[0] = current_position

            self.x_meas = np.zeros((int(np.ceil(self.n_iter / meas_per)), len(x0)))
            self.measurements = np.zeros(int(np.ceil(self.n_iter / meas_per)))
            self.grad = np.zeros((int(np.ceil(self.n_iter / meas_per)), 2))

            if filter is not False:
                self.grad_filter = np.zeros((int(np.ceil(self.n_iter / meas_per)), 2))

            # Init state control law
            self.control = self.init_heading[:2] * dynamics.speed / np.linalg.norm([self.init_heading[:2]])

            self.meas_index = 0
            self.init_flag = True

            # Initialisation
            self.i = 0

            # Graphing
            # Plot initial trajectory point
            ax.set_aspect('equal')
            xx, yy = np.meshgrid(self.grid.lon, self.grid.lat, indexing='ij')
            p = plt.pcolormesh(xx, yy, self.grid.data[:,:,self.grid.t_idx], cmap='viridis', shading='auto', vmin=0, vmax=10)
            cax = fig.add_axes([ax.get_position().x1+0.01,ax.get_position().y0,0.02,ax.get_position().height])
            cp = fig.colorbar(p, cax=cax)
            cp.set_label("Chl a density [mm/mm3]")
            ax.contour(xx, yy, grid.data[:,:,grid.t_idx], levels=[self.delta_ref])
            plt.pause(0.0001)

        # ==============================================================================================================

        if self.i % int(self.n_iter/100) == 0:
            rospy.loginfo("Current iteration: %d" % self.i)

        offset = self.i % meas_per

        # Init state - 5% tolerance from front
        if (self.i < self.estimation_trigger_val-1 or self.measurements[self.meas_index-1] < 0.95*dynamics.delta_ref) and self.init_flag is True:
            if offset == 0:
                self.x_meas[self.meas_index] = current_position

                
                self.measurements[self.meas_index] = grid.field(self.x_meas[self.meas_index]) + np.random.normal(0, estimator.s)
                self.grad[self.meas_index] = self.init_heading[:2] / np.linalg.norm(self.init_heading[:2])
                rospy.loginfo('Taking measurement : {}'.format(self.measurements[self.meas_index]))

                if filter is not False:
                    self.grad_filter[self.meas_index] = self.grad[self.meas_index]

                self.meas_index = self.meas_index + 1

        # Estimation state
        else:
            if self.init_flag is True:
                print("Following the front...")
                # init_flag = False

            if offset == 0:
                self.x_meas[self.meas_index] = current_position

                # Take measurement
                val = grid.field(self.x_meas[self.meas_index]) + np.random.normal(0, estimator.s)
                rospy.loginfo('Taking measurement near front: {}'.format(val))
                if np.isnan(val):
                    print("Warning: NaN value measured.")
                    self.measurements[self.meas_index] = self.measurements[self.meas_index-1] # Avoid plots problems
                    return

                else:
                    self.measurements[self.meas_index] = val

                self.measurements[self.meas_index] = np.average(self.measurements[self.meas_index + 1 - self.meas_filter_len:self.meas_index+1], weights=self.weights_meas)

                # Estimate gradient
                if include_time:
                    self.grad[self.meas_index] = np.array(estimator.est_grad(self.x_meas[self.meas_index-self.n_meas:self.meas_index+1, :2], \
                                                            self.measurements[self.meas_index-self.n_meas:self.meas_index+1])).squeeze()
                else:
                    self.grad[self.meas_index] = np.array(estimator.est_grad(self.x_meas[self.meas_index-self.n_meas:self.meas_index+1], \
                                                            self.measurements[self.meas_index-self.n_meas:self.meas_index+1])).squeeze()

                self.grad[self.meas_index] = self.grad[self.meas_index] / np.linalg.norm(self.grad[self.meas_index])

                # Filter gradient
                if filter:
                    self.grad_filter[self.meas_index] = grad_moving_average_2D(self.grad, self.meas_index, self.grad_filter_len, self.weights)
                    self.grad_filter[self.meas_index] = self.grad_filter[self.meas_index-1]*self.alpha + self.grad_filter[self.meas_index]*(1-self.alpha)
                    self.control = dynamics(self.measurements[self.meas_index], self.grad_filter[self.meas_index], include_time=include_time)

                else:
                    self.grad[self.meas_index] = self.grad[self.meas_index-1, :]*self.alpha + self.grad[self.meas_index, :]*(1-self.alpha)
                    self.control = dynamics(self.measurements[self.meas_index], self.grad[self.meas_index], include_time=include_time)

                self.meas_index = self.meas_index + 1

        self.traj[self.i+1] =self.traj[self.i] + step*self.control
        self.true_traj[self.i] = current_position

        # Send next waypoint
        self.following_waypoint = True
        self.publishWaypoint(lat=self.traj[self.i+1][1],lon = self.traj[self.i+1][0],depth=0)

        # SAM moving outsife of scope of current mission
        if not grid.is_within_limits(self.traj[self.i+1, :], include_time=include_time):
            print("Warning:self.trajectory got out of boundary limits.")
            return 

        # Plot calculated waypoint
        if self.i%50 == 0:
            ax.plot(self.traj[self.i][0],self.traj[self.i][1],'r.', linewidth=1)
            plt.pause(0.0001)

        # Tick the counter
        self.i+=1

        # if filter is not False:
        #     return self.traj[:i], self.measurements[:self.meas_index], self.grad_filter[:self.meas_index]
        # else:
        #     return self.traj[:self.i], self.measurements[:self.meas_index], self.grad[:self.meas_index]

    def dispatch_waypoint(self):
        pass       
    
    def run_node(self):

        init_coords = [20.87, 61.492]

        update_period = self.time_step
        rate = rospy.Rate(1/update_period)
        # while True:
        while not rospy.is_shutdown():
        
            # self.tick_control(init_coords, self.time_step, self.dynamics, self.grid, self.est, init_heading, self.meas_per, include_time=False, filter=False)

            # if self.init:
            #     pass

            self.dispatch_waypoint()

            rate.sleep()

if __name__ == '__main__':

    rospy.init_node("algalbloom_tracker")
    tracking = algalbloom_tracker_node()
    tracking.init_tracker()
    tracking.run_node()
        