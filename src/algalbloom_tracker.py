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
# import rospy
import os

# Smarc imports
# from vehicle import Vehicle
# from auv_config import AUVConfig
# from geographic_msgs.msg import GeoPoint
# from std_msgs.msg import Float64
# import common_globals

import matplotlib.pyplot as plt
fig,ax = plt.subplots()

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

        return dx @ self.__model.alpha_, dy @ self.__model.alpha_


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
def read_mat_data(timestamp,include_time=False):

    # Read mat files
    chl = scipy.io.loadmat('data/chl.mat')['chl']
    lat = scipy.io.loadmat('data/lat.mat')['lat']
    lon = scipy.io.loadmat('data/lon.mat')['lon']
    time = scipy.io.loadmat('data/time.mat')['time']

    # Reshape
    lat = np.reshape(lat,[-1,])
    lon = np.reshape(lon,[-1,])
    chl = np.swapaxes(chl,0,2)
    time = np.reshape(time,[-1,])

    t_idx = np.argmin(np.abs(timestamp - time))

    return GeoGrid(chl, lon, lat, time, t_idx, include_time=include_time)

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

    # Init
    def __init__(self):

        # Set initial values
        self.depth = None
        self.lat = None
        self.lon = None
        self.x = None
        self.y = None

        # Subscribe to relevant topics
        # self.depth_sub = rospy.Subscriber('/sam/dr/depth', Float64, self.depth__cb, queue_size=2)
        # self.depth_sub = rospy.Subscriber('/sam/dr/x', Float64, self.x__cb, queue_size=2)
        # self.depth_sub = rospy.Subscriber('/sam/dr/y', Float64, self.y__cb, queue_size=2)
        # self.depth_sub = rospy.Subscriber('/sam/dr/lat_lon', GeoPoint, self.lat_lon__cb, queue_size=2)

        # Setup dynamics
        self.alpha_seek = 30
        self.alpha_follow = 1
        self.delta_ref = 7.45
        self.speed = 0.00004497 # 5m/s
        self.dynamics = Dynamics(self.alpha_seek, self.alpha_follow, self.delta_ref, self.speed)

        # WGS84 grid
        self.args = 1618610399
        self.include_time = False
        self.timestamp = 1618610399
        self.grid = read_mat_data(self.timestamp, include_time=False)

        # Gaussian Process Regression
        self.kernel = "MAT"
        self.std = 1e-3
        self.range = 200
        self.params = [44.29588721, 0.54654887, 0.26656638]
        self.time_step = 1.5
        self.meas_per = int(10 / self.time_step) # measurement period
        self.est = GPEstimator(kernel=self.kernel, s=self.std, range_m=self.range, params=self.params)

        # Algorithm settings (commented values are ofself.trajectory for IROS paper)
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

    def tick_control(self,x0, step, dynamics, grid, estimator, init_heading, meas_per, include_time=False, filter=False):
        """ Perform the control law """

        print("Ticking control law")

        # ==============================================================================================================
        if not self.init_flag:

            print("Initialising")

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

            if include_time is not False:
                x0 = [x0[0], x0[1], grid.time[grid.t_idx]]
                self.init_heading = np.array([init_heading[0] - x0[0], init_heading[1] - x0[1], 1])

            else:
                self.init_heading = np.array([init_heading[0] - x0[0], init_heading[1] - x0[1]])

            self.traj = np.zeros((self.n_iter, len(x0)))
            self.traj[0] = x0

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
            # Plot trajectory
            ax.set_aspect('equal')
            xx, yy = np.meshgrid(self.grid.lon, self.grid.lat, indexing='ij')
            p = plt.pcolormesh(xx, yy, self.grid.data[:,:,self.grid.t_idx], cmap='viridis', shading='auto', vmin=0, vmax=10)
            cax = fig.add_axes([ax.get_position().x1+0.01,ax.get_position().y0,0.02,ax.get_position().height])
            cp = fig.colorbar(p, cax=cax)
            cp.set_label("Chl a density [mm/mm3]")
            ax.contour(xx, yy, grid.data[:,:,grid.t_idx], levels=[self.delta_ref])
            plt.pause(0.0001)

        # ==============================================================================================================

        # for i in range(self.n_iter-1):
        if self.i % int(self.n_iter/100) == 0:
            print("Current iteration: %d" % self.i)

        offset = self.i % meas_per

        # Init state - 5% tolerance from front
        if (self.i < self.estimation_trigger_val-1 or self.measurements[self.meas_index-1] < 0.95*dynamics.delta_ref) and self.init_flag is True:
            if offset == 0:
                self.x_meas[self.meas_index] =self.traj[self.i]

                self.measurements[self.meas_index] = grid.field(self.x_meas[self.meas_index]) + np.random.normal(0, estimator.s)
                self.grad[self.meas_index] = self.init_heading[:2] / np.linalg.norm(self.init_heading[:2])

                if filter is not False:
                    self.grad_filter[self.meas_index] = self.grad[self.meas_index]

                self.meas_index = self.meas_index + 1

        # Estimation state
        else:
            if self.init_flag is True:
                print("Following the front...")
                # init_flag = False

            if offset == 0:
                self.x_meas[self.meas_index] =self.traj[self.i]

                # Take measurement
                val = grid.field(self.x_meas[self.meas_index]) + np.random.normal(0, estimator.s)
                if np.isnan(val):
                    print("Warning: NaN value measured.")
                    self.measurements[self.meas_index] = self.measurements[self.meas_index-1] # Avoid plots problems
                    return

                else:
                    self.measurements[self.meas_index] = val

                self.measurements[self.meas_index] = np.average(self.measurements[self.meas_index + 1 - self.meas_filter_len:self.meas_index+1], weights=self.weights_meas)

                # Estimate gradient
                if include_time is not False:
                    self.grad[self.meas_index] = np.array(estimator.est_grad(self.x_meas[self.meas_index-self.n_meas:self.meas_index+1, :2], \
                                                            self.measurements[self.meas_index-self.n_meas:self.meas_index+1])).squeeze()
                else:
                    self.grad[self.meas_index] = np.array(estimator.est_grad(self.x_meas[self.meas_index-self.n_meas:self.meas_index+1], \
                                                            self.measurements[self.meas_index-self.n_meas:self.meas_index+1])).squeeze()

                self.grad[self.meas_index] = self.grad[self.meas_index] / np.linalg.norm(self.grad[self.meas_index])

                # Filter gradient
                if filter is not False:
                    self.grad_filter[self.meas_index] = grad_moving_average_2D(self.grad, self.meas_index, self.grad_filter_len, self.weights)
                    self.grad_filter[self.meas_index] = self.grad_filter[self.meas_index-1]*self.alpha + self.grad_filter[self.meas_index]*(1-self.alpha)
                    self.control = dynamics(self.measurements[self.meas_index], self.grad_filter[self.meas_index], include_time=include_time)

                else:
                    self.grad[self.meas_index] = self.grad[self.meas_index-1, :]*self.alpha + self.grad[self.meas_index, :]*(1-self.alpha)
                    self.control = dynamics(self.measurements[self.meas_index], self.grad[self.meas_index], include_time=include_time)

                self.meas_index = self.meas_index + 1

        self.traj[self.i+1] =self.traj[self.i] + step*self.control

        if not grid.is_within_limits(self.traj[self.i+1, :], include_time=include_time):
            print("Warning:self.trajectory got out of boundary limits.")
            return 

        # Plotting
        if self.i%50 == 0:
            ax.plot(self.traj[self.i][0],self.traj[self.i][1],'r.', linewidth=1)
            plt.pause(0.0001)

        # Tick the counter
        self.i+=1

        # if filter is not False:
        #     return self.traj[:i], self.measurements[:self.meas_index], self.grad_filter[:self.meas_index]
        # else:
        #     return self.traj[:self.i], self.measurements[:self.meas_index], self.grad[:self.meas_index]

        

    
    def run_node(self):

        #self.trajectory parameters
        init_heading = np.array([21, 61.492])
        init_coords = [20.87, 61.492]

        update_period = self.time_step
        # rate = rospy.Rate(1/update_period)
        # while not rospy.is_shutdown():

            # print("Depth: {}".format(self.depth))
        while True:
            self.tick_control(init_coords, self.time_step, self.dynamics, self.grid, self.est, init_heading, self.meas_per, include_time=False, filter=False)

            # rate.sleep()

# Main runtime function
if __name__ == '__main__':

    # rospy.init_node("algalbloom_tracker")

    tracking = algalbloom_tracker_node()
    tracking.run_node()
        