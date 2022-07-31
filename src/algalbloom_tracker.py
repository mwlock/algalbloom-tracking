#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8

# Python imports
from array import array
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
from smarc_msgs.msg import GotoWaypointActionResult,ChlorophyllSample,AlgaeFrontGradient
from geographic_msgs.msg import GeoPointStamped

# Sampler import
from simulated_chlorophyll_sampler import GeoGrid
from simulated_chlorophyll_sampler import read_mat_data

# Controller
from controller.positions import RelativePosition
from controller.controller_parameters import ControllerParameters
from controller.controller_state import ControllerState

# Utils
from utils import Utils

# Publisher methods
from publishers.waypoints import publish_waypoint
from publishers.gradient import publish_gradient
from publishers.positions import publish_vp

# Gradient estimation
from estimators.gp import GPEstimator

# Constants
CHLOROPHYLL_TOPIC = '/sam/algae_tracking/chlorophyll_sampling'
GRADIENT_TOPIC = '/sam/algae_tracking/gradient'
VITUAL_POSITION_TOPIC = '/sam/algae_tracking/vp'

LATLONG_TOPIC = '/sam/dr/lat_lon'
GOT_TO_WAYPOINT_RESULT = '/sam/ctrl/goto_waypoint/result'
LIVE_WP_BASE_TOPIC = 'sam/smarc_bt/live_wp/'

WAPOINT_TOPIC=LIVE_WP_BASE_TOPIC+'wp'
WAPOINT_ENABLE_TOPIC=LIVE_WP_BASE_TOPIC+'enable'

class algalbloom_tracker_node(object):

    # Algal bloom tracker node
    def __init__(self):

        # Arguments
        self.args = {}
        self.args['initial_heading']  = rospy.get_param('~initial_heading')                         # initial heading [degrees]
        self.args['delta_ref']  = rospy.get_param('~delta_ref')                                     # target chlorophyll value
        self.args['following_gain']  = rospy.get_param('~following_gain')                           # following gain
        self.args['seeking_gain']  = rospy.get_param('~seeking_gain')                               # seeking gain
        self.args['wp_distance']  = rospy.get_param('~wp_distance')                                 # wp_distance [m]
        self.args['estimation_trigger_val'] = rospy.get_param('~estimation_trigger_val')            # number of samples before estimation
        self.args['speed'] = rospy.get_param('~speed')                                              # waypoint following speed [m/s]  
        self.args['waypoint_tolerance'] = rospy.get_param('~waypoint_tolerance')                    # waypoint tolerance [m]
        self.args['range'] = rospy.get_param('~range')                                              # estimation circle radius [m]

        # Move these elsewhere (TODO)
        # Gaussian Process Regression
        self.kernel = "MAT"
        self.std = 1e-3
        # TODO : Should this be 200, this is apparently the estimation circle radius?
        self.range = self.args['range'] 
        self.params = [44.29588721, 0.54654887, 0.26656638]
        self.time_step = 1
        self.meas_per = int(10 / self.time_step) # measurement period

        # Move these elsewhere (TODO)
        # Algorithm settings
        self.n_meas = 125 # 125
        self.grad_filter_len = 2 # 2
        self.meas_filter_len = 3 # 3
        self.alpha = 0.95 # Gradient update factor, 0.95            

        # Chlorophyl samples
        self.samples = np.array([])
        self.samples_positions =  np.empty((0,2), dtype=float)
        self.last_sample = -1

        # Gradient
        self.gradients = np.empty((0,2), dtype=float)

        # Controller 
        self.controller_state = ControllerState()
        self.controller_params = ControllerParameters()

        self.inited = False
        self.waypoints_cleared = True
        self.front_crossed = False

        # Subscriber topics
        self.chlorophyll_topic= CHLOROPHYLL_TOPIC 
        self.gradient_topic= GRADIENT_TOPIC 
        self.vitual_position_topic = VITUAL_POSITION_TOPIC
        self.latlong_topic= LATLONG_TOPIC 
        self.got_to_waypoint_result= GOT_TO_WAYPOINT_RESULT 
        self.wapoint_topic= WAPOINT_TOPIC 
        self.wapoint_enable_topic = WAPOINT_ENABLE_TOPIC 

        # Gradient publisher
        self.gradient_pub = rospy.Publisher(self.gradient_topic, AlgaeFrontGradient ,queue_size=1)

        # Virtual position publisher
        self.vp_pub = rospy.Publisher(self.vitual_position_topic,GeoPointStamped,queue_size=1)

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

    def init_tracker(self):
        """ Initialise controller and such """

        # Init controller state
        self.controller_state.direction = math.radians(self.args['initial_heading'])  # [radians]

        # Init controller params
        self.controller_params.distance = self.args['wp_distance']
        self.controller_params.following_gain = self.args['following_gain']
        self.controller_params.seeking_gain = self.args['seeking_gain']
        self.controller_params.speed = self.args['speed']
        self.controller_params.waypoint_tolerance = self.args['waypoint_tolerance']

        # Setup estimator
        self.est = GPEstimator(kernel=self.kernel, s=self.std, range_m=self.range, params=self.params)

        # Subscribe to topics
        self.depth_sub = rospy.Subscriber(self.latlong_topic, GeoPoint, self.lat_lon__cb, queue_size=2)        
        self.chlorophyll_sub = rospy.Subscriber(self.chlorophyll_topic, ChlorophyllSample, self.chlorophyl__cb, queue_size=2)      
        self.goal_reached_sub = rospy.Subscriber(self.got_to_waypoint_result, GotoWaypointActionResult, self.waypoint_reached__cb, queue_size=2)

        rospy.loginfo("Subscribed to {}".format(self.latlong_topic))
        rospy.loginfo("Subscribed to {}".format(self.chlorophyll_topic))
        rospy.loginfo("Subscribed to {}".format(self.got_to_waypoint_result))

    # Subscriber callbacks
    def depth__cb(self,fb):
        self.depth = fb.data

    def x__cb(self,fb):
        self.x = fb.data

    def y__cb(self,fb):
        self.y = fb.data

    def lat_lon__cb(self,fb):
        """ update virtual position of the robot using dead reckoning"""

        fb.latitude = fb.latitude 
        fb.longitude = fb.longitude

        # Get position
        self.controller_state.absolute_position.lat = fb.latitude
        self.controller_state.absolute_position.lon = fb.longitude
        # rospy.loginfo(self.controller_state)

        # Set virtual postion (initalisation of vp)
        if not self.inited:           
            self.controller_state.virtual_position.lat = fb.latitude
            self.controller_state.virtual_position.lon = fb.longitude

        # Calculate displacement (in m)
        dx,dy = Utils.displacement(virtual_position=self.controller_state.absolute_position,current_position=self.controller_state.virtual_position)
        self.controller_state.relative_postion.x = dx
        self.controller_state.relative_postion.y = dy

        # Update ref if mission not started
        if not self.inited:            
            self.update_ref()  
            self.inited = True            


    def waypoint_reached__cb(self,fb):
        """ Waypoint reached
        
        Logic checking for proximity threshold is handled by the line following action"""

        # Determine if waypoint has been reached
        if fb.status.text == "WP Reached":

            rospy.loginfo("Waypoint reached signal received")

            # Check distance to waypoint
            x,y = Utils.displacement(self.controller_state.absolute_position,self.controller_state.waypoint_position)
            dist = np.linalg.norm(np.array([x,y]))
            rospy.loginfo("Distance to the waypoint : {}".format(dist))
            if dist < self.controller_params.waypoint_tolerance:
                self.waypoints_cleared = True

            # Check that previous waypoints were reached
            if not self.waypoints_cleared:
                return 

            # Count waypoints reached
            self.controller_state.n_waypoints +=1

            self.update_direction()             # Determine control direction
            self.update_virtual_position()      # Update virtual postion
            self.update_ref()                   # Send new waypoint

            self.waypoints_cleared = False


    def chlorophyl__cb(self,fb):
        """ Callback when a sensor reading is received 
        
        The sensor reading should be appended to the list of sensor readings, along with the associated
        lat lon position where the reading was taken. """

        # read values (the sensor is responsible for providing the Geo stamp i.e. lat lon co-ordinates)
        position = np.array([[fb.lon,fb.lat]])
        sample = fb.sample
        self.last_sample = fb.header.stamp

        # Apply moving average filter (size 3)
        self.samples = np.append(self.samples,sample) 
        self.samples[-1] = np.average(self.samples[-3:])

        # Record sample position   
        self.samples_positions = np.append(self.samples_positions, position,axis=0)

        # Check if front has been reached
        if not self.front_crossed:
            if self.samples[-1] >= 0.95*self.args['delta_ref']:
                rospy.loginfo("FRONT HAS BEEN REACHED")
                self.front_crossed = True
                self.update_virtual_position()
                self.update_direction()                
                self.update_ref()
                self.waypoints_cleared = False

        grad_angle = None
        if self.inited:
            grad = self.estimate_gradient()
            grad_angle = math.degrees(math.atan2(grad[1],grad[0]))

        # logging stuff :)
        rospy.loginfo('Sample : {} at {},{} est gradient {:.2f} degrees (sample #{})'.format(fb.sample,fb.lat,fb.lon,grad_angle,len(self.samples)))    

    def dispatch_waypoint(self):
        pass       
    
    def run_node(self):

        init_coords = [20.87, 61.492]

        update_period = self.time_step
        rate = rospy.Rate(1/update_period)
        # while True:
        while not rospy.is_shutdown():

            self.dispatch_waypoint()

            rate.sleep()

    def update_ref(self):
        """ Update referece and publish new waypoint"""

        rospy.loginfo("Determining new waypoint")

        # Determine if y displacement should be positive or negative
        sign = 2 * (self.controller_state.n_waypoints % 2) - 1

        # Determine if we have reached the front
        i = len(self.samples)
        front_crossed = self.has_crossed_the_front()
        # rospy.loginfo("Crossed the front : {}".format(front_crossed))
        # rospy.loginfo("Samples taken : {}/{}".format(i,self.args['estimation_trigger_val']))
        if len(self.samples)>0:
            rospy.loginfo("Latest sample : {}/{}".format(self.samples[-1],0.95*self.args['delta_ref']))

        distance = self.controller_params.distance if front_crossed else 0
        along_track_displacement = distance / math.tan(math.radians(self.controller_params.angle)) if front_crossed else self.controller_params.distance*2

        # Determine the next waypoint
        next_wp = RelativePosition(x=along_track_displacement,y=sign*distance)
        next_wp_theta = math.degrees(math.atan2(next_wp.y,next_wp.x))
        rospy.loginfo("Next waypoint is {} m, {} m relative to current position ({:.2f} degrees)".format(next_wp.x,next_wp.y,next_wp_theta))

        # Bearing should always be 45%?
        bearing, range = Utils.toPolar(x=next_wp.x,y= next_wp.y)
        rospy.loginfo("HEADING {} degrees".format(math.degrees(self.controller_state.direction)))
        rospy.loginfo("BEARING {} degrees".format(math.degrees(bearing)))

        # Add current direction to bearing
        bearing += self.controller_state.direction
        rospy.loginfo("Next waypoint is has bearing and range : {} degrees {} m".format(math.degrees(bearing),range))

        # calculate change from current position
        dx = range*math.cos(bearing)
        dy = range*math.sin(bearing)

        # calculate displacement for waypoint
        lat, lon = Utils.displace(current_position=self.controller_state.virtual_position,dx=dx,dy=dy)

        # Store waypoint
        self.controller_state.waypoint_position.lat = lat
        self.controller_state.waypoint_position.lon = lon
        publish_waypoint(latlontoutm_service = self.LATLONTOUTM_SERVICE,controller_params=self.controller_params,waypoint_pub=self.waypoint_pub,enable_waypoint_pub=self.enable_waypoint_pub,lat=self.controller_state.waypoint_position.lat,lon=self.controller_state.waypoint_position.lon,depth=0)

    def get_track_position(self,origin,use_relative_position=True):
        """ Return distance along the track """

        if use_relative_position:

            rospy.loginfo("Track position dx : {}".format(self.controller_state.relative_postion.x))
            rospy.loginfo("Track position dy : {}".format(self.controller_state.relative_postion.y))
            
            bearing, range = Utils.toPolar(x=self.controller_state.relative_postion.x,y=self.controller_state.relative_postion.y)
            rospy.loginfo("Track postion bearing and range : {:.2f} (degrees) {} (m)".format(math.degrees(bearing),range))
            bearing = bearing - self.controller_state.direction
            rospy.loginfo("Controller bearing : {}".format(math.degrees(self.controller_state.direction)))
            rospy.loginfo("Track postion bearing and range : {:.2f} (degrees) {} (m)".format(math.degrees(bearing),range))

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
        publish_vp(lat=self.controller_state.virtual_position.lat,lon=self.controller_state.virtual_position.lon,vp_pub=self.vp_pub)

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
            return

        # Estimate direction of the front
        grad = self.estimate_gradient()
        grad_heading = math.atan2(grad[1],grad[0]) # rad
        rospy.loginfo("Estimated gradient : {} ({} degrees)".format(grad,math.degrees(grad_heading)))

        # Perform control
        self.controller_state.direction = self.perform_control(grad=grad)

    def estimate_gradient(self):
        """ Estimate gradient """

        # TODO (Add windowed filtering on samples to smoothen out?)

        # Estimate the gradient
        if self.has_crossed_the_front():
            grad = np.array(self.est.est_grad(self.samples_positions[-(self.n_meas+1):], \
                                                                self.samples[-(self.n_meas+1):])).squeeze()
        else:
            grad = np.array([math.cos(self.controller_state.direction),math.sin(self.controller_state.direction)])

        self.gradients = np.append(self.gradients,[grad],axis=0)

        # Normalise gradient (unit vector)
        self.gradients[-1] = self.gradients[-1] / np.linalg.norm(self.gradients[-1])

        # Apply decaying factor to gradient (not sure if this will work)
        if len(self.gradients)>1:
            self.gradients[-1] = self.gradients[-2] * self.alpha + self.gradients[-1] * (1-self.alpha)

        # Publish calculated gradient
        grad_norm = self.gradients[-1]
        publish_gradient(lon =self.samples_positions[-1][0], lat=self.samples_positions[-1][1],x=grad_norm[0],y=grad_norm[1],gradient_pub=self.gradient_pub)

        return grad_norm

    def perform_control(self,grad):
        """ Calculate the control action 
        
        returns the new heading"""

        # Create vector orthogonal to gradient
        epsi = np.array([-grad[1],grad[0]])

        error = self.args['delta_ref'] - self.samples[-1]
        u_seek = self.controller_params.seeking_gain * error * grad
        u_follow = self.controller_params.following_gain * epsi
        u = u_seek + u_follow

        seek_angle =  math.degrees(math.atan2(u_seek[1],u_seek[0])) 
        follow_angle = math.degrees(math.atan2(u_follow[1],u_follow[0]))  

        rospy.loginfo(("error : {}".format(error)))
        rospy.loginfo(("seek control : {} ({} degrees)".format(u_seek,seek_angle)))
        rospy.loginfo(("follow control : {} ({} degrees)".format(u_follow,follow_angle)))

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