import rospy

# Smarc
from smarc_msgs.msg import GotoWaypoint, LatLonOdometry
from std_msgs.msg import Float64, Header, Bool, Empty

# Services
from services.latlon_to_utm import latlon_to_utm

def publish_waypoint(latlontoutm_service,controller_params,waypoint_pub,enable_waypoint_pub,lat,lon,depth,travel_rpm):    

    """ Publish waypoint to SAM"""

    # Convert lat,lon to UTM
    x, y = latlon_to_utm(service=latlontoutm_service, lat=lat,lon=lon,z=depth)

    # Speed and z controls
    z_control_modes = [GotoWaypoint.Z_CONTROL_DEPTH]
    speed_control_mode = [GotoWaypoint.SPEED_CONTROL_RPM,GotoWaypoint.SPEED_CONTROL_SPEED]

    # Waypoint message
    msg = GotoWaypoint()
    msg.travel_depth = -1
    msg.goal_tolerance = controller_params.waypoint_tolerance
    msg.lat = lat
    msg.lon = lon
    msg.z_control_mode = z_control_modes[0]
    msg.travel_rpm = travel_rpm
    msg.speed_control_mode = speed_control_mode[0]
    msg.travel_speed = controller_params.speed
    msg.pose.header.frame_id = 'utm'
    msg.pose.header.stamp = rospy.Time(0)
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y

    # Enable waypoint following
    enable_waypoint_following = Bool()
    enable_waypoint_following.data = True
    enable_waypoint_pub.publish(enable_waypoint_following)

    # Publish waypoint
    waypoint_pub.publish(msg)
    rospy.loginfo('Published waypoint : {},{}'.format(lat,lon))