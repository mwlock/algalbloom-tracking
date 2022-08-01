import rospy

# Smarc imports
from geographic_msgs.msg import GeoPointStamped

def publish_vp(lat,lon,vp_pub):
    """ Publish current vp """

    msg = GeoPointStamped()
    msg.header.stamp = rospy.Time.now()
    msg.position.latitude = lat
    msg.position.longitude = lon
    msg.position.altitude = -1

    vp_pub.publish(msg)   