import rospy

# Smarc imports
from geographic_msgs.msg import GeoPointStamped

def publish_offset(lat,lon,pub):
    """ Publish lat_lon offset"""

    msg = GeoPointStamped()
    msg.header.stamp = rospy.Time.now()
    msg.position.latitude = lat
    msg.position.longitude = lon
    msg.position.altitude = -1

    pub.publish(msg)   