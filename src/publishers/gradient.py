import rospy

# SMaRC imports
from smarc_msgs.msg import AlgaeFrontGradient

def publish_gradient(lat,lon,x,y,gradient_pub):

    """ 
    Publish gradient information
    """

    msg = AlgaeFrontGradient()
    msg.header.stamp = rospy.Time.now()
    msg.lat = lat
    msg.lon = lon
    msg.x = x
    msg.y = y

    gradient_pub.publish(msg)  