import rospy

from smarc_msgs.srv import LatLonToUTM
from geographic_msgs.msg import GeoPoint

def latlon_to_utm(service,lat,lon,z,in_degrees=False):

    """
    Use proxy service to convert lat/lon to utmZ
    """

    try:
        rospy.wait_for_service(service, timeout=1)
    except:
        rospy.logwarn(str(service)+" service not found!")
        return (None, None)
    try:
        latlontoutm_service = rospy.ServiceProxy(service,
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
        rospy.logerr_throttle_identical(5, "LatLon to UTM service failed! namespace:{}".format(service))
        return (None, None)