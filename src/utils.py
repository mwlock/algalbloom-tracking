import math
import utm

from positions import AbsolutePosition

class Utils():

    @staticmethod
    def toPolar(x, y):
        angle = math.atan2(y, x)
        norm = math.sqrt(x * x + y * y)
        return (angle),norm

    @staticmethod
    def displace(current_position,dx,dy):
        """ Returns lat lon after dx and dy displacement """

        current_utm_coords = utm.from_latlon(current_position.lat, current_position.lon)
        x = current_utm_coords[0] + dx
        y = current_utm_coords[1] + dy

        displaced = utm.to_latlon(x,y,current_utm_coords[2],current_utm_coords[3])
        return displaced[0],displaced[1]

    @staticmethod
    def displacement(current_position,virtual_position):
        """ Return the displacement in m between current positon and virtual position"""

        current_position_utm_coords = utm.from_latlon(current_position.lat, current_position.lon)
        virtual_position_utm_coords = utm.from_latlon(virtual_position.lat, virtual_position.lon)

        dx = virtual_position_utm_coords[0] - current_position_utm_coords[0]
        dy = virtual_position_utm_coords[1] - current_position_utm_coords[1]

        return dx,dy