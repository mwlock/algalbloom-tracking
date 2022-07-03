import math
import utm

from AbsolutePosition import AbsolutePosition

class Utils():

    @staticmethod
    def toPolar(x, y):
        angle = math.atan2(y, x)
        norm = math.sqrt(x * x + y * y)
        return math.degrees(angle),norm

    @staticmethod
    def displace(current_position,dx,dy):

        current_utm_coords = utm.from_latlon((current_position.lat, current_position.lon))
        x = current_utm_coords[0] + dx
        y = current_utm_coords[1] + dy

        displaced = utm.to_latlon(x,y,current_utm_coords[2],current_utm_coords[3])
        return displaced[0],displaced[1]