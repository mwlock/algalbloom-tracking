from AbsolutePosition import AbsolutePosition
from RelativePosition import RelativePosition

class ControllerState():

    def __init__(self,virtual_position,relative_postion,direction,speed,n_waypoints):

        # Position of the vehicle
        self.virtual_position = virtual_position

        # Position of the vehicle relative to the virtual vehicle
        self.relative_postion = relative_postion

        # Track bearing (rad)
        self.direction = direction

        # Speed in m/s
        self.speed = speed

        # Number of waypoints since last update
        self.n_waypoints = n_waypoints