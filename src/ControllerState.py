from AbsolutePosition import AbsolutePosition
from RelativePosition import RelativePosition

class ControllerState():

    def __init__(self):

        # Position of the vehicle
        self.virtual_position = AbsolutePosition()

        # Position of the vehicle relative to the virtual vehicle
        self.relative_postion = RelativePosition()

        # Track bearing (rad)
        self.direction = 0

        # Speed in m/s
        self.speed = 0

        # Number of waypoints since last update
        self.n_waypoints = 0