
from positions import AbsolutePosition,RelativePosition,VirtualPosition

class ControllerState():
    """ Controler state"""

    def __init__(self):

        # Position of the vehicle
        self.absolute_position = AbsolutePosition()

        # Position of the vertual vehicle
        self.virtual_position = VirtualPosition()

        # Position of the vehicle relative to the virtual vehicle
        self.relative_postion = RelativePosition()

        # Track bearing (rad)
        self.direction = 0

        # Speed in m/s
        self.speed = 0

        # Number of waypoints since last update
        self.n_waypoints = 0

    def __str__(self):
        return "virtual postion : {},{}\nrelative position : {},{}\ndirection:{}\nspeed:{}\nn_waypoints:{}\n".format(self.virtual_position.lat,self.virtual_position.lon,self.relative_postion.x,self.relative_postion.y,self.direction,self.speed,self.n_waypoints)