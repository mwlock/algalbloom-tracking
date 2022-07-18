class ControllerParameters:

    def __init__(self):
        
        # Angle with track bearing (rad)
        self.angle = 0

        # Horizontal distance travelled by the vehicle.
        self.distance = 0

        # Front seeking gain
        self.seeking_gain = 0

        # Front following gain
        self.following_gain = 0

        # Vehicle speed
        self.speed = 0

        # Waypoint tolerance (m)
        self.waypoint_tolerance = 0