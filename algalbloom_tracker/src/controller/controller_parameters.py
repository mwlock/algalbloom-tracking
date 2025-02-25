class ControllerParameters:

    def __init__(self):

        # Distance between waypoints.
        self.distance = 0

        # Front seeking gain
        self.seeking_gain = 0

        # Front following gain
        self.following_gain = 0

        # Vehicle speed
        self.speed = 0

        # Waypoint tolerance (m)
        self.waypoint_tolerance = 0

        # Set exponential gradient decay
        self.grad_decay = 0.95