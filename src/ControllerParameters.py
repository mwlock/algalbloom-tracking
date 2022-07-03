class ControllerParameters:

    def __init__(self,angle,distance,seeking_gain,following_gain):
        
        # Angle with track bearing (rad)
        self.angle = angle

        # Horizontal distance travelled by the vehicle.
        self.distance

        # Front seeking gain
        self.seeking_gain

        # Front following gain
        self.following_gain