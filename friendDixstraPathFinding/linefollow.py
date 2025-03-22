from XRPLib.differential_drive import DifferentialDrive
from linesensors import Linesensors
ls = Linesensors()

class Linefollow:

    def __init__(self) -> None:

        # initiate a line tracking object
        self.drivetrain = DifferentialDrive.get_default_differential_drive()
        self.line_sense = Linesensors()
        # adjust drive effort and kp to desired values
        self.Drive_Effort: float = 0.4
        self.KP: float = 0.5


    def follow_to_intersection(self) -> None:

        # proportional control algorithm
        print("Starting Follow To Intersection")
        # follow line until intersection
        while not ls.at_cross():
          self.drivetrain.arcade(self.Drive_Effort, -self.KP*ls.get_error())
        self.drivetrain.stop()
        # drive through intersection so line isn't counted twice 
        self.drive_past_intersection()
    

    def turn_counterclockwise(self) -> None:

        # turn counterclockwise (left) at intersection
        # drive past line to optimal turning location
        self.drivetrain.straight(6)
        self.drivetrain.stop()
        # turn 80 degrees
        self.drivetrain.turn(80, 0.5)
        # slow down and continue turning until line
        while not ls.either_on_line:
            self.drivetrain.set_effort(0.5, -0.5)
        self.drivetrain.stop()

    
    def turn_clockwise(self) -> None:

        # turn clockwise (right) at intersection
        # drive past line to optimal turning position
        self.drivetrain.straight(6)
        self.drivetrain.stop()
        # turn 80 degrees
        self.drivetrain.turn(-80, 0.5)
        # slow down and continue turning until line
        while not ls.either_on_line:
            self.drivetrain.set_effort(0.5, -0.5)
        self.drivetrain.stop()
    
    def turn_180(self) -> None:
        
        # turn around at intersection
        # drive past line to optimal turning position
        self.drivetrain.straight(6)
        self.drivetrain.stop()
        # turn 170 degrees
        self.drivetrain.turn(170, 0.5)
        # slow down and continue turning until line
        while not ls.either_on_line:
            self.drivetrain.set_effort(-0.5, 0.5)
        self.drivetrain.stop()


    def drive_past_intersection(self) -> None:

        # Drive through intersection and stop
        while ls.at_cross():
            self.drivetrain.set_effort(self.Drive_Effort - (ls.get_error()*self.KP), self.Drive_Effort + (ls.get_error()*self.KP))
        self.drivetrain.stop()


    def stop_robot(self) -> None:
        
        # kill all drivetrain function
        self.drivetrain.stop()
        