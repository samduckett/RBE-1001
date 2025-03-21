from vex import *

class RBEDrivetrain:
    # constructor for the drivetrain
    def __init__(self, 
                 LeftMotor:Motor, 
                 RightMotor:Motor, 
                 GearRatio:float = 1, 
                 wheelRadius:float = 1, 
                 TrackWidth:float = 1, 
                 Wheelbase:float = 1):
          
          self.LeftMotor = LeftMotor
          self.RightMotor = RightMotor


          # the gear ratio on the drive motor
          self.GearRatio = GearRatio

          #the radius of the drive wheels
          self.wheelRadius = wheelRadius

          #how for the robot will move with 1 rotation
          self.wheelTravel = wheelRadius * math.PI

          # distance from left wheal to right wheel
          self.TrackWidth = TrackWidth

          # distance from front to back wheel
          self.Wheelbase = Wheelbase


    
    #drives the chassis distance in inches
    def drive_inches(self, inches:float):
          pass