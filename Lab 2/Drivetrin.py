from vex import *


class RBEDrivetrain:
    """drivetrain class for RBE 1001, will build on the entire class"""

    def __init__(
        self,
        LeftMotor: Motor,
        RightMotor: Motor,
        GearRatio: float = 1,
        wheelDiameter: float = 1,
        TrackWidth: float = 1,
        Wheelbase: float = 1,
    ):

        self.LeftMotor = LeftMotor
        self.RightMotor = RightMotor

        self.gearRatio = GearRatio
        """the gear ratio on the drive motor"""

        self.wheelDiameter = wheelDiameter
        """the radius of the drive wheels"""

        self.wheelCircumference = wheelDiameter * 3.141592
        """how for the robot will move with 1 rotation"""

        self.trackWidth = TrackWidth
        """distance from left Wheel to right wheel"""

        self.wheelbase = Wheelbase
        """distance from front to back wheel"""

        self.rotationsPerInch = 1 / self.wheelCircumference
        """how Rotations will move the robot 1 inch"""

    def driveStraight(self, Inches: float):
        """Function to drive BaseBot straight for some number of inches"""
        Velocity = 100
        self.LeftMotor.spin_for(
            FORWARD,
            Inches * self.gearRatio * self.rotationsPerInch,
            RotationUnits.REV,
            Velocity,
            RPM,
            False,
        )
        self.RightMotor.spin_for(
            FORWARD,
            Inches * self.gearRatio * self.rotationsPerInch,
            RotationUnits.REV,
            Velocity,
            RPM,
            True,
        )

    def turnInPlace(self, Rotations: float):
        """Function to turn BaseBot for some number of Rotations"""
        Velocity = 100
        self.LeftMotor.spin_for(
            REVERSE,
            Rotations * self.gearRatio * self.trackWidth / self.wheelDiameter,
            RotationUnits.REV,
            Velocity,
            RPM,
            False,
        )
        self.RightMotor.spin_for(
            FORWARD,
            Rotations * self.gearRatio * self.trackWidth / self.wheelDiameter,
            RotationUnits.REV,
            Velocity,
            RPM,
            True,
        )

    def turnAroundWheel(self, Rotations: float):
        Velocity = 100
        if Rotations > 0:
            self.LeftMotor.spin_for(
                FORWARD,
                Rotations * self.gearRatio * self.trackWidth / self.wheelDiameter * 2,
                RotationUnits.REV,
                Velocity,
                RPM,
                True,
            )
        else:
            self.RightMotor.spin_for(
                REVERSE,
                Rotations * self.gearRatio * self.trackWidth / self.wheelDiameter * 2,
                RotationUnits.REV,
                Velocity,
                RPM,
                True,
            )


# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       nbertozzi                                                    #
# 	Created:      10/19/2023, 6:27:34 PM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

left_motor = Motor(Ports.PORT1, 18_1, True)
right_motor = Motor(Ports.PORT10, 18_1, False)
rangeFinderFront = Sonar(brain.three_wire_port.g)
rangeFinderRightSide = Sonar(brain.three_wire_port.a)

WHEEL_DIAMETER = 4.0
GEAR_RATIO = 5.0
WHEEL_TRACK = 11.0 

K_P = 10
SET_WALL_FOLLOW_SPEED = 70         # RPM
SET_WALL_DISTANCE = 11            # inches for wall following
TURN_ANGLE_AT_WALL = 90           # turn left angle in degrees
SET_DISTANCE_TO_START_TURN = 8      # inches from wall in front to begin turn

# drive function - For negative values of direction, the robot turns right, and for positive
# values of direction, the robot turns left.  For values of direction with small magnitudes, the
# robot gradually turns.  For values of direction with large magnitudes, the robot turns more
# quickly.

def drive(speed, direction):
   left_motor.set_velocity(speed * direction, RPM)
   right_motor.set_velocity(speed * direction, RPM)
   left_motor.spin(FORWARD)
   right_motor.spin(FORWARD)

# Function to wall follow at set distance from wall

def wallFollowInches(setDistanceFromWall):
    rightError = setDistanceFromWall - rangeFinderFront.distance()
    drive(SET_WALL_FOLLOW_SPEED, -K_P*rightError)

# Function to turn BaseBot for some number of degrees

def turnInPlace(robotTurnInDegrees):
    left_motor.spin_for(REVERSE, robotTurnInDegrees*GEAR_RATIO*WHEEL_TRACK/WHEEL_DIAMETER, DEGREES, 100, RPM, False)
    right_motor.spin_for(FORWARD, robotTurnInDegrees*GEAR_RATIO*WHEEL_TRACK/WHEEL_DIAMETER, DEGREES, 100, RPM, True)

      
# Program to follow wall, turn left at next wall, and follow wall indefinitely

rangeFinderFront.distance(DistanceUnits.IN)     # acquire initial distance values
rangeFinderRightSide.distance(DistanceUnits.IN)

wait(500)

while(rangeFinderFront.distance(DistanceUnits.IN) != SET_WALL_DISTANCE):
    wallFollowInches(SET_WALL_DISTANCE)

turnInPlace(TURN_ANGLE_AT_WALL)

while(True):
    wallFollowInches(SET_WALL_DISTANCE)