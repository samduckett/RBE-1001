# region VEXcode Generated Robot Configuration
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


# initialize the brian
brain = Brain()

# initialize the left and right motor
left_motor = Motor(Ports.PORT1, 18_1, False)
right_motor = Motor(Ports.PORT10, 18_1, True)

brain.screen.print("Hello, World")

drivetrain = RBEDrivetrain(left_motor, right_motor, 5, 4, 11.0)


def polygon(sides: int, inches: float):
    for _ in range(sides):
        drivetrain.driveStraight(inches)
        drivetrain.turnInPlace(1 / sides)


def maze():
    # forward
    # turn left
    # forward
    # forward
    # turn right
    # forward
    # turn right
    # forward
    drivetrain.driveStraight(20)
    drivetrain.turnAroundWheel(1 / 4)
    drivetrain.driveStraight(16.5)
    drivetrain.turnAroundWheel(-1 / 4)
    drivetrain.driveStraight(9)
    drivetrain.turnAroundWheel(-1 / 4)
    drivetrain.driveStraight(5.5)


maze()
