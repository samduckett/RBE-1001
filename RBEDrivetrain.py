from vex import *


class RBEDrivetrain:
    # constructor for the drivetrain
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

        # the gear ratio on the drive motor
        self.gearRatio = GearRatio

        # the radius of the drive wheels
        self.wheelDiameter = wheelDiameter

        # how for the robot will move with 1 rotation
        self.wheelCircumference = wheelDiameter * math.PI

        # distance from left wheal to right wheel
        self.trackWidth = TrackWidth

        # distance from front to back wheel
        self.wheelbase = Wheelbase

        # hoe Rotations will move the robot 1 inch
        self.rotationsPerInch = 1 / self.wheelCircumference

    # Function to drive BaseBot straight for some number of inches
    def driveStraight(self, Inches: float):
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

    # Function to turn BaseBot for some number of Rotations
    def turnInPlace(self, Rotations: float):
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
