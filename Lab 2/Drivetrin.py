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

        self.leftMotor = LeftMotor
        self.rightMotor = RightMotor

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





    def configPIDWallFollow(
        self,
        K_P: float,
        wallFollowSpeed: float,
        frontRangeFinder: Sonar,
        rightRangeFInder: Sonar,
        RightFollowDistance: float = 4.3,
        ForwardWallDistance: float = 10.5,
        ForwardWallDistance2: float = 51
    ):
        # 51
        # 10 3/4
        self.K_P = K_P
        self.wallFollowSpeed = wallFollowSpeed
        self.frontRangeFinder = frontRangeFinder
        self.rightRangeFInder = rightRangeFInder

        self.rightFollowDistance = RightFollowDistance
        """Distance to follow on right side, lab step 1"""

        self.forwardWallDistance = ForwardWallDistance
        """Distance required to move to next step"""

        self.forwardWallDistance2 = ForwardWallDistance2
        """Distance required to move to the last step"""

    def drive(self, speed):
        """drive function - For negative values of direction, the robot turns right, and for positive values of direction, the robot turns left.  For values of direction with small magnitudes, the robot gradually turns.  For values of direction with large magnitudes, the robot turns more quickly."""
        self.leftMotor.spin(FORWARD, speed, RPM)
        self.rightMotor.spin(FORWARD, speed, RPM)

    def wallFollowInches(self, setDistanceFromWall):
        while self.frontRangeFinder.distance(DistanceUnits.IN) != setDistanceFromWall:
            rightError = setDistanceFromWall - self.frontRangeFinder.distance()
            self.drive(-self.K_P * rightError)

    def driveDistance(self, Inches: float):
        """Function to drive BaseBot straight for some number of inches"""
        Velocity = 100
        self.leftMotor.spin_for(
            FORWARD,
            Inches * self.gearRatio * self.rotationsPerInch,
            RotationUnits.REV,
            Velocity,
            RPM,
            False,
        )
        self.rightMotor.spin_for(
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
        self.leftMotor.spin_for(
            REVERSE,
            Rotations * self.gearRatio * self.trackWidth / self.wheelDiameter,
            RotationUnits.REV,
            Velocity,
            RPM,
            False,
        )
        self.rightMotor.spin_for(
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
            self.leftMotor.spin_for(
                FORWARD,
                Rotations * self.gearRatio * self.trackWidth / self.wheelDiameter * 2,
                RotationUnits.REV,
                Velocity,
                RPM,
                True,
            )
        else:
            self.rightMotor.spin_for(
                REVERSE,
                Rotations * self.gearRatio * self.trackWidth / self.wheelDiameter * 2,
                RotationUnits.REV,
                Velocity,
                RPM,
                True,
            )
