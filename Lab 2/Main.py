from vex import *


class RBEDrivetrain:
    """drivetrain class for RBE 1001, will build on the entire class"""

    def __init__(
        self,
        frontRangeFinder: Sonar,
        rightRangeFinder: Sonar,
        motorLeft: Motor,
        motorRight: Motor,
    ):

        self.frontRangeFinder = frontRangeFinder
        self.rightRangeFinder = rightRangeFinder
        self.motorLeft = motorLeft
        self.motorRight = motorRight

        self.finalDrive = 5
        self.wheelDiameter = 3
        self.wheelBase = 8
        self.wheelTrack = 10.25

        self.wheelCircumference = self.wheelDiameter * math.pi
        self.rotationsPerInch = 1 / self.wheelCircumference
        self.wheelRotDegPerBodyTurnDeg = (self.wheelTrack * 2) / self.wheelDiameter

    def driveLeftMotor(self, speed):
        """drive function - For negative values of direction, the robot turns right, and for positive values of direction, the robot turns left.  For values of direction with small magnitudes, the robot gradually turns.  For values of direction with large magnitudes, the robot turns more quickly."""
        self.motorLeft.set_velocity(speed, RPM)
        self.motorLeft.spin(FORWARD)

    def driveRightMotor(self, speed):
        self.motorRight.set_velocity(speed, RPM)
        self.motorRight.spin(FORWARD)

    def spin(self, speed):
        pass

    def spinAboutWheel(self, speed, deg, wheel, pause):
        if wheel == "LEFT":
            self.motorRight.spin_for(
                REVERSE,
                self.finalDrive * deg * self.wheelRotDegPerBodyTurnDeg,
                DEGREES,
                speed,
                RPM,
                pause,
            )
        elif wheel == "RIGHT":
            self.motorLeft.spin_for(
                REVERSE,
                self.finalDrive * deg * self.wheelRotDegPerBodyTurnDeg,
                DEGREES,
                speed,
                RPM,
                pause,
            )

    def spinInPlace(self, speed, deg, pause):
        self.motorLeft.spin_for(
            FORWARD,
            self.finalDrive * deg / 2 * self.wheelRotDegPerBodyTurnDeg,
            DEGREES,
            speed,
            RPM,
            False,
        )
        self.motorRight.spin_for(
            FORWARD,
            -self.finalDrive * deg / 2 * self.wheelRotDegPerBodyTurnDeg,
            DEGREES,
            speed,
            RPM,
            pause,
        )

    def brazeWallUntilDistance(self, rightFollowDist, forwardDistWall, speed, kp):
        while self.frontRangeFinder.distance(DistanceUnits.IN) >= forwardDistWall:
            rightError = rightFollowDist - self.rightRangeFinder.distance(
                DistanceUnits.IN
            )
            brain.screen.print_at(
                "FRONT SENSOR", rangeFinderFront.distance(DistanceUnits.IN), x=40, y=40
            )
            brain.screen.print_at(
                "RIGHT SENSOR", rangeFinderRight.distance(DistanceUnits.IN), x=40, y=90
            )
            self.driveRightMotor(speed)
            self.driveLeftMotor(kp * rightError + speed)
        self.motorLeft.stop(HOLD)
        self.motorRight.stop(HOLD)


# configs
brain = Brain()

leftMotor = Motor(Ports.PORT1, 18_1, False)
rightMotor = Motor(Ports.PORT10, 18_1, True)
rangeFinderFront = Sonar(brain.three_wire_port.e)
rangeFinderRight = Sonar(brain.three_wire_port.c)

# vars
kp = float(12)

rightFollowDistance = float(4.3)
forwardWallDistance = float(5)
forwardWallDistance2 = float(51)

rbeDriveTrain = RBEDrivetrain(
    rangeFinderFront,
    rangeFinderRight,
    leftMotor,
    rightMotor,
)

brain.screen.print("running /n")

rangeFinderFront.distance(DistanceUnits.IN)
rangeFinderRight.distance(DistanceUnits.IN)

wait(500)


def part1():
    pass


def part2():
    pass


def part3():
    pass


part2()
