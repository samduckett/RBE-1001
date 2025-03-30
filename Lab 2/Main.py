from vex import *


class RBEDrivetrain:
    """drivetrain class for RBE 1001, will build on the entire class"""

    def __init__(
        self,
        frontRangeFinder: Sonar,
        rightRangeFinder: Sonar,
        motorLeft: Motor,
        motorRight: Motor,
        gyro: Inertial,
    ):
        self.frontRangeFinder = frontRangeFinder
        self.rightRangeFinder = rightRangeFinder

        self.motorLeft = motorLeft
        self.motorRight = motorRight

        self.gyro = gyro

        self.driveGearRatio = 5
        self.wheelDiameter = 3
        self.wheelBase = 8
        self.wheelTrack = 10.25

        self.wheelCircumference = self.wheelDiameter * math.pi
        self.degPerInch = 360 / self.wheelCircumference

    def driveLeftMotor(self, speed):
        self.motorLeft.spin(FORWARD, speed, RPM)

    def driveRightMotor(self, speed):
        self.motorRight.spin(FORWARD, speed, RPM)

    def spin(
        self,
        speed: float,
        deg: float,
        rotationCOE: float = 0,
        pause: bool = True,
        useGyro: bool = False,
    ):
        """
        The Velocity the robot wheels will spin \n
        the degrees the robot will rotate \n
        rotates the robot around the back wheels, centered around rotationCOE being [-1, 1], default to 0\n
        - where right wheal is -1 \n
        - where left wheal is 1 \n
        - and center is 0 \n
        - with any numbers between being in between \n
        if the robot will pause until the robot will be compleat, defaults to TRUE
        if will use the gyro to true with more accuracy
        """
        if useGyro:
            kP = 5
            goalHeading = self.gyro.heading() + deg

            error = 999
            while abs(error) < 3:  # error less than 3 degrees
                error = goalHeading - self.gyro.heading()
                self.motorLeft.spin(FORWARD, speed + kP * error, RPM)
                self.motorRight.spin(FORWARD, speed - kP * error, RPM)
            self.motorLeft.stop(HOLD)
            self.motorRight.stop(HOLD)
        else:
            motorSpinFor = (
                deg * self.driveGearRatio * self.wheelTrack / self.wheelDiameter
            )

            self.motorLeft.spin_for(
                FORWARD,
                motorSpinFor * (1 + rotationCOE),
                DEGREES,
                speed,
                RPM,
                False,
            )
            self.motorRight.spin_for(
                FORWARD,
                motorSpinFor * (1 - rotationCOE),
                DEGREES,
                speed,
                RPM,
                pause,
            )

    # Drive both motors for a set distance in inches
    # where dist is distance desired, can also be negative
    # where speed is motor RPM
    # where pause is whether or not following code should be executed or wait until finished

    def driveForwardDist(self, dist, speed, pause):
        self.motorLeft.spin_for(
            FORWARD, dist / self.wheelCircumference, TURNS, speed, RPM, False
        )
        self.motorRight.spin_for(
            FORWARD, dist / self.wheelCircumference, TURNS, speed, RPM, pause
        )

    # Drive both motors at a set speed
    # where speed is motor RPM
    def driveForward(self, speed):
        self.driveLeftMotor(speed)
        self.driveRightMotor(speed)

    def driveForwardUntilDistance(self, forwardDistWall, speed):
        while self.frontRangeFinder.distance(DistanceUnits.IN) >= forwardDistWall:
            self.driveForward(speed)
            brain.screen.print_at(
                "FRONT SENSOR", rangeFinderFront.distance(DistanceUnits.IN), x=40, y=40
            )
        self.motorLeft.stop(HOLD)
        self.motorRight.stop(HOLD)

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


class Arm:
    def __init__(self, armMotor: Motor):
        self.armMotor = armMotor

    def goToSetPoint(self, position):
        pass

    def log(self):
        pass


# configs
brain = Brain()

leftMotor = Motor(Ports.PORT1, 18_1, False)
rightMotor = Motor(Ports.PORT10, 18_1, True)
armMotor = Motor(Ports.PORT10, 18_1, True)

rangeFinderFront = Sonar(brain.three_wire_port.e)
rangeFinderRight = Sonar(brain.three_wire_port.c)

imu = Inertial(6)

# vars
kp = float(12)

rightFollowDistance = float(4.3)
forwardWallDistance = float(5)
forwardWallDistance2 = float(51)

# config sensors
brain.screen.print("running /n")

rangeFinderFront.distance(DistanceUnits.IN)
rangeFinderRight.distance(DistanceUnits.IN)

imu.calibrate()
while imu.is_calibrating:
    wait(5)

# config drivetrain
rbeDriveTrain = RBEDrivetrain(
    rangeFinderFront,
    rangeFinderRight,
    leftMotor,
    rightMotor,
    imu,
)

arm = Arm()


def part1():
    rbeDriveTrain.driveForwardUntilDistance(8, 200)
    rbeDriveTrain.driveForwardDist(-2.5, 200, False)
    # rbeDriveTrain.spinAboutWheel(200, 90, "LEFT", false)
    rbeDriveTrain.spin(200, 90, 1, False)
    rbeDriveTrain.brazeWallUntilDistance(4.2, 52.5, 200, kp)
    rbeDriveTrain.driveForwardUntilDistance(30, 200)
    pass


def part2():
    # spins 90 deg
    rbeDriveTrain.spin(100, 90)
    # spins -90 degres around left wheal
    rbeDriveTrain.spin(100, -90, 1)
    # spinds 90 degrees aroudn right wheal
    rbeDriveTrain.spin(100, 90, -1)
    # spins -90 degrees around .75 left wheal
    rbeDriveTrain.spin(100, -90, 0.75)


def part3():
    pass


def part4():
    while True:
        pass


# ZERO HEADING FOE GYRO
imu.set_heading(0, DEGREES)
part2()
