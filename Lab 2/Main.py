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
        lightLeft: Light,
        lightRight: Light,
    ):
        self.frontRangeFinder = frontRangeFinder
        self.rightRangeFinder = rightRangeFinder

        self.motorLeft = motorLeft
        self.motorRight = motorRight

        self.gyro = gyro
        self.lightLeft = lightLeft
        self.lightRight = lightRight

        self.driveGearRatio = 1
        self.wheelDiameter = 3
        self.wheelBase = 8
        self.wheelTrack = 11

        self.wheelRotDegPerBodyTurnDeg = self.wheelTrack / self.wheelDiameter
        self.wheelCircumference = self.wheelDiameter * math.pi
        self.degPerInch = 360 / self.wheelCircumference

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
            kP = 2.5
            kSpeed = 25
            goalHeading = self.gyro.rotation() + deg

            error = 999
            while abs(error) > 3:  # error less than 3 degrees\
                error = goalHeading - self.gyro.rotation()
                mult = math.copysign(kSpeed + abs(kP * error), error)
                self.motorLeft.spin(FORWARD, -mult, RPM)
                self.motorRight.spin(FORWARD, mult, RPM)
                brain.screen.print_at("rotation", self.gyro.rotation(), x=40, y=90)
                brain.screen.print_at("error", error, x=40, y=50)
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
                -motorSpinFor * (1 - rotationCOE),
                DEGREES,
                speed,
                RPM,
                pause,
            )

    def driveForwardTillWall(self, wallDist, frontSensor: bool = True):
        kP_gyro = 1
        kP_wall = 7.5

        kSpeed = 10

        goalHeading = self.gyro.rotation()

        GyroError = 999
        distError = 999

        while abs(distError) >= 1:

            # error
            if frontSensor:
                distError = self.frontRangeFinder.distance(DistanceUnits.IN) - wallDist
            else:
                distError = self.rightRangeFinder.distance(DistanceUnits.IN) - wallDist

            GyroError = goalHeading - self.gyro.rotation()

            self.motorLeft.spin(
                FORWARD, kSpeed + distError * kP_wall - kP_gyro * GyroError, RPM
            )
            self.motorRight.spin(
                FORWARD, kSpeed + distError * kP_wall + kP_gyro * GyroError, RPM
            )

            # Logging
            brain.screen.print_at("rotation", self.gyro.rotation(), x=40, y=90)
            brain.screen.print_at(
                "Dist",
                distError,
                x=40,
                y=110,
            )
            brain.screen.print_at("error", GyroError, x=40, y=50)

        self.motorLeft.stop(HOLD)
        self.motorRight.stop(HOLD)

    def driveForwardGyroTillWall(self, wallDist, frontSensor: bool = True):
        kP_wall = 7.5

        kSpeed = 10

        distError = 999

        while abs(distError) >= 1:

            # error
            if frontSensor:
                distError = self.frontRangeFinder.distance(DistanceUnits.IN) - wallDist
            else:
                distError = self.rightRangeFinder.distance(DistanceUnits.IN) - wallDist

            self.motorLeft.spin(FORWARD, kSpeed + distError * kP_wall, RPM)
            self.motorRight.spin(FORWARD, kSpeed + distError * kP_wall, RPM)

            # Logging
            brain.screen.print_at("rotation", self.gyro.rotation(), x=40, y=90)
            brain.screen.print_at(
                "Dist",
                distError,
                x=40,
                y=110,
            )

        self.motorLeft.stop(HOLD)
        self.motorRight.stop(HOLD)

    # Drive both motors for a set distance in inches
    # where dist is distance desired, can also be negative
    # where speed is motor RPM
    # where pause is whether or not following code should be executed or wait until finished

    def driveForwardDist(self, dist, speed, pause):
        self.motorLeft.spin_for(
            FORWARD,
            self.driveGearRatio * dist / self.wheelCircumference,
            TURNS,
            speed,
            RPM,
            False,
        )
        self.motorRight.spin_for(
            FORWARD,
            self.driveGearRatio * dist / self.wheelCircumference,
            TURNS,
            speed,
            RPM,
            pause,
        )

    # # Drive both motors at a set speed
    # # where speed is motor RPM
    # def driveForward(self, speed):
    #     self.driveLeftMotor(speed)
    #     self.driveRightMotor(speed)

    # def driveForwardUntilDistance(self, forwardDistWall, speed):
    #     while self.frontRangeFinder.distance(DistanceUnits.IN) >= forwardDistWall:
    #         self.driveForward(speed)
    #         brain.screen.print_at(
    #             "FRONT SENSOR", rangeFinderFront.distance(DistanceUnits.IN), x=40, y=40
    #         )
    #     self.motorLeft.stop(HOLD)
    #     self.motorRight.stop(HOLD)

    # def brazeWallUntilDistance(self, rightFollowDist, forwardDistWall, speed, kp):
    #     while self.frontRangeFinder.distance(DistanceUnits.IN) >= forwardDistWall:
    #         rightError = rightFollowDist - self.rightRangeFinder.distance(
    #             DistanceUnits.IN
    #         )
    #         brain.screen.print_at(
    #             "FRONT SENSOR", rangeFinderFront.distance(DistanceUnits.IN), x=40, y=40
    #         )
    #         brain.screen.print_at(
    #             "RIGHT SENSOR", rangeFinderRight.distance(DistanceUnits.IN), x=40, y=90
    #         )
    #         self.driveRightMotor(speed)
    #         self.driveLeftMotor(kp * rightError + speed)
    #     self.motorLeft.stop(HOLD)
    #     self.motorRight.stop(HOLD)

    # def brazeWallForDistane(self, rightFollowDist, dist, speed, kp):
    #     self.motorRight.spin_for(
    #         FORWARD,
    #         self.driveGearRatio * dist / self.wheelCircumference,
    #         TURNS,
    #         speed,
    #         RPM,
    #         False,
    #     )
    #     while self.motorRight.is_spinning():
    #         rightError = rightFollowDist - self.rightRangeFinder.distance(
    #             DistanceUnits.IN
    #         )
    #         brain.screen.print_at(
    #             "RIGHT SENSOR", rangeFinderRight.distance(DistanceUnits.IN), x=40, y=90
    #         )
    #         self.driveLeftMotor(kp * rightError + speed)
    #     self.motorLeft.stop(HOLD)
    #     self.motorRight.stop(HOLD)


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
armMotor = Motor(Ports.PORT8, 18_1, True)

rangeFinderFront = Sonar(brain.three_wire_port.e)
rangeFinderRight = Sonar(brain.three_wire_port.c)

imu = Inertial(Ports.PORT6)

lightLeft = Light(brain.three_wire_port.a)
lightRight = Light(brain.three_wire_port.b)

# config sensors
brain.screen.print("Calibrating \n")


rangeFinderFront.distance(DistanceUnits.IN)
rangeFinderRight.distance(DistanceUnits.IN)

imu.calibrate()
while imu.is_calibrating():
    wait(5)

brain.screen.print("Finished Calibrating \n")

# config drivetrain
rbeDriveTrain = RBEDrivetrain(
    rangeFinderFront,
    rangeFinderRight,
    leftMotor,
    rightMotor,
    imu,
    lightLeft,
    lightRight,
)

arm = Arm(armMotor)


def part1():
    rbeDriveTrain.driveForwardTillWall(7)
    rbeDriveTrain.spin(100, 90)
    rbeDriveTrain.driveForwardTillWall(50)
    rbeDriveTrain.spin(100, 90)
    rbeDriveTrain.driveForwardDist(13, 200, True)


def part2():
    # spins 90 deg
    rbeDriveTrain.driveForwardGyroTillWall(7)
    rbeDriveTrain.spin(120, -90, 0, True, True)
    rbeDriveTrain.driveForwardGyroTillWall(50)
    rbeDriveTrain.spin(120, -90, 0, True, True)
    rbeDriveTrain.driveForwardDist(10, 200, True)


def part3():
    pass


def part4():
    while True:
        arm.log()


# ZERO HEADING FOE GYRO
imu.set_heading(0, DEGREES)
part1()
