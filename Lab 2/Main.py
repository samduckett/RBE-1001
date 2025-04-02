from vex import *


# ONLY ADD DEFINITIONS
class RBEDrivetrain:
    """drivetrain class for RBE 1001, will build on the entire class"""

    def __init__(
        self,
        frontRangeFinder: Sonar,
        rightRangeFinder: Sonar,
        motorLeft: Motor,
        motorRight: Motor,
        gyro: Inertial,
        lineLeft: Line,
        lineRight: Line,
        bumperButton1: Bumper,
    ):
        self.frontRangeFinder = frontRangeFinder
        self.rightRangeFinder = rightRangeFinder

        self.motorLeft = motorLeft
        self.motorRight = motorRight

        self.gyro = gyro
        self.lineLeft = lineLeft
        self.lineRight = lineRight

        self.bumperButton1 = bumperButton1

        self.finalDrive = 5
        self.wheelDiameter = 3
        self.wheelBase = 8
        self.wheelTrack = 9

        self.wheelRotDegPerBodyTurnDeg = self.wheelTrack / self.wheelDiameter
        self.wheelCircumference = self.wheelDiameter * math.pi
        self.degPerInch = 360 / self.wheelCircumference

    def driveLeftMotor(self, speed):
        self.motorLeft.spin(FORWARD, speed, RPM)

    def driveRightMotor(self, speed):
        self.motorRight.spin(FORWARD, speed, RPM)

    def spin(
        self,
        deg: float,
        speed: float = 175,
        rotationCOE: float = 0,
        pause: bool = True,
        useGyro: bool = True,
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
            kSpeed = speed
            # kSpeed = 175
            goalHeading = self.gyro.rotation() + deg

            error = 999
            while abs(error) > 3:  # error less than 3 degrees\
                error = goalHeading - self.gyro.rotation()
                if error != 0:
                    mult = (kSpeed + abs(kP * error)) * error / abs(error)
                self.motorLeft.spin(FORWARD, -mult, RPM)
                self.motorRight.spin(FORWARD, mult, RPM)
                brain.screen.print_at("rotation", self.gyro.rotation(), x=40, y=90)
                brain.screen.print_at("error", error, x=40, y=50)
            self.motorLeft.stop(HOLD)
            self.motorRight.stop(HOLD)
        else:
            motorSpinFor = deg * self.finalDrive * self.wheelTrack / self.wheelDiameter

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

    def driveForwardDist(self, dist, speed, pause):
        self.motorLeft.spin_for(
            FORWARD,
            self.finalDrive * dist / self.wheelCircumference,
            TURNS,
            speed,
            RPM,
            False,
        )
        self.motorRight.spin_for(
            FORWARD,
            self.finalDrive * dist / self.wheelCircumference,
            TURNS,
            speed,
            RPM,
            pause,
        )

    def driveSideWallTillWall(
        self, frontWallStopDistance, sideWallFollowDistance, direction: DirectionType
    ):
        # kP's
        kPSide = 10
        kPFront = 10
        kSpeed = 50

        # errors
        errorFront = 999
        errorSide = 0

        # within 1 in of target
        while abs(errorFront) >= 1:
            errorFront = (
                self.frontRangeFinder.distance(DistanceUnits.IN) - frontWallStopDistance
            )
            errorSide = (
                self.rightRangeFinder.distance(DistanceUnits.IN)
                - sideWallFollowDistance
            )

            mult = kSpeed + errorFront * kPFront

            # max speed of 130
            if mult > 130:
                mult = 130
            elif mult < -130:
                mult = -130

            self.motorLeft.spin(direction, mult - kPSide * errorSide, RPM)
            self.motorRight.spin(direction, mult + kPSide * errorSide, RPM)
            brain.screen.print_at(
                "front", self.frontRangeFinder.distance(DistanceUnits.IN), x=40, y=50
            )

            # inout d - ouput c
            brain.screen.print_at(
                "right", self.rightRangeFinder.distance(DistanceUnits.IN), x=40, y=90
            )
            brain.screen.print_at("mult", mult, x=40, y=110)

        self.motorLeft.stop(HOLD)
        self.motorRight.stop(HOLD)

    def driveSideWallGyroTillWall(
        self, frontWallStopDistance, sideWallFollowDistance, direction: DirectionType
    ):
        goalHeading = Gyro.heading
        # kP's
        kPSide = 10
        kPFront = 10
        kPGyro = 10

        kSpeed = 50

        # errors
        errorFront = 999
        errorSide = 0

        # within 1 in of target
        while abs(errorFront) >= 1:
            errorFront = (
                self.frontRangeFinder.distance(DistanceUnits.IN) - frontWallStopDistance
            )
            errorSide = (
                self.rightRangeFinder.distance(DistanceUnits.IN)
                - sideWallFollowDistance
            )
            errorGyro = Gyro.heading - goalHeading

            mult = kSpeed + errorFront * kPFront

            # max speed of 130
            if mult > 130:
                mult = 130
            elif mult < -130:
                mult = -130

            self.motorLeft.spin(
                direction, mult - kPSide * errorSide - kPGyro * errorGyro, RPM
            )
            self.motorRight.spin(
                direction, mult + kPSide * errorSide + kPGyro * errorGyro, RPM
            )
            brain.screen.print_at(
                "front", self.frontRangeFinder.distance(DistanceUnits.IN), x=40, y=50
            )

            # inout d - ouput c
            brain.screen.print_at(
                "right", self.rightRangeFinder.distance(DistanceUnits.IN), x=40, y=90
            )
            brain.screen.print_at("mult", mult, x=40, y=110)

        self.motorLeft.stop(HOLD)
        self.motorRight.stop(HOLD)

    def driveAlongLineUntilEnd(self, direction: DirectionType):
        speed = 150
        leftError = 100 - self.lineLeft.reflectivity(PercentUnits.PERCENT)
        rightError = 100 - self.lineRight.reflectivity(PercentUnits.PERCENT)
        wait(500)
        kp = 0.5
        while (
            self.lineLeft.reflectivity(PercentUnits.PERCENT) > 15
            or self.lineRight.reflectivity(PercentUnits.PERCENT) > 15
        ):
            brain.screen.print_at(
                "left line sensor",
                self.lineLeft.reflectivity(PercentUnits.PERCENT),
                x=40,
                y=90,
            )
            brain.screen.print_at(
                "right line sensor",
                self.lineRight.reflectivity(PercentUnits.PERCENT),
                x=40,
                y=50,
            )
            leftError = 100 - self.lineLeft.reflectivity(PercentUnits.PERCENT)
            rightError = 100 - self.lineRight.reflectivity(PercentUnits.PERCENT)

            self.motorRight.spin(direction, speed - rightError * kp, RPM)
            self.motorLeft.spin(direction, speed - leftError * kp, RPM)
        self.motorLeft.stop(HOLD)
        self.motorRight.stop(HOLD)

    def driveAlongLineUntilWall(self, wallDist, direction: DirectionType):
        speed = 150
        leftError = 100 - self.lineLeft.reflectivity(PercentUnits.PERCENT)
        rightError = 100 - self.lineRight.reflectivity(PercentUnits.PERCENT)
        wait(500)
        kp = 0.5
        while self.frontRangeFinder.distance(DistanceUnits.IN) >= wallDist:
            brain.screen.print_at(
                "left line sensor",
                self.lineLeft.reflectivity(PercentUnits.PERCENT),
                x=40,
                y=90,
            )
            brain.screen.print_at(
                "right line sensor",
                self.lineRight.reflectivity(PercentUnits.PERCENT),
                x=40,
                y=50,
            )
            leftError = 100 - self.lineLeft.reflectivity(PercentUnits.PERCENT)
            rightError = 100 - self.lineRight.reflectivity(PercentUnits.PERCENT)

            self.motorRight.spin(direction, speed - rightError * kp, RPM)
            self.motorLeft.spin(direction, speed - leftError * kp, RPM)
        self.motorLeft.stop(HOLD)
        self.motorRight.stop(HOLD)


# configs
brain = Brain()

leftMotor = Motor(Ports.PORT1, 18_1, False)
rightMotor = Motor(Ports.PORT10, 18_1, True)
armMotor = Motor(Ports.PORT8, 18_1, True)

rangeFinderFront = Sonar(brain.three_wire_port.e)
rangeFinderRight = Sonar(brain.three_wire_port.c)

imu = Inertial(Ports.PORT6)

lineLeft = Line(brain.three_wire_port.b)
lineRight = Line(brain.three_wire_port.a)

bumperButton1 = Bumper(brain.three_wire_port.g)

brain.screen.print("Calibrating")

rangeFinderFront.distance(DistanceUnits.IN)
rangeFinderRight.distance(DistanceUnits.IN)

imu.calibrate()
while imu.is_calibrating():
    wait(5)

# ZERO HEADING FOE GYRO
imu.set_heading(0, DEGREES)

brain.screen.print("Finished Calibrating")

# config drivetrain
rbeDriveTrain = RBEDrivetrain(
    rangeFinderFront,
    rangeFinderRight,
    leftMotor,
    rightMotor,
    imu,
    lineLeft,
    lineRight,
    bumperButton1,
)


def part1():
    # go forward
    rbeDriveTrain.driveSideWallTillWall(5, 5, DirectionType.FORWARD)
    # spin
    rbeDriveTrain.spin(-90)
    # go forwad
    rbeDriveTrain.driveSideWallTillWall(35, 10, DirectionType.FORWARD)
    # spin
    rbeDriveTrain.spin(-90)
    # goforward
    rbeDriveTrain.driveForwardDist(15, 150, True)


def part2():
    # go forward
    rbeDriveTrain.driveSideWallGyroTillWall(2, 5, DirectionType.FORWARD)
    # spin
    rbeDriveTrain.spin(-90)
    # go forwad
    rbeDriveTrain.driveSideWallGyroTillWall(35, 10, DirectionType.FORWARD)
    # spin
    rbeDriveTrain.spin(-90)
    # goforward
    rbeDriveTrain.driveForwardDist(15, 150, True)


def part3():
    # go forward
    rbeDriveTrain.driveAlongLineUntilEnd(DirectionType.FORWARD)
    rbeDriveTrain.driveSideWallTillWall(2, 7, DirectionType.FORWARD)
    # spin
    rbeDriveTrain.spin(-90)
    # go forwad
    rbeDriveTrain.driveAlongLineUntilWall(35, DirectionType.FORWARD)
    # spin
    rbeDriveTrain.spin(-90)
    # goforward
    rbeDriveTrain.driveForwardDist(15, 150, True)


def part4():
    armMotorSlop = 5
    armFinalDrive = 5
    while True:
        brain.screen.print_at("Motor Current", armMotor.current(), x=40, y=50)
        brain.screen.print_at("Motor Position", armMotor.position(), x=40, y=70)
        brain.screen.print_at("Motor Temperature", armMotor.temperature(), x=40, y=90)
        brain.screen.print_at("Motor Torque", armMotor.torque(), x=40, y=110)
        if rbeDriveTrain.bumperButton1.pressing() and (
            armMotor.position() - (45 * armFinalDrive) >= -armMotorSlop
            and armMotor.position() - (45 * armFinalDrive) <= armMotorSlop
        ):
            wait(50)
            armMotor.spin_to_position(0, DEGREES, 50, RPM)
        elif rbeDriveTrain.bumperButton1.pressing() and (
            armMotor.position() >= -armMotorSlop and armMotor.position() <= armMotorSlop
        ):
            wait(50)
            armMotor.spin_to_position(45 * armFinalDrive, DEGREES, 50, RPM)


part3()
