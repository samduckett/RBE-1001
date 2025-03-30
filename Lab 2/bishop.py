# # # ---------------------------------------------------------------------------- #
# # #                                                                              #
# # # 	Module:       main.py                                                      #
# # # 	Author:       bruta                                                        #
# # # 	Created:      3/26/2025, 11:30:02 AM                                       #
# # # 	Description:  V5 project                                                   #
# # #                                                                              #
# # # ---------------------------------------------------------------------------- #

# # Library imports
# from vex import *


# # Brain should be defined by default
# brain=Brain()
# leftMotor = Motor(Ports.PORT1, 18_1, False)
# rightMotor = Motor(Ports.PORT10, 18_1, True)


# rangeFinderFront = Sonar(brain.three_wire_port.e)
# rangeFinderRight = Sonar(brain.three_wire_port.c)


# while(True):
#     brain.screen.print("Hello V5")
#     brain.screen.print_at("FRONT SENSOR", rangeFinderFront.distance(DistanceUnits.IN), x=40, y=40)
#     brain.screen.print_at("RIGHT SENSOR", rangeFinderRight.distance(DistanceUnits.IN), x=40, y=90)
#     wait(500)


from vex import *


class arm:
    pass


class RBEDrivetrain:
    """drivetrain class for RBE 1001, will build on the entire class"""

    def __init__(
        self,
        frontRangeFinder: Sonar,
        rightRangeFinder: Sonar,
        motorLeft: Motor,
        motorRight: Motor,
        finalDrive: float,
        wheelDiameter: float,
        wheelBase: float,
        wheelTrack: float,
    ):
        # 51
        # 10 3/4
        self.frontRangeFinder = frontRangeFinder
        self.rightRangeFinder = rightRangeFinder
        self.motorLeft = motorLeft
        self.motorRight = motorRight
        self.finalDrive = finalDrive
        self.wheelDiameter = wheelDiameter
        self.wheelBase = wheelBase
        self.wheelTrack = wheelTrack
        self.wheelCircumference = self.wheelDiameter * math.pi
        self.rotationsPerInch = 1 / self.wheelCircumference
	"""drivetrain class for RBE 1001, will build on the entire class"""
	def __init__(
		self,
		frontRangeFinder: Sonar,
		rightRangeFinder: Sonar,
		motorLeft: Motor,
		motorRight: Motor,
		finalDrive: float,
		wheelDiameter: float,
		wheelBase: float,
		wheelTrack: float,
	):
		# 51
		# 10 3/4
		self.frontRangeFinder = frontRangeFinder
		self.rightRangeFinder = rightRangeFinder
		self.motorLeft = motorLeft
		self.motorRight = motorRight
		self.finalDrive = finalDrive
		self.wheelDiameter = wheelDiameter
		self.wheelBase = wheelBase
		self.wheelTrack = wheelTrack
		self.wheelCircumference = self.wheelDiameter * math.pi
		self.rotationsPerInch = 1 / self.wheelCircumference
		self.wheelRotDegPerBodyTurnDeg = ((wheelTrack * 2) / wheelDiameter)

    def driveLeftMotor(self, speed):
        """drive function - For negative values of direction, the robot turns right, and for positive values of direction, the robot turns left.  For values of direction with small magnitudes, the robot gradually turns.  For values of direction with large magnitudes, the robot turns more quickly."""
        self.motorLeft.set_velocity(speed, RPM)
        self.motorLeft.spin(FORWARD)

    def driveRightMotor(self, speed):
        self.motorRight.set_velocity(speed, RPM)
        self.motorRight.spin(FORWARD)

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

    # def wallFollowInches(self, setDistanceFromWall):
    #     while frontRangeFinder.distance(DistanceUnits.IN) != setDistanceFromWall:
    #         rightError = setDistanceFromWall - frontRangeFinder.distance(DistanceUnits.IN)
    #         drive(-K_P * rightError)
	
	def spinAboutWheel(self, speed, deg, wheel, pause){
		if wheel == "LEFT":
			self.motorLeft.spin_for(FORWARD, deg * self.wheelRotDegPerBodyTurnDeg * (deg / abs(deg)), DEGREES, speed, RPM, pause)
		else if wheel == "RIGHT":
			self.motorRight.spin_for(FORWARD, deg * self.wheelRotDegPerBodyTurnDeg * (deg / abs(deg)), DEGREES, speed, RPM, pause)
	}
	def brazeWallUntilDistance(self, rightFollowDist, forwardDistWall, speed, kp):
		while self.frontRangeFinder.distance(DistanceUnits.IN) >= forwardDistWall:
			rightError = rightFollowDist - self.rightRangeFinder.distance(DistanceUnits.IN)
			brain.screen.print_at("FRONT SENSOR", rangeFinderFront.distance(DistanceUnits.IN), x=40, y=40)
			brain.screen.print_at("RIGHT SENSOR", rangeFinderRight.distance(DistanceUnits.IN), x=40, y=90)
			self.driveRightMotor(speed)
			self.driveLeftMotor(kp * rightError + speed)
		self.motorLeft.stop(HOLD)
		self.motorRight.stop(HOLD)
		
	# def wallFollowInches(self, setDistanceFromWall):
	#     while frontRangeFinder.distance(DistanceUnits.IN) != setDistanceFromWall:
	#         rightError = setDistanceFromWall - frontRangeFinder.distance(DistanceUnits.IN)
	#         drive(-K_P * rightError)

    def driveDistance(self, Inches: float):
        """Function to drive BaseBot straight for some number of inches"""
        Velocity = 100
        self.motorLeft.spin_for(
            FORWARD,
            Inches * self.finalDrive * self.rotationsPerInch,
            RotationUnits.REV,
            Velocity,
            RPM,
            False,
        )
        self.motorRight.spin_for(
            FORWARD,
            Inches * self.finalDrive * self.rotationsPerInch,
            RotationUnits.REV,
            Velocity,
            RPM,
            True,
        )
	# def driveDistance(self, Inches: float):
	# 	"""Function to drive BaseBot straight for some number of inches"""
	# 	Velocity = 100
	# 	self.motorLeft.spin_for(
	# 		FORWARD,
	# 		Inches * self.finalDrive * self.rotationsPerInch,
	# 		RotationUnits.REV,
	# 		Velocity,
	# 		RPM,
	# 		False,
	# 	)
	# 	self.motorRight.spin_for(
	# 		FORWARD,
	# 		Inches * self.finalDrive * self.rotationsPerInch,
	# 		RotationUnits.REV,
	# 		Velocity,
	# 		RPM,
	# 		True,
	# 	)

    def turnInPlace(self, Rotations: float):
        """Function to turn BaseBot for some number of Rotations"""
        Velocity = 100
        self.motorLeft.spin_for(
            REVERSE,
            Rotations * self.finalDrive * self.wheelTrack / self.wheelDiameter,
            RotationUnits.REV,
            Velocity,
            RPM,
            False,
        )
        self.motorRight.spin_for(
            FORWARD,
            Rotations * self.finalDrive * self.wheelTrack / self.wheelDiameter,
            RotationUnits.REV,
            Velocity,
            RPM,
            True,
        )
	# def turnInPlace(self, Rotations: float):
	# 	"""Function to turn BaseBot for some number of Rotations"""
	# 	Velocity = 100
	# 	self.motorLeft.spin_for(
	# 		REVERSE,
	# 		Rotations * self.finalDrive * self.wheelTrack / self.wheelDiameter,
	# 		RotationUnits.REV,
	# 		Velocity,
	# 		RPM,
	# 		False,
	# 	)
	# 	self.motorRight.spin_for(
	# 		FORWARD,
	# 		Rotations * self.finalDrive * self.wheelTrack / self.wheelDiameter,
	# 		RotationUnits.REV,
	# 		Velocity,
	# 		RPM,
	# 		True,
	# 	)

    def turnAroundWheel(self, Rotations: float):
        Velocity = 100
        if Rotations > 0:
            self.motorLeft.spin_for(
                FORWARD,
                Rotations * self.finalDrive * self.wheelTrack / self.wheelDiameter * 2,
                RotationUnits.REV,
                Velocity,
                RPM,
                True,
            )
        else:
            self.motorRight.spin_for(
                REVERSE,
                Rotations * finalDrive * self.wheelTrack / self.wheelDiameter * 2,
                RotationUnits.REV,
                Velocity,
                RPM,
                True,
            )


# Functions
brain = Brain()
	# def turnAroundWheel(self, Rotations: float):
	# 	Velocity = 100
	# 	if Rotations > 0:
	# 		self.motorLeft.spin_for(
	# 			FORWARD,
	# 			Rotations * self.finalDrive * self.wheelTrack / self.wheelDiameter * 2,
	# 			RotationUnits.REV,
	# 			Velocity,
	# 			RPM,
	# 			True,
	# 		)
	# 	else:
	# 		self.motorRight.spin_for(
	# 			REVERSE,
	# 			Rotations * finalDrive * self.wheelTrack / self.wheelDiameter * 2,
	# 			RotationUnits.REV,
	# 			Velocity,
	# 			RPM,
	# 			True,
	# 		)
kp = float(12)
brain=Brain()

leftMotor = Motor(Ports.PORT1, 18_1, False)
rightMotor = Motor(Ports.PORT10, 18_1, True)
rangeFinderFront = Sonar(brain.three_wire_port.e)
rangeFinderRight = Sonar(brain.three_wire_port.c)

# conts
kp = float(12)
finalDrive = 5
wheelDiameter = 4
wheelBase = 8
wheelTrack = 10.25

rightFollowDistance = float(4.3)
forwardWallDistance = float(5)
forwardWallDistance2 = float(51)


rbeDriveTrain = RBEDrivetrain(
    rangeFinderFront,
    rangeFinderRight,
    leftMotor,
    rightMotor,
    finalDrive,
    wheelDiameter,
    wheelBase,
    wheelTrack,
)

brain.screen.print("running /n")

rangeFinderFront.distance(DistanceUnits.IN)
rangeFinderRight.distance(DistanceUnits.IN)

wait(500)

# line follow test
rbeDriveTrain.brazeWallUntilDistance(rightFollowDistance, forwardWallDistance, 100, kp)
brain.screen.print("stopping /n")
