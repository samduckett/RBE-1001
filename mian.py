from vex import *
from math import *


# -------------- Classes that should probably only be made in java but they are here
class Fruit:
    """
    Fruit class that is similar to ai object but also has fruit color and width Height Ratio
    """

    def __init__(self):
        self.originX: float = 0
        self.originY: float = 0
        self.centerX: float = 0
        self.centerY: float = 0
        self.width: float = 0
        self.height: float = 0
        self.score: float = 0
        # one of 3 colors the fruit can be
        self.fruitColor: str = ""
        # the ratio between the width and height of the fruit
        self.widthHeightRatio: float = 0


class Transform2D:
    def __init__(self, dX, dY, dTheta=0):
        self.dX: float = dX
        self.dY: float = dY
        self.dTheta: float = dTheta % 360

    def __repr__(self):
        return (
            "Transform2D(dX="
            + str(self.dX)
            + ", dY="
            + str(self.dY)
            + ", dTheta="
            + (self.dTheta)
        )

    def __eq__(self, other: "Transform2D"):
        return (
            isinstance(other, Transform2D)
            and isclose(self.dX, other.dX)
            and isclose(self.dY, other.dY)
            and isclose(self.dTheta % 360, other.dTheta % 360)
        )

    def __add__(self, other: "Transform2D") -> "Transform2D":
        if not isinstance(other, Transform2D):
            return NotImplemented

        # Rotate other's translation by self's heading
        angle_rad = radians(self.dTheta)
        rotated_dx = cos(angle_rad) * other.dX - sin(angle_rad) * other.dY
        rotated_dy = sin(angle_rad) * other.dX + cos(angle_rad) * other.dY

        # Combine translations and rotations
        new_dx = self.dX + rotated_dx
        new_dy = self.dY + rotated_dy
        new_dtheta = (self.dTheta + other.dTheta) % 360

        return Transform2D(new_dx, new_dy, new_dtheta)

    def __hash__(self):
        return hash((self.dX, self.dY, self.dTheta))

    def inverse(self) -> "Transform2D":
        angle_rad = radians(-self.dTheta)
        new_dx = -cos(angle_rad) * self.dX - sin(angle_rad) * self.dY
        new_dy = sin(angle_rad) * self.dX - cos(angle_rad) * self.dY
        return Transform2D(new_dx, new_dy, -self.dTheta)

    def norm(self) -> float:
        return (self.dX**2 + self.dY**2) ** 0.5


class Pose2D:
    def __init__(self, x, y, heading=0):
        self.x: float = x
        self.y: float = y
        self.heading: float = heading

    def __add__(self, transform2D: Transform2D):  # Apply transform
        rad = radians(self.heading)
        newX = self.x + cos(rad) * transform2D.dX - sin(rad) * transform2D.dY
        newY = self.y + sin(rad) * transform2D.dX + cos(rad) * transform2D.dY
        newHeading = (self.heading + transform2D.dTheta) % 360
        return Pose2D(newX, newY, newHeading)

    def __sub__(self, other: "Pose2D") -> Transform2D:
        dx = self.x - other.x
        dy = self.y - other.y
        dTheta = (self.heading - other.heading) % 360
        rad = radians(-other.heading)
        # Rotate into other's frame
        relDX = cos(rad) * dx - sin(rad) * dy
        relDY = sin(rad) * dx + cos(rad) * dy
        return Transform2D(relDX, relDY, dTheta)

    def __eq__(self, other: "Pose2D"):
        return (
            isinstance(other, Pose2D)
            and isclose(self.x, other.x)
            and isclose(self.y, other.y)
            and isclose(self.heading % 360, other.heading % 360)
        )

    def __hash__(self):
        return hash((self.x, self.y, self.heading))

    def __lt__(self, other):
        if not isinstance(other, Pose2D):
            return NotImplemented
        return (self.x, self.y, self.heading) < (other.x, other.y, other.heading)

    def toString(self):
        return (
            "Pose2D(x="
            + str(self.x)
            + ", y="
            + str(self.y)
            + ", heading="
            + str(self.heading)
            + ")"
        )

    def normalizeAngle(self, degrees) -> None:
        return degrees % 360

    def transformBy(self, transform: Transform2D) -> "Pose2D":
        return self + transform

    def distanceTo(self, other: "Pose2D") -> float:
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5

    def headingTo(self, other: "Pose2D") -> float:
        from math import atan2, degrees

        dx = other.x - self.x
        dy = other.y - self.y
        return degrees(atan2(dy, dx)) % 360


# ----------------------------- control classes
class PID:
    def __init__(
        self,
        brain: Brain,
        Kp: float,
        Ki: float,
        Kd: float,
        setpoint: float = 0.0,
        tolerance: float = 0.0,
        continuous: bool = False,
        minimumInput: float = 0.0,
        maximumInput: float = 360.0,
    ):
        # For logging and timer
        self.brain: Brain = brain

        # PID
        self.Kp: float = Kp
        self.Ki: float = Ki
        self.Kd: float = Kd

        # setpoint and tolerance
        self.setpoint: float = setpoint
        self.tolerance: float = tolerance

        # Continuous
        self.continuous = continuous
        self.minimumInput = minimumInput
        self.maximumInput = maximumInput

        # internal state
        self.lastError: float = 0.0
        self.integral: float = 0.0
        self.lastTime: float = 0

    # to control heading/Continuous control stems
    def warpError(self, error: float) -> float:
        range = self.maximumInput - self.minimumInput
        error = (error + range / 2) % range - range / 2
        return error

    def setSetpoint(self, newSetpoint: float, reset: bool = True) -> None:
        self.setpoint: float = newSetpoint
        if reset:
            self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.lastError = 0.0
        self.lastTime = 0

    def getSetpoint(self) -> float:
        return self.setpoint

    def update(self, measured: float, setpoint: float = None) -> float:
        currentTime = self.brain.timer.system()

        if self.lastTime == 0:
            self.lastTime = currentTime
            return 0.0

        dt = currentTime - self.lastTime
        self.lastTime = currentTime

        if setpoint is not None:
            self.setpoint = setpoint

        error = self.setpoint - measured
        if self.continuous:
            error = self.warpError(error)

        # P term
        P = self.Kp * error

        # I term
        self.integral += error * dt
        I = self.Ki * self.integral

        # D term
        derivative = (error - self.lastError) / dt if dt > 0 else 0.0
        D = self.Kd * derivative

        # store for next cycle for D
        self.lastError = error

        return P + I + D

    def atGoal(self, measured) -> bool:
        error = self.setpoint - measured
        if self.continuous:
            error = self.warpError(error)
        return abs(error) <= self.tolerance


class VisionFruit:
    def __init__(
        self,
        brain: Brain,
        tagMap: dict[int, Pose2D],
    ):
        # For logging
        self.brain: Brain = brain

        # fruit color configs
        self.AIGreenFruit = Colordesc(1, 12, 167, 71, 20, 0.2)
        self.AIOrangeFruit = Colordesc(2, 222, 66, 67, 10, 0.2)
        self.AIYellowFruit = Colordesc(3, 166, 114, 59, 15, 0.2)

        # self.CamGreenFruit = Signature(1, 12, 167, 71, 20, 0.2)  # TODO!!!!!!!!!
        # self.camOrangeFruit = Signature(2, 222, 66, 67, 10, 0.2)  # TODO!!!!!!!!!
        # self.camYellowFruit = Signature(3, 166, 114, 59, 15, 0.2)  # TODO!!!!!!!!!

        # AI Camera const
        self.camWidth = 320
        self.camHeight = 240
        self.camVertFOV = 68
        self.camHorizFOV = 74
        self.degPerPixelWidth = self.camHorizFOV / self.camWidth

        # vision config
        self.aiVision = AiVision(
            Ports.PORT14,
            self.AIGreenFruit,
            self.AIOrangeFruit,
            self.AIYellowFruit,
            AiVision.ALL_TAGS,
        )

        # self.fruitCam = Vision(
        #     Ports.PORT15,
        #     50,
        #     self.CamGreenFruit,
        #     self.camOrangeFruit,
        #     self.camYellowFruit,
        # )

        # arrays of fruit
        self.fruitObjects: dict[str, list[Fruit]] = {}

        self.strategy: list[str] = []
        self.strategyColors: list[str] = []

        self.largeFruitRatio = 1.1  # guess
        self.smallFruitRatio = 0.5  # guess
        self.fruitSizeTolerance = 0.05  # guess

        # ----------- April Tags
        self.tagWidth: float = 5 + 15 / 16  # guess
        self.cameraToRobot = Transform2D(0, 0, 0)  # guess
        self.fruitToRobot = Transform2D(0, 0, 0)  # guess

        self.tagMap = tagMap  # guess

    def setStrategy(self, strategy: list[str]):
        self.strategy = strategy
        self.strategyColors: list[str] = self.colorsFromStrategy(self.strategy)

    def colorsFromStrategy(self, fruitPickingStrategy: list[str]) -> list[str]:
        colors = []
        seen = set()
        for col in fruitPickingStrategy:
            color = col.split("_")[1]
            if color not in seen:
                colors.append(color)
                seen.add(color)
        return colors

    def makeFruitFromVisionObject(self, obj: AiVisionObject, color: str) -> Fruit:
        fruit = Fruit()
        fruit.originX = obj.originX
        fruit.originY = obj.originY
        fruit.centerX = obj.centerX
        fruit.centerY = obj.centerY
        fruit.width = obj.width
        fruit.height = obj.height
        fruit.score = obj.score
        fruit.fruitColor = color
        fruit.widthHeightRatio = fruit.width / fruit.height

        return fruit

    def fruitDist(self, pixelWidth, ObjectWidthIn):
        # OLD FIX!! deferent size fruit, take fruit object
        angularWidth = self.degPerPixelWidth * pixelWidth
        return (ObjectWidthIn * 0.5) / tan(radians(angularWidth * 0.5))

    # def updateFruit(self) -> bool:
    #     """
    #     Updates the vision-based fruit detection. Returns True if any fruit matching
    #     the strategy is detected, otherwise False.
    #     """
    #     self.fruitObjects.clear()

    #     colorSensors = {
    #         "green": self.greenFruit,
    #         "orange": self.orangeFruit,
    #         "yellow": self.yellowFruit,
    #     }

    #     for color, target in zip(self.strategyColors, self.strategy):
    #         color = color.lower()
    #         if color not in colorSensors:
    #             continue

    #         for obj in self.aiVision.take_snapshot(colorSensors[color]):
    #             fruit = self.makeFruitFromVisionObject(obj, color)

    #             # Check for large fruit
    #             if (
    #                 abs(fruit.widthHeightRatio - self.largeFruitRatio)
    #                 < self.fruitSizeTolerance
    #             ):
    #                 self.fruitObjects.setdefault(
    #                     f"Large_{color.capitalize()}", []
    #                 ).append(fruit)

    #             # Check for small fruit
    #             if (
    #                 abs(fruit.widthHeightRatio - self.smallFruitRatio)
    #                 < self.fruitSizeTolerance
    #             ):
    #                 self.fruitObjects.setdefault(
    #                     f"Small_{color.capitalize()}", []
    #                 ).append(fruit)

    #         if target in self.fruitObjects:
    #             return True

    #     return bool(self.fruitObjects)

    def averageAnglesWeighted(self, degreesList: list[float], weights):
        sumSin = sum(w * sin(radians(a)) for a, w in zip(degreesList, weights))
        sumCos = sum(w * cos(radians(a)) for a, w in zip(degreesList, weights))
        return degrees(atan2(sumSin, sumCos)) % 360

    def cameraPlaneToTransform2d(
        self,
        x: float,
        width: float,
        angleDeg: float,
    ) -> Transform2D:
        # Correct the tag width for yaw angle
        yaw_rad = radians(angleDeg)
        corrected_width = width / cos(yaw_rad) if cos(yaw_rad) != 0 else float("inf")

        # Estimate depth using corrected width
        depth = (self.tagWidth * self.degPerPixelWidth) / corrected_width

        # Scale x offset to real-world units
        scale = depth / self.degPerPixelWidth
        realX = x * scale

        return Transform2D(depth, realX, angleDeg)

    def captureTags(self) -> list[dict] | None:
        tags = self.aiVision.take_snapshot(AiVision.ALL_TAGS)

        formattedTags: list[dict] = []
        # brain.screen.print_at(formattedTags, x=40, y=60)
        if tags:
            for tag in tags:
                tempDict = {}

                tempDict["id"] = tag.id
                tempDict["transform"] = self.cameraPlaneToTransform2d(
                    tag.centerX, tag.width, tag.angle
                )
                tempDict["vision_score"] = tag.score
                formattedTags.append(tempDict)
        else:
            return None
        return formattedTags

    def getRobotPoseWithTags(self) -> Pose2D | None:
        tags = self.captureTags()

        if tags == None:
            return None

        # --- Process detections ---
        weightedX = 0.0
        weightedY = 0.0
        weightedHeadings = []
        weights = []

        for tag in tags:
            tagID: int = tag["id"]
            cameraToTag: Transform2D = tag["transform"]
            visionScore: float = tag["vision_score"]

            # does not add tag if score is less than 25% or not in tag map
            if tagID not in self.tagMap or visionScore <= 0.25:
                continue

            tagPose = self.tagMap[tagID]

            tagToCamera = cameraToTag.inverse()
            tagToRobot = tagToCamera + self.cameraToRobot
            robotPose: Pose2D = tagPose + tagToRobot
            # self.brain.screen.print_at(robotPose, x=30, y=50)

            # Distance-based score
            distance = sqrt(cameraToTag.dX**2 + cameraToTag.dY**2)
            if distance != 0:
                distanceScore = 1 / (distance)
            else:
                distanceScore = 0

            # can add weight to make vision score have more of a impact
            finalScore = visionScore * distanceScore

            weightedX += robotPose.x * finalScore
            weightedY += robotPose.y * finalScore
            weightedHeadings.append(robotPose.heading)
            weights.append(finalScore)
        # --- Final weighted average ---
        if weights:
            totalWeight = sum(weights)
            avgX = weightedX / totalWeight
            avgY = weightedY / totalWeight
            avg_heading = self.averageAnglesWeighted(weightedHeadings, weights)

            finalPose = Pose2D(avgX, avgY, avg_heading)
            return finalPose
        else:
            return None

    def getBestFruitLocationFruitCam(self):
        return (Pose2D(0, 0, 0), 0)
        # camera frame of refrance
        # fruit = self.fruitCam.take_snapshot(self.CamGreenFruit)

        # if not (fruit and len(fruit) > 0):
        #     return None

        # # makes it robot frame of refrance
        # return (
        #     Pose2D(fruit[0].centerX, self.fruitDist(fruit[0].centerX), 0),
        #     fruit[0].centerY,
        # )


class Odometry:
    def __init__(self, brain, visionFruit: VisionFruit):
        # For logging
        self.brain: Brain = brain

        self.visionFruit: VisionFruit = visionFruit
        self.pose: Pose2D = Pose2D(0, 0, 0)

    def update(self) -> Pose2D:
        pose = self.visionFruit.getRobotPoseWithTags()
        if pose == None:
            return self.getPosition()
        self.pose = pose
        return self.getPosition()

    def getPosition(self) -> Pose2D:
        return self.pose


class HDrive:
    def __init__(
        self,
        brain: Brain,
        odometry: Odometry,
        imu: Inertial,
        lineLeft: Line,
        lineRight: Line,
    ):
        # For logging
        self.brain: Brain = brain

        self.odometry: Odometry = odometry
        # Robot Const

        self.wheelBase: float = 10
        self.wheelTrack: float = 10

        self.wheelDiameter: float = 4
        self.wheelCircumstance: float = self.wheelDiameter * pi

        self.maxWheelSpeed: float = 175
        # Motors
        self.frontLeftMotor: Motor = Motor(Ports.PORT1, 18_1, False)
        self.frontRightMotor: Motor = Motor(Ports.PORT2, 18_1, True)
        self.backLeftMotor: Motor = Motor(Ports.PORT3, 18_1, False)
        self.backRightMotor: Motor = Motor(Ports.PORT4, 18_1, True)
        self.frontSideMotor: Motor = Motor(Ports.PORT5, 18_1, False)
        self.backSideMotor: Motor = Motor(Ports.PORT6, 18_1, True)

        # Sensors
        self.imu: Inertial = imu

        self.lineLeft: Line = lineLeft
        self.lineRight: Line = lineRight

        # PID'S
        self.headingPID: PID = PID(  # TUNE!!!
            self.brain,
            Kp=5,
            Ki=0,
            Kd=0,
            setpoint=0.0,
            tolerance=1.0,
            continuous=True,
            minimumInput=0.0,
            maximumInput=360.0,
        )

        self.positionPIDX = PID(self.brain, Kp=75, Ki=0.0, Kd=0, tolerance=0.5)
        self.positionPIDY = PID(self.brain, Kp=75, Ki=0.0, Kd=0, tolerance=0.5)

        self.controllerIndex = 0

    def configMotors(self):
        self.frontLeftMotor.reset_position()
        self.frontRightMotor.reset_position()
        self.backLeftMotor.reset_position()
        self.backRightMotor.reset_position()
        self.frontSideMotor.reset_position()
        self.backSideMotor.reset_position()

    def rotateVector(self, x: float, y: float, angle: float) -> tuple[float, float]:
        angleRad = radians(angle)
        cosRad = cos(angleRad)
        sinRad = sin(angleRad)
        return (x * cosRad - y * sinRad, x * sinRad + y * cosRad)

    def joystickToAngleDeg(self, x: float, y: float) -> float:
        rad = atan2(y, x)
        deg = degrees(rad)
        return deg % 360

    def stop(self) -> None:
        self.frontLeftMotor.stop(BRAKE)
        self.frontRightMotor.stop(BRAKE)
        self.backLeftMotor.stop(BRAKE)
        self.backRightMotor.stop(BRAKE)
        self.frontSideMotor.stop(BRAKE)
        self.backSideMotor.stop(BRAKE)

    def dumbVelocityDrive(
        self,
        flSpeed: float,
        frSpeed: float,
        rlSpeed: float,
        rrSpeed: float,
        fsSpeed: float,
        bsSpeed: float,
    ) -> None:
        self.frontLeftMotor.spin(FORWARD, flSpeed, RPM)
        self.frontRightMotor.spin(FORWARD, frSpeed, RPM)
        self.backLeftMotor.spin(FORWARD, rlSpeed, RPM)
        self.backRightMotor.spin(FORWARD, rrSpeed, RPM)

        self.frontSideMotor.spin(FORWARD, fsSpeed, RPM)
        self.backSideMotor.spin(FORWARD, bsSpeed, RPM)

    def arcadeDrive(
        self,
        forward: float,
        strafe: float,
        rotation: float,
    ) -> None:
        # Calculate individual wheel speeds
        fl = forward + rotation
        fr = forward - rotation
        rl = forward + rotation
        rr = forward - rotation
        fs = strafe
        bs = strafe

        # Pack into list for normalization
        speeds = [fl, fr, rl, rr, fs, bs]
        maxMagnitude = max(abs(s) for s in speeds)

        # Normalize if needed
        if maxMagnitude > self.maxWheelSpeed:
            speeds = [s / maxMagnitude * self.maxWheelSpeed for s in speeds]

        # Unpack back
        fl, fr, rl, rr, fs, bs = speeds

        # Send to motor control
        self.dumbVelocityDrive(fl, fr, rl, rr, fs, bs)

    def fieldCentricDrive(
        self,
        forward: float,
        strafe: float,
        rotation: float,
    ) -> None:
        # Boost strafe if needed
        strafeBoost = 2.0
        strafe *= strafeBoost

        # Field-oriented translation
        strafeFC, forwardFC = self.rotateVector(strafe, forward, self.imu.heading())

        self.arcadeDrive(forwardFC, strafeFC, rotation)

    def fieldCentricTargetAngleDrive(
        self,
        forward: float,
        strafe: float,
        targetAngle: float,
    ):
        # PID computes rotation power to reach target angle
        rotation = self.headingPID.update(measured=imu.heading(), setpoint=targetAngle)

        # brain.screen.print_at(imu.heading(), x=100, y=40)
        self.fieldCentricDrive(forward, strafe, rotation)

    def driveController(self, controller: Controller):
        driveModes = [
            "arcadeDrive",
            "fieldCentricDrive",
            "fieldCentric0AngleDrive",
            "fieldCentricTargetAngleDrive",
        ]

        # if controller.buttonA.pressed():
        #     self.controllerIndex = (self.controllerIndex + 1) % len(driveModes)
        #     controller.screen.clear_screen()
        #     controller.screen.print(driveModes[self.controllerIndex])

        current_mode = driveModes[0]

        if current_mode == "fieldCentricDrive":
            self.fieldCentricDrive(
                controller.axis3.position(),
                controller.axis4.position(),
                controller.axis1.position(),
            )
        elif current_mode == "fieldCentric0AngleDrive":
            self.fieldCentricTargetAngleDrive(
                controller.axis3.position(),
                controller.axis4.position(),
                0,
            )
        elif current_mode == "fieldCentricTargetAngleDrive":
            self.fieldCentricTargetAngleDrive(
                controller.axis3.position(),
                controller.axis4.position(),
                self.joystickToAngleDeg(
                    controller.axis1.position(), controller.axis1.position()
                ),
            )
        else:
            self.arcadeDrive(
                controller.axis3.position(),
                controller.axis1.position(),
                controller.axis4.position(),
            )

    # center of robot
    def driveToPosition(self, target: Pose2D, speed: float = 50.0):
        pose: Pose2D = self.odometry.getPosition()

        forward = self.positionPIDY.update(pose.x, target.x)
        strafe = self.positionPIDX.update(pose.y, target.y)

        self.fieldCentricTargetAngleDrive(forward, strafe, target.heading)

        return self.positionPIDY.atGoal(pose.y) and self.positionPIDY.atGoal(pose.x)

    def followPath(self, path: list[Pose2D], speed: float = 50.0):
        """Follow a path from start to goal."""
        for target in path:
            while not self.driveToPosition(target, speed):
                pass
            print("Arrived at " + str(target.x) + "," + str(target.y))

        print("Path following complete!")

    def pathFindToPosition(self, target: Pose2D, speed: float = 50.0):
        # path = self.pathFinding.a_star_pose2d(odometry.getPosition(), target)
        path = None
        print(path)
        self.followPath(path)


class Arm:
    def __init__(self, brain: Brain):
        # For logging
        self.brain: Brain = brain
        self.driveRatio = 4
        self.armMotor: Motor = Motor(Ports.PORT8, 18_1, True)

        self.PIDArm = PID(self.brain, Kp=75, Ki=0.0, Kd=0, tolerance=0.5)

    def pidControl(self, controller: Controller):
        # horizontal 0
        # vertical -680
        # other horizontal -1250
        if controller.buttonA:
            self.PIDArm.setSetpoint(-680)
        if controller.buttonB:
            self.PIDArm.setSetpoint(-340)
        if controller.buttonX:
            self.PIDArm.setSetpoint(0)

        self.armMotor.spin(
            FORWARD, self.PIDArm.update(self.armMotor.position(DEGREES)), RPM
        )


class Elevator:
    def __init__(self, brain: Brain):
        # For logging
        self.brain: Brain = brain
        self.driveRatio = 5
        self.ElevatorMotor: Motor = Motor(Ports.PORT8, 18_1, True)

        self.PIDElevator = PID(self.brain, Kp=75, Ki=0.0, Kd=0, tolerance=0.5)
        # NEGATIVE IS DOWN POSITIVE IS UP

        self.normalTorque = 10
        self.stallTorque = 15

        # elevator setpoint's

    def zero(self) -> None:
        while (self.ElevatorMotor.torque(TorqueUnits.NM) - self.normalTorque) < (
            self.stallTorque - self.normalTorque
        ) / 2:
            self.ElevatorMotor.spin(FORWARD, -100, RPM)
        self.ElevatorMotor.stop(COAST)
        self.ElevatorMotor.reset_position()

    def driveMotorEncoderPID(self, rotations) -> None:
        self.ElevatorMotor.spin_to_position(rotations, DEGREES, 200, RPM, False)

    def driveFruitPicking(self, dZ: float) -> None:
        self.spinMotor(self.PIDElevator.update(dZ))
        return self.PIDElevator.atGoal(dZ)

    def pick(self) -> None:
        self.ElevatorMotor.spin_for(
            FORWARD,
            -2,
            RotationUnits.REV,
            200,
            RPM,
            False,
        )

    def home(self) -> None:
        self.driveFruitPicking(72000)

    def bottom(self) -> None:
        self.driveFruitPicking(0)

    def spinMotor(self, spin) -> None:
        self.ElevatorMotor.spin(FORWARD, spin, RPM)


# --------------- configs and stuff
fruitPickingStrategy = [
    "Large_Green",
    "Large_Yellow",
    "Small_Green",
    "Large_Orange",
    "Small_Yellow",
    "Small_Orange",
]

# CENTER OF TAG!!!
# inches
tagMap: dict[int, Pose2D] = {
    1: Pose2D(0, 0, 0),
}


# ------------------------- Initial Robot and stuff
brain = Brain()

imu = Inertial(Ports.PORT7)

lineLeft = Line(brain.three_wire_port.a)
lineRight = Line(brain.three_wire_port.b)

controller = Controller(PRIMARY)

# Classes
vision = VisionFruit(brain, tagMap)
odometry = Odometry(brain, vision)

hDrive = HDrive(brain, odometry, imu, lineLeft, lineRight)

# arm = Arm(brain)

elevator = Elevator(brain)
# ------------------------  calibrate shit here and zero

brain.screen.print("Calibrating")

vision.setStrategy(fruitPickingStrategy)

hDrive.configMotors()

imu.calibrate()

while imu.is_calibrating():
    wait(5)

brain.timer.clear()

imu.set_heading(0, DEGREES)
imu.set_rotation(0, DEGREES)

brain.screen.print("Finished Calibrating")


def pickFruit() -> None:
    detection = vision.getBestFruitLocationFruitCam()

    if detection == None:
        return None

    pose, z = detection
    # y forward backwards
    # x left right
    # z up down

    newPose = odometry.getPosition() + pose

    return hDrive.driveToPosition(newPose) and elevator.driveFruitPicking(z)


# ---------------------------   RUN CODE HERE
# elevator.zero()

while True:
    odometry.update()
    brain.screen.print_at("Location: " + odometry.getPosition().toString(), x=30, y=40)
    wait(50)
    # pickFruit()

    # brain.screen.print_at(odometry.getPosition(), x=40, y=40)
    # hDrive.driveController(controller)
    # arm.pidControl(controller)
