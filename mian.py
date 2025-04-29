from vex import *
from math import *
from random import *


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
        self.depth = 0
        self.area = 0


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

        # Clamp
        self.min_output = None
        self.max_output = None

    def clamp(self, value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))

    def addClamp(self, min_value: float, max_value: float) -> None:
        self.min_output = min_value
        self.max_output = max_value

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

        # calculate output
        output = P + I + D

        # Clamp output if limits are set
        if self.min_output is not None and self.max_output is not None:
            output = self.clamp(output, self.min_output, self.max_output)

        return output

    def atGoal(self, measured) -> bool:
        error = self.setpoint - measured
        if self.continuous:
            error = self.warpError(error)
        return abs(error) <= self.tolerance


class VisionFruit:
    def __init__(
        self,
        brain: Brain,
    ):
        # For logging
        self.brain: Brain = brain

        # fruit color configs
        self.AIGreenFruit = Colordesc(1, 12, 167, 71, 20, 0.2)
        self.AIOrangeFruit = Colordesc(2, 222, 66, 67, 10, 0.2)
        self.AIYellowFruit = Colordesc(3, 166, 114, 59, 15, 0.2)

        # AI Camera const
        self.camWidth = 320
        self.camHeight = 240
        self.camVertFOV = 68
        self.camHorizFOV = 74
        self.degPerPixelWidth = self.camHorizFOV / self.camWidth
        self.degPerPixelHeight = self.camVertFOV / self.camHeight

        # vision config
        self.aiVision = AiVision(
            Ports.PORT10,
            self.AIGreenFruit,
            self.AIOrangeFruit,
            self.AIYellowFruit,
        )

        # arrays of fruit
        self.fruitHeight = 3
        self.smallFruitWidth = 2.25
        self.largeFruitWidth = 3

        self.smallFruitRatio = self.fruitHeight / self.smallFruitWidth
        self.largeFruitRatio = self.fruitHeight / self.largeFruitWidth

        self.fruitSizeTolerance = 0.05

        self.objects: list[Fruit] = []

        self.tags: list[AiVisionObject] = []
        self.aiVision.tag_detection(True)

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
        fruit.depth = self.fruitDist(fruit)
        fruit.area = obj.area
        return fruit

    def fruitDist(self, fruit: Fruit):
        self.dist(self.fruitHeight, fruit.height)

    def tagDist(self, obj: AiVisionObject):
        self.dist(self.fruitHeight, obj.height)

    def dist(self, height, objHeight):
        distY = (objHeight * 0.5) / tan(radians(self.degPerPixelHeight * height * 0.5))
        return distY

    # changes origan to center of camera
    def centerScreen(self, centerX, centerY) -> tuple[float, float]:
        return (centerX - self.camWidth / 2, centerY - self.camHeight / 2)

    def update(self, colors: list[str]):
        self.objects: list[Fruit] = []
        for color in colors:
            if color == "green":
                for obj in self.aiVision.take_snapshot(self.AIGreenFruit):
                    self.objects.append(self.makeFruitFromVisionObject(obj, "green"))
            if color == "orange":
                for obj in self.aiVision.take_snapshot(self.AIOrangeFruit):
                    self.objects.append(self.makeFruitFromVisionObject(obj, "orange"))
            if color == "yellow":
                for obj in self.aiVision.take_snapshot(self.AIYellowFruit):
                    self.objects.append(self.makeFruitFromVisionObject(obj, "yellow"))

        self.tags = self.aiVision.take_snapshot(self.aiVision.ALL_TAGS)


class HDrive:
    def __init__(
        self,
        brain: Brain,
        imu: Inertial,
        vision: VisionFruit,
    ):
        # For logging
        self.brain: Brain = brain

        self.vision: VisionFruit = vision

        # Robot Const
        self.wheelBase: float = 10
        self.wheelTrack: float = 10

        self.wheelDiameter: float = 4
        self.wheelCircumstance: float = self.wheelDiameter * pi

        self.maxWheelSpeed: float = 175
        # Motors
        self.frontLeftMotor: Motor = Motor(Ports.PORT4, 18_1, False)
        self.frontRightMotor: Motor = Motor(Ports.PORT3, 18_1, True)
        self.backLeftMotor: Motor = Motor(Ports.PORT2, 18_1, False)
        self.backRightMotor: Motor = Motor(Ports.PORT1, 18_1, True)
        self.frontSideMotor: Motor = Motor(Ports.PORT6, 18_1, False)
        self.backSideMotor: Motor = Motor(Ports.PORT5, 18_1, True)

        # Sensors
        self.imu: Inertial = imu

        self.lineFront = Line(self.brain.three_wire_port.d)
        self.lineBack = Line(self.brain.three_wire_port.a)

        # PID'S
        self.headingPID: PID = PID(
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

        self.friendXPID: PID = PID(
            self.brain, Kp=2, Ki=0, Kd=0, setpoint=0, tolerance=0.5
        )
        self.friendXPID.addClamp(-30, 30)
        self.friendYPID: PID = PID(
            self.brain, Kp=3, Ki=0, Kd=0, setpoint=0, tolerance=0.5
        )
        self.friendYPID.addClamp(-50, 50)

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

    def driveController(self, controller: Controller, index):
        driveModes = {
            "arcadeDrive": 1,
            "fieldCentricDrive": 2,
            "fieldCentric0AngleDrive": 3,
            "fieldCentricTargetAngleDrive": 4,
        }

        current_mode = list(driveModes.keys())[index]

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

    def onLine(self) -> bool:
        return self.frontLineOnLine() or self.backLineOnLine()

    def onCross(self) -> bool:
        return self.frontLineOnLine() and self.backLineOnLine()

    def frontLineReflectivity(self):
        return self.lineFront.value(PERCENT)

    def backLineReflectivity(self):
        return self.lineBack.value(PERCENT)

    def frontLineOnLine(self):
        return self.frontLineReflectivity() < 65

    def backLineOnLine(self):
        return self.backLineReflectivity() < 65

    def strafeLine(self, strafeSpeed, angle: float = 0):
        straddleLineReflectivity: float = 50
        Kp: float = 0.5

        # off line positive on line zero
        frontError = self.frontLineReflectivity() - straddleLineReflectivity
        backError = self.backLineReflectivity() - straddleLineReflectivity

        # brain.screen.print_at(frontError, x=100, y=200)
        # brain.screen.print_at(backError, x=100, y=220)

        # brain.screen.print_at(self.frontLineReflectivity(), x=40, y=200)
        # brain.screen.print_at(self.backLineReflectivity(), x=40, y=220)
        # front pos forward
        # back pos backwards

        # drive motors
        self.fieldCentricTargetAngleDrive(
            -frontError * Kp + backError * Kp, strafeSpeed, angle
        )

    def friendToFruit(self, fruit: Fruit):

        fruitDistance = self.vision.fruitDist(fruit)
        x, y = self.vision.centerScreen(fruit.centerX, fruit.centerY)

        brain.screen.print_at(x, x=40, y=80)
        brain.screen.print_at(fruitDistance, x=40, y=100)

        self.fieldCentricTargetAngleDrive(
            -self.friendYPID.update(fruitDistance),
            -self.friendXPID.update(x),
            45,
        )

    def friendToFruitSlow(self, fruit: Fruit):

        fruitDistance = self.vision.fruitDist(fruit)
        x, y = self.vision.centerScreen(fruit.centerX, fruit.centerY)

        speedFW = fruitDistance * 5
        speedRot = x * 0.5
        # max speed
        maxSpeed = 170
        if speedFW > maxSpeed:
            speedFW = maxSpeed
        elif speedFW < -maxSpeed:
            speedFW = -maxSpeed

        maxSpeedRot = 170
        if speedRot > maxSpeedRot:
            speedRot = maxSpeedRot
        elif speedRot < -maxSpeedRot:
            speedRot = -maxSpeedRot

        self.arcadeDrive(speedFW, 0, speedRot)


class Arm:
    def __init__(self, brain: Brain, vision: VisionFruit):
        # For logging
        self.brain: Brain = brain
        self.vision: VisionFruit = vision

        self.driveRatio: float = (12 / 84) * (36 / 60)

        self.armMotor: Motor = Motor(Ports.PORT8, 18_1, True)
        self.armMotor.set_stopping(HOLD)

        self.LineBucket: Line = Line(brain.three_wire_port.b)

        self.bucketServo: Servo = Servo(self.brain.three_wire_port.c)
        self.bucketServoTwo: Servo = Servo(self.brain.three_wire_port.e)

        self.PIDArm = PID(self.brain, Kp=-1.5, Ki=0.0, Kd=0, tolerance=0.25, setpoint=0)
        self.PIDArm.addClamp(-175, 175)

    def hasFruit(self) -> bool:
        return self.LineBucket.value(PERCENT) < 69.0

    def spinToPose(self, pose, stop=False) -> None:
        # makes sure motor does not stall
        if pose == 0:
            self.armMotor.set_stopping(COAST)
        elif pose < 0:
            brain.screen.print("ERROR")
            pose = 0
        else:
            self.armMotor.set_stopping(HOLD)

        self.armMotor.spin_to_position(pose, DEGREES, 200, RPM, stop)

    def zero(self) -> None:
        if self.armMotor.temperature(PERCENT) < 47:
            # THE motor is not being limited
            while self.armMotor.torque(TorqueUnits.NM) < 0.8:
                self.brain.screen.print_at(
                    self.armMotor.torque(TorqueUnits.NM), x=40, y=40
                )
                self.armMotor.spin(FORWARD, -200, RPM)
        else:
            # the motor is being limited
            self.armMotor.spin(FORWARD, -200, RPM)
            wait(2000)  # drive backwards for 5 seconds and then zeros
        self.armMotor.stop(COAST)
        self.armMotor.reset_position()

    def toHome(self, pause: bool = False):
        self.spinToPose(0, pause)

    def atHome(self):
        return abs(self.armMotor.position(DEGREES)) < 20

    def toFruitPickingHeight(self, pause: bool = False):
        self.spinToPose(2000, pause)

    def atFruitPickingHeight(self):
        return abs(arm.armMotor.position(DEGREES) - 2000) < 15  # TUNE

    def toVisionFruitHeight(self, fruit: Fruit | None) -> bool:
        if fruit is None:
            self.stop()
        else:
            x, y = vision.centerScreen(fruit.centerX, fruit.centerY)
            if y == 0:
                self.stop()
            else:
                if (
                    self.armMotor.torque(TorqueUnits.NM) > 1.0
                    or self.armMotor.position(DEGREES) < 1000
                ):  # TODO arm at vertical position
                    self.stop
                else:
                    self.armMotor.spin(FORWARD, self.PIDArm.update(y), RPM)
                return self.PIDArm.atGoal(y)
        return False

    def closeBucket(self):
        self.bucketServo.set_position(150, PERCENT)
        self.bucketServoTwo.set_position(-150, PERCENT)

    def openBucket(self):
        self.bucketServo.set_position(-100, PERCENT)
        self.bucketServoTwo.set_position(100, PERCENT)

    def stop(self):
        self.armMotor.stop()


# ------------------- State Machine
class State:
    def __init__(self, timeout, fallBack, color):
        self.timeout = timeout * 1000
        self.fallback = fallBack
        self.color = color

    def getTimeout(self):
        return self.timeout

    def getFallback(self):
        return self.fallback

    def getColor(self):
        return self.color


STATE: dict[str, State] = {
    "FIND_LINE": State(10.0, "ERROR", Color.WHITE),
    "LINE_FOLLOW_TO_TREE": State(20, "FIND_LINE", Color.RED),
    "GO_TO_FRUIT": State(10.0, "FIND_LINE", Color.YELLOW),
    "GRAB_FRUIT": State(5.0, "FIND_LINE", Color.GREEN),
    "BACK_TO_Line": State(5.0, "FIND_LINE", Color.ORANGE),
    "LINE_FOLLOW_TO_BIN": State(15.0, "FIND_LINE", Color.CYAN),
    "TRACK_BIN": State(10.0, "FIND_LINE", Color.BLUE),
    "RAM_BUCKET": State(5.0, "FIND_LINE", Color.PURPLE),
    "ERROR": State(0, "FIND_LINE", Color.BLACK),
}


class StateMachine:
    def __init__(
        self,
        brain: Brain,
        drive: HDrive,
        arm: Arm,
        vision: VisionFruit,
    ):
        self.state: State = STATE["FIND_LINE"]
        self.brain: Brain = brain

        self.drive: HDrive = drive
        self.arm: Arm = arm
        self.vision: VisionFruit = vision

        self.stateStartTime = self.brain.timer.system()

        self.fruitColors = ["green", "yellow", "orange"]
        #              Tag ID   0        1        2

        self.wantedFruitColorIndex = 0

    def update(self):
        self.vision.update([self.fruitColors[self.wantedFruitColorIndex]])

        elapsed = self.brain.timer.system() - self.stateStartTime
        if self.state.getTimeout() is not None:
            if elapsed > self.state.getTimeout():
                fallback: State = self.state.getFallback()
                self.transition(fallback)
                return

        # Dispatch logic
        runner = {
            STATE["FIND_LINE"]: self.runFindLine,
            STATE["LINE_FOLLOW_TO_TREE"]: self.runLineFollowToTree,
            STATE["GO_TO_FRUIT"]: self.runGoToFruit,
            STATE["GRAB_FRUIT"]: self.runGrabFruit,
            STATE["BACK_TO_Line"]: self.runBackToLine,
            STATE["LINE_FOLLOW_TO_BIN"]: self.runLineFollowToBin,
            STATE["TRACK_BIN"]: self.runTrackBin,
            STATE["RAM_BUCKET"]: self.runRamBucket,
            STATE["ERROR"]: self.runError,
        }.get(self.state)

        if runner:
            runner()

    def transition(self, newState: State):
        self.brain.screen.draw_rectangle(0, 120, 480, 120, newState.getColor())
        self.state = newState
        self.stateStartTime = self.brain.timer.system()
        # adding pause between states
        wait(500)

    def stateCheck(self, checks: dict[str, bool]) -> bool:
        if not checks:
            return False

        width = floor(480 / len(checks))
        done = True

        for index, (name, check) in enumerate(checks.items()):
            color = Color.GREEN if check else Color.RED
            self.brain.screen.draw_rectangle(width * index, 0, width, 120, color)
            self.brain.screen.print_at(name, x=width * index, y=40)
            if check and done:
                done = True
            else:
                done = False

        return done

    def runFindLine(self):
        # arm out in front of robot
        self.arm.toFruitPickingHeight()
        # drives backward
        self.drive.fieldCentricTargetAngleDrive(-30, 0, 45)

        checks = {
            "on Line": self.drive.onLine(),
            "arm at hight": self.arm.atFruitPickingHeight(),
        }
        if self.stateCheck(checks):
            self.drive.stop()
            self.arm.stop()
            self.transition(STATE["LINE_FOLLOW_TO_TREE"])

    def runLineFollowToTree(self):
        # line follows the line at a 45 so the arm can grab all of the fruit
        self.drive.strafeLine(20, 45)

        self.arm.openBucket()

        self.arm.PIDArm.setpoint = 0

        objects = self.vision.objects
        if objects:
            x, y = self.vision.centerScreen(objects[0].centerX, objects[0].centerY)
            self.arm.toVisionFruitHeight(objects[0])

            # if close in the x derection and close in the depth derectin go to it
            checks = {
                "close to fruit x": abs(x) < 30,
                "close to fruit depth": objects[0].depth < 20,  # TODO check
            }
            if self.stateCheck(checks):
                self.transition(STATE["GO_TO_FRUIT"])

    def runGoToFruit(self):
        self.arm.openBucket()

        objects = self.vision.objects
        if objects:
            # gets close to fruit but does not go into gripper
            if objects[0].depth > 6:
                self.drive.friendToFruit(objects[0])
            else:
                self.drive.stop()

            # changes arm setpoint when close
            if objects[0].depth < 9:
                self.arm.PIDArm.setpoint = 65

            checks = {
                "close to fruit depth": objects[0].depth < 7,
                "arm at height": self.arm.toVisionFruitHeight(objects[0]),
            }
            if self.stateCheck(checks):
                self.transition(STATE["GRAB_FRUIT"])
        else:
            # if lose fruit go back to line
            self.transition(STATE["FIND_LINE"])

    def runGrabFruit(self):
        objects = self.vision.objects
        if objects:
            if self.arm.toVisionFruitHeight(objects[0]):
                self.drive.friendToFruitSlow(objects[0])

        checks = {
            "has fruit": self.arm.hasFruit(),
        }
        if self.stateCheck(checks):
            self.arm.closeBucket()
            self.drive.stop()
            self.arm.stop()
            self.transition(STATE["BACK_TO_Line"])

    def runBackToLine(self):
        # drives back to line
        self.drive.fieldCentricTargetAngleDrive(-50, 0, 45)

        checks = {
            "on line": self.drive.onLine(),
        }
        if self.stateCheck(checks):
            self.transition(STATE["LINE_FOLLOW_TO_BIN"])

    def runLineFollowToBin(self):
        self.arm.toHome()
        self.drive.strafeLine(-50, -90)

        tags = self.vision.tags
        if tags:
            dist = self.vision.tagDist(tags[0])
            if dist < 10:
                self.transition(STATE["TRACK_BIN"])

    def runTrackBin(self):
        self.drive.fieldCentricTargetAngleDrive(20, 0, -90)
        tags = self.vision.tags
        # Drive forwards while facing forward and depth from camera
        # TODO: friend to bin
        # tag center x is like 0
        if tags:
            fruitTag = None
            for tag in tags:
                if tag.id == self.wantedFruitColorIndex:
                    fruitTag = tag

            x, y = self.vision.centerScreen(fruitTag.centerX, fruitTag.centerY)
            self.transition(STATE["RAM_BUCKET"])
        # else:
        #     self.transition(STATE["LINE_FOLLOW_TO_BIN"])

    def runRamBucket(self):
        arm.openBucket()
        self.drive.fieldCentricTargetAngleDrive(-200, 0, -90)

        if not self.arm.hasFruit():
            self.wantedFruitColorIndex = (self.wantedFruitColorIndex + 1) % len(
                self.fruitColors
            )
            self.transition(STATE["FIND_LINE"])

    def runError(self):
        # Could add error recovery
        self.drive.arcadeDrive(
            50 * random(),
            50 * random(),
            50 * random(),
        )
        wait(1000)
        self.transition(STATE["FIND_LINE"])


# ------------------------- Initial Robot and stuff
brain = Brain()

imu = Inertial(Ports.PORT7)

controller = Controller(PRIMARY)

# Classes
vision = VisionFruit(brain)
hDrive = HDrive(brain, imu, vision)
arm = Arm(brain, vision)

stateMachine = StateMachine(brain, hDrive, arm, vision)


# ------------------------  calibrate shit here and zero
# ---- Initial Loop
# Zero Arm -> to completion
# zero imu -> to completion
# Line follow up the ramp -> till cross -> Off ramp
# Drive to next line -> till see next line
# Zero IMU -> till completion

brain.screen.print("Calibrating")

hDrive.configMotors()

imu.calibrate()

arm.zero()
arm.openBucket()

brain.timer.clear()

while imu.is_calibrating():
    wait(5)

imu.set_heading(0, DEGREES)
imu.set_rotation(0, DEGREES)
wait(1500)
brain.screen.print("Ready TO Pick Fruit")
# --------------- Drive up ramp
if True:
    while imu.orientation(ROLL, DEGREES) > 8:
        hDrive.strafeLine(150)

    t = brain.timer.system()
    # drvies forward for 1.5 seconds and then cheks if on line
    while (not hDrive.onLine()) or (brain.timer.system() - t) < 1500:
        hDrive.fieldCentricTargetAngleDrive(0, 30, -90)

    hDrive.stop()

    imu.calibrate()
    while imu.is_calibrating():
        wait(5)

    imu.set_heading(0, DEGREES)
    imu.set_rotation(0, DEGREES)
    arm.toFruitPickingHeight(True)
# ---------------------------   RUN CODE HERE
while True:
    stateMachine.update()
    # if controller.buttonA.pressing():
    #     arm.openBucket()
    # if controller.buttonB.pressing():
    #     arm.closeBucket()

    # hDrive.strafeLine(20, 45)
    # vision.update(["green"])
    # if vision.objects:
    #     if vision.objects[0].depth > 6:
    #         hDrive.friendToFruit(vision.objects[0])
    #     else:
    #         hDrive.stop()

    #     if vision.objects[0].depth < 9:
    #         arm.PIDArm.setpoint = 65

    #     if vision.objects[0].depth < 7 and arm.toVisionFruitHeight(vision.objects[0]):
    #         arm.armMotor.stop()
    #         brain.screen.print("DONE")
    #         break
    # else:
    #     hDrive.stop()
    #     arm.armMotor.stop()
