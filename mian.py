from vex import *
from math import *
import random


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
        self.camVertFOV = 63
        self.camHorizFOV = 74
        self.degPerPixelWidth = self.camHorizFOV / self.camWidth
        self.degPerPixelHeight = self.camVertFOV / self.camHeight

        # vision config
        self.aiVision = AiVision(
            Ports.PORT10,
            self.AIGreenFruit,
            self.AIOrangeFruit,
            self.AIYellowFruit,
            AiVision.ALL_TAGS,
        )

        # arrays of fruit
        self.fruitHeight = 2 + 15 / 16
        self.smallFruitWidth = 2.25
        self.largeFruitWidth = 3

        self.smallFruitRatio = self.fruitHeight / self.smallFruitWidth
        self.largeFruitRatio = self.fruitHeight / self.largeFruitWidth

        self.fruitSizeTolerance = 0.05

        self.objects: list[Fruit] = []

        self.tags: list[AiVisionObject] = []

        self.tagWidth = 1 + 11 / 16

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
        return self.dist(fruit.height, self.fruitHeight)

    def tagDist(self, obj: AiVisionObject):
        return self.dist(obj.height, self.tagWidth)

    def dist(self, detectedHeight, realHeight):
        if detectedHeight <= 0:
            return float("inf")  # big number
        angle_deg = self.degPerPixelHeight * detectedHeight * 0.5
        distY = (realHeight * 0.5) / tan(radians(angle_deg))
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

        self.lineFront = Line(self.brain.three_wire_port.f)
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

        # PIDs used to friend to fruit
        self.friendXPID: PID = PID(
            self.brain, Kp=0.5, Ki=0, Kd=0, setpoint=0, tolerance=0.5
        )
        # clamping max speed
        self.friendXPID.addClamp(-50, 50)
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

    # makes field sentric work
    def rotateVector(self, x: float, y: float, angle: float) -> tuple[float, float]:
        angleRad = radians(angle)
        cosRad = cos(angleRad)
        sinRad = sin(angleRad)
        return (x * cosRad - y * sinRad, x * sinRad + y * cosRad)

    # fun driving scheam
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

    # drives all motors with velocity controll
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

    # drives with robot relitive
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

    # makes the robot field relitive
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

    # adds pid compensation
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
        return self.frontLineReflectivity() < 60

    def backLineOnLine(self):
        return self.backLineReflectivity() < 60

    def strafeLine(self, strafeSpeed, angle: float = 0):
        straddleLineReflectivity: float = 50
        Kp: float = 0.3

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

    def friendToFruitSlow(self, fruit: Fruit):

        fruitDistance = self.vision.fruitDist(fruit)
        x, y = self.vision.centerScreen(fruit.centerX, fruit.centerY)

        speedFW = fruitDistance * 5
        speedRot = x * 0.5
        # max speed
        maxSpeed = 70
        if speedFW > maxSpeed:
            speedFW = maxSpeed
        elif speedFW < -maxSpeed:
            speedFW = -maxSpeed

        maxSpeedRot = 70
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

        self.PIDArm = PID(self.brain, Kp=-1.5, Ki=0.0, Kd=0, tolerance=5, setpoint=0)
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

    def toFruitDropHeight(self, pause: bool = False):
        self.spinToPose(2400, pause)

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
                if self.armMotor.torque(TorqueUnits.NM) > 1.0:
                    pass
                else:
                    self.armMotor.spin(FORWARD, self.PIDArm.update(y), RPM)
                self.brain.screen.print_at(self.armMotor.position(DEGREES), x=20, y=140)
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
    def __init__(self, name, timeout, fallBack, color):
        self.name = name
        self.timeout = timeout * 1000 if timeout is not None or timeout == 0 else None
        self.fallback = fallBack
        self.color = color

    def getTimeout(self):
        return self.timeout

    def getFallback(self):
        return self.fallback

    def getColor(self):
        return self.color


STATE: dict[str, State] = {
    "FIND_LINE": State("FIND_LINE", 10.0, "ERROR", Color.WHITE),
    "LINE_FOLLOW_TO_TREE": State("LINE_FOLLOW_TO_TREE", 20, "FIND_LINE", Color.RED),
    "GO_TO_FRUIT": State("GO_TO_FRUIT", 10.0, "FIND_LINE", Color.YELLOW),
    "GRAB_FRUIT": State("GRAB_FRUIT", 7.5, "FIND_LINE", Color.GREEN),
    "BACK_TO_Line": State("BACK_TO_Line", 10.0, "FIND_LINE", Color.ORANGE),
    "LINE_FOLLOW_TO_BIN": State("LINE_FOLLOW_TO_BIN", 20.0, "FIND_LINE", Color.CYAN),
    "TRACK_BIN": State("TRACK_BIN", 10.0, "FIND_LINE", Color.BLUE),
    "RAM_BUCKET": State("RAM_BUCKET", 7.5, "FIND_LINE", Color.PURPLE),
    "ERROR": State("ERROR", 0, "FIND_LINE", Color.BLACK),
    "ESTOP": State("ESTOP", 0, "", Color.BLACK),
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
        self.preEStopState: State = None

        self.brain: Brain = brain

        self.drive: HDrive = drive
        self.arm: Arm = arm
        self.vision: VisionFruit = vision

        self.stateStartTime = self.brain.timer.system()

        self.fruitColors = ["green", "yellow", "orange"]
        self.fruitColorsForScreen = [Color.GREEN, Color.YELLOW, Color.YELLOW]
        #              Tag ID   0        1        2

        self.wantedFruitColorIndex = 0

        self.setFruitColor()

        self.lastChecks: dict[str, bool] = {}

    def update(self):
        # updates the state machine
        self.vision.update([self.fruitColors[self.wantedFruitColorIndex]])

        elapsed = self.brain.timer.system() - self.stateStartTime
        if self.state.getTimeout() is not None:
            num = floor((self.state.getTimeout() - elapsed) / 1000)
            num = str(num) if num > 9 else "0" + str(num)
            self.brain.screen.print_at(
                "Time Left: " + num,
                x=270,
                y=160,
            )
            if elapsed > self.state.getTimeout():
                fallback: State = STATE[self.state.getFallback()]
                # self.arm.closeBucket() # FOR TESTING
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
            STATE["ESTOP"]: self.runEStop,
        }.get(self.state)

        if runner:
            runner()

    def transition(self, newState: State):
        # chagnes state and displays color
        brain.screen.clear_screen()

        brain.screen.set_fill_color(newState.getColor())
        self.brain.screen.draw_rectangle(240, 120, 240, 120)
        self.brain.screen.print_at(newState.name, x=260, y=140)

        if newState.getColor() is not Color.WHITE:
            brain.screen.set_fill_color(Color.TRANSPARENT)
        else:
            brain.screen.set_fill_color(Color.BLACK)

        self.state = newState
        self.stateStartTime = self.brain.timer.system()
        # adding pause between states
        wait(500)

    def stateCheck(self, checks: dict[str, bool]) -> bool:
        # displays color and returns if all states are done
        # Skips checks if empty
        if not checks:
            return False

        # Skip if checks are identical to last run
        if checks == self.lastChecks:
            return False

        boxWidth = floor(480 / len(checks))
        done = True

        for name, check in checks.items():
            index = list(checks.keys()).index(name)
            brain.screen.set_fill_color(Color.GREEN if check else Color.RED)
            self.brain.screen.draw_rectangle(boxWidth * index, 0, boxWidth, 120)
            brain.screen.set_fill_color(Color.TRANSPARENT)
            self.brain.screen.print_at(name, x=boxWidth * index + 10, y=20)
            if not check:
                done = False

        return done

    # changes fruit to next fruit
    def nextFruit(self):
        self.wantedFruitColorIndex = (self.wantedFruitColorIndex + 1) % len(
            self.fruitColors
        )
        self.setFruitColor()

    def setFruitColor(self):
        brain.screen.draw_rectangle(
            0, 120, 240, 120, self.fruitColorsForScreen[self.wantedFruitColorIndex]
        )

    def runFindLine(self):
        # arm out in front of robot
        self.arm.toFruitPickingHeight()
        # drives backward
        self.drive.fieldCentricTargetAngleDrive(-50, 0, 45)

        checks = {
            "on Line": self.drive.onLine(),
            "arm at hight": self.arm.atFruitPickingHeight(),
        }
        if self.stateCheck(checks):
            self.drive.stop()
            self.arm.stop()
            self.transition(STATE["LINE_FOLLOW_TO_TREE"])

    # lines follow till it sees a fruit
    def runLineFollowToTree(self):
        # line follows the line at a 45 so the arm can grab all of the fruit
        self.drive.strafeLine(20, 45)

        self.arm.openBucket()

        objects = self.vision.objects
        if objects:
            x, y = self.vision.centerScreen(objects[0].centerX, objects[0].centerY)
            self.arm.toVisionFruitHeight(objects[0])

            self.brain.screen.print_at(objects[0].depth, x=280, y=50)
            # if close in the x direction and close in the depth direction go to it
            checks = {
                "close to fruit x": abs(x) < 60,
                "close to fruit depth": objects[0].depth < 20,
            }
            if self.stateCheck(checks):
                self.drive.stop()
                self.transition(STATE["GO_TO_FRUIT"])
        else:
            checks = {
                "close to fruit x": False,
                "close to fruit depth": False,
            }
            self.stateCheck(checks)

    # gets us close to the fruit
    def runGoToFruit(self):
        self.arm.openBucket()

        self.arm.PIDArm.setpoint = 25
        self.drive.friendYPID.setpoint = 6

        objects = self.vision.objects
        if objects:
            depth = objects[0].depth
            x, y = self.vision.centerScreen(objects[0].centerX, objects[0].centerY)

            self.drive.arcadeDrive(
                -self.drive.friendYPID.update(depth),
                -self.drive.friendXPID.update(x),
                0,
            )

            checks = {
                "close to fruit depth": objects[0].depth < 7,
                "close to fruit X": abs(x) < 10,
                "arm at height": self.arm.toVisionFruitHeight(objects[0]),
            }
            if self.stateCheck(checks):
                self.drive.stop()
                self.transition(STATE["GRAB_FRUIT"])
        else:
            # if lose fruit go back to line
            # self.transition(STATE["FIND_LINE"])
            pass

    # a slower drive sqance that grabs fruit
    def runGrabFruit(self):
        self.arm.PIDArm.setpoint = 67
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

    # can add more feadback from aprial tags if i want to
    def runBackToLine(self):
        self.arm.closeBucket()
        # drives back to line
        self.drive.fieldCentricTargetAngleDrive(-30, 0, -90)

        checks = {
            "on line": self.drive.onLine(),
            "has fruit": self.arm.hasFruit(),
        }
        if self.stateCheck(checks):
            self.transition(STATE["LINE_FOLLOW_TO_BIN"])

    # can add more feadback from aprial tags
    def runLineFollowToBin(self):
        self.arm.toFruitDropHeight()
        self.arm.closeBucket()
        self.drive.strafeLine(-30, -90)  # make work with feedback from tags

        tags = self.vision.tags
        if tags:
            dist = self.vision.tagDist(tags[0])

            checks = {
                "close to tag dist": dist < 15,
                "has fruit": self.arm.hasFruit(),
            }
            self.brain.screen.print_at(dist, x=20, y=160)
            if self.stateCheck(checks):
                self.drive.stop()
                self.transition(STATE["TRACK_BIN"])
        else:
            self.brain.screen.print_at("NO TAGS DETECTED", x=20, y=50)
            checks = {
                "close to tag dist": False,
                "has fruit": self.arm.hasFruit(),
            }
            self.stateCheck(checks)

    # drives along the bins untill it sees the right one
    def runTrackBin(self):
        self.arm.toFruitDropHeight()
        self.arm.closeBucket()
        # Drive forwards while facing forward and depth from camera
        self.drive.fieldCentricTargetAngleDrive(
            20, 0, -90
        )  # make work with feedback from tags
        tags = self.vision.tags

        if tags:
            fruitTag = None
            for tag in tags:
                if tag.id == self.wantedFruitColorIndex:
                    fruitTag = tag

            # if fruitTag == None:
            #     self.transition(STATE["BACK_TO_Line"])
            if fruitTag is not None:
                x, y = self.vision.centerScreen(fruitTag.centerX, fruitTag.centerY)
                depth = self.vision.tagDist(fruitTag)
                self.brain.screen.print_at("Correct X:" + str(x), x=20, y=160)
                self.brain.screen.print_at("Correct Depth:" + str(depth), x=20, y=180)

            checks = {
                "close to tag x": (abs(x) < 20) if fruitTag is not None else False,
                "has fruit": self.arm.hasFruit(),
            }

            if self.stateCheck(checks):
                self.transition(STATE["RAM_BUCKET"])
        else:
            # self.transition(STATE["BACK_TO_Line"])
            pass

    # rambs the robot into the bucket
    def runRamBucket(self):
        arm.openBucket()
        self.drive.fieldCentricTargetAngleDrive(0, -100, -90)

        checks = {
            "not has fruit": not self.arm.hasFruit(),
        }
        if self.stateCheck(checks):
            self.nextFruit()
            self.transition(STATE["FIND_LINE"])

    # drives the robot in a random derection
    def runError(self):
        # Could add error recovery
        self.drive.arcadeDrive(
            random.randint(-100, 100),
            random.randint(-100, 100),
            random.randint(-100, 100),
        )
        wait(1000)
        self.transition(STATE["FIND_LINE"])

    def runEStop(self):
        pass


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
# ---- Initial Loop
# Zero Arm -> to completion
# zero imu -> to completion
# Line follow up the ramp -> till cross -> Off ramp
# Drive to next line -> till see next line
# Zero IMU -> till completion
if True:
    while imu.orientation(ROLL, DEGREES) > 8:
        hDrive.strafeLine(150)

    t = brain.timer.system()
    # drives forward for 1.5 seconds and then checks if on line to get of the start line
    while (brain.timer.system() - t) < 1500:
        hDrive.fieldCentricTargetAngleDrive(40, 0, -90)

    while (not hDrive.onLine()) or (brain.timer.system() - t) < 3000:
        hDrive.fieldCentricTargetAngleDrive(0, 40, -90)

    hDrive.stop()

    imu.calibrate()
    while imu.is_calibrating():
        wait(5)

    imu.set_heading(0, DEGREES)
    imu.set_rotation(0, DEGREES)
arm.toFruitPickingHeight(True)
# ---------------------------   RUN CODE HERE

# use for debuggin to jump states
# stateMachine.transition(STATE["BACK_TO_Line"])
toggle = True
while True:
    stateMachine.update()

    controller.buttonA.pressed(stateMachine.nextFruit)
