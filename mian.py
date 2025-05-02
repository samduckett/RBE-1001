from vex import *
from math import *
import random


# -------------- Classes that should probably only be made in java but they are here
class Fruit:
    """
    Represents a fruit object with position, size, color, width-height ratio, depth, and area attributes.
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
        min_input_value: float = 0.0,
        max_input_value: float = 360.0,
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
        self.min_input_value = min_input_value
        self.max_input_value = max_input_value

        # internal state
        self.previous_error: float = 0.0
        self.integral: float = 0.0
        self.previous_time: float = 0

        # Clamp
        self.min_velocity_output = None
        self.max_velocity_output = None
        self.min_position_output = None  # Added for position clamp
        self.max_position_output = None  # Added for position clamp

    def clamp(self, value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))

    def addClamp(
        self,
        min_velocity: float,
        max_velocity: float,
        min_position: float = None,  # Added for position clamp
        max_position: float = None,  # Added for position clamp
    ) -> None:
        self.min_velocity_output = min_velocity
        self.max_velocity_output = max_velocity
        self.min_position_output = min_position  # Store position limits
        self.max_position_output = max_position  # Store position limits

    def warpError(self, error: float) -> float:
        range = self.max_input_value - self.min_input_value
        error = (error + range / 2) % range - range / 2
        return error

    def setSetpoint(self, newSetpoint: float, reset: bool = True) -> None:
        self.setpoint: float = newSetpoint
        if reset:
            self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = 0

    def getSetpoint(self) -> float:
        return self.setpoint

    def update(
        self,
        measured_value: float,
        setpoint: float = None,
        current_position: float = None,
    ) -> float:
        current_time = self.brain.timer.system()

        if self.previous_time == 0:
            self.previous_time = current_time
            return 0.0

        delta_time = current_time - self.previous_time
        self.previous_time = current_time

        if setpoint is not None:
            self.setpoint = setpoint

        error = self.setpoint - measured_value
        if self.continuous:
            error = self.warpError(error)

        # P term
        P = self.Kp * error

        # I term
        self.integral += error * delta_time
        I = self.Ki * self.integral

        # D term
        derivative = (
            (error - self.previous_error) / delta_time if delta_time > 0 else 0.0
        )
        D = self.Kd * derivative

        # store for next cycle for D
        self.previous_error = error

        # calculate output
        output = P + I + D

        # Clamp output if limits are set
        if (
            self.min_velocity_output is not None
            and self.max_velocity_output is not None
        ):
            output = self.clamp(
                output, self.min_velocity_output, self.max_velocity_output
            )
        return output

    def atGoal(self, measured_value: float) -> bool:
        error = self.setpoint - measured_value
        if self.continuous:
            error = self.warpError(error)
        return abs(error) <= self.tolerance

    def ifInPositionClampRange(self, current_position: float) -> bool:
        if self.min_position_output is None or self.max_position_output is None:
            return True  # No position clamping is active
        return self.min_position_output <= current_position <= self.max_position_output


class VisionFruit:
    def __init__(
        self,
        brain: Brain,
    ):
        # For logging
        self.brain: Brain = brain

        # fruit color configs
        self.AIGreenFruit = Colordesc(1, 13, 169, 72, 16, 0.2)
        self.AIOrangeFruit = Colordesc(3, 246, 151, 131, 10, 0.21)
        self.AIYellowFruit = Colordesc(2, 193, 142, 75, 4, 0.4)

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
            Kp=4,  # TODO was 5 is now 4 tune more
            Ki=0,
            Kd=0,
            setpoint=0.0,
            tolerance=1.0,
            continuous=True,
            min_input_value=0.0,
            max_input_value=360.0,
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

        self.startingHeading = None

    def configMotors(self):
        self.frontLeftMotor.reset_position()
        self.frontRightMotor.reset_position()
        self.backLeftMotor.reset_position()
        self.backRightMotor.reset_position()
        self.frontSideMotor.reset_position()
        self.backSideMotor.reset_position()

    # makes field centric work
    def rotateVector(self, x: float, y: float, angle: float) -> tuple[float, float]:
        angleRad = radians(angle)
        cosRad = cos(angleRad)
        sinRad = sin(angleRad)
        return (x * cosRad - y * sinRad, x * sinRad + y * cosRad)

    # fun driving scheme
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

    # drives all motors with velocity control
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

    # drives with robot relative
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

    # makes the robot field relative

    def arcadeDriveCorrectedHeading(self, forward: float, strafe: float):
        if self.startingHeading == None:
            self.startingHeading = imu.heading()
        # PID computes rotation power to reach target angle
        rotation = self.headingPID.update(
            measured_value=imu.heading(), setpoint=self.startingHeading
        )

        # brain.screen.print_at(imu.heading(), x=100, y=40)
        self.arcadeDrive(forward, strafe, rotation)

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
        rotation = self.headingPID.update(
            measured_value=imu.heading(), setpoint=targetAngle
        )

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

    def onCross(self) -> bool:  # TODO !!!!! IMPORTANT NEED TO FIXFix
        return self.lineFront.value(PERCENT) < 20 and self.lineBack.value(PERCENT) < 40

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

        self.setpoint = 0

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
        return self.atHome()

    def atHome(self):
        return abs(self.armMotor.position(DEGREES)) < 20

    def toFruitPickingHeight(self, pause: bool = False):
        self.spinToPose(2000, pause)

    def toFruitDropHeight(self, pause: bool = False):
        self.spinToPose(2300, pause)

    def atFruitPickingHeight(self):
        return abs(arm.armMotor.position(DEGREES) - 2000) < 15  # TUNE

    def toVisionFruitHeight(self, fruit: Fruit | None) -> bool:
        if 1100 > self.armMotor.position(DEGREES):
            self.spinToPose(1150, False)
            return False
        elif self.armMotor.position(DEGREES) > 2200:
            self.spinToPose(2150, False)
            return False

        if fruit is None:
            self.stop()
            return False

        # TODO add clamp and if not in clamp range go to clamp range
        x, y = vision.centerScreen(fruit.centerX, fruit.centerY)
        yError = y - self.setpoint
        if abs(yError) < 5:
            self.stop()
            return True

        if self.armMotor.torque(TorqueUnits.NM) > 1.0:
            return False

        speed = yError * 1.5
        speed = max(min(170, speed), -170)
        self.armMotor.spin(FORWARD, speed)
        return False

    def inClampRange(self, tolerance: float = 1):
        return 1100 < self.armMotor.position(DEGREES) < 2200

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

    def getName(self):
        return self.name


STATE: dict[str, State] = {
    "FIND_LINE": State("FIND_LINE", 10.0, "ERROR", Color.WHITE),
    ## -------- Line Follows
    "LINE_FOLLOW_TO_END": State("LINE_FOLLOW_TO_END", 20, "FIND_LINE", Color.RED),
    "LINE_FOLLOW_TO_CROSS": State("LINE_FOLLOW_TO_CROSS", 20, "FIND_LINE", Color.RED),
    "LINE_FOLLOW_TO_START": State(
        "LINE_FOLLOW_TO_START", 20, "LINE_FOLLOW_TO_END", Color.RED
    ),
    ## -- Picking Fruit
    "CLOSE_TO_FRUIT": State("CLOSE_TO_FRUIT", 10.0, "FIND_LINE", Color.YELLOW),
    "GRAB_FRUIT": State("GRAB_FRUIT", 7.5, "FIND_LINE", Color.GREEN),
    ## ---- droping fruit
    "LINE_FOLLOW_TO_BIN": State("LINE_FOLLOW_TO_BIN", 20.0, "FIND_LINE", Color.CYAN),
    "TRACK_BIN": State("TRACK_BIN", 10.0, "FIND_LINE", Color.BLUE),
    "RAM_BIN": State("RAM_BIN", 7.5, "FIND_LINE", Color.PURPLE),
    "BACK_UP": State("BACK_UP", 7.5, "FIND_LINE", Color.PURPLE),
    ## ------ Other
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
        self.fruitColorsForScreen = [Color.GREEN, Color.YELLOW, Color.ORANGE]
        #              Tag ID   0        1        2

        self.wantedFruitColorIndex = 1

        self.setFruitColor()

        self.lastChecks: dict[str, bool] = {}

        self.attemptedPicks = 0
        self.grabFruitArmAtSetpoint = False

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
            # find
            STATE["LINE_FOLLOW_TO_END"]: self.runLineFollowToEnd,
            STATE["LINE_FOLLOW_TO_CROSS"]: self.runLineFollowToCross,
            STATE["LINE_FOLLOW_TO_START"]: self.runLineFollowToStart,
            # pick
            STATE["CLOSE_TO_FRUIT"]: self.runGoToFruit,
            STATE["GRAB_FRUIT"]: self.runGrabFruit,
            # drop
            STATE["LINE_FOLLOW_TO_BIN"]: self.runLineFollowToBin,
            STATE["TRACK_BIN"]: self.runTrackBin,
            STATE["RAM_BIN"]: self.runRamBucket,
            STATE["BACK_UP"]: self.runBackUp,
            # othor
            STATE["ERROR"]: self.runError,
            STATE["ESTOP"]: self.runEStop,
        }.get(self.state)

        if runner:
            runner()

    def transition(self, newState: State):

        self.grabFruitArmAtSetpoint = False
        self.drive.startingHeading = None
        # changes state and displays color
        brain.screen.clear_screen()

        brain.screen.set_fill_color(newState.getColor())
        self.brain.screen.draw_rectangle(240, 120, 240, 120)
        self.brain.screen.print_at(newState.getName(), x=260, y=140)
        self.setFruitColor()

        if newState.getColor() is not Color.WHITE:
            brain.screen.set_fill_color(Color.TRANSPARENT)
        else:
            brain.screen.set_fill_color(Color.BLACK)

        if newState.getName() != "ESTOP":
            self.preEStopState = self.state

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

        self.lastChecks = checks

        return done

    # changes fruit to next fruit
    def nextFruit(self):
        self.wantedFruitColorIndex = (self.wantedFruitColorIndex + 1) % len(
            self.fruitColors
        )
        self.attemptedPicks = 0
        self.setFruitColor()

    def setFruitColor(self):
        brain.screen.draw_rectangle(
            0, 120, 240, 120, self.fruitColorsForScreen[self.wantedFruitColorIndex]
        )

    def runFindLine(self):
        # arm out in front of robot
        # drives backward
        self.drive.arcadeDriveCorrectedHeading(-40, 0)

        if self.drive.onLine():
            self.arm.toHome()
            self.drive.stop()

        if self.attemptedPicks > 3:
            self.nextFruit()

        checks = {
            "on Line": self.drive.onLine(),
            "arm at hight": self.arm.atHome(),
        }
        if self.stateCheck(checks):
            self.drive.stop()
            self.arm.stop()

            if self.arm.hasFruit():
                self.transition(STATE["LINE_FOLLOW_TO_BIN"])
            elif self.wantedFruitColorIndex == 0:
                self.transition(STATE["LINE_FOLLOW_TO_CROSS"])
                self.attemptedPicks += 1
            else:
                self.transition(STATE["LINE_FOLLOW_TO_END"])
                self.attemptedPicks += 1

    # ------------- Line Follow
    def runLineFollowToEnd(self):
        # line follows the line at a 45 so the arm can grab all of the fruit
        objects = self.vision.objects

        self.arm.setpoint = 0
        self.arm.openBucket()

        if self.drive.onCross():
            self.transition(STATE["LINE_FOLLOW_TO_START"])

        if arm.inClampRange():
            self.drive.strafeLine(20, 45)
            if objects:
                x, y = self.vision.centerScreen(objects[0].centerX, objects[0].centerY)
                self.arm.toVisionFruitHeight(objects[0])

                # if close in the x direction and close in the depth direction go to it
                checks = {
                    "close to fruit x": abs(x) < 25,  # TODO TUNE, it was 60
                    "close to fruit depth": objects[0].depth
                    < 20,  # TODO might want to make 15
                }
                if self.stateCheck(checks):
                    self.drive.stop()
                    self.arm.stop()
                    self.transition(STATE["CLOSE_TO_FRUIT"])
            else:
                arm.stop()
                checks = {
                    "close to fruit x": False,
                    "close to fruit depth": False,
                }
                self.stateCheck(checks)

        else:
            arm.toFruitPickingHeight()
            checks = {
                "close to fruit x": False,
                "close to fruit depth": False,
            }
            self.stateCheck(checks)

    def runLineFollowToStart(self):
        # line follows the line at a 45 so the arm can grab all of the fruit
        objects = self.vision.objects
        tags = self.vision.tags

        self.arm.setpoint = 0
        self.arm.openBucket()

        if tags:
            if self.vision.tagDist(tags[0]) < 25:
                self.transition(STATE["LINE_FOLLOW_TO_END"])

        if arm.inClampRange():
            self.drive.strafeLine(-20, -45)
            if objects:
                x, y = self.vision.centerScreen(objects[0].centerX, objects[0].centerY)
                self.arm.toVisionFruitHeight(objects[0])

                # if close in the x direction and close in the depth direction go to it
                checks = {
                    "close to fruit x": abs(x) < 25,  # TODO TUNE, it was 60
                    "close to fruit depth": objects[0].depth
                    < 20,  # TODO might want to make 15
                }
                if self.stateCheck(checks):
                    self.drive.stop()
                    self.arm.stop()
                    self.transition(STATE["CLOSE_TO_FRUIT"])
            else:
                arm.stop()
                checks = {
                    "close to fruit x": False,
                    "close to fruit depth": False,
                }
                self.stateCheck(checks)

        else:
            arm.toFruitPickingHeight()
            checks = {
                "close to fruit x": False,
                "close to fruit depth": False,
            }
            self.stateCheck(checks)

    def runLineFollowToCross(self):
        self.drive.strafeLine(20, 90)
        checks = {
            "on cross": self.drive.onCross(),
        }
        if self.stateCheck(checks):
            self.drive.stop()
            pass
            # self.transition(STATE["LINE_FOLLOW_TO_START"])

    # --- Grab Fruit
    def runGoToFruit(self):
        self.arm.openBucket()

        self.arm.setpoint = 25
        self.drive.friendYPID.setpoint = 6

        objects = self.vision.objects
        if objects:
            depth = objects[0].depth
            x, y = self.vision.centerScreen(objects[0].centerX, objects[0].centerY)

            self.drive.arcadeDriveCorrectedHeading(
                -self.drive.friendYPID.update(depth),
                -self.drive.friendXPID.update(x),
            )

            checks = {
                "close to fruit depth": objects[0].depth < 7,
                "close to fruit X": abs(x) < 10,
                "arm at height": self.arm.toVisionFruitHeight(objects[0]),
            }
            if self.stateCheck(checks):
                self.drive.stop()
                self.arm.stop()
                self.transition(STATE["GRAB_FRUIT"])
        else:
            checks = {
                "close to fruit depth": False,
                "close to fruit X": False,
                "arm at height": False,
            }
            self.stateCheck(checks)

    def runGrabFruit(self):
        self.arm.setpoint = 68
        objects = self.vision.objects
        if objects:
            if not self.grabFruitArmAtSetpoint:
                if self.arm.toVisionFruitHeight(objects[0]):
                    self.grabFruitArmAtSetpoint = True
            else:
                self.arm.stop()
                self.drive.friendToFruitSlow(objects[0])

        checks = {
            "has fruit": self.arm.hasFruit(),
        }
        if self.stateCheck(checks):
            self.arm.closeBucket()
            self.drive.stop()
            self.transition(STATE["FIND_LINE"])

    # ------------- Drop Fruit
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

    def runTrackBin(self):
        self.arm.toFruitDropHeight()
        self.arm.closeBucket()
        # Drive forwards while facing forward and depth from camera
        self.drive.fieldCentricTargetAngleDrive(
            30, 0, -90
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
                self.transition(STATE["RAM_BIN"])
        else:
            # self.transition(STATE["BACK_TO_Line"])
            pass

    def runRamBucket(self):
        arm.openBucket()
        self.drive.fieldCentricTargetAngleDrive(0, -50, -90)

        checks = {
            "not has fruit": not self.arm.hasFruit(),
        }
        if self.stateCheck(checks):
            self.nextFruit()
            self.transition(STATE["BACK_UP"])

    def runBackUp(self):
        self.arm.toHome()
        self.drive.fieldCentricTargetAngleDrive(0, 30, 0)

        if self.stateStartTime > 500:
            self.drive.stop()
            self.transition(STATE["FIND_LINE"])

    # ------------ othor
    def runError(self):
        # Could add error recovery
        self.drive.arcadeDriveCorrectedHeading(40, 0)
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
# ---------------------------   RUN CODE HERE

stateMachine.transition(STATE["LINE_FOLLOW_TO_CROSS"])
while True:
    stateMachine.update()
