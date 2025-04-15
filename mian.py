from vex import *


class Fruit:
    originX = 0
    originY = 0
    centerX = 0
    centerY = 0
    width = 0
    height = 0
    score = 0
    fruitColor = ""
    widthHeightRatio = 0

    def colorsFromStrategy(self, fruitPickingStrategy: list[str]) -> list[str]:
        colors = []
        seen = set()
        for col in fruitPickingStrategy:
            color = col.split("_")[1]
            if color not in seen:
                colors.append(color)
                seen.add(color)
        return colors


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
        self.brain = brain
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

    def warpError(self, error: float) -> float:
        range = self.maximumInput - self.minimumInput
        return (error + range / 2) % range - range / 2

    def setSetpoint(self, newSetpoint: float):
        self.setpoint: float = newSetpoint
        self.integral = 0.0
        self.lastError = 0.0

    def getSetpoint(self) -> float:
        return self.setpoint

    def update(self, measured: float, setpoint: float = None) -> float:
        dt = self.brain.timer.system() - self.lastTime
        self.lastTime = self.brain.timer.system()
        # brain.screen.print_at("dt" + str(dt), x=200, y=40)
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

    def atGoal(self, measured_value) -> bool:
        return abs(self.setpoint - measured_value) <= self.tolerance


class GridNode:
    def __init__(self, row, col):
        self.row: int = row
        self.col: int = col
        self.dist: int = 100
        self.previous: GridNode = None
        self.neighbors: list[GridNode] = []
        self.blocked: bool = False


class Grid:
    def __init__(self, rows: int, cols: int, blocked: list[tuple[int]]):
        # initiate a dijkstra object with grid size and blocked intersections
        self.rows: int = rows
        self.cols: int = cols
        self.current: tuple[int] = (0, 0)
        self.grid: dict[tuple[int], GridNode] = {}  # creates a dictionary of nodes

        for row in range(self.rows):
            for col in range(self.cols):
                self.grid[(row, col)] = GridNode(row, col)

        for block in blocked:
            self.grid[block].blocked = True

        for n in self.grid.values():
            node: GridNode = n
            if node.row < self.rows - 1:
                node.neighbors.append(self.grid[(node.row + 1, node.col)])
            if node.row > 0:
                node.neighbors.append(self.grid[(node.row - 1, node.col)])
            if node.col < self.cols - 1:
                node.neighbors.append(self.grid[(node.row, node.col + 1)])
            if node.col > 0:
                node.neighbors.append(self.grid[(node.row, node.col - 1)])

    def get_node(self, loc: tuple[int, int]):
        return self.grid[loc]

    def computePath(self, final: tuple[int, int]) -> list[tuple[int, int]]:
        # creates a queue setting all distances to 100 and the current to previous using dictionary
        for node in self.grid:
            self.grid[node].dist = 100
        self.grid[self.current].dist = 0
        self.grid[self.current].previous = None

        startNode: GridNode = self.get_node(self.current)
        queue: list[GridNode] = [startNode]

        # sets distance between nodes and prior node
        while queue:
            currentNode = queue.pop(0)
            if currentNode.blocked:
                continue
            for neighbor in currentNode.neighbors:
                if neighbor.blocked:
                    continue
                newDist = currentNode.dist + 1
                if newDist < neighbor.dist:
                    neighbor.dist = newDist
                    neighbor.previous = currentNode
                    if neighbor not in queue:
                        queue.append(neighbor)
        # makes path end to start
        path = []
        finalNode = self.get_node(final)
        while finalNode:
            path.append((finalNode.row, finalNode.col))
            finalNode = finalNode.previous

        # flips and returns path
        path.reverse()
        self.current = final
        path = self.optimizePath(path)
        return path

    def optimizePath(self, path: list[tuple[int, int]]) -> list[tuple[int, int]]:
        if len(path) < 3:
            return path

        simplified = [path[0]]  # Start with the first point

        i = 1
        while i < len(path) - 1:
            start = simplified[-1]
            curr = path[i]
            next = path[i + 1]

            # Get direction vectors
            dx1, dy1 = curr[0] - start[0], curr[1] - start[1]
            dx2, dy2 = next[0] - curr[0], next[1] - curr[1]

            # Normalize directions
            if dx1 != 0:
                dx1 //= abs(dx1)
            if dy1 != 0:
                dy1 //= abs(dy1)
            if dx2 != 0:
                dx2 //= abs(dx2)
            if dy2 != 0:
                dy2 //= abs(dy2)

            if (dx1, dy1) == (dx2, dy2):
                # Same direction, check how long it goes
                direction = (dx1, dy1)
                runEnd = i + 1
                while runEnd + 1 < len(path):
                    nextDir = (
                        path[runEnd + 1][0] - path[runEnd][0],
                        path[runEnd + 1][1] - path[runEnd][1],
                    )
                    if nextDir[0] != 0:
                        nextDir = (nextDir[0] // abs(nextDir[0]), nextDir[1])
                    if nextDir[1] != 0:
                        nextDir = (nextDir[0], nextDir[1] // abs(nextDir[1]))
                    if nextDir == direction:
                        runEnd += 1
                    else:
                        break

                # Add the endpoint of the run and skip the middle ones
                simplified.append(path[runEnd])
                i = runEnd + 1
            else:
                simplified.append(curr)
                i += 1

        # Add last point if it wasn't included
        if simplified[-1] != path[-1]:
            simplified.append(path[-1])

        return simplified

    # def print_grid(self):
    #     spacing = 1
    #     print("  " + " " * (spacing + 1), end="")
    #     for col in range(self.cols):
    #         print(" " * spacing + f"{col}".zfill(2), end="")
    #     for row in range(self.rows):
    #         print("\n" + f"{row}".zfill(2) + " " * spacing + "|", end="")
    #         for col in range(self.cols):
    #             node = self.get_node((row, col))
    #             print(
    #                 " " * spacing + ("  " if node.blocked else f"{node.dist}".zfill(2)),
    #                 end="",
    #             )
    #     print()


class Odometry:
    def __init__(
        self,
        rangeFinderFront: Sonar,
        rangeFinderRight: Sonar,
    ):
        self.rangeFinderFront = rangeFinderFront
        self.rangeFinderRight = rangeFinderRight

    def getPosition(self) -> tuple[int, int]:
        return (0, 0)


class HDrive:
    def __init__(
        self,
        odometry: Odometry,
        brain: Brain,
        imu: Inertial,
        lineLeft: Line,
        lineRight: Line,
    ):
        self.odometry = odometry
        # Robot Const

        self.wheelBase: float = 10
        self.wheelTrack: float = 10

        self.wheelDiameter: float = 4
        self.wheelCircumstance: float = self.wheelDiameter * math.pi

        self.brain = brain

        self.maxWhealSpeed: float = 175
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

    def configMotors(self):
        self.frontLeftMotor.reset_position()
        self.frontRightMotor.reset_position()
        self.backLeftMotor.reset_position()
        self.backRightMotor.reset_position()
        self.frontSideMotor.reset_position()
        self.backSideMotor.reset_position()

    def rotateVector(self, x: float, y: float, angle: float):
        angleRad = math.radians(angle)
        cos = math.cos(angleRad)
        sin = math.sin(angleRad)
        return (x * cos - y * sin, x * sin + y * cos)

    def joystickToAngleDeg(self, x: float, y: float) -> float:
        rad = math.atan2(y, x)
        deg = math.degrees(rad)
        return deg % 360

    def stop(self):
        self.frontLeftMotor.stop(HOLD)
        self.frontRightMotor.stop(HOLD)
        self.backLeftMotor.stop(HOLD)
        self.backRightMotor.stop(HOLD)
        self.frontSideMotor.stop(HOLD)
        self.backSideMotor.stop(HOLD)

    def dumbVelocityDrive(
        self,
        flSpeed: float,
        frSpeed: float,
        rlSpeed: float,
        rrSpeed: float,
        fsSpeed: float,
        bsSpeed: float,
    ):
        self.frontLeftMotor.spin(FORWARD, flSpeed, RPM)
        self.frontRightMotor.spin(FORWARD, frSpeed, RPM)
        self.backLeftMotor.spin(FORWARD, rlSpeed, RPM)
        self.backRightMotor.spin(FORWARD, rrSpeed, RPM)

        self.frontSideMotor.spin(FORWARD, fsSpeed, RPM)
        self.backSideMotor.spin(FORWARD, bsSpeed, RPM)

        brain.screen.print_at(flSpeed, x=40, y=40)
        brain.screen.print_at(frSpeed, x=40, y=60)
        brain.screen.print_at(rlSpeed, x=40, y=80)
        brain.screen.print_at(rrSpeed, x=40, y=100)
        brain.screen.print_at(fsSpeed, x=40, y=120)
        brain.screen.print_at(bsSpeed, x=40, y=140)

    def arcadeDrive(
        self,
        forward: float,
        strafe: float,
        rotation: float,
    ):
        flSpeed = forward + rotation
        frSpeed = forward - rotation
        rlSpeed = forward + rotation
        rrSpeed = forward - rotation

        # 2 center wheels: strafe only
        fsSpeed = strafe
        bsSpeed = strafe

        # Normalize powers
        maxSpeed = max(
            abs(flSpeed),
            abs(frSpeed),
            abs(rlSpeed),
            abs(rrSpeed),
            abs(fsSpeed),
            abs(bsSpeed),
            self.maxWhealSpeed,
        )

        frSpeed /= maxSpeed
        flSpeed /= maxSpeed
        rlSpeed /= maxSpeed
        rrSpeed /= maxSpeed
        fsSpeed /= maxSpeed
        bsSpeed /= maxSpeed

        frSpeed *= self.maxWhealSpeed
        flSpeed *= self.maxWhealSpeed
        rlSpeed *= self.maxWhealSpeed
        rrSpeed *= self.maxWhealSpeed
        fsSpeed *= self.maxWhealSpeed
        bsSpeed *= self.maxWhealSpeed

        self.dumbVelocityDrive(flSpeed, frSpeed, rlSpeed, rrSpeed, fsSpeed, bsSpeed)

    def fieldCentricDrive(
        self,
        forward: float,
        strafe: float,
        rotation: float,
    ):
        # Boost strafe if needed
        strafeBoost = 2.0
        strafe *= strafeBoost

        # Field-oriented translation
        strafeFC, forwardFC = self.rotateVector(strafe, forward, imu.heading())

        self.arcadeDrive(forwardFC, strafeFC, rotation)

    def fieldCentricTargetAngleDrive(
        self,
        forward: float,
        strafe: float,
        targetAngle: float,
    ):
        # PID computes rotation power to reach target angle
        rotation = self.headingPID.update(measured=imu.heading(), setpoint=targetAngle)
        brain.screen.print_at(targetAngle, x=40, y=160)
        brain.screen.print_at(imu.heading(), x=40, y=180)
        # brain.screen.print_at(imu.heading(), x=100, y=40)
        self.fieldCentricDrive(forward, strafe, rotation)

    def driveController(self, controller: Controller):
        self.arcadeDrive(
            controller.axis3.position(),
            controller.axis1.position(),
            controller.axis4.position(),
        )

        # self.fieldCentricDrive(
        #     controller.axis3.position(),
        #     controller.axis4.position(),
        #     controller.axis1.position(),
        # )

        # self.fieldCentricTargetAngleDrive(
        #     controller.axis3.position(),
        #     controller.axis4.position(),
        #     0,
        # )

        # self.fieldCentricTargetAngleDrive(
        #     controller.axis3.position(),
        #     controller.axis4.position(),
        #     self.joystickToAngleDeg(
        #         controller.axis1.position(), controller.axis1.position()
        #     ),
        # )

    # center of robot
    def driveToPosition(self, posX: float, posY: float, targetAngle: float):
        x, y = self.odometry.getPosition()

        forward = self.positionPIDY.update(measured=y, setpoint=posY)
        strafe = self.positionPIDX.update(measured=x, setpoint=posX)

        self.fieldCentricTargetAngleDrive(forward, strafe, targetAngle)

        return self.positionPIDY.atGoal(y) and self.positionPIDY.atGoal(x)


class VisionFruit:
    def __init__(
        self,
    ):
        # fruit color configs
        self.greenFruit = Colordesc(1, 12, 167, 71, 20, 0.2)
        self.orangeFruit = Colordesc(2, 222, 66, 67, 10, 0.2)
        self.yellowFruit = Colordesc(3, 166, 114, 59, 15, 0.2)

        # camera const
        self.camWidth = 320
        self.camHeight = 240
        self.camVertFOV = 68
        self.camHorizFOV = 74
        self.degPerPixelWidth = self.camHorizFOV / self.camWidth
        self.degPerPixelHeight = self.camVertFOV / self.camHeight

        # vision config
        self.aiVision = AiVision(
            Ports.PORT14, self.greenFruit, self.orangeFruit, self.yellowFruit
        )

        # arrays of fruit
        self.objects = {}

        self.strategy = []
        self.strategyColors: list[str] = []

        self.largeFruitRatio = 1.1  # guess
        self.smallFruitRatio = 0.5  # guess
        self.fruitSizeTolerance = 0.05  # guess

    def setStrategy(self, strategy: list[str]):
        self.strategy = strategy
        self.strategyColors: list[str] = Fruit.colorsFromStrategy(self.strategy)

    def fruitDist(self, pixelWidth, ObjectWidthIn):
        angularWidth = self.degPerPixelWidth * pixelWidth
        return (ObjectWidthIn * 0.5) / math.tan(math.radians(angularWidth * 0.5))

    def update(self) -> bool:
        """
        ### Parameters
        none
        ### What it does
        updates the objects inside the vision class with what the camera seas
        ### return
        returns true if the camera detects and object"""
        self.objects.clear()
        print(self.strategyColors)

        for color in self.strategyColors:
            match color.lower():
                case "green":
                    for obj in self.aiVision.take_snapshot(self.greenFruit):
                        tempFruit = self.makeFruitFromVisionObject(obj, "green")
                        if (
                            abs(tempFruit.widthHeightRatio - self.largeFruitRatio)
                            < self.fruitSizeTolerance
                        ):
                            if "Large_Green" not in self.objects:
                                self.objects["Large_Green"] = []
                            self.objects["Large_Green"].append(tempFruit)
                        if (
                            abs(tempFruit.widthHeightRatio - self.smallFruitRatio)
                            < self.fruitSizeTolerance
                        ):
                            if "Small_Green" not in self.objects:
                                self.objects["Small_Green"] = []
                            self.objects["Small_Green"].append(tempFruit)
                case "orange":
                    for obj in self.aiVision.take_snapshot(self.orangeFruit):
                        tempFruit = self.makeFruitFromVisionObject(obj, "orange")
                        if (
                            abs(tempFruit.widthHeightRatio - self.largeFruitRatio)
                            < self.fruitSizeTolerance
                        ):
                            if "Large_Orange" not in self.objects:
                                self.objects["Large_Orange"] = []
                            self.objects["Large_Orange"].append(tempFruit)
                        if (
                            abs(tempFruit.widthHeightRatio - self.smallFruitRatio)
                            < self.fruitSizeTolerance
                        ):
                            if "Small_Orange" not in self.objects:
                                self.objects["Small_Orange"] = []
                            self.objects["Small_Orange"].append(tempFruit)
                case "yellow":
                    for obj in self.aiVision.take_snapshot(self.yellowFruit):
                        tempFruit = self.makeFruitFromVisionObject(obj, "yellow")
                        if (
                            abs(tempFruit.widthHeightRatio - self.largeFruitRatio)
                            < self.fruitSizeTolerance
                        ):
                            if "Large_Yellow" not in self.objects:
                                self.objects["Large_Yellow"] = []
                            self.objects["Large_Yellow"].append(tempFruit)
                        if (
                            abs(tempFruit.widthHeightRatio - self.smallFruitRatio)
                            < self.fruitSizeTolerance
                        ):
                            if "Small_Yellow" not in self.objects:
                                self.objects["Small_Yellow"] = []
                            self.objects["Small_Yellow"].append(tempFruit)
                case _:
                    pass
            if self.strategy[self.strategyColors.index(color)] in self.objects:
                return True
        # need the not not to return a boolean
        return not not self.objects

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


class Arm:
    def __init__(self):

        self.driveRatio = 4
        armMotor: Motor = Motor(Ports.PORT8, 18_1, True)
        # self.positionPIDX = PID(self.brain, Kp=75, Ki=0.0, Kd=0, tolerance=0.5)

    def controlController(self, controller: Controller):
        # horizontal 0
        # vertical -680
        # other horizontal -1250

        brain.screen.print_at(armMotor.position(DEGREES), x=40, y=40)
        if not abs(controller.axis2.position()) < 0.05:
            armMotor.spin(FORWARD, controller.axis2.position() * 50, RPM)
        else:
            armMotor.stop(HOLD)


# const
fruitPickingStrategy = [
    "Large_Green",
    "Large_Yellow",
    "Small_Green",
    "Large_Orange",
    "Small_Yellow",
    "Small_Orange",
]

blocked: list[tuple[int, int]] = [
    (3, 1),
    (3, 3),
    (2, 2),
    (3, 7),
    (2, 8),
    (2, 5),
    (4, 6),
]

treeBranchLocations: dict[str, list[tuple[int]]] = {
    "Green": [(4, 9), (3, 9), (2, 9)],
    "Yellow": [(4, 8), (3, 8), (2, 8)],
}

# Initial Robot and stuff
brain = Brain()

imu = Inertial(Ports.PORT7)

lineLeft = Line(brain.three_wire_port.a)
lineRight = Line(brain.three_wire_port.b)

rangeFinderFront = None  # Sonar(brain.three_wire_port.e)
rangeFinderRight = None  # Sonar(brain.three_wire_port.c)

controller = Controller(PRIMARY)

# 5ft x 10ft guess
field = Grid(20, 50, blocked)

# classes
odometry = Odometry(rangeFinderFront, rangeFinderRight)
hDrive = HDrive(odometry, brain, imu, lineLeft, lineRight)
arm = Arm()
vision = VisionFruit()


# calibrate shit here and zero

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

# RUN CODE HERE
while True:
    hDrive.driveController(controller)
