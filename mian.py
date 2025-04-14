from vex import *


class Fruit:
    def __init__(self):
        self.originX = 0
        self.originY = 0
        self.centerX = 0
        self.centerY = 0
        self.width = 0
        self.height = 0
        self.score = 0
        self.fruitColor = ""
        self.widthHeightRatio = 0

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
        Kp: float,
        Ki: float,
        Kd: float,
        setpoint: float = 0.0,
        tolerance: float = 0.0,
        continuous: bool = False,
        minimumInput: float = 0.0,
        maximumInput: float = 360.0,
    ):
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
        dt = timer.system() - self.lastTime
        self.lastTime = timer.system()

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

    def at_goal(self, measured_value) -> bool:
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

    def get_node(self, loc: tuple[int]):
        return self.grid[loc]

    def compute_path(self, final):
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
                run_end = i + 1
                while run_end + 1 < len(path):
                    next_dir = (
                        path[run_end + 1][0] - path[run_end][0],
                        path[run_end + 1][1] - path[run_end][1],
                    )
                    if next_dir[0] != 0:
                        next_dir = (next_dir[0] // abs(next_dir[0]), next_dir[1])
                    if next_dir[1] != 0:
                        next_dir = (next_dir[0], next_dir[1] // abs(next_dir[1]))
                    if next_dir == direction:
                        run_end += 1
                    else:
                        break

                # Add the endpoint of the run and skip the middle ones
                simplified.append(path[run_end])
                i = run_end + 1
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
    #         print("\033[4m" + " " * spacing + f"{col}".zfill(2) + "\033[0m", end="")
    #     for row in range(self.rows):
    #         print("\n" + f"{row}".zfill(2) + " " * spacing + "|", end="")
    #         for col in range(self.cols):
    #             node = self.get_node((row, col))
    #             print(
    #                 " " * spacing + ("  " if node.blocked else f"{node.dist}".zfill(2)),
    #                 end="",
    #             )
    #     print()


class HDrive:
    def __init__(
        self,
    ):
        self.heading: float = 0

        self.wheelBase = 10
        self.wheelTrack = 10

        self.frontLeftMotor = Motor(Ports.PORT1, 18_1, False)
        self.frontRightMotor = Motor(Ports.PORT2, 18_1, True)
        self.backLeftMotor = Motor(Ports.PORT3, 18_1, False)
        self.backRightMotor = Motor(Ports.PORT4, 18_1, True)
        self.frontSideMotor = Motor(Ports.PORT5, 18_1, False)
        self.backSideMotor = Motor(Ports.PORT6, 18_1, True)

    def configMotors(self):
        self.frontLeftMotor.reset_position()
        self.frontRightMotor.reset_position()
        self.backLeftMotor.reset_position()
        self.backRightMotor.reset_position()
        self.frontSideMotor.reset_position()
        self.backSideMotor.reset_position()

    def drive(self, velocityX: float, velocityY: float, heading: float = None):
        """velocity in ft/s"""
        if heading is not None:
            self.heading = heading

        self.frontLeftMotor.spin(FORWARD, velocityY, RPM)
        self.frontRightMotor.spin(FORWARD, velocityY, RPM)
        self.backLeftMotor.spin(FORWARD, velocityY, RPM)
        self.backRightMotor.spin(FORWARD, velocityY, RPM)

        self.frontSideMotor.spin(FORWARD, velocityX, RPM)
        self.backSideMotor.spin(FORWARD, velocityX, RPM)

    def driveController(self):
        xSpeed = 100
        ySpeed = 100

        self.drive(
            controller.axis3.position() * ySpeed, controller.axis1.position() * xSpeed
        )

    def drivePosition(self, posX: float, posY: float):
        pass


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


# Initial Robot
brain = Brain()

# seonsors
imu = Inertial(Ports.PORT7)

lineLeft = Line(brain.three_wire_port.a)
lineRight = Line(brain.three_wire_port.b)

controller = Controller(PRIMARY)

# classes
hDrive = HDrive()
vision = VisionFruit()


brain.screen.print("Calibrating")

# calibrate shit here and zero

hDrive.configMotors()

imu.calibrate()

while imu.is_calibrating():
    wait(5)

timer.clear()
imu.set_heading(0, DEGREES)
imu.set_rotation(0, DEGREES)


brain.screen.print("Finished Calibrating")

# Field Const
fruitPickingStrategy = [
    "Large_Green",
    "Large_Yellow",
    "Small_Green",
    "Large_Orange",
    "Small_Yellow",
    "Small_Orange",
]

vision.setStrategy(fruitPickingStrategy)

blocked: list[tuple[int, int]] = [
    (3, 1),
    (3, 3),
    (2, 2),
    (3, 7),
    (2, 8),
    (2, 5),
    (4, 6),
]
treeBranchLocations: dict[str, list[tuple[int]]] = {"Green": [(4, 9)]}

# grid on the inch
field = Grid(5, 10, blocked)

print(
    "Planned Path to first branch in strategy",
    field.compute_path(
        treeBranchLocations.get(Fruit.colorsFromStrategy(fruitPickingStrategy)[0])[0]
    ),
)

while True:
    hDrive.driveController()

# drive up ramp till line cross then run code

# navigate to wanted tree
