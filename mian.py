# endregion VEXcode Generated Robot Configuration
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
        self.dTheta: float = dTheta

    def inverse(self):
        angle_rad = radians(-self.dTheta)
        new_dx = -cos(angle_rad) * self.dX - sin(angle_rad) * self.dY
        new_dy = sin(angle_rad) * self.dX - cos(angle_rad) * self.dY
        return Transform2D(new_dx, new_dy, -self.dTheta)


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

    def __eq__(self, other):
        return (
            isinstance(other, Pose2D)
            and self.x == other.x
            and self.y == other.y
            and self.heading == other.heading
        )

    def __hash__(self):
        return hash((self.x, self.y, self.heading))

    def __lt__(self, other):
        if not isinstance(other, Pose2D):
            return NotImplemented
        return (self.x, self.y, self.heading) < (other.x, other.y, other.heading)

    def __repr__(self):
        return (
            "Pose2D(x="
            + str(self.x)
            + ","
            + str(self.y)
            + ","
            + str(self.heading)
            + ")"
        )


# -------------------------  Consts
# Cardinal movements
CARDINAL_DIRECTIONS = {
    "N": Transform2D(0, -1),
    "S": Transform2D(0, 1),
    "E": Transform2D(1, 0),
    "W": Transform2D(-1, 0),
}

DIAGONAL_DIRECTIONS = {
    "NE": Transform2D(1, -1),
    "SE": Transform2D(1, 1),
    "SW": Transform2D(-1, 1),
    "NW": Transform2D(-1, -1),
}


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
        return (error + range / 2) % range - range / 2

    def setSetpoint(self, newSetpoint: float):
        self.setpoint: float = newSetpoint
        self.integral: float = 0.0
        self.lastError: float = 0.0

    def getSetpoint(self) -> float:
        return self.setpoint

    def update(self, measured: float, setpoint: float = None) -> float:
        dt: float = self.brain.timer.system() - self.lastTime
        self.lastTime: float = self.brain.timer.system()

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


# class AStarSolver:
#     def __init__(
#         self,
#         fieldSize: Tuple[int, int],
#         robotRadius: float = 0.5,
#         obstacles: set[Pose2D] = set(),
#     ):
#         self.fieldSize: float = fieldSize
#         self.robotRadius: float = robotRadius
#         self.lastObstacles: set[Pose2D] = obstacles

#         self.inflatedObstacles: set[Pose2D] = set()
#         self.inflatedObstacles: set[Pose2D] = self.inflateObstacles(self.lastObstacles)

#         self.turn_penalty = 10
#         self.wall_penalty = 4
#         self.move_penalty = 1
#         self.diagonal_penalty = 2.5
#         self.obstacles_penalty = 3

#     def inflateObstacles(self, obstacles: set[Pose2D]) -> set[Pose2D]:
#         pass

#     #     """Inflate obstacles and store them in memory only when obstacles change."""
#     #     # If the obstacles haven't changed, return the cached inflated obstacles
#     #     if obstacles == self.lastObstacles and self.inflatedObstacles:
#     #         return self.inflatedObstacles

#     #     inflated = set()
#     #     inflationRange = ceil(self.robotRadius)

#     #     for obs in obstacles:
#     #         for dx in range(-inflationRange, inflationRange + 1):
#     #             for dy in range(-inflationRange, inflationRange + 1):
#     #                 if abs(dx) + abs(dy) <= inflationRange:
#     #                     inflatedX = obs.x + dx
#     #                     inflatedY = obs.y + dy
#     #                     if (
#     #                         0 <= inflatedX < self.fieldSize[0]
#     #                         and 0 <= inflatedY < self.fieldSize[1]
#     #                     ):
#     #                         inflated.add(Pose2D(inflatedX, inflatedY, obs.heading))

#     #     # Update the cached values
#     #     self.inflatedObstacles = inflated
#     #     self.lastObstacles = obstacles  # Store the current obstacles for comparison
#     #     return inflated

#     # def heuristic(self, a: Pose2D, b: Pose2D) -> float:
#     #     """Calculate Manhattan distance heuristic."""
#     #     distance = abs(a.x - b.x) + abs(a.y - b.y)
#     #     return distance

#     # def simplifyPathWithDiagonals(self, path: list[Pose2D]) -> list[Pose2D]:
#     #     """Simplify the path by removing unnecessary waypoints, including diagonal directions."""
#     #     if len(path) < 3:
#     #         return path

#     #     simplified = [path[0]]

#     #     def direction(a: Pose2D, b: Pose2D) -> Tuple[int, int]:
#     #         dx = b.x - a.x
#     #         dy = b.y - a.y
#     #         return (dx, dy)

#     #     prev_dir = direction(path[0], path[1])

#     #     for i in range(1, len(path) - 1):
#     #         curr_dir = direction(path[i], path[i + 1])
#     #         if curr_dir != prev_dir:
#     #             simplified.append(path[i])
#     #             prev_dir = curr_dir

#     #     simplified.append(path[-1])
#     #     return simplified

#     # def printGrid(self, path: list[Pose2D], start: Pose2D, goal: Pose2D) -> None:
#     #     pass
#     #     # """Print grid with obstacles, path, and start/goal, including axis numbers (double-digit safe)."""
#     #     # width, height = self.fieldSize
#     #     # grid = [[" " for _ in range(width)] for _ in range(height)]

#     #     # def clamp(value, max_val):
#     #     #     return max(0, min(int(value), max_val - 1))

#     #     # # Place inflated obstacles
#     #     # for obs in self.inflatedObstacles:
#     #     #     x = clamp(obs.x, width)
#     #     #     y = clamp(obs.y, height)
#     #     #     grid[y][x] = "#"

#     #     # # Place original obstacles
#     #     # for obs in self.lastObstacles:
#     #     #     x = clamp(obs.x, width)
#     #     #     y = clamp(obs.y, height)
#     #     #     grid[y][x] = "X"

#     #     # # Place path
#     #     # for pose in path:
#     #     #     x = clamp(pose.x, width)
#     #     #     y = clamp(pose.y, height)
#     #     #     grid[y][x] = "*"

#     #     # # Place start and goal
#     #     # sx = clamp(start.x, width)
#     #     # sy = clamp(start.y, height)
#     #     # gx = clamp(goal.x, width)
#     #     # gy = clamp(goal.y, height)
#     #     # grid[sy][sx] = "R"
#     #     # grid[gy][gx] = "G"

#     #     # print("\nGrid:")

#     #     # # Calculate padding based on max column number
#     #     # col_width = len(str(width - 1)) + 1  # +1 for spacing

#     #     # # Print grid rows with Y-axis labels
#     #     # for y in reversed(range(height)):
#     #     #     row_label = f"{y:>{col_width}} |"
#     #     #     row_data = "".join(f"{cell:>{col_width}}" for cell in grid[y])
#     #     #     print(row_label + row_data)

#     #     # # Print separator
#     #     # print(" " * (col_width + 1) + "-" * (width * col_width))

#     #     # # Print X-axis labels
#     #     # col_numbers = " " * (col_width + 1)
#     #     # for x in range(width):
#     #     #     col_numbers += f"{x:>{col_width}}"
#     #     # print(col_numbers)

#     # def distance_to_obstacles(self, point: Pose2D, obstacles: Set[Pose2D]) -> float:
#     #     """Calculate the minimum distance from a point to the nearest obstacle."""
#     #     min_distance = float("inf")
#     #     for obs in obstacles:
#     #         dist = sqrt((point.x - obs.x) ** 2 + (point.y - obs.y) ** 2)
#     #         min_distance = min(min_distance, dist)
#     #     return min_distance

#     # def is_near_boundary(self, position: Pose2D) -> bool:
#     #     return (
#     #         position.x == 0
#     #         or position.x == self.fieldSize[0] - 1
#     #         or position.y == 0
#     #         or position.y == self.fieldSize[1] - 1
#     #     )

#     # def a_star_pose2d(
#     #     self,
#     #     start: Pose2D,
#     #     goal: Pose2D,
#     #     obstacles: Set[Pose2D] = None,
#     # ):
#     #     """A* algorithm to find the shortest path with penalties for turning, wall proximity, and obstacle proximity."""
#     #     obstacles = obstacles or self.inflatedObstacles

#     #     open_set = []
#     #     heapq.heappush(open_set, (0, start, None))  # (f, position, last_direction)

#     #     came_from = {}
#     #     g_score = {(start, None): 0}

#     #     while open_set:
#     #         _, current, last_dir = heapq.heappop(open_set)

#     #         if current == goal:
#     #             path = [current]
#     #             key = (current, last_dir)
#     #             while key in came_from:
#     #                 key = came_from[key]
#     #                 path.append(key[0])

#     #             # Print the grid and path
#     #             self.printGrid(path, start, goal)

#     #             # Optimize path by simplifying direction changes
#     #             path = self.simplifyPathWithDiagonals(path)

#     #             return path[::-1]

#     #         for direction, delta in {
#     #             **CARDINAL_DIRECTIONS,
#     #             **DIAGONAL_DIRECTIONS,
#     #         }.items():
#     #             neighbor = current + delta

#     #             if not (
#     #                 0 <= neighbor.x < self.fieldSize[0]
#     #                 and 0 <= neighbor.y < self.fieldSize[1]
#     #             ):
#     #                 continue
#     #             if neighbor in obstacles:
#     #                 continue

#     #             # Apply penalties:
#     #             move_cost = self.move_penalty

#     #             # Extra cost for diagonal movement
#     #             if direction in DIAGONAL_DIRECTIONS:
#     #                 move_cost += self.diagonal_penalty

#     #             # Large penalty for turning to encourage smoother motion
#     #             if last_dir and last_dir != direction:
#     #                 move_cost += self.turn_penalty

#     #             # Smaller penalty for proximity to walls
#     #             if self.is_near_boundary(neighbor):
#     #                 move_cost += self.wall_penalty

#     #             # Penalty for being too close to obstacles
#     #             if self.distance_to_obstacles(neighbor, obstacles) < 2:
#     #                 move_cost += self.obstacles_penalty

#     #             tentative_g = g_score[(current, last_dir)] + move_cost
#     #             neighbor_key = (neighbor, direction)

#     #             if neighbor_key not in g_score or tentative_g < g_score[neighbor_key]:
#     #                 came_from[neighbor_key] = (current, last_dir)
#     #                 g_score[neighbor_key] = tentative_g
#     #                 f_score = tentative_g + self.heuristic(neighbor, goal)
#     #                 heapq.heappush(open_set, (f_score, neighbor, direction))

#     #     return None


class VisionFruit:
    def __init__(
        self,
        brain: Brain,
        tagMap: dict[int, Pose2D],
    ):
        # For logging
        self.brain: Brain = brain

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
        # vision config
        self.aiVision = AiVision(
            Ports.PORT14,
            self.greenFruit,
            self.orangeFruit,
            self.yellowFruit,
            AiVision.ALL_TAGS,
        )

        # arrays of fruit
        self.fruitObjects: dict[str, list[Fruit]] = {}

        self.strategy: list[str] = []
        self.strategyColors: list[str] = []

        self.largeFruitRatio = 1.1  # guess
        self.smallFruitRatio = 0.5  # guess
        self.fruitSizeTolerance = 0.05  # guess

        # ----------- April Tags
        self.tagWidth: float = 10  # guess
        self.cameraToRobot = Transform2D(-20, 10, 0)  # guess

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
        # Estimate distance using width of tag and focal length
        # depth = (real_width * focal_length) / apparent_width
        depth = (self.tagWidth * self.degPerPixelWidth) / width

        # Convert image-plane x, y into real-world x, y at the estimated depth
        # Assuming camera is centered at (0, 0)
        scale = depth / self.degPerPixelWidth
        realX = x * scale

        return Transform2D(depth, realX, angleDeg)

    def captureTags(self) -> list[dict]:
        tags = self.aiVision.take_snapshot(AiVision.ALL_TAGS)

        formattedTags: list[dict] = []
        brain.screen.print_at(formattedTags, x=40, y=60)
        if tags:
            for tag in tags:
                tempDict = {}

                tempDict["id"] = tag.id
                tempDict["transform"] = self.cameraPlaneToTransform2d(
                    tag.centerX, tag.width, tag.angle
                )
                tempDict["vision_score"] = tag.score
                formattedTags.append(tempDict)
        return formattedTags

    def getRobotPoseWithTags(self) -> Pose2D:
        tags = self.captureTags()
        # --- Process detections ---
        weightedX = 0.0
        weightedY = 0.0
        weightedHeadings = []
        weights = []

        for tag in tags:
            tagID: int = tag["id"]
            relTransform2d: Transform2D = tag["transform"]
            visionScore: float = tag["vision_score"]

            if tagID not in self.tagMap or visionScore <= 0:
                continue

            tagPose = self.tagMap[tagID]

            tagToCamera = relTransform2d.inverse()
            tagToRobot = tagToCamera + self.cameraToRobot
            robotPose: Pose2D = tagPose + tagToRobot

            # Distance-based score
            distance = sqrt(relTransform2d.dX**2 + relTransform2d.dY**2)
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


class Odometry:
    def __init__(self, brain, visionFruit: VisionFruit):
        # For logging
        self.brain: Brain = brain

        self.visionFruit: VisionFruit = visionFruit
        self.pose: Pose2D = Pose2D(0, 0, 0)

    def update(self):
        self.pose = self.visionFruit.getRobotPoseWithTags()

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

        self.controlerIndex = 0

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
        #     self.controlerIndex = (self.controlerIndex + 1) % len(driveModes)
        #     controller.screen.clear_screen()
        #     controller.screen.print(driveModes[self.controlerIndex])

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
        pass
        """Follow a path from start to goal."""
        # for target in path:
        #     while not self.driveToPosition(target):
        #         pass
        #     print(f"Arrived at {target.x}, {target.y}")

        # print("Path following complete!")

    def pathFindToPosition(self, target: Pose2D, speed: float = 50.0):
        path = self.pathFinding.a_star_pose2d(odometry.getPosition(), target)
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


# --------------- configs and stuff
fruitPickingStrategy = [
    "Large_Green",
    "Large_Yellow",
    "Small_Green",
    "Large_Orange",
    "Small_Yellow",
    "Small_Orange",
]

# Create a field size (width, height) for the grid (2 in grid)
fieldSize = (58, 39)  # 146x98 / 2.5


# Define the obstacles as Pose2D objects
def generatePoseList(
    coords: list[tuple[int, int]], heading: float = 0.0
) -> set[Pose2D]:
    return {Pose2D(x, y, heading) for x, y in coords}


def generatePoseRec(
    x_range: range, y_range: range, heading: float = 0.0
) -> set[Pose2D]:
    return {Pose2D(x, y, heading) for x in x_range for y in y_range}


obstacles: set[Pose2D] = set()
# obstacles |= generatePoseList(
#     [
#         (16, 12),
#         (16, 19),
#         (16, 26),
#         (31, 12),
#         (31, 19),
#         (31, 26),
#         (45, 12),
#         (45, 19),
#         (45, 26),
#     ]
# )
# obstacles |= generatePoseRec(range(20, 28), range(17, 22))
# obstacles |= generatePoseRec(range(35, 42), range(17, 22))

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

# ---------------------------   RUN CODE HERE
while True:
    pass
    # odometry.update()
    # brain.screen.print_at(odometry.getPosition(), x=40, y=40)
    # hDrive.driveController(controller)
    # arm.pidControl(controller)

    # hDrive.pathFindToPosition(Pose2D(44, 23))
