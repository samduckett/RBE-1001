from vex import *


def colorsFromStrategy(fruitPickingStrategy: list[str]):
    colors = []
    seen = set()
    for col in fruitPickingStrategy:
        color = col.split("_")[1]
        if color not in seen:
            colors.append(color)
            seen.add(color)
    return colors


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

    def g():
        pass


class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0.0, tolerance=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # setpoint and tolerance
        self.setpoint = setpoint
        self.tolerance = tolerance

        # internal state
        self.lastError = 0.0
        self.integral = 0.0
        self.lastTime = 0

    def setSetpoint(self, newSetpoint):
        self.setpoint = newSetpoint
        self.integral = 0.0
        self.lastError = 0.0

    def update(self, measured, setpoint=None):
        dt = Timer.system() - self.lastTime
        self.lastTime = Timer.system()

        if setpoint is not None:
            self.setpoint = setpoint

        error = self.setpoint - measured

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

    def at_goal(self, measured_value):
        return abs(self.setpoint - measured_value) <= self.tolerance


class HDrive:
    def __init__(
        self,
    ):
        pass

    def turn():
        pass

    def drive():
        pass


class Vision:
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
        self.strategyColors = []

        self.largeFruitRatio = 1.1  # guess
        self.smallFruitRatio = 0.5  # guess
        self.fruitSizeTolerance = 0.05  # guess

    def setStrategy(self, strategy: list[str]):
        self.strategy = strategy
        self.strategyColors = colorsFromStrategy(self.strategy)

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

        for index, color in enumerate(self.strategyColors):
            match color.lower():
                case "green":
                    for obj in self.aiVision.take_snapshot(self.greenFruit):
                        tempFruit = self.makeFruit(obj, "green")
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
                        tempFruit = self.makeFruit(obj, "orange")
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
                        tempFruit = self.makeFruit(obj, "yellow")
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
            if self.strategy[index] in self.objects:
                return True
        # need the not not to return a boolean
        return not not self.objects

    def makeFruit(self, obj: AiVisionObject, color: str) -> Fruit:
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

leftFrontMotor = Motor(Ports.PORT1, 18_1, False)
rightFrontMotor = Motor(Ports.PORT2, 18_1, False)
leftBackMotor = Motor(Ports.PORT3, 18_1, False)
rightBackMotor = Motor(Ports.PORT4, 18_1, False)
frontSideMotor = Motor(Ports.PORT5, 18_1, False)
backSideMotor = Motor(Ports.PORT6, 18_1, False)

imu = Inertial(Ports.PORT7)

lineLeft = Line(brain.three_wire_port.a)
lineRight = Line(brain.three_wire_port.b)


brain.screen.print("Calibrating")

# calibrate shit here
# Timer.clear() # UNCOMMENT
imu.calibrate()


while imu.is_calibrating():
    wait(5)

imu.set_heading(0, DEGREES)


brain.screen.print("Finished Calibrating")

# class definitions
hDrive = HDrive()
vision = Vision()

fruitPickingStrategy = [
    "Large_Green",
    "Large_Yellow",
    "Small_Green",
    "Large_Orange",
    "Small_Yellow",
    "Small_Orange",
]

vision.setStrategy(fruitPickingStrategy)

vision.update()
