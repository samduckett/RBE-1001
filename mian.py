from vex import *


class HDrive:
    def __init__(
        self,
    ):
        pass


class Vision:
    def __init__(
        self,
    ):
        # fruit color configs
        greenFruit = Colordesc(1, 12, 167, 71, 20, 0.2)
        orangeFruit = Colordesc(2, 222, 66, 67, 10, 0.2)
        yellowFruit = Colordesc(3, 166, 114, 59, 15, 0.2)

        # camera const
        self.camWidth = 320
        self.camHeight = 240
        self.camVertFOV = 68
        self.camHorizFOV = 74
        self.degPerPixelWidth = self.camHorizFOV / self.camWidth
        self.degPerPixelHeight = self.camVertFOV / self.camHeight

        # vision config
        self.aiVision = AiVision(Ports.PORT14, greenFruit, orangeFruit, yellowFruit)

    def objectDist(self, pixelWidth, ObjectWidthIn):
        angularWidth = self.degPerPixelWidth * pixelWidth
        return (ObjectWidthIn * 0.5) / math.tan(math.radians(angularWidth * 0.5))


class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0.0, tolerance=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # setpoint and tolerance
        self.setpoint = setpoint
        self.tolerance = tolerance

        # internal state
        self.last_error = 0.0
        self.integral = 0.0
        self.lastTime = 0

    def set_setpoint(self, newSetpoint):
        """Change the target setpoint and reset I/D"""
        self.setpoint = newSetpoint
        self.integral = 0.0
        self.last_error = 0.0

    def update(self, measured, dt, setpoint=None):
        """
        Compute the PID output.
        """
        if setpoint is not None:
            self.setpoint = setpoint

        error = self.setpoint - measured

        # P term
        P = self.Kp * error

        # I term
        self.integral += error * dt
        I = self.Ki * self.integral

        # D term
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        D = self.Kd * derivative

        # store for next cycle for D
        self.last_error = error

        return P + I + D


# Initial Robot
brain = Brain()

leftFrontMotor = Motor(Ports.PORT1, 18_1, False)

imu = Inertial(Ports.PORT6)

lineLeft = Line(brain.three_wire_port.b)
lineRight = Line(brain.three_wire_port.a)


brain.screen.print("Calibrating")

# calibrate shit here
Timer.clear()
imu.calibrate()


while imu.is_calibrating():
    wait(5)

imu.set_heading(0, DEGREES)


brain.screen.print("Finished Calibrating")

hDrive = HDrive()
