# endregion VEXcode Generated Robot Configuration
from vex import *


class XDrive:

    def __init__(
        self,
        LFMotor: Motor,
        LBMotor: Motor,
        RFMotor: Motor,
        RBMotor: Motor,
        MaxVelocity: float,
    ):
        self.LFMotor = LFMotor
        self.LBMotor = LBMotor
        self.RFMotor = RFMotor
        self.RBMotor = RBMotor

        self.MaxVelocity = MaxVelocity

    def resetMotorRotations(self):
        self.LFMotor.reset_position()
        self.LBMotor.reset_position()
        self.RFMotor.reset_position()
        self.RBMotor.reset_position()

    def setVelocity(
        self, LFVelocity: float, LBVelocity: float, RFVelocity: float, RBVelocity: float
    ):
        self.LFMotor.spin(FORWARD, LFVelocity, RPM)
        self.LBMotor.spin(FORWARD, LBVelocity, RPM)
        self.RFMotor.spin(FORWARD, RFVelocity, RPM)
        self.RBMotor.spin(FORWARD, RBVelocity, RPM)

    def stop(self, brakeType: BrakeType):
        self.LFMotor.stop(brakeType)
        self.LBMotor.stop(brakeType)
        self.RFMotor.stop(brakeType)
        self.RBMotor.stop(brakeType)

    def robotCentric(self, XVelocity, YVelocity, turning):
        # TODO figure out turing units / math
        LF = (YVelocity + XVelocity + turning) / 2
        LB = (YVelocity - XVelocity + turning) / 2
        RF = (YVelocity - XVelocity - turning) / 2
        RB = (YVelocity + XVelocity - turning) / 2

        self.setVelocity(LF, LB, RF, RB)

    def fieldCentric(self, XVelocity, YVelocity, turning):

        magnitude = math.sqrt(pow(XVelocity, 2) + pow(YVelocity, 2))
        theta = math.atan2(XVelocity, YVelocity) + imu.rotation() * 2 * math.pi

        XRobot = magnitude * math.cos(theta)
        YRobot = magnitude * math.sin(theta)

        LF = (YRobot + XRobot + turning) / 2
        LB = (YRobot - XRobot + turning) / 2
        RF = (YRobot - XRobot - turning) / 2
        RB = (YRobot + XRobot - turning) / 2

        self.setVelocity(LF, LB, RF, RB)

    def teleOpDrive(self, driver: Controller):
        # setting axis [-127, 127]
        xAxis = driver.axis4.value()
        yAxis = driver.axis3.value()
        rotationAxis = driver.axis1.value()

        # Dead band [-127, -5] u [ 5, 127]
        xAxis = 0 if abs(xAxis) < 5 else xAxis
        yAxis = 0 if abs(yAxis) < 5 else yAxis
        rotationAxis = 0 if abs(rotationAxis) < 5 else rotationAxis

        # scaling axis [-1, 1]
        xAxis /= 127
        yAxis /= 127
        rotationAxis /= 127

        # power, better controlling at low speeds, non linear scale for control [-1, 1]
        xAxis *= xAxis
        yAxis *= yAxis
        rotationAxis *= rotationAxis

        # multiply by velocity [-MaxVelocity, MaxVelocity]
        XVelocity *= self.MaxVelocity
        YVelocity *= self.MaxVelocity
        turning *= self.MaxVelocity / 10

        # drive field centric
        self.fieldCentric(XVelocity, YVelocity, turning)
        # self.robotCentric(XVelocity, YVelocity, turning)

        if driver.buttonA.pressing():
            imu.reset_rotation()

        if driver.buttonX.pressing():
            self.moveTo(0, 0, 0)

    def moveTo(self, x, y, rotations):
        pass


# X Drive Code

brain = Brain()

# LF LB RF RB
LFMotor = Motor(Ports.PORT1, 18_1, False)
LBMotor = Motor(Ports.PORT2, 18_1, False)
RFMotor = Motor(Ports.PORT10, 18_1, True)
RBMotor = Motor(Ports.PORT11, 18_1, True)

imu = Gyro(brain.three_wire_port.a)

xDrive = XDrive(LFMotor, LBMotor, RFMotor, RBMotor, 500)

driver = Controller()

imu.calibrate()

wait(2000, MSEC)


while True:
    xDrive.teleOpDrive(driver)
