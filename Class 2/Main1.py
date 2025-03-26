# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       nbertozzi                                                    #
# 	Created:      10/19/2023, 6:27:34 PM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain = Brain()


left_motor = Motor(Ports.PORT1, 18_1, True)
right_motor = Motor(Ports.PORT10, 18_1, False)

wheelDiameter = 4.0  # wheel diameter in inches
wheelCircumference = 3.14 * wheelDiameter
gearRatio = 5.0
motorDegreesPerInch = 360.0 * gearRatio / wheelCircumference
Kp = 0.5  # left-right count error proportionality constant

brain.screen.print("Hello V5")  # say hello to the brain


def driveStraightVoltage(
    motorVoltage, distance
):  # provide a voltage (effort) from 0 to 12 volts
    degreeCount = (
        distance * motorDegreesPerInch
    )  # calculate total degrees motor must turn
    left_motor.reset_position()  # reset motor positions to zero
    right_motor.reset_position()
    print(
        "left motor position: ", left_motor.position()
    )  # check to see if motor positions have been reset to zero
    print("right motor position: ", right_motor.position())
    wait(5000)
    while left_motor.position() < degreeCount:
        leftRightDegreeDiff = (
            left_motor.position() - right_motor.position()
        )  # calculate position error
        left_motor.spin(
            FORWARD, motorVoltage * leftRightDegreeDiff * Kp, VOLT
        )  # adjust effort to correct for error
        right_motor.spin(FORWARD, motorVoltage * leftRightDegreeDiff * Kp, VOLT)
    left_motor.stop(BRAKE)  # Brake once you reach the desired position
    right_motor.stop(BRAKE)


setMotorVoltage = 6.0  # set motor effort
distanceInches = 11  # set distance

driveStraightVoltage(
    setMotorVoltage, distanceInches
)  # Pass the desired voltage and distance to the function
print(
    "final left motor position: ", left_motor.position()
)  # Print the final motor positions
print("final right motor position: ", right_motor.position())
