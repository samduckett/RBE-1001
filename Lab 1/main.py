# region VEXcode Generated Robot Configuration
from vex import *
from Drivetrin import *

# initialize the brian
brain = Brain()

# initialize the left and right motor
left_motor = Motor(Ports.PORT1, 18_1, False)
right_motor = Motor(Ports.PORT10, 18_1, True)

brain.screen.print("Hello, World")

drivetrain = RBEDrivetrain(left_motor, right_motor, 5, 4, 11.0)


def polygon(sides: int, inches: float):
    for _ in range(sides):
        drivetrain.driveStraight(inches)
        drivetrain.turnInPlace(1 / sides)


def maze():
    # forward
    # turn left
    # forward
    # forward
    # turn right
    # forward
    # turn right
    # forward
    drivetrain.driveStraight(20)
    drivetrain.turnAroundWheel(1 / 4)
    drivetrain.driveStraight(16.5)
    drivetrain.turnAroundWheel(-1 / 4)
    drivetrain.driveStraight(9)
    drivetrain.turnAroundWheel(-1 / 4)
    drivetrain.driveStraight(5.5)


maze()
