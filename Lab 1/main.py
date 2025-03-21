# basic Hello world program

from vex import *
from RBEDrivetrain import *

# initialize the brian
brain = Brain()

# initialize the left and right motor
left_motor = Motor(Ports.PORT1, 18_1, True)
right_motor = Motor(Ports.PORT10, 18_1, False)

brain.screen.print("Hellow, World")

drivetrain = RBEDrivetrain(left_motor, right_motor, 5, 4, 11.0)
