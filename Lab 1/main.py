# basic Hello world program

from vex import *
from RBEDrivetrin import *
brain = Brain()

brain.screen.print("Hellow, World")

drivetrin = RBEDrivetrin()

drivetrin.set_gear_ratio(90)
