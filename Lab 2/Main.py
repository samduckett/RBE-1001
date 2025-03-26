from vex import *
from Drivetrin import *

# TEST COMMTI FROM BISHOP


# initialize the brian
brain = Brain()

# initialize the left and right motor
left_motor = Motor(Ports.PORT1, 18_1, False)
right_motor = Motor(Ports.PORT10, 18_1, True)

frontRangeFinder = Sonar(brain.three_wire_port.g)
rightRangeFInder = Sonar(brain.three_wire_port.a)

frontRangeFinder.distance(DistanceUnits.IN)  # acquire initial distance values
rightRangeFInder.distance(DistanceUnits.IN)

wait(500)

brain.screen.print("Hello, World")

drivetrain = RBEDrivetrain(left_motor, right_motor, 5, 4, 11.0)

drivetrain.configPIDWallFollow(10, 100, frontRangeFinder, rightRangeFInder)

drivetrain.wallFollowInches(10)

# drive forward while 5 in away from a wall, as fast as possible
# drive forward until robot sees wall 5 in in front of it
# turn with gyroscope

# prints right and front distens to brain
