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
brain=Brain()

left_motor = Motor(Ports.PORT1, 18_1, True)
right_motor = Motor(Ports.PORT10, 18_1, False)

wheelDiameter = 4.0
wheelCircumference = 3.14 * wheelDiameter
gearRatio = 5.0
wheelTrack = 11.0 
degreesPerInch = 360.0 / wheelCircumference

# Drive BaseBot straight for 20 inches

distanceInInches = 20.0

left_motor.spin_for(FORWARD, ?, DEGREES, 100, RPM, False)
right_motor.spin_for(FORWARD, ?, DEGREES, 100, RPM, True)

# Turn BaseBot 90 degrees counterclockwise

robotTurnInDegrees = 90.0

left_motor.spin_for(REVERSE, ?, DEGREES, 100, RPM, False)
right_motor.spin_for(FORWARD, ?, DEGREES, 100, RPM, True)




wait(3000)