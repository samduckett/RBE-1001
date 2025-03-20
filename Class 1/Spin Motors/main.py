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
arm_motor = Motor(Ports.PORT8, 18_1, True)

brain.screen.print("Hello V5")

left_motor.spin_for(FORWARD, 2, TURNS, 100, RPM, False)
right_motor.spin_for(FORWARD, 2, TURNS, 100, RPM, True)
wait(3000)
arm_motor.spin_for(REVERSE, .5, TURNS, 100, RPM)


        
