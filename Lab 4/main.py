# region VEXcode Generated Robot Configuration
from vex import *
import math

# Brain should be defined by default
brain = Brain()

greenFruit = Colordesc(1, 15, 202, 113, 18, 0.25)

AiVision = AiVision(Ports.PORT14, greenFruit)

camWidth = 320
camHeight = 240
camVertFOV = 68
camHorizFOV = 74
ObjectWidthIn = 3.5
degPerPixel = camWidth / camHorizFOV


def objectDist(pixelWidth):
    angularWidth = degPerPixel * pixelWidth
    return (ObjectWidthIn*.5) / math.tan(angularWidth * .5)

    


def DetectObject():

    # takes a snapshot and searches for SIG_3_RED_BALL
    # you’ll want to use the signature that you defined above

    objects = AiVision.take_snapshot(greenFruit)

    # print the coordinates of the center of the object

    if objects:
        print(
            "x:",
            AiVision.largest_object().centerX,
            "   y:",
            AiVision.largest_object().centerY,
            "   width:",
            AiVision.largest_object().width,
        )
        brain.screen.print_at("x: ", AiVision.largest_object().centerX, x=50, y=40)
        brain.screen.print_at("   y:", AiVision.largest_object().centerY, x=150, y=40)
        brain.screen.print_at("  width:", AiVision.largest_object().width, x=250, y=40)
        wait(90)
        brain.screen.clear_screen()


while True:
    DetectObject()
    wait(10)
