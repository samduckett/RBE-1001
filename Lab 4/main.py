# region VEXcode Generated Robot Configuration
from vex import *
import math

# Brain should be defined by default
brain = Brain()

leftMotor = Motor(Ports.PORT1, 18_1, False)
rightMotor = Motor(Ports.PORT10, 18_1, True)


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

    


def driveForward(speed):
    leftMotor.spin(FORWARD, speed, RPM)
    rightMotor.spin(FORWARD, speed, RPM)


def spin(speed):
    leftMotor.spin(FORWARD, speed, RPM)
    rightMotor.spin(FORWARD, speed, RPM)


def DetectObject():

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

    return objects


def part1():
    # inches away from fruit that the robot will get to
    goalDistanceAway = 5
    kPDistanceAway = 50

    while True:

        objects = AiVision.take_snapshot(greenFruit)

        if objects:
            fruitDistance = objectSize(objects[0].width)

            # to close negative, to far positive
            error = fruitDistance - goalDistanceAway

            speed = error * kPDistanceAway

            # max speed
            maxSpeed = 170
            if speed > maxSpeed:
                speed = maxSpeed
            elif speed < -maxSpeed:
                speed = -maxSpeed

            driveForward(speed)

        wait(10)


while True:
    objectSize()
