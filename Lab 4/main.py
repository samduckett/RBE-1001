# endregion VEXcode Generated Robot Configuration
# region VEXcode Generated Robot Configuration
from vex import *
import math

# Brain should be defined by default
brain = Brain()

leftMotor = Motor(Ports.PORT1, 18_1, False)
rightMotor = Motor(Ports.PORT10, 18_1, True)


greenFruit = Colordesc(1, 12, 167, 71, 20, 0.2)
orangeFruit = Colordesc(2, 222, 66, 67, 10, 0.2)
yellowFruit = Colordesc(3, 166, 114, 59, 15, 0.2)
aiVision = AiVision(Ports.PORT14, greenFruit, orangeFruit, yellowFruit)

camWidth = 320
camHeight = 240
camVertFOV = 68
camHorizFOV = 74
ObjectWidthIn = 3.5
degPerPixel = camHorizFOV / camWidth


def objectDist(pixelWidth):
    angularWidth = degPerPixel * pixelWidth
    return (ObjectWidthIn * 0.5) / math.tan(math.radians(angularWidth * 0.5))


def driveForward(speed):
    leftMotor.spin(FORWARD, speed, RPM)
    rightMotor.spin(FORWARD, speed, RPM)


def spin(speed):
    leftMotor.spin(REVERSE, speed, RPM)
    rightMotor.spin(FORWARD, speed, RPM)


def DetectObject():

    objects = aiVision.take_snapshot(greenFruit)

    # print the coordinates of the center of the object

    if objects:
        while True:
            print(
                "x:",
                aiVision.largest_object().centerX,
                "   y:",
                aiVision.largest_object().centerY,
                "   width:",
                aiVision.largest_object().width,
            )
            brain.screen.print_at("x: ", aiVision.largest_object().centerX, x=50, y=40)
            brain.screen.print_at(
                "   y:", aiVision.largest_object().centerY, x=150, y=40
            )
            brain.screen.print_at(
                "  width:", aiVision.largest_object().width, x=250, y=40
            )
            brain.screen.clear_screen()
            wait(90)
            brain.screen.clear_screen()
            wait(10)


def part1():
    # inches away from fruit that the robot will get to
    goalDistanceAway = 5
    kPDistanceAway = 65

    while True:

        objects = aiVision.take_snapshot(greenFruit)

        if objects:
            fruitDistance = objectDist(objects[0].width)

            # to close negative, to far positive
            error = fruitDistance - goalDistanceAway

            speed = error * kPDistanceAway

            # max speed
            maxSpeed = 170
            if speed > maxSpeed:
                speed = maxSpeed
            elif speed < -maxSpeed:
                speed = -maxSpeed

            brain.screen.print_at("objectDist: ", fruitDistance, x=40, y=50)
            brain.screen.print_at("speed: ", speed, x=40, y=70)
            brain.screen.print_at("objectWidth", objects[0].width, x=40, y=90)

            driveForward(speed)
        else:
            pass
            driveForward(0)
        wait(100)


def part2():

    kPRotation = 0.9

    while True:

        objects = aiVision.take_snapshot((greenFruit, orangeFruit, yellowFruit))

        if objects:
            rot = objects[0].centerX

            # to close negative, to far positive
            error = rot - camWidth / 2

            speed = error * kPRotation

            # max speed
            maxSpeed = 170
            if speed > maxSpeed:
                speed = maxSpeed
            elif speed < -maxSpeed:
                speed = -maxSpeed

            brain.screen.print_at("objectDist: ", rot, x=40, y=50)
            brain.screen.print_at("speed: ", speed, x=40, y=70)
            brain.screen.print_at("objectWidth", objects[0].width, x=40, y=90)

            spin(speed)
        else:
            driveForward(0)
        wait(100)


def part3():

    kPRotation = 0.9
    goalDistanceAway = 10
    kPDistanceAway = 65

    while True:
        objects = []
        for o in aiVision.take_snapshot(greenFruit):
            objects.append(o)
        for o in aiVision.take_snapshot(orangeFruit):
            objects.append(o)
        for o in aiVision.take_snapshot(yellowFruit):
            objects.append(o)

        if objects:
            rot = objects[0].centerX
            fruitDistance = objectDist(objects[0].width)

            errorRot = rot - camWidth / 2
            # to close negative, to far positive
            errorDit = fruitDistance - goalDistanceAway

            speedFW = errorDit * kPDistanceAway
            speedRot = errorRot * kPRotation
            # max speed
            maxSpeed = 170
            if speedFW > maxSpeed:
                speedFW = maxSpeed
            elif speedFW < -maxSpeed:
                speedFW = -maxSpeed

            maxSpeedRot = 170
            if speedRot > maxSpeedRot:
                speedRot = maxSpeedRot
            elif speedRot < -maxSpeedRot:
                speedRot = -maxSpeedRot

            brain.screen.print_at("dist: ", fruitDistance, x=40, y=30)
            brain.screen.print_at("rot: ", rot, x=40, y=50)
            brain.screen.print_at("speed: ", speedFW, x=40, y=70)
            brain.screen.print_at("objectWidth", objects[0].width, x=40, y=90)

            rightMotor.spin(FORWARD, speedFW + speedRot, RPM)
            leftMotor.spin(FORWARD, speedFW - speedRot, RPM)
        else:
            driveForward(0)
        wait(100)


part3()
