# region VEXcode Generated Robot Configuration
from vex import *

brain = Brain()

# Robot configuration code
# AI Vision Color Descriptions
# AI Vision Code Descriptions
ai_vision_1 = AiVision(Ports.PORT1, AiVision.ALL_TAGS)

while True:
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)
    # Take a snapshot of all AprilTags.
    snapshot_objects = ai_vision_1.take_snapshot(AiVision.ALL_TAGS)
    # Check to see if an AprilTag exists in this snapshot.
    if snapshot_objects and len(snapshot_objects) > 0:
        # Determine which AprilTag is detected.
        if snapshot_objects[0].id == 1:
            # Conditional based on finding TagID #1.
            brain.screen.print("Found TagID 1")
        elif snapshot_objects[0].id == 2:
            # Conditional based on finding TagID #2.
            brain.screen.print("Found TagID 2")
        else:
            # Else condition will print any other TagID found.
            brain.screen.print("Found TagID")
            brain.screen.next_row()
            brain.screen.print("TagID: ")
            brain.screen.print(snapshot_objects[0].id)
    else:
        # If no AprilTags are found in this snapshot, display a message.
        brain.screen.print("No AprilTags")
    # Wait some time and restart loop.
    wait(0.3, SECONDS)
    wait(5, MSEC)
