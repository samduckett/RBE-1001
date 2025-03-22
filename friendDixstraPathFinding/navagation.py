from linefollow import Linefollow
from dijkstra import Dijkstra


class Navigation:

    def __init__(self, start):

        # initiate a navigation object
        self.current = start
        self.heading = 0 # assumuing facing north at start
        self.robot = Linefollow()

    def get_heading(self, next_intersection):
        
        # calculate heading in x
        heading_x = next_intersection[0] - self.current[0]
        # calculate heading in y
        heading_y = next_intersection[1] - self.current[1]

        if heading_x > 0:
            target_heading = 0 # north
        elif heading_x < 0:
            target_heading = 180 # south
        elif heading_y > 0:
            target_heading =  90 # east
        elif heading_y < 0:
            target_heading = 270 # west
        return target_heading
        
    def turn_to_heading(self, target_heading):

        # calculate degrees to turn (must be within 360)
        turn_heading = (target_heading - self.heading) %360
        self.heading = target_heading
        # print all heading values for debug
        print("Start Heading: ", self.heading)
        print("Target Heading: ", target_heading)
        print("Turn Heading: ", turn_heading)

        # turn left if target is west of robot
        if turn_heading == 270 or turn_heading == -90:
            self.robot.turn_counterclockwise()
        # turn right if target is east of robot
        elif turn_heading == 90 or turn_heading == -270:
            self.robot.turn_clockwise()
        # turn 180 if target is behind robot
        elif turn_heading == 180 or turn_heading == -180:
            self.robot.turn_180()


    def navigate(self, path):

        # loop for all intersections in paths
        for next_intersection in path:
            # print desired heading for debug
            print('driving from', self.current, "to", next_intersection)
            print("turing to heading", self.get_heading(next_intersection))
            self.turn_to_heading(self.get_heading(next_intersection))
            # print next point in path for debug
            print("goint to point", next_intersection)
            self.robot.follow_to_intersection()
            # reset current position
            self.current = next_intersection
            self.robot.stop_robot()
        print("path complete")
        
