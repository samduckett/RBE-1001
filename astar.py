from math import *
import heapq


class Transform2D:
    def __init__(self, dX, dY, dTheta=0):
        self.dX: float = dX
        self.dY: float = dY
        self.dTheta: float = dTheta

    def inverse(self):
        angle_rad = radians(-self.dTheta)
        new_dx = -cos(angle_rad) * self.dX - sin(angle_rad) * self.dY
        new_dy = sin(angle_rad) * self.dX - cos(angle_rad) * self.dY
        return Transform2D(new_dx, new_dy, -self.dTheta)


class Pose2D:
    def __init__(self, x, y, heading=0):
        self.x: float = x
        self.y: float = y
        self.heading: float = heading

    def __add__(self, transform2D: Transform2D):  # Apply transform
        rad = radians(self.heading)
        newX = self.x + cos(rad) * transform2D.dX - sin(rad) * transform2D.dY
        newY = self.y + sin(rad) * transform2D.dX + cos(rad) * transform2D.dY
        newHeading = (self.heading + transform2D.dTheta) % 360
        return Pose2D(newX, newY, newHeading)

    def __eq__(self, other):
        return (
            isinstance(other, Pose2D)
            and self.x == other.x
            and self.y == other.y
            and self.heading == other.heading
        )

    def __hash__(self):
        return hash((self.x, self.y, self.heading))

    def __lt__(self, other):
        if not isinstance(other, Pose2D):
            return NotImplemented
        return (self.x, self.y, self.heading) < (other.x, other.y, other.heading)

    def __repr__(self):
        return (
            "Pose2D(x="
            + str(self.x)
            + ","
            + str(self.y)
            + ","
            + str(self.heading)
            + ")"
        )


# -------------------------  Consts
# Cardinal movements
CARDINAL_DIRECTIONS = {
    "N": Transform2D(0, -1),
    "S": Transform2D(0, 1),
    "E": Transform2D(1, 0),
    "W": Transform2D(-1, 0),
}

DIAGONAL_DIRECTIONS = {
    "NE": Transform2D(1, -1),
    "SE": Transform2D(1, 1),
    "SW": Transform2D(-1, 1),
    "NW": Transform2D(-1, -1),
}


class AStarSolver:
    def __init__(
        self,
        fieldSize: tuple[int, int],
        robotRadius: float = 0.5,
        obstacles: set[Pose2D] = set(),
    ):
        self.fieldSize: float = fieldSize
        self.robotRadius: float = robotRadius
        self.lastObstacles: set[Pose2D] = obstacles

        self.inflatedObstacles: set[Pose2D] = set()
        self.inflatedObstacles: set[Pose2D] = self.inflateObstacles(self.lastObstacles)

        self.turn_penalty = 10
        self.wall_penalty = 4
        self.move_penalty = 1
        self.diagonal_penalty = 2.5
        self.obstacles_penalty = 3

    def inflateObstacles(self, obstacles: set[Pose2D]) -> set[Pose2D]:
        pass

        """Inflate obstacles and store them in memory only when obstacles change."""
        # If the obstacles haven't changed, return the cached inflated obstacles
        if obstacles == self.lastObstacles and self.inflatedObstacles:
            return self.inflatedObstacles

        inflated = set()
        inflationRange = ceil(self.robotRadius)

        for obs in obstacles:
            for dx in range(-inflationRange, inflationRange + 1):
                for dy in range(-inflationRange, inflationRange + 1):
                    if abs(dx) + abs(dy) <= inflationRange:
                        inflatedX = obs.x + dx
                        inflatedY = obs.y + dy
                        if (
                            0 <= inflatedX < self.fieldSize[0]
                            and 0 <= inflatedY < self.fieldSize[1]
                        ):
                            inflated.add(Pose2D(inflatedX, inflatedY, obs.heading))

        # Update the cached values
        self.inflatedObstacles = inflated
        self.lastObstacles = obstacles  # Store the current obstacles for comparison
        return inflated

    def heuristic(self, a: Pose2D, b: Pose2D) -> float:
        """Calculate Manhattan distance heuristic."""
        distance = abs(a.x - b.x) + abs(a.y - b.y)
        return distance

    def simplifyPathWithDiagonals(self, path: list[Pose2D]) -> list[Pose2D]:
        """Simplify the path by removing unnecessary waypoints, including diagonal directions."""
        if len(path) < 3:
            return path

        simplified = [path[0]]

        def direction(a: Pose2D, b: Pose2D) -> tuple[int, int]:
            dx = b.x - a.x
            dy = b.y - a.y
            return (dx, dy)

        prev_dir = direction(path[0], path[1])

        for i in range(1, len(path) - 1):
            curr_dir = direction(path[i], path[i + 1])
            if curr_dir != prev_dir:
                simplified.append(path[i])
                prev_dir = curr_dir

        simplified.append(path[-1])
        return simplified

    def printGrid(self, path: list[Pose2D], start: Pose2D, goal: Pose2D) -> None:
        """Print grid with obstacles, path, and start/goal, including axis numbers (double-digit safe)."""
        width, height = self.fieldSize
        grid = [[" " for _ in range(width)] for _ in range(height)]

        def clamp(value, max_val):
            return max(0, min(int(value), max_val - 1))

        # Place inflated obstacles
        for obs in self.inflatedObstacles:
            x = clamp(obs.x, width)
            y = clamp(obs.y, height)
            grid[y][x] = "#"

        # Place original obstacles
        for obs in self.lastObstacles:
            x = clamp(obs.x, width)
            y = clamp(obs.y, height)
            grid[y][x] = "X"

        # Place path
        for pose in path:
            x = clamp(pose.x, width)
            y = clamp(pose.y, height)
            grid[y][x] = "*"

        # Place start and goal
        sx = clamp(start.x, width)
        sy = clamp(start.y, height)
        gx = clamp(goal.x, width)
        gy = clamp(goal.y, height)
        grid[sy][sx] = "R"
        grid[gy][gx] = "G"

        print("\nGrid:")

        # Calculate padding based on max column number
        col_width = len(str(width - 1)) + 1  # +1 for spacing

        # Print grid rows with Y-axis labels
        for y in reversed(range(height)):
            row_label = f"{y:>{col_width}} |"
            row_data = "".join(f"{cell:>{col_width}}" for cell in grid[y])
            print(row_label + row_data)

        # Print separator
        print(" " * (col_width + 1) + "-" * (width * col_width))

        # Print X-axis labels
        col_numbers = " " * (col_width + 1)
        for x in range(width):
            col_numbers += f"{x:>{col_width}}"
        print(col_numbers)

    def distance_to_obstacles(self, point: Pose2D, obstacles: set[Pose2D]) -> float:
        """Calculate the minimum distance from a point to the nearest obstacle."""
        min_distance = float("inf")
        for obs in obstacles:
            dist = sqrt((point.x - obs.x) ** 2 + (point.y - obs.y) ** 2)
            min_distance = min(min_distance, dist)
        return min_distance

    def is_near_boundary(self, position: Pose2D) -> bool:
        return (
            position.x == 0
            or position.x == self.fieldSize[0] - 1
            or position.y == 0
            or position.y == self.fieldSize[1] - 1
        )

    def a_star_pose2d(
        self,
        start: Pose2D,
        goal: Pose2D,
        obstacles: set[Pose2D] = None,
    ):
        """A* algorithm to find the shortest path with penalties for turning, wall proximity, and obstacle proximity."""
        obstacles = obstacles or self.inflatedObstacles

        open_set = []
        heapq.heappush(open_set, (0, start, None))  # (f, position, last_direction)

        came_from = {}
        g_score = {(start, None): 0}

        while open_set:
            _, current, last_dir = heapq.heappop(open_set)

            if current == goal:
                path = [current]
                key = (current, last_dir)
                while key in came_from:
                    key = came_from[key]
                    path.append(key[0])

                # Print the grid and path
                self.printGrid(path, start, goal)

                # Optimize path by simplifying direction changes
                path = self.simplifyPathWithDiagonals(path)

                return path[::-1]

            for direction, delta in {
                **CARDINAL_DIRECTIONS,
                **DIAGONAL_DIRECTIONS,
            }.items():
                neighbor = current + delta

                if not (
                    0 <= neighbor.x < self.fieldSize[0]
                    and 0 <= neighbor.y < self.fieldSize[1]
                ):
                    continue
                if neighbor in obstacles:
                    continue

                # Apply penalties:
                move_cost = self.move_penalty

                # Extra cost for diagonal movement
                if direction in DIAGONAL_DIRECTIONS:
                    move_cost += self.diagonal_penalty

                # Large penalty for turning to encourage smoother motion
                if last_dir and last_dir != direction:
                    move_cost += self.turn_penalty

                # Smaller penalty for proximity to walls
                if self.is_near_boundary(neighbor):
                    move_cost += self.wall_penalty

                # Penalty for being too close to obstacles
                if self.distance_to_obstacles(neighbor, obstacles) < 2:
                    move_cost += self.obstacles_penalty

                tentative_g = g_score[(current, last_dir)] + move_cost
                neighbor_key = (neighbor, direction)

                if neighbor_key not in g_score or tentative_g < g_score[neighbor_key]:
                    came_from[neighbor_key] = (current, last_dir)
                    g_score[neighbor_key] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor, direction))

        return None
