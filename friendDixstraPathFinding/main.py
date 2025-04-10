class GridNode:
    def __init__(self, row, col):
        self.row: int = row
        self.col: int = col
        self.dist: int = 100
        self.previous: GridNode = None
        self.neighbors: list[GridNode] = []
        self.blocked: bool = False


class Grid:
    def __init__(self, rows: int, cols: int, blocked: list[tuple[int, int]]):
        # initiate a dijkstra object with grid size and blocked intersections
        self.rows: int = rows
        self.cols: int = cols
        self.current: tuple[int, int] = (0, 0)
        self.grid: dict[tuple[int, int], GridNode] = {}  # creates a dictionary of nodes

        for row in range(self.rows):
            for col in range(self.cols):
                self.grid[(row, col)] = GridNode(row, col)

        for block in blocked:
            self.grid[block].blocked = True

        for n in self.grid.values():
            node: GridNode = n
            if node.row < self.rows - 1:
                node.neighbors.append(self.grid[(node.row + 1, node.col)])
            if node.row > 0:
                node.neighbors.append(self.grid[(node.row - 1, node.col)])
            if node.col < self.cols - 1:
                node.neighbors.append(self.grid[(node.row, node.col + 1)])
            if node.col > 0:
                node.neighbors.append(self.grid[(node.row, node.col - 1)])

    def get_node(self, loc: tuple[int, int]):
        return self.grid[loc]

    def compute_path(self, final: tuple[int, int]) -> list[tuple[int, int]]:
        # creates a queue setting all distances to 100 and the current to previous using dictionary
        for node in self.grid:
            self.grid[node].dist = 100
        self.grid[self.current].dist = 0
        self.grid[self.current].previous = None

        startNode: GridNode = self.get_node(self.current)
        queue: list[GridNode] = [startNode]

        # sets distance between nodes and prior node
        while queue:
            currentNode = queue.pop(0)
            if currentNode.blocked:
                continue
            for neighbor in currentNode.neighbors:
                if neighbor.blocked:
                    continue
                newDist = currentNode.dist + 1
                if newDist < neighbor.dist:
                    neighbor.dist = newDist
                    neighbor.previous = currentNode
                    if neighbor not in queue:
                        queue.append(neighbor)
        # makes path end to start
        path = []
        finalNode = self.get_node(final)
        while finalNode:
            path.append((finalNode.row, finalNode.col))
            finalNode = finalNode.previous

        # flips and returns path
        path.reverse()
        self.current = final
        path = self.optimizePath(path)
        return path

    def optimizePath(self, path: list[tuple[int, int]]) -> list[tuple[int, int]]:
        if len(path) < 3:
            return path

        simplified = [path[0]]  # Start with the first point

        i = 1
        while i < len(path) - 1:
            start = simplified[-1]
            curr = path[i]
            next = path[i + 1]

            # Get direction vectors
            dx1, dy1 = curr[0] - start[0], curr[1] - start[1]
            dx2, dy2 = next[0] - curr[0], next[1] - curr[1]

            # Normalize directions
            if dx1 != 0:
                dx1 //= abs(dx1)
            if dy1 != 0:
                dy1 //= abs(dy1)
            if dx2 != 0:
                dx2 //= abs(dx2)
            if dy2 != 0:
                dy2 //= abs(dy2)

            if (dx1, dy1) == (dx2, dy2):
                # Same direction, check how long it goes
                direction = (dx1, dy1)
                run_start = i - 1
                run_end = i + 1
                while run_end + 1 < len(path):
                    next_dir = (
                        path[run_end + 1][0] - path[run_end][0],
                        path[run_end + 1][1] - path[run_end][1],
                    )
                    if next_dir[0] != 0:
                        next_dir = (next_dir[0] // abs(next_dir[0]), next_dir[1])
                    if next_dir[1] != 0:
                        next_dir = (next_dir[0], next_dir[1] // abs(next_dir[1]))
                    if next_dir == direction:
                        run_end += 1
                    else:
                        break

                # Add the endpoint of the run and skip the middle ones
                simplified.append(path[run_end])
                i = run_end + 1
            else:
                simplified.append(curr)
                i += 1

        # Add last point if it wasn't included
        if simplified[-1] != path[-1]:
            simplified.append(path[-1])

        return simplified

    def print_grid(self):
        spacing = 1
        print("  " + " " * (spacing + 1), end="")
        for col in range(self.cols):
            print("\033[4m" + " " * spacing + f"{col}".zfill(2) + "\033[0m", end="")
        for row in range(self.rows):
            print("\n" + f"{row}".zfill(2) + " " * spacing + "|", end="")
            for col in range(self.cols):
                node = self.get_node((row, col))
                print(
                    " " * spacing + ("  " if node.blocked else f"{node.dist}".zfill(2)),
                    end="",
                )
        print()


# set desired stops and blocked nodes
# row col
stops = [(4, 9)]

blocked: list[tuple[int, int]] = [
    (3, 1),
    (3, 3),
    (2, 2),
    (3, 7),
    (2, 8),
    (2, 5),
    (4, 6),
]
# call Dijkstra and set grid size
grid = Grid(5, 10, blocked)
# loop for all stops in list
for stop in stops:
    path = grid.compute_path(stop)
    print("Planned Path", path)
    grid.print_grid()
