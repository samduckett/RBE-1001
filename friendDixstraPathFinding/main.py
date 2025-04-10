class GridNode:
    def __init__(self, row, col):
        self.row: int = row
        self.col: int = col
        self.dist: int = 100
        self.previous: GridNode = None
        self.neighbors: list[GridNode] = []
        self.blocked: bool = False


class Grid:
    def __init__(self, rows: int, cols: int, blocked: list[tuple[int]]):
        # initiate a dijkstra object with grid size and blocked intersections
        self.rows: int = rows
        self.cols: int = cols
        self.current: tuple[int] = (0, 0)
        self.grid: dict[tuple[int], GridNode] = {}  # creates a dictionary of nodes

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

    def get_node(self, loc: tuple[int]):
        return self.grid[loc]

    def compute_path(self, final):
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
        return path

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
stops = [(4, 3), (0, 2)]
blocked = [(4, 0), (3, 3), (2, 2), (0, 3)]
# call Dijkstra and set grid size
grid = Grid(20, 40, blocked)
# loop for all stops in list
for stop in stops:
    path = grid.compute_path(stop)
    print("Planned Path", path)
    grid.print_grid()
