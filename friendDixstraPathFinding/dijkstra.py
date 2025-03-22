from node import Node

class Dijkstra:


    def __init__(self, rows, cols, blocked):
        
        # initiate a dijkstra object with grid size and blocked intersections
        self.rows = rows
        self.cols = cols
        self.current = (0, 0)
        self.grid = {} # creates a dictionary of nodes
        
        for r in range(rows):
            for c in range(cols):
                self.grid[(r, c)] = Node(r, c)

        for b in blocked:
            self.grid[b].blocked = True

        for loc in self.grid:
            n = self.grid[loc]
            if n.row < self.rows - 1:
                n.neighbors.append(self.grid[(n.row + 1, n.col)])
            if n.row > 0:
                n.neighbors.append(self.grid[(n.row - 1, n.col)])
            if n.col < self.cols - 1:
                n.neighbors.append(self.grid[(n.row, n.col + 1)])
            if n.col > 0:
                n.neighbors.append(self.grid[(n.row, n.col - 1)])

    def get_node(self, loc):

        # return a row and column given a nodes location
        row, col = loc
        return self.grid[(row, col)]

    def make_queue(self):

        # creates a queue setting all distances to 100 and the current to previous using dictionary
        for n in self.grid:
            self.grid[n].dist = 100
        self.grid[self.current].dist = 0
        self.grid[self.current].previous = None

    
    def compute_path(self, dest):

        # computes a path from current position to a given destination using the queue
        self.make_queue()
        start_node = self.get_node(self.current)
        queue = [start_node]

        while queue:
            current_node = queue.pop(0)

            if current_node.blocked:
                continue

            for neighbor in current_node.neighbors:
                if neighbor.blocked:
                    continue
                new_dist = current_node.dist + 1
                if new_dist < neighbor.dist:
                    neighbor.dist = new_dist
                    neighbor.previous = current_node
                    if neighbor not in queue:
                        queue.append(neighbor)
        
        path = []
        dest_node = self.get_node(dest)
        while dest_node:
            path.append((dest_node.row, dest_node.col))
            dest_node = dest_node.previous
        path.reverse()
        self.current = dest   
        return path

    def print_grid(self):

        #print columns
        print("\n ", end='')
        for col in range(0, self.cols):
            print(f'{col:3}', end='')
        print('\n' + '-' * (self.cols * 3 + 2))
        #print rows
        for row in range(0, self.rows):
            print(f'{row:2}|', end='')
            for col in range(0, self.cols):
                n = self.get_node((row, col))
                if n.blocked:
                    print(f'   X', end='')
                else:
                    d = n.dist
                    print(f'{d:4}', end='')
            print()

# PRINT FOR DEBUG
# stops = [(4, 3), (3, 1), (2, 3), (0, 0)]
# blocked = [(4, 0), (3, 3), (2, 2), (0, 3)]
# grid = Dijkstra(5, 10, blocked_nodes)
# for s in stops:
    # path = grid.compute_path(s)
    # print('Planned Path', path)
    # grid.print_grid()
    # print('         ')
