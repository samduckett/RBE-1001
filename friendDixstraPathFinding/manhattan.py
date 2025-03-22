class Manhattan:

    def __init__(self, width, height):

        # initiate path object with grid size (grid bounds not currently used)
        self.width = width
        self.height = height
        # starting position is (0, 0)
        self.current = (0, 0)

    def compute_path(self, finish):
        
        # calculate x and y points in path
        path = []
        x1, y1 = self.current
        x2, y2 = finish
        # append points to path until start = finish
        while x1 != x2:
            if x1 < x2:
                x1 += 1
            elif x1 > x2:
                x1 -= 1
            # append added point to path
            current = (x1, y1)
            path.append(current)
        # append points to path until start = finish
        while y1 != y2:
            if y1 < y2:
                y1 += 1
            elif y1 > y2:
                y1 -= 1
            # append added point to path
            current = (x1, y1)
            path.append(current)
        # return paths
        self.current = finish
        return path

#Print Virtual Path for Debug (uncomment if needed)

grid = Manhattan(8, 6)
stops = [(5, 3), (3, 1), (2, 0), (0, 0)] 
print('initializing')
for s in stops:
    path = grid.compute_path(s)
    print("Planned Path:", path)


