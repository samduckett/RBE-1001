class Node:


    def __init__(self, row, col):

        # initiate a node object
        # declare necesary variables for nodes
        self.row = row
        self.col = col
        self.dist = 100
        self.previous = None
        self.neighbors = []
        self.blocked = False

