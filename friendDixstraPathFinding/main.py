from dijkstra import Dijkstra
from navagation import Navigation
from linefollow import Linefollow

# set desired stops and blocked nodes
stops = [(4, 3), (3, 1), (2, 3), (0, 0)]
blocked = [(4, 0), (3, 3), (2, 2), (0, 3)]
# call Dijkstra and set grid size
grid = Dijkstra(5, 10, blocked)
# call navagation and set starting position
nav = Navigation((0, 0))
print('initializing')
# loop for all stops in list
for s in stops:
    path = grid.compute_path(s)[1:]
    print('Planned Path', path)
    grid.print_grid()
    print(' ')
    nav.navigate(path)
print('completed')

