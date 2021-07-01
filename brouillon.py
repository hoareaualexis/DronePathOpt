# from sympy import *
# import matplotlib.pyplot as plt
# import numpy as np
# poly = Polygon([Point(0, 5), Point(1, 1), Point(3, 0), Point(7, 2), Point(7, 6), Point(2, 7)])
# point = Point(5.5, 2.5)
# poly.contains(point)

# from sympy import Point, Line, Segment, Polygon
# p1, p2, p3 = Point(0, 0), Point(1, 1), Point(7, 7)
# l1 = Line(p1, p2)
# p1, p2, p3, p4, p5 = map(Point, [(0, 0), (1, 0), (5, 1), (0, 1), (3, 0)])
# poly = Polygon(p1, p2, p3, p4)
# point = Point(5.5, 2.5)
#
# print(poly.contains(point))
# print(l1.intersection(p3))
# print(poly)
# print(poly.area)
# print(poly.angles[p1])
# print(poly.centroid)

# from sympy import Polygon, Line, Point, convex_hull
# a, b = 20, 10
# p1, p2, p3, p4 = [(0, b), (0, 0), (a, 0), (a, b)]
# rectangle = Polygon(p1, p2, p3, p4)
# t = rectangle.cut_section(Line((0, 5), slope=0))
# print(rectangle)
# print(t)
# points = [(1, 1), (1, 2), (3, 1), (-5, 2), (15, 4)]
# print(convex_hull(*points))
# upper_segment, lower_segment = t
# upper_segment.area
#
# upper_segment.centroid
#
# lower_segment.centroid
from sympy import Point, Polygon, Ray, Segment, Line
from collections import deque
import numpy as np
from matplotlib.path import Path
import math

def find_paths_bfs(graph, start, end):

    queue = deque()
    queue.append((start, [start]))

    while queue:
        node, path = queue.popleft()
        adjacent_nodes = [n for n in graph[node] if n not in path]
        for adjacent_node in adjacent_nodes:
            if adjacent_node == end:
                path += [adjacent_node]
            else:
                queue.append((adjacent_node, path + [adjacent_node]))

def recursive_dfs(graph, node, visited=None):

    if visited is None:
        visited = []

    if node not in visited:
        visited.append(node)

    unvisited = [n for n in graph[node] if n not in visited]

    for node in unvisited:
        recursive_dfs(graph, node, visited)

    return visited


def iterative_dfs(graph, node):

    visited = []
    stack = deque()
    stack.append(node)

    while stack:
        node = stack.pop()
        if node not in visited:
            visited.append(node)
            unvisited = [n for n in graph[node] if n not in visited]
            stack.extend(unvisited)
    return visited

def create_field (poly, step= 0.1, direction = 'cw'):

    if direction == 'ccw':

        pathPadding = 1E-8

    elif direction == 'cw':

        pathPadding = -1E-8

    else:

        pathPadding = 0

    x_lst = [x[0] for x in liste]
    y_lst = [y[1] for y in liste]

    xmin = np.min(x_lst) - step
    xmax = np.max(x_lst) + 2*step

    ymin = np.min(y_lst) - step
    ymax = np.max(y_lst) + 2 * step

    x_values = np.arange(xmin, xmax, step)
    y_values = np.arange(ymin, ymax, step)

    nx = len(x_values)
    ny = len(y_values)

    xVec = np.repeat( x_values, ny ) # [1,1,1.....2,2,2......3,3,3.....]
    yVec = np.tile( y_values , nx ) # [1,2,3......1,2,3......1,2,3.....]

    coors = [(x, y) for x, y in zip(xVec, yVec)]

    poly_path = Path(poly)
    mask = poly_path.contains_points(coors, radius=pathPadding)

    import matplotlib.pyplot as plt
    matrix = mask.reshape(nx, ny)
    plt.imshow(matrix)
    plt.show()

lis = [(4, 10), (8, 10), (8, 8), (4, 8), (4, 7), (8, 7), (9, 5), (9, 2), (6, 5), (6, 2), (1, 4), (1, 8)]
p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11 = map(Point, lis)
# lis = [(3, 9), (9, 9), (12, 5), (8, 5), (8, 2), (5, 4), (1, 4)]
# liste = [p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11]
# poly = Polygon(*liste)

def shape_gen():
    liste, arcs, tuple_list = [], [], []
    while True:
        abscisse = input("Veuillez entrer l'abscisse du point : ")
        if abscisse == '':
            break
        else:
            while True:
                ordonnee = input("Veuillez entrer l'ordonnee du point : ")

                if ordonnee != '':
                    break
            pt = Point(abscisse, ordonnee)
            tuple_list.append((abscisse, ordonnee))
            liste.append(pt)

    n = len(liste)
    for i in range(n - 1):
        arcs.append((liste[i], liste[i + 1]))
    arcs.append((liste[n - 1], liste[0]))
    print(liste)
    print(arcs)

    poly = Polygon(*liste)

    return liste, arcs, tuple_list, poly

liste, arcs, tuple_list, poly = shape_gen()

create_field(lis, step= 0.05, direction = 'cw')





