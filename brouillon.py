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

# from sympy import symbols
# from sympy.plotting import plot
# x = symbols('x')
# p1 = plot(x*x, show=False)
# p2 = plot(x, show=False)
# # p1.append(p2[0])
# p1.extend(p2)
# print(p1)
# p1.show()

# from sympy.geometry import Point, Line
# p1, p2 = Point(1, 1), Point(4, 5)
# l = Line((3, 1), (2, 2))
# p1.distance(p2)
# print(p1.distance(l))
# print(p1.dot(p2))

from sympy import Point, Line, Polygon, Ray, Segment, convex_hull
import copy

lis = [(4, 10), (8, 10), (8, 8), (4, 8), (4, 7), (8, 7), (9, 5), (9, 2), (6, 5), (6, 2), (1, 4), (1, 8)]

## Definition du polygone##

def poly_def(lis):

    p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11 = map(Point, lis)

    liste = [p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11]  ## liste des points de la figures

    arcs = [(p0, p1), (p1, p2), (p2, p3), (p3, p4), (p4, p5), (p5, p6),
            (p6, p7), (p7, p8), (p8, p9), (p9, p10), (p10, p11), (p11, p0)] ## Liste des cotes du polygone definis sous formes d'arcs

    poly = Polygon(*liste) ## definition du polygone

    return liste, arcs, poly

## test de concavite ##
def ccw(A, B, C):
    """Tests whether the turn formed by A, B, and C is ccw"""
    return (B.x - A.x) * (C.y - A.y) > (B.y - A.y) * (C.x - A.x)


## Determination des points concave du polygone ##

def concave_points(lis):

    concav_points = []
    concave_index = []

    liste, _, _ = poly_def(lis) ##recuperation de la liste des points definissant le polygone

    num = len(liste)

    for i in range(num):

        if (i == 0):
            a, b, c = liste[num - 1], liste[i], liste[i + 1]
        elif (i == num - 1):
            a, b, c = liste[i - 1], liste[i], liste[0]
        else:
            a, b, c = liste[i - 1], liste[i], liste[i + 1]
        if ccw(a, b, c):
            concav_points.append(liste[i])
            concave_index.append(i)
            print(i, ':', liste[i])

    return concav_points, concave_index

def intersection_points(lis):

    _, conc_index = concave_points(lis)

    print("concave", conc_index)


    liste, arcs, poly = poly_def(lis)
    num = len(liste)




    for j in range(len(conc_index)):
        index = conc_index[j]

        print("index", index)
        if index == 0:
                r = Ray(liste[num - 1], liste[index])
        elif index == num:
                r = Ray(liste[0], liste[index])
        else:
                r = Ray(liste[index + 1], liste[index])
                print("r", r)

        show_inter = r.intersection(poly)



        near_point = show_inter[0]

        if len(show_inter)>2:
            ## Point d'intersection le plus  proche du ieme concave point
           near_point = short_distance(show_inter, liste, index)

            ##Ajout du nouvel arc
        arcs_add(near_point, index, arcs, liste)

        print(near_point)

        inter_seg = 0
        segment = 0


##Calcul du point d'intersection le plus proche du concave_point
def short_distance(vector_points, liste, index):
    short_dist = 1000000

    for i in range(len(vector_points) - 1):

        dist = liste[index].distance(vector_points[i])

        if short_dist > dist:

            short_dist = dist
            near_point = vector_points[i]

    return near_point


def arcs_add(near_point, index, arcs, liste):


    nbre_arc = len(arcs)
    indic = 0

    for i in range(nbre_arc):
        """Recherche du segment auxquel appartient I et ajout des nouveaux cotes dans arcs"""
        for j in range(1):
            segment = Segment(arcs[i][j], arcs[i][j+1])
            dist = near_point.distance(segment)
            if dist == 0:
                if (near_point == arcs[i][j]) or (near_point == arcs[i][j+1]):
                    arcs.insert(index, (liste[index], near_point))
                else:
                    arcs.insert(index, (liste[index], near_point))
                    arcs.insert(i + 1, (arcs[i][j], near_point))
                    arcs.insert(i + 2, (near_point, arcs[i][j+1]))
                nbre_arc = len(arcs)
                indic = 1
        if indic == 1: break

    print(len(arcs))
    for i in range(nbre_arc):
        print(i + 1, ':', arcs[i])

    return arcs


def execution_main():
    lis = [(4, 10), (8, 10), (8, 8), (4, 8), (4, 7), (8, 7), (9, 5), (9, 2), (6, 5), (6, 2), (1, 4), (1, 8)]

    poly_def(lis)
    intersection_points(lis)
# total_point = copy.deepcopy(liste)
# print(concave_points)
# total_point.extend(concave_points)
# print(total_point)
execution_main()