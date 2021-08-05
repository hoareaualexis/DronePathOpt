from sympy import Point, Line, Polygon, Ray, Segment
import numpy as np
from matplotlib.path import Path
import math
import copy

lis = [(1, 3), (5, 1), (9, 3), (9, 7), (5, 8), (1, 7)]
# lis = [(1, 1), (50, 1), (50, 50), (1, 50)]


def arc_generator(liste):
    arcs, liste1 = [], []
    n = len(liste)
    for i in range(n - 1):
        point1, point2 = Point(liste[i]), Point(liste[i + 1])
        arcs.append((point1, point2))
    arcs.append((Point(liste[n - 1]), Point(liste[0])))

    for pt in liste:
        point1 = Point(pt)
        liste1.append(point1)

    poly = Polygon(*liste)

    return poly, arcs, liste1


def long_cote(arcs):

    global grand_cote
    big_edge = 0
    for cote in arcs:
        dist = cote[0].distance(cote[1])
        if big_edge <= dist:
            big_edge = dist
            grand_cote = cote
    return grand_cote


def nombre_BF(liste, arcs):

    cote = long_cote(arcs)
    line = Line(cote[0], cote[1])

    point1, point2 = None, None
    long_dist = 0
    far_arc, spread_width = None, None

    ##Calcul de la profondeur du champ
    for pt in liste:
        dist = float(pt.distance(line))

        if long_dist < dist:
            long_dist = dist

    print('long_dist: ', long_dist)

    nb_BF = float(long_dist / 2)

    decimal = nb_BF % 1

    if decimal > 0.2:

        nb_BF = int(nb_BF) + 1

        spread_width = decimal + 1
    else:
        nb_BF = int(long_dist / 2)

    return spread_width, long_dist


def vect_translation_coors(arcs, spread_width, p_width):       #Calcul des coors du vecteur directeur et normal

    cote = long_cote(arcs)

    u_x = cote[0][0] - cote[1][0]
    u_y = cote[0][1] - cote[1][1]

    n_x = u_y
    n_y = -u_x

    norme = lambda x, y: float(math.sqrt(x**2 + y**2))

    norm_n = norme(n_x, n_y)
    if spread_width is not None:

        vec = p_width-spread_width*0.5

        t_x = float((vec/norm_n)*n_x)
        t_y = float((vec/norm_n)*n_y)

        vec_trans = Point(t_x, t_y)

        radius = norme(t_x, t_y)
        print('Radius/Spreading width: ', radius)
        print('translation coors: ', t_x, 'et', t_y)
    else:
        vec_trans = None

    vec_trans1 = Point((2 / norm_n) * n_x, (2 / norm_n) * n_y)

    return vec_trans, vec_trans1


def path(arcs, poly, vec_trans, vec_trans1, p_width):
    # liste, arcs, poly = poly_def()
    cote = long_cote(arcs)

    print('Plus long cote: ', cote)

    # vec_trans, far_pt = vect_translation_coors(arcs)         ## Vecteur translation

    lines_path = []                ## Insertion de la premiere ligne de parcours BF

    ech1, ech2 = (1, 1), (0, 0)
    type_seg = type(Segment(ech1, ech2))         ## recuperation du type Segment

    # point_A = cote[0] + vec_trans/2
    # point_B = cote[1] + vec_trans/2

    point_A = cote[0] + vec_trans1/2
    point_B = cote[1] + vec_trans1/2

    BF_line_nber = 1

    line = Line(point_A, point_B)

    intersection_line = poly.intersection(line)

    dim_list = len(intersection_line)

    if dim_list == 0:
        point_A = cote[0] - vec_trans1 / 2
        point_B = cote[1] - vec_trans1 / 2

        line = Line(point_A, point_B)

        intersection_line = poly.intersection(line)

        def translation(a, b, vector_trans):
            return a - vector_trans, b - vector_trans
    else:
        def translation(a, b, vector_trans):
            return a + vector_trans, b + vector_trans

    lines_path.append(intersection_line[0])
    lines_path.append(intersection_line[1])

    point_A, point_B = translation(intersection_line[0], intersection_line[1], vec_trans)