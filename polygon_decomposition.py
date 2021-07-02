import math
from collections import deque
from sympy import Point, Polygon, Ray, Segment, Line
import itertools

lis = [(4, 10), (8, 10), (8, 8), (4, 8), (4, 7), (8, 7), (9, 5), (9, 2), (6, 5), (6, 2), (1, 4), (1, 8)]
# lis = [(3, 9), (9, 9), (12, 5), (8, 5), (8, 2), (5, 4), (1, 4)]


def poly_def(lis):
    p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11 = map(Point, lis)
    # p0, p1, p2, p3, p4, p5, p6 = map(Point, lis)

    liste = [p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11]  ## liste des points de la figures
    # liste = [p0, p1, p2, p3, p4, p5, p6]

    arcs = [(p0, p1), (p1, p2), (p2, p3), (p3, p4), (p4, p5), (p5, p6),         ## Liste des cotes du polygone definis sous formes d'arcs
            (p6, p7), (p7, p8), (p8, p9), (p9, p10), (p10, p11),
            (p11, p0)]
    # arcs = [(p0, p1), (p1, p2), (p2, p3), (p3, p4), (p4, p5), (p5, p6), (p6, p0)]

    poly = Polygon(*liste)  ## definition du polygone

    return liste, arcs, poly


def ccw(A, B, C):
    """Tests whether the turn formed by A, B, and C is ccw"""
    return (B.x - A.x) * (C.y - A.y) > (B.y - A.y) * (C.x - A.x)

## Ajout des arcs dans la liste des ccotes


def concave_points(lis):
    concav_points = []

    liste, _, _ = poly_def(lis)  ##recuperation de la liste des points definissant le polygone

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
            print(i, ':', liste[i])

    nbre = len(concav_points)
    print('\n')

    return concav_points, nbre


def short_distance(vector_points, liste, index):
    global near_point

    short_dist = 1000e+1024

    distance = lambda a, b: math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)

    for pt in vector_points:

        dist = distance(liste[index], pt)    #liste[index].distance(pt)

        if short_dist > dist:
            short_dist = dist
            near_point = pt

    return near_point


def section_droite1(liste, num, index):

    if index == 0:
        r = Ray(liste[num - 1], liste[index])

    elif index == num:
        r = Ray(liste[0], liste[index])

    else:
        r = Ray(liste[index + 1], liste[index])

    return r


def section_droite2(liste, num, index):

    if index == 0:
        r = Ray(liste[1], liste[index])

    elif index == num:
        r = Ray(liste[num - 1], liste[index])

    else:
        r = Ray(liste[index - 1], liste[index])

    return r


def intersection_points(lis, concaves_pt, conf):

    liste, arcs, poly = poly_def(lis)

    arcs_intern = []

    n = len(liste)

    i = 0
    for con_pt in concaves_pt:

        index = liste.index(con_pt)
        if conf[i] == 0:
            r = section_droite1(liste, n, index)
        else:
            r = section_droite2(liste, n, index)
        i += 1
        print('semi_droite de section: ', r)

        show_inter = r.intersection(poly)

        del show_inter[-1]

        if arcs_intern != []:

            for arc in arcs_intern:
                seg = Segment(arc[0], arc[1])
                int_pt = r.intersection(seg)

                if int_pt and (int_pt[0] != r._args[0]) and (int_pt[0] != r._args[1]):

                    show_inter.append(int_pt[0])

        near_point = show_inter[0]

        if len(show_inter) > 1:

            near_point = short_distance(show_inter, liste, index)

        arcs_add(near_point, index, arcs, liste, arcs_intern)

    print('\n')

    return arcs, liste


def arcs_add(near_point, index, arcs, liste, interior_arc):

    arret = 0

    for arc in arcs:
        segment = Segment(arc[0], arc[1])
        # print('print me segment and near_point: ', segment, near_point)
        dist = near_point.distance(segment)
        if dist == 0:

            if near_point == arc[0] or near_point == arc[1]:

                arcs.append((liste[index], near_point))

                interior_arc.append((liste[index], near_point))
            else:
                arcs.append((liste[index], near_point))

                arcs.append((arc[0], near_point))

                arcs.append((near_point, arc[1]))

                interior_arc.append((liste[index], near_point))

                arcs.remove(arc)

                if arc in interior_arc:
                    interior_arc.append((arc[0], near_point))
                    interior_arc.append((near_point, arc[1]))
                    interior_arc.remove(arc)

                k = liste.index(arc[1])
                liste.insert(k, near_point)

            arret = 1

        if arret == 1:
            break

    return arcs, interior_arc


def graphe_def(arcs, liste):

    # arcs, liste = intersection_points(lis)
    dic_graph = {}

    for pt in liste:

        tmp = []
        for arc in arcs:

            if pt == arc[0]:
                tmp.append(arc[1])
            elif pt == arc[1]:
                tmp.append(arc[0])

        dic_graph[pt] = tmp

        # print(pt, '=>', dic_graph[pt])

    start_pt_list = []
    for key, values in dic_graph.items():
        nb_vois = len(values)
        if nb_vois > 2:
            start_pt_list.append(key)

    return dic_graph, start_pt_list


def find_paths_dfs(graph, start, end):

    stack = deque()
    stack.append((start, [start]))

    while stack:
        node, path = stack.pop()
        adjacent_nodes = [n for n in graph[node] if n not in path]
        for adjacent_node in adjacent_nodes:
            if adjacent_node == end:
                yield path + [adjacent_node]
            else:
                stack.append((adjacent_node, path + [adjacent_node]))


def clockwiseangle_and_distance(point, origin):
    refvec = [0, 1]

    # Vector between point and the origin: v = p - o
    vector = [point[0]-origin[0], point[1]-origin[1]]
    # Length of vector: ||v||
    lenvector = math.hypot(vector[0], vector[1])
    # If length is zero there is no angle
    if lenvector == 0:
        return -math.pi, 0
    # Normalize vector: v/||v||
    normalized = [vector[0]/lenvector, vector[1]/lenvector]
    dotprod  = normalized[0]*refvec[0] + normalized[1]*refvec[1]     # x1*x2 + y1*y2
    diffprod = refvec[1]*normalized[0] - refvec[0]*normalized[1]     # x1*y2 - y1*x2
    angle = math.atan2(diffprod, dotprod)
    # Negative angles represent counter-clockwise angles so we need to subtract them
    # from 2*pi (360 degrees)
    if angle < 0:
        return 2*math.pi+angle, lenvector
    # I return first the angle because that's the primary sorting criterium
    # but if two vectors have the same angle then the shorter distance should come first.
    return angle, lenvector


def sorted_cw(pts):

    origin = pts[0]

    listed = []

    for pt in pts:
        angle, _ = clockwiseangle_and_distance(pt, origin)

        listed.append((pt, round(angle, 5)))

    takeSecond = lambda elem: elem[1]

    listed.sort(key=takeSecond)   # sort list from second tuple elt

    sorted_list = []
    for li in listed:
        # print('Sorted list:', li[0])
        sorted_list.append(li[0])

    return sorted_list


def convex_select(path_gen, arcs):
    total_convex = []
    for vect in path_gen:

        recueil = sorted_cw(vect)
        n = len(recueil)
        condi = 1
        for i in range(n-1):

            tupl1 = (recueil[i], recueil[i+1])
            tupl2 = (recueil[i+1], recueil[i])

            if not ((tupl1 in arcs) or (tupl2 in arcs)):
                break
            else:
                condi += 1

        if condi == n:
            ptA, ptB = recueil[0], recueil[1]

            arret = 2

            for i in range(2, n):
                if ccw(ptA, ptB, recueil[i]):
                    # print('Is not convex')
                    break
                else:
                    ptA, ptB = ptB, recueil[i]
                    arret += 1
            if arret == n:
                total_convex.append(recueil)

    return total_convex


def convex_shapes(dic_graph, start_pt, arcs):

    # dic_graph, _, start_pt, arcs = graphe_def(lis)
    total_convex = []

    for start in start_pt:
        vertices = dic_graph[start]

        for vertex in vertices:
            paths_gen = find_paths_dfs(dic_graph, start, vertex)
            total_convex += convex_select(paths_gen, arcs)

    # new_list = copy.deepcopy(total_convex)
    new_list = redondance_del(total_convex)

    for li in new_list:
        print(li)
    print('\n')

    return new_list


def redondance_del(total_convex):

    new_list, new_list1 = [], []

    for elt in total_convex:
        new_list.append(set(elt))

    for elt in new_list:
        if not elt in new_list1:
            if len(elt) > 2:
                new_list1.append(elt)

    new_list = []
    for elt in new_list1:
        for ls in total_convex:
            if elt == set(ls):
                new_list.append(ls)
                break

    return new_list


def arc_generator(liste):
    arcs = []
    n = len(liste)
    for i in range(n - 1):
        arcs.append((liste[i], liste[i + 1]))
    arcs.append((liste[n - 1], liste[0]))

    # print(liste)
    # print(arcs)

    poly = Polygon(*liste)

    return poly, arcs


def long_cote(arcs):

    grand_cote = ''
    big_edge = 0

    distance = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    for cote in arcs:
        # dist = cote[0].distance(cote[1])
        dist = distance(cote[0], cote[1])
        if big_edge <= dist:
            big_edge = dist
            grand_cote = cote
    return grand_cote


def nombre_BF(liste, arcs):

    cote = long_cote(arcs)
    line = Line(cote[0], cote[1])

    far_point1 = 0
    long_dist1, long_dist2 = 0, 0

    ##Calcul de la profondeur du champ
    for pt in liste:
        dist = float(pt.distance(line))

        if long_dist1 <= dist:
            long_dist2 = long_dist1
            long_dist1 = dist
            far_point2 = far_point1
            far_point1 = pt

        elif long_dist2 <= dist:
            long_dist2 = dist
            far_point2 = pt

    k = 1
    spread_width = float(long_dist1/k)
    while spread_width > 2:
        k += 1
        spread_width = float(long_dist1/k)
    # print('spread_width et nombre de BF: ', spread_width, 'et', k)

    # return far_point1, far_point2, long_dist1, long_dist2
    return spread_width, k


def execution_main():

    conc_pt, nbre = concave_points(lis)
    config_list = list(itertools.product([0, 1], repeat=nbre))

    all_permut = list(itertools.permutations(conc_pt))

    for co_pts in all_permut:
        for pt in co_pts:
            print(pt)

        for conf in config_list:
            arcs, liste = intersection_points(lis, co_pts, conf)

            dic_graph, start_pts = graphe_def(arcs, liste)

            shape_list = convex_shapes(dic_graph, start_pts, arcs)
            # print('shape_list:', shape_list)

            for convex_sh in shape_list:
                pol, shape_arc = arc_generator(convex_sh)

                spread_width, k = nombre_BF(convex_sh, shape_arc)
                print('nbre de BF and sp_width: ', k, 'et ', round(spread_width, 2))
            print('\n')


execution_main()

