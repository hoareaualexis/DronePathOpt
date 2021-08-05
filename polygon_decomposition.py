import math
from collections import deque
from sympy import Point, Polygon, Ray, Segment, Line
import itertools, copy

# lis = [(4, 10), (8, 10), (8, 8), (4, 8), (4, 7), (8, 7), (9, 5), (9, 2), (6, 5), (6, 2), (1, 4), (1, 8)]
# lis = [(4, 7), (9, 7), (10, 5), (9, 4), (10, 3), (10, 2), (9, 1), (6, 1), (5, 2), (4, 1), (2, 2), (2, 5)]
# lis = [(3, 9), (9, 9), (12, 5), (8, 5), (8, 2), (5, 4), (1, 4)]

lis = [(3, 16), (7, 13), (11, 16), (16, 14), (15, 7), (13, 7), (13, 2), (7, 1), (4, 3), (6, 4), (6, 6), (3, 7), (3, 10),
       (1, 14)]


def poly_def(lis):

    poly, arcs, liste = arc_generator(lis)

    return liste, arcs, poly


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

    distance = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    for pt in vector_points:

        dist = distance(liste[index], pt)  # liste[index].distance(pt)

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
        # print('show me the list:', liste)

        show_inter = r.intersection(poly)

        # print('show_inter:', show_inter)
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

        arcs_add(near_point, index, arcs, liste, arcs_intern, concaves_pt)

    print('\n')

    return arcs, liste


def arcs_add(near_point, index, arcs, liste, interior_arc, concaves_pt):
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

                if (arc[0] or arc[1]) in concaves_pt:
                    # liste.insert(-1, near_point)
                    liste.append(near_point)
                else:
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
    vector = [point[0] - origin[0], point[1] - origin[1]]
    # Length of vector: ||v||
    lenvector = math.hypot(vector[0], vector[1])
    # If length is zero there is no angle
    if lenvector == 0:
        return -math.pi, 0
    # Normalize vector: v/||v||
    normalized = [vector[0] / lenvector, vector[1] / lenvector]
    dotprod = normalized[0] * refvec[0] + normalized[1] * refvec[1]  # x1*x2 + y1*y2
    diffprod = refvec[1] * normalized[0] - refvec[0] * normalized[1]  # x1*y2 - y1*x2
    angle = math.atan2(diffprod, dotprod)
    # Negative angles represent counter-clockwise angles so we need to subtract them
    # from 2*pi (360 degrees)
    if angle < 0:
        return 2 * math.pi + angle, lenvector
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

    listed.sort(key=takeSecond)  # sort list from second tuple elt

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
        for i in range(n - 1):

            tupl1 = (recueil[i], recueil[i + 1])
            tupl2 = (recueil[i + 1], recueil[i])

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

            ptA, ptB = ptB, recueil[-1]
            if arret == n:
                if not ccw(ptA, ptB, recueil[0]):
                    if not ccw(ptB, recueil[0], recueil[1]):
                        total_convex.append(recueil)

    return total_convex


def convex_shapes(dic_graph, start_pt, arcs, nbre):
    # dic_graph, _, start_pt, arcs = graphe_def(lis)
    total_convex = []

    for start in start_pt:
        vertices = dic_graph[start]

        for vertex in vertices:
            paths_gen = find_paths_dfs(dic_graph, start, vertex)
            total_convex += convex_select(paths_gen, arcs)

    # new_list = copy.deepcopy(total_convex)
    new_list = redondance_del(total_convex)
    new_list1 = []
    for vect in new_list:

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
            if arret == n and len(recueil) > 2:
                new_list1.append(recueil)

    new_list1 = edging(new_list1)

    for li in new_list1:
        print(li)
    print('\n')

    return new_list1


def edging(vector):

    typ_pt = type(Point(0, 0))
    indice = 0
    for vec in vector:

        if len(vec) <= 3:
            indice += 1
            continue
        tmp = []
        for i in range(2, len(vec)):
            l1 = Line(vec[i - 2], vec[i - 1])
            l2 = Line(vec[i - 1], vec[i])
            intersect = l1.intersection(l2)

            if type(intersect[0]) == typ_pt:
                tmp.append(intersect[0])

        l1, l2 = Line(tmp[-1], vec[-1]), Line(vec[-1], vec[0])
        intersect = l1.intersection(l2)
        if type(intersect[0]) == typ_pt:
            tmp.append(intersect[0])

        l1, l2 = Line(tmp[-1], vec[0]), Line(vec[0], tmp[0])
        intersect = l1.intersection(l2)
        if type(intersect[0]) == typ_pt:
            tmp.append(intersect[0])

        vector[indice] = tmp
        indice += 1

    # for v in vector:
    #     print('vec:', v)

    copy_vec = copy.deepcopy(vector)
    # print('copy_vec:', copy_vec)

    pas = 1
    for vec in vector:
        poly = Polygon(*vec)
        # print(vec)
        area1 = abs(poly.area)

        for i in range(pas, len(vector)):
            poly1 = Polygon(*vector[i])
            inter_pol = poly.intersection(poly1)

            if len(inter_pol) > 1:
                area2 = abs(poly1.area)
                if area1 >= area2:
                    copy_vec.remove(vector[i])
                    # del copy_vec[i]
                else:
                    copy_vec.remove(vec)
                    # del copy_vec[pas-1]
        if len(copy_vec) == 1:
            break
        pas += 1

    # for copy_ in copy_vec:
    #     print('exec: ', copy_)
    return copy_vec


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


def arc_generator2(liste):
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

    point1, point2 = None, None
    long_dist1 = 0
    far_arc, spread_width = None, None

    ##Calcul de la profondeur du champ
    for pt in liste:
        dist = float(pt.distance(line))

        if long_dist1 < dist:
            long_dist1 = dist
            point1 = pt
        elif long_dist1 == dist:
            point2 = pt

    if point2 is not None:
        if (point1, point2) in arcs:
            far_arc = (point1, point2)
        elif (point2, point1) in arcs:
            far_arc = (point2, point1)

    # print('long_dist: ', long_dist1)

    nb_BF = float(long_dist1 / 2)

    decimal = nb_BF % 1

    if decimal > 0.2:

        nb_BF = int(nb_BF) + 1

        spread_width = decimal + 1
    else:
        nb_BF = int(long_dist1 / 2)

    # print('returned value by nbre_BF:', far_arc, spread_width, nb_BF)

    return far_arc, spread_width, nb_BF


def execution_main():
    conc_pt, nbre = concave_points(lis)

    config_list = list(itertools.product([0, 1], repeat=nbre))

    all_permut = list(itertools.permutations(conc_pt))

    opt_shape, opt_order = [], []
    opt_BF, opt_spw = 0, 0

    for co_pts in all_permut:
        for pt in co_pts:
            print(pt)

        for conf in config_list:
            arcs, liste = intersection_points(lis, co_pts, conf)

            dic_graph, start_pts = graphe_def(arcs, liste)

            shape_list = convex_shapes(dic_graph, start_pts, arcs, nbre)

            n = len(shape_list)
            if n < nbre:
                continue

            sum_nBF, sum_spw = 0, 0

            for convex_sh in shape_list:
                _, shape_arc = arc_generator2(convex_sh)

                _, spread_width, nbre_BF = nombre_BF(convex_sh, shape_arc)

                sum_nBF += nbre_BF
                if spread_width is None:
                    spread_width = 2
                    sum_spw += spread_width
                else:
                    sum_spw += spread_width
                print('nbre de BF and sp_width: ', nbre_BF, 'et ', round(spread_width, 2))
            # print('shape_list:', shape_list)

            print('sum_nBF de BF and sum_spw: ', sum_nBF, 'et ', round(sum_spw / n, 2))

            if opt_BF == 0:

                opt_shape = shape_list
                opt_spw = sum_spw / n
                opt_BF = sum_nBF

            elif sum_nBF < opt_BF:
                opt_shape = [shape_list]
                opt_spw = sum_spw / n
                opt_BF = sum_nBF

            elif sum_nBF == opt_BF:

                if sum_spw / n > opt_spw:
                    opt_shape = shape_list
                    opt_spw = sum_spw / n

                elif sum_spw / n == opt_spw:
                    opt_shape.append(shape_list)

            print('\n')

    print('optimals convex shapes are:')
    h = 1
    for opt_sh in opt_shape:
        print(h, 'shape:')
        for op in opt_sh:
            print(op)
        print('\n')
        h += 1
    print('with nbre_BF et spw_moyen: ', opt_BF, 'et', round(opt_spw, 2))

    return opt_shape


execution_main()

