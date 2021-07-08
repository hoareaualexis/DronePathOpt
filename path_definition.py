from sympy import Point, Line, Polygon, Ray, Segment
import numpy as np
from matplotlib.path import Path
import math
import copy

# lis = [(1, 3), (5, 1), (9, 3), (9, 7), (5, 8), (1, 7)]
lis = [(1, 1), (50, 1), (50, 50), (1, 50)]

def poly_def():
    # p0, p1, p2, p3, p4, p5 = map(Point, lis)
    p0, p1, p2, p3 = map(Point, lis)
    # liste = [p0, p1, p2, p3, p4, p5]
    liste = [p0, p1, p2, p3]
    # arcs = [(p0, p1), (p1, p2), (p2, p3), (p3, p4), (p4, p5), (p5, p0)]
    arcs = [(p0, p1), (p1, p2), (p2, p3), (p3, p0)]
    # lis = [(1, 1), (10, 1), (10, 7), (1, 7)]
    # p0, p1, p2, p3 = map(Point, lis)
    # liste = [p0, p1, p2, p3]
    poly = Polygon(*liste)
    return liste, arcs, poly

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

    print('long_dist: ', long_dist1)

    nb_BF = float(long_dist1 / 2)

    decimal = nb_BF % 1

    if decimal > 0.2:

        nb_BF = int(nb_BF) + 1

        spread_width = decimal + 1
    else:
        nb_BF = int(long_dist1 / 2)

    print('returned value by nbre_BF:', far_arc, spread_width, nb_BF)

    return far_arc, spread_width, nb_BF


def vect_translation_coors(arcs, spread_width):       #Calcul des coors du vecteur directeur et normal
    # _, arcs, _ = poly_def()
    cote = long_cote(arcs)

    u_x = cote[0][0] - cote[1][0]
    u_y = cote[0][1] - cote[1][1]

    n_x = u_y
    n_y = -u_x

    norme = lambda x, y: float(math.sqrt(x**2 + y**2))

    norm_n = norme(n_x, n_y)
    if spread_width is not None:
        t_x = float((spread_width/norm_n)*n_x)
        t_y = float((spread_width/norm_n)*n_y)

        vec_trans = Point(t_x, t_y)

        radius = norme(t_x, t_y)
        print('Radius/Spreading width: ', radius)
        print('translation coors: ', t_x, 'et', t_y)
    else:
        vec_trans = None

    vec_trans1 = Point((2 / norm_n) * n_x, (2 / norm_n) * n_y)

    return vec_trans, vec_trans1


def path(arcs, poly, vec_trans, vec_trans1, nb_BF):
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

    while True:

        BF_line_nber += 1

        if BF_line_nber >= nb_BF - 1 and (vec_trans is not None):

            point_A, point_B = translation(intersection_line[0], intersection_line[1], vec_trans)
        else:

            point_A, point_B = translation(intersection_line[0], intersection_line[1], vec_trans1)

        line = Line(point_A, point_B)

        intersection_line = poly.intersection(line)

        dim_list = len(intersection_line)          ## Recuperation de taille d'intersection_line

        if dim_list == 0:
            break

        elif dim_list == 2:
            lines_path.append(intersection_line[0])
            lines_path.append(intersection_line[1])

        elif type(intersection_line[0]) == type_seg:
            lines_path.append(intersection_line[0]._args[0])
            lines_path.append(intersection_line[0]._args[1])
            break

        elif dim_list == 1 and type_seg != type(intersection_line[0]):
            lines_path.append(intersection_line[0])
            break

        if BF_line_nber == nb_BF:
            break

    lines_tuple, trajet = [], []
    for point in lines_path:
        x = float(point[0])
        y = float(point[1])
        tup = (x, y)
        lines_tuple.append(tup)

    swapper = 0
    for i in range(0, len(lines_tuple), 2):
        if swapper % 2 == 0:
            couple = (lines_tuple[i], lines_tuple[i + 1])
            trajet.append(couple)
        else:
            couple = (lines_tuple[i + 1], lines_tuple[i])
            trajet.append(couple)
        swapper += 1

    print(trajet)

    return trajet


def trajectory(itineraire, spread_width, nb_BF):

    Fr, AD = 18.33, 25

    # velo_vec = Fr/(AD * spread_width)
    # velo_vec = 0

    norme = lambda x, y: float(math.sqrt(x ** 2 + y ** 2))

    trajet = []
    BF_line_nber = 0

    for cpl in itineraire:
        u_x = cpl[1][0] - cpl[0][0]
        u_y = cpl[1][1] - cpl[0][1]

        BF_line_nber += 1

        norm_u = norme(u_x, u_y)

        if BF_line_nber >= nb_BF - 1 and spread_width is not None:
            velo_vec = Fr/(AD * spread_width)
        else:
            velo_vec = Fr / (AD * 2)

        velo_x = float((velo_vec / norm_u) * u_x)
        velo_y = float((velo_vec / norm_u) * u_y)

        # step = norme(velo_x, velo_y)

        velocity = Point(velo_x, velo_y)

        segment = Segment(cpl[0], cpl[1])

        transla_pt = Point(cpl[0]) + velocity

        distance = segment.distance(transla_pt)
        tmp = []
        while distance == 0:
            pt = (float(transla_pt[0]), float(transla_pt[1]))
            tmp.append(pt)
            transla_pt = transla_pt + velocity
            distance = segment.distance(transla_pt)
        tmp.insert(0, cpl[0])
        tmp.append(cpl[1])
        trajet.append(tmp)

    print('print trajectory:')
    for li in trajet:
        print(li)

    return trajet


def distance_AB(A, B):
    return float(Point(A).distance(Point(B)))


def energy_comp(a, b, dr_weight, spread_width):
    # pi = math.pi
    # sin_phi = math.sin(pi)
    # rotor_radius = 2.5
    # Pc = 0.5*drone_weight(velocity*sin_phi + math.sqrt((velocity*sin_phi)**2 + 2*drone_weight/(air_density*pi*rotor_radius**2)))
    # Pd = 0.5*drone_weight(velocity*sin_phi - math.sqrt((velocity*sin_phi)**2 - 2*drone_weight/(air_density*pi*rotor_radius**2)))
    # Pv = Pc + Pd
    # energy = (Pv + Ph) * time

    dist = distance_AB(a, b)

    velocity = feed_rate / (dosage * spread_width)

    time = dist/velocity

    # dr_weight -= feed_rate*0.001
    dr_weight -= feed_rate*0.001
    released_weight = feed_rate*0.001

    Ph = 0.5 * drag_coeff * front_area * air_density * velocity ** 3 + dr_weight ** 2 / (
                air_density * velocity * drone_width ** 2)

    energy = Ph * time * coef_vrais
    print('la petite energie: ', energy)

    return energy, time, dr_weight, released_weight


def calcul_metric(trajet, drone_weight, spread_width):
    # trajet = trajectory()
    # _, spread_width, _ = nombre_BF()
    energy = 0
    time = 0
    released_weight = 0
    # cur_weight = drone_weight
    n = len(trajet) - 2
    for i in range(n):

        metrics = energy_comp((trajet[i]), trajet[i+1], drone_weight, spread_width)
        print(metrics)
        energy += metrics[0]
        time += metrics[1]
        released_weight += metrics[3]

    return energy, time, drone_weight


def f_objective(solution, maximum_energy_of_the_drone, maximum_drone_weight, drone_weight, spread_width, nb_BF):

    cs_list = []
    scs_list = []

    total_travel_time, total_seed_weight_released = 0, 0

    remaining_energy = maximum_energy_of_the_drone
    remaining_weight = maximum_drone_weight

    nb_line_nber = 0
    for sol in solution:

        nb_line_nber += 1

        for j in range(len(sol)-1):

            a = sol[j]
            b = sol[j+1]

            if nb_line_nber >= nb_BF - 1:
                spw = spread_width
            else:
                spw = 2

            metric = energy_comp(a, b, remaining_weight, spw)
            remaining_energy -= metric[0]
            remaining_weight = metric[2]
            total_travel_time += metric[1]
            total_seed_weight_released += metric[3]

            print('print me metric: ', metric)

            if remaining_energy <= 0:

                cs_list.append((sol[j], total_travel_time, remaining_weight))

                remaining_energy = maximum_energy_of_the_drone

            if remaining_weight <= drone_weight:

                scs_list.append((sol[j], total_travel_time, remaining_energy))

                remaining_weight = maximum_drone_weight

        print('remaining_energy: ', remaining_energy)
        print('remaining_weight: ', remaining_weight)
        print('total travel time: ', total_travel_time)

    velocity = feed_rate / (dosage * spread_width)

    total_distance = velocity*total_travel_time

    print('\ntotal_distance: ', total_distance)
    print('total_seed_weight_released: ', total_seed_weight_released)
    print('total_travel_time: ', total_travel_time)

    # pos = np.where(np.array(cs_list) == 1)[0]
    # pos1 = np.where(np.array(scs_list) == 1)[0]
    print('\nPoint de recharge en energie:')

    for cs in cs_list:
        print('Point coords:', cs[0], 'at time: ', cs[1], 'remaining_weight: ', cs[2])  #sol[j], total_travel_time, remaining_weight

    print('\n Point de recharge en semence:')

    for scs in scs_list:
        print('Point coords:', scs[0], 'at time: ', scs[1], 'remaining_energy: ', scs[2])

    return cs_list, scs_list


##Drone specs##
drone_weight = 19 ## en Kg
maximum_take_off_weight = 45
take_off_weight = [19, 36]
flight_time = [10, 25]
overall_size = 1650*1650*580

feed_rate = 18.33
dosage = 25
drag_coeff = 0.0613
drone_width = 580*0.01
air_density = 1.2754     ##kg / m³
front_area = 1650*580*0.0001    #95.7
maximum_energy_of_the_drone = float(4000*600)  # 4000W pour un flight time de 10-25min
maximum_drone_weight = 36
coef_vrais = 54.1415108851846        ## Coefficient d'ajustement


def execution_main():

    liste, arcs, poly = poly_def()

    far_arc, spread_width, nb_BF = nombre_BF(liste, arcs)

    vec_trans, vec_trans1 = vect_translation_coors(arcs, spread_width)

    ligne_BF = path(arcs, poly, vec_trans, vec_trans1, nb_BF)

    trajectoire = trajectory(ligne_BF, spread_width, nb_BF)

    f_objective(trajectoire, maximum_energy_of_the_drone, maximum_drone_weight, drone_weight, spread_width, nb_BF)


execution_main()
