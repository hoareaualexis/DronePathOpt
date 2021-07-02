from sympy import Point, Line, Polygon, Ray, Segment
import numpy as np
from matplotlib.path import Path
import math
import copy

lis = [(1, 3), (5, 1), (9, 3), (9, 7), (5, 8), (1, 7)]

def poly_def():
    p0, p1, p2, p3, p4, p5 = map(Point, lis)
    liste = [p0, p1, p2, p3, p4, p5]
    arcs = [(p0, p1), (p1, p2), (p2, p3), (p3, p4), (p4, p5), (p5, p0)]

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


def vect_translation_coors(arcs):       #Calcul des coors du vecteur directeur et normal
    # _, arcs, _ = poly_def()
    cote = long_cote(arcs)

    u_x = cote[0][0] - cote[1][0]
    u_y = cote[0][1] - cote[1][1]

    n_x = u_y
    n_y = -u_x

    norme = lambda x, y: float(math.sqrt(x**2 + y**2))

    norm_n = norme(n_x, n_y)

    far_point1, spread_width, k = nombre_BF()               #Definition de la Largeur du couloir d'epandage

    t_x = float((spread_width/norm_n)*n_x)
    t_y = float((spread_width/norm_n)*n_y)

    radius = norme(t_x, t_y)

    print('Radius/Spreading width: ', radius)

    print('translation vector: (', t_x, ',', t_y, ')')

    translation_vector = Point(t_x, t_y)

    return translation_vector, far_point1


#Determination du signe du vecteur translation

def isPositive():
    _, arcs, poly = poly_def()

    cote = long_cote(arcs)

    vec_trans, _ = vect_translation_coors(arcs)

    point_A = cote[0] + vec_trans
    point_B = cote[1] + vec_trans

    line = Line(point_A, point_B)

    intersection_line = poly.intersection(line)

    dim_list = len(intersection_line)

    if dim_list == 0:

        return False

    else:
        return True


def path():
    liste, arcs, poly = poly_def()
    cote = long_cote(arcs)

    print('Plus long cote: ', cote)

    vec_trans, far_pt = vect_translation_coors(arcs)         ## Vecteur translation

    lines_path = []                ## Insertion de la premiere ligne de parcours BF

    type_seg = type(Segment((1, 1), (0, 0)))         ## recuperation du type Segment

    if isPositive():

        point_A = cote[0] + vec_trans/2
        point_B = cote[1] + vec_trans/2

        line = Line(point_A, point_B)

        intersection_line = poly.intersection(line)

        lines_path.append(intersection_line[0])
        lines_path.append(intersection_line[1])

        while True:

            point_A = intersection_line[0] + vec_trans
            point_B = intersection_line[1] + vec_trans

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

    else:
        point_A = cote[0] - vec_trans
        point_B = cote[1] - vec_trans

        line = Line(point_A, point_B)

        intersection_line = poly.intersection(line)

        lines_path.append(intersection_line[0])
        lines_path.append(intersection_line[1])

        while True:

            point_A = intersection_line[0] - vec_trans
            point_B = intersection_line[1] - vec_trans

            line = Line(point_A, point_B)

            intersection_line = poly.intersection(line)

            dim_list = len(intersection_line)  ## Recuperation de taille d'intersection_line

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

def trajectory():
    ligne_BF = path()
    _, spread_width, _ = nombre_BF()  # Definition de la Largeur du couloir d'epandage
    Fr, AD = 18.33, 25
    velo_vec = Fr/(AD * spread_width)
    print('velo_vec: ', velo_vec)
    norme = lambda x, y: float(math.sqrt(x ** 2 + y ** 2))

    trajet = []
    for cpl in ligne_BF:
        u_x = cpl[1][0] - cpl[0][0]
        u_y = cpl[1][1] - cpl[0][1]

        norm_u = norme(u_x, u_y)

        velo_x = float((velo_vec / norm_u) * u_x)
        velo_y = float((velo_vec / norm_u) * u_y)

        step = norme(velo_x, velo_y)

        velocity = Point(velo_x, velo_y)
        # print('vecteur vitesse: (', velo_x, ',', velo_y, ')')

        segment = Segment(cpl[0], cpl[1])

        transla_pt = Point(cpl[0]) + velocity

        distance = segment.distance(transla_pt)

        while distance == 0:
            pt = (float(transla_pt[0]), float(transla_pt[1]))
            trajet.append(pt)
            transla_pt = transla_pt + velocity
            distance = segment.distance(transla_pt)

    trajet.insert(0, ligne_BF[0][0])
    trajet.append(ligne_BF[-1][1])

    print('print trajectory: ', trajet)

    dis = 0
    for i in range(len(trajet) - 1):
        dis += distance_AB(trajet[i], trajet[i + 1])
    print('print distance parcouru', round(dis, 2))
    time = round(dis / velo_vec, 2)
    print('print time: ', time)
    return trajet



def nombre_BF():
    liste, arcs, poly = poly_def()

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
    return far_point1, spread_width, k


def distance_AB(A, B):
    return float(Point(A).distance(Point(B)))


def energy_comp(a, b, dr_weight):
    # pi = math.pi
    # sin_phi = math.sin(pi)
    # rotor_radius = 2.5
    # Pc = 0.5*drone_weight(velocity*sin_phi + math.sqrt((velocity*sin_phi)**2 + 2*drone_weight/(air_density*pi*rotor_radius**2)))
    # Pd = 0.5*drone_weight(velocity*sin_phi - math.sqrt((velocity*sin_phi)**2 - 2*drone_weight/(air_density*pi*rotor_radius**2)))
    # Pv = Pc + Pd
    # energy = (Pv + Ph) * time

    feed_rate = 18.33
    dosage = 25
    drag_coeff = 0.0613
    drone_width = 580 * 0.01
    air_density = 1.2754  ##kg / m³
    front_area = 1650 * 580 * 0.0001 #95.7
    coef_vrais = 54.1415108851846        ## Coefficient d'ajustement
    _, spread_width, _ = nombre_BF()

    dist = distance_AB(a, b)

    velocity = feed_rate / (dosage * spread_width)

    time = dist/velocity

    dr_weight -= feed_rate*0.001

    Ph = 0.5 * drag_coeff * front_area * air_density * velocity ** 3 + dr_weight ** 2 / (
                air_density * velocity * drone_width ** 2)

    energy = Ph * time * coef_vrais

    return energy, time, dr_weight

def calcul_metric(trajet, drone_weight):
    # trajet = trajectory()
    _, spread_width, _ = nombre_BF()
    energy = 0
    time = 0
    # cur_weight = drone_weight
    n = len(trajet) - 2
    for i in range(n):
        # dist += distance_AB(trajet[i], trajet[i+1])
        # time += dist/velocity
        # drone_weight -= feed_rate
        metrics = energy_comp((trajet[i]), trajet[i+1], drone_weight)
        print(metrics)
        energy += metrics[0]
        time += metrics[1]

    return energy, time, drone_weight


def f_objective(solution, maximum_energy_of_the_drone, maximum_drone_weight, spread_width, drone_weight):

    # sol = copy.copy(solution[0])
    cs_list = []
    scs_list = []
    energy_list = []
    dr_weight_list = []

    for i in range(0, len(solution)):
        cs_list.append(0)
        scs_list.append(0)

    solution_representation = [
        solution,
        cs_list,
        scs_list,
        energy_list,
        dr_weight_list,
    ]

    remaining_energy = maximum_energy_of_the_drone
    remaining_weight = maximum_drone_weight

    # metrics = calcul_metric(soltution, remaining_weight)

    # total_energy = round(metrics[0], 2)
    # total_travel_time = round(metrics[1], 2)
    # total_seed_weight = round(metrics[2], 2)

    total_travel_time = 0
    for i in range(0, len(solution)-1):

        a = solution[i]
        b = solution[i+1]

        metric = energy_comp(a, b, remaining_weight)
        remaining_energy -= metric[0]
        remaining_weight = metric[2]
        total_travel_time += metric[1]

        print('print me metric: ', metric)

        # if remaining_energy <= 0 and remaining_weight <= drone_weight:
        #     solution_representation[1][i] = 1
        #     solution_representation[2][i] = 1
        #     solution_representation[3][i] = round(remaining_energy, 2)
        #     solution_representation[4][i] = round(remaining_weight, 2)
        #
        #     remaining_energy = maximum_energy_of_the_drone
        #     remaining_weight = maximum_drone_weight

        if remaining_energy <= 0:
            solution_representation[1][i] = 1
            # solution_representation[3][i] = round(remaining_energy, 2)

            remaining_energy = maximum_energy_of_the_drone


        if remaining_weight <= drone_weight:
            solution_representation[2][i] = 1
            # solution_representation[4][i] = round(remaining_weight, 2)

            remaining_weight = maximum_drone_weight


        print('remaining_energy: ', remaining_energy)
        print('remaining_weight: ', remaining_weight)
        print('total travel time: ', total_travel_time)

    nbre_sc = sum(solution_representation[1])
    nbre_scs = sum(solution_representation[2])

    print('cs_list: ', solution_representation[1])
    print('scs_list: ', solution_representation[2])
    print('energy_list: ', solution_representation[3])
    print('drone_list: ', solution_representation[4])

    return nbre_sc, nbre_scs, solution_representation



# if __name__ = '__main__':

##Drone specs##
# maximum_drone_energy = 4000  #*3600
drone_weight = 19 ## en Kg
maximum_take_off_weight = 45
take_off_weight = [19, 36]
flight_time = [10, 25]
overall_size = 1650*1650*580

feed_rate = 18.33
dosage = 25
drag_coeff = 0.0613
drone_width = 580*0.01
air_density = 1.2754 ##kg / m³
front_area = 1650*580*0.0001
maximum_energy_of_the_drone = float(4000*600)  # 4000W pour un flight time de 10-25min
maximum_drone_weight = 36

def execution_main():
    # poly_def()
    # path()
    solution = trajectory()
    _, spread_width, _ = nombre_BF()
    f_objective(solution, maximum_energy_of_the_drone, maximum_drone_weight, spread_width, drone_weight)


execution_main()
