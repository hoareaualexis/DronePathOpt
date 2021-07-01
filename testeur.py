import shapely
import shapely.geometry as shgeo
from sympy import Point, Line, Polygon, Ray, Segment
# from shapely.geometry import Polygon
import numpy as np
from matplotlib.path import Path
import math

import copy
# lis = [(4, 10), (8, 10), (8, 8), (4, 8), (4, 7), (8, 7), (9, 5), (9, 2), (6, 5), (6, 2), (1, 4), (1, 8)]
# p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11 = map(Point, lis)
# liste = [p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11]

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


# lis = [(1, 3), (5, 1), (9, 3), (9, 7), (5, 8), (1, 7)]
lis = [(1, 1), (10, 1), (10, 7), (1, 7)]
p0, p1, p2, p3 = map(Point, lis)
liste = [p0, p1, p2, p3]
poly = Polygon(*liste)
# shapely_poly = shapely.geometry.Polygon(liste)

# create_field(liste, step=0.03, direction='ccw')

# seg =0


def long_cote(liste):
    long_edge = 0
    index = 0
    for i in range(len(liste)-1):
        dist = math.sqrt((liste[i][0]-liste[i+1][0])**2 + (liste[i][1] - liste[i+1][1])**2)
        # dist = liste[i].distance(liste[i+1])
        if long_edge <= dist:
            long_edge = dist
            # seg = Segment(liste[i], liste[i+1])
            index = i

    return index

#Calcul des coors du vecteur directeur et normal
def vect_translation_coors(liste):
    index = long_cote(liste)
    u_x = liste[index][0] - liste[index+1][0]
    u_y = liste[index][1] - liste[index+1][1]
    print('listage: ', liste[index], liste[index+1])

    n_x = u_y
    n_y = -u_x
    norme = lambda x, y: float(math.sqrt(x**2 + y**2))
    norm_n = norme(n_x, n_y)
    width_spread = 2

    t_x = float((width_spread/norm_n)*n_x)
    t_y = float((width_spread/norm_n)*n_y)

    radius = norme(t_x, t_y)
    print('radius norm: ', radius)

    print('coords translation: ', t_x, 'et', t_y)
    return t_x, t_y


#Determination du signe du vecteur translation

def isPositive(liste):

    index = long_cote(liste)
    t_x, t_y = vect_translation_coors(liste)

    a_x = float(liste[index][0] + t_x)
    a_y = float(liste[index][1] + t_y)

    b_x = float(liste[index+1][0] + t_x)
    b_y = float(liste[index+1][1] + t_y)

    point_A, point_B = (a_x, a_y), (b_x, b_y)

    # line = [point_A, point_B]
    line = Line(point_A, point_B)
    # shapely_line = shapely.geometry.LineString(line)

    # intersection_line = list(shapely_poly.intersection(shapely_line).coords)
    intersection_line = poly.intersection(line)

    dim_list = len(intersection_line)
    print(intersection_line)
    if (dim_list == 0):
        return False
    else:
        return True
print(isPositive(lis))

def path(liste):

    index = long_cote(liste)
    t_x, t_y = vect_translation_coors(liste)
    lines_path = [Point(liste[index][0], liste[index][1]), Point(liste[index+1][0],liste[index+1][1])]

    if isPositive(liste):
        a_x = float(liste[index][0] + t_x)
        a_y = float(liste[index][1] + t_y)

        b_x = float(liste[index + 1][0] + t_x)
        b_y = float(liste[index + 1][1] + t_y)

        point_A, point_B = (a_x, a_y), (b_x, b_y)

        # line = [point_A, point_B]
        # shapely_line = shapely.geometry.LineString(line)
        #
        # intersection_line = list(shapely_poly.intersection(shapely_line).coords)
        line = Line(point_A, point_B)

        intersection_line = poly.intersection(line)
        dim_list = len(intersection_line)
        print('Premier test: ', intersection_line)

        lines_path.append(intersection_line[0])
        lines_path.append(intersection_line[1])

        while True:
            if dim_list == 2:
                a_x = float(intersection_line[0][0] + t_x)
                a_y = float(intersection_line[0][1] + t_y)

                b_x = float(intersection_line[1][0] + t_x)
                b_y = float(intersection_line[1][1] + t_y)
            else:
                a_x = float(intersection_line[0]._args[0][0] + t_x)
                a_y = float(intersection_line[0]._args[0][1] + t_y)

                b_x = float(intersection_line[0]._args[1][0] + t_x)
                b_y = float(intersection_line[0]._args[1][1] + t_y)

            point_A, point_B = (a_x, a_y), (b_x, b_y)

            line = Line(point_A, point_B)

            intersection_line = poly.intersection(line)

            type_seg = type(Segment((1, 1), (10, 1)))
            dim_list = len(intersection_line)  # taille de l'ensemble de point d'intersection

            if type(intersection_line[0]) == type_seg:
                lines_path.append(intersection_line[0]._args[0])
                lines_path.append(intersection_line[0]._args[1])
                # dim_list = 2
            elif dim_list == 2:
                lines_path.append(intersection_line[0])
                lines_path.append(intersection_line[1])
            elif dim_list == 1 and type_seg != type(intersection_line[0]):
                lines_path.append(intersection_line[0])

            print('intersection lines:', intersection_line)

            print('lines_path: ', lines_path)

            if dim_list != 2:
                break

    # else:
    #     a_x = float(liste[index][0] - t_x)
    #     a_y = float(liste[index][1] - t_y)
    #
    #     b_x = float(liste[index + 1][0] - t_x)
    #     b_y = float(liste[index + 1][1] - t_y)
    #
    #     point_A, point_B = (a_x, a_y), (b_x, b_y)
    #
    #     line = [point_A, point_B]
    #     shapely_line = shapely.geometry.LineString(line)
    #
    #     intersection_line = list(shapely_poly.intersection(shapely_line).coords)
    #     dim_list = len(intersection_line)
    #
    #     while dim_list == 2:
    #         lines_path.append(intersection_line[0])
    #         lines_path.append(intersection_line[1])
    #
    #         a_x = float(intersection_line[0][0] - t_x)
    #         a_y = float(intersection_line[0][1] - t_y)
    #
    #         b_x = float(intersection_line[1][0] - t_x)
    #         b_y = float(intersection_line[1][1] - t_x)
    #
    #         point_A, point_B = (a_x, a_y), (b_x, b_y)
    #
    #         line = [point_A, point_B]
    #         shapely_line = shapely.geometry.LineString(line)
    #         intersection_line = list(shapely_poly.intersection(shapely_line).coords)
    #
    #         dim_list = len(intersection_line)
    # print('lines_path', lines_path)
    return lines_path

path(lis)
# print(point_A, point_B)
# show_inter = list(poly.intersection(Line(point_A, point_B)))

# print('listage des point inter: ', show_inter)
create_field (lis, step= 0.1, direction = 'cw')
