from sympy import Point, Polygon, Ray, Segment, Line
import copy

vector = [
    [Point(3, 16), Point(7, 13), Point(3, 10), Point(1, 14)],
    [Point(3, 16), Point(7, 13), Point(3, 10)],
    [Point(3, 16), Point(3, 10), Point(1, 14)],
    [Point(7, 13), Point(11, 16), Point(13, 76/5), Point(13, 7), Point(13, 11/3), Point(6, 6), Point(3, 7), Point(3, 10)],
    [Point(13, 76/5), Point(16, 14), Point(15, 7), Point(13, 7)],
    [Point(6, 5/3), Point(6, 4), Point(6, 6), Point(13, 11/3), Point(13, 2), Point(7, 1)],
    [Point(6, 4), Point(6, 5/3), Point(4, 3)]
]

vector1 = [
        [Point(2, 7), Point(4, 7), Point(6, 7), Point(8, 7), Point(10, 7), Point(10, 5), Point(10, 3),
        Point(10, 1), Point(8, 1), Point(6, 1), Point(4, 1), Point(2, 1), Point(2, 3), Point(2, 5)],
        [Point(2, 7), Point(4, 7), Point(6, 7), Point(8, 7), Point(10, 7), Point(10, 5), Point(10, 3),
        Point(10, 1), Point(8, 1), Point(6, 1), Point(4, 1), Point(2, 1), Point(2, 3), Point(2, 5)],
        [Point(2, 7), Point(4, 7), Point(6, 7), Point(8, 7), Point(10, 7), Point(10, 5), Point(10, 3),
        Point(10, 1), Point(8, 1), Point(6, 1), Point(4, 1), Point(2, 1), Point(2, 3), Point(2, 5)],
        # [Point(7, 13), Point(11, 16), Point(13, 76/5), Point(13, 7), Point(13, 11/3), Point(6, 6), Point(3, 7),
        #  Point(3, 10)],
       ]

typ_pt = type(Point(0, 0))
indice = 0
for vec in vector:

    if len(vec) <= 3:
        indice += 1
        continue
    tmp = []
    for i in range(2, len(vec)):
         l1 = Line(vec[i-2], vec[i-1])
         l2 = Line(vec[i-1], vec[i])
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

for v in vector:
    print('vec:', v)
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

for copy_ in copy_vec:
    print('exec: ', copy_)
