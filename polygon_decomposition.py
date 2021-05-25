from sympy import Point, Line, Polygon, Ray, Segment
from math import pi
import math
# p1, p2, p3, p4, p5, p6, p7 = map(Point, [(3, 9), (9, 9), (12, 5), (8, 5), (8, 2), (5, 4), (1, 4)])
# poly = Polygon(p1, p2, p3, p4, p5, p6, p7)
# l = Line(p4, p5)
# cutSection = poly.cut_section(l)
# poly1, poly2 = cutSection
# # print(cutSection)
# # print(poly1)
# l1 = Line(p5, p6)
# cutSection1 = poly2.cut_section(l1)
# poly3, poly4 = cutSection1
# print(poly3)
# print(poly4)

e1, e2, e3, e4, e5, e6, e7, e8, e9, e10, e11, e12 =(4, 10), (8, 10), (8, 8), (4, 8), (4, 7),\
                                                   (8, 7),(9, 5), (9, 2), (6, 5), (6, 2), (1, 4), (1, 8)

liste0 = [e1, e2, e3, e4, e5, e6, e7, e8, e9, e10, e11, e12]

p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12 = map(Point, liste0)

liste1 = p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12

poly = Polygon(*liste1)
print(liste1)

##Recherche d'angle concave##
num = len(liste1)
concave_points = []
concave_index = []

def ccw(A, B, C):
    """Tests whether the turn formed by A, B, and C is ccw"""
    return (B.x - A.x) * (C.y - A.y) > (B.y - A.y) * (C.x - A.x)

for i in range(num):
    if (i == 0):
         a, b, c = liste1[num-1], liste1[i], liste1[i+1]
    elif (i == num-1):
       a, b, c = liste1[i-1], liste1[i], liste1[0]
    else:
         a, b, c = liste1[i-1], liste1[i], liste1[i+1]
    if ccw(a, b, c):
        concave_points.append(liste1[i])
        concave_index.append(i)
        print(i+1, ':', liste1[i])

##Verification du nombre de points d'intersection##
for j in range(len(concave_index)):
    index = concave_index[i]
    if index == 0:
        r = Ray(liste1[num-1], liste1[index])
    elif index == num:
        r = Ray(liste1[0], liste1[index])
    else:
        r = Ray(liste1[index-1], liste1[index])
    show_inter = r.intersection(poly)
    minter = 0
    if len(show_inter)>1:
       for cpt in range(num):
           if cpt+1 > num:
               j = 0
           else:
               j = cpt + 1
           segment = Segment(liste1[cpt], liste1[j])
           dist = show_inter[0].distance(segment)
           if dist == 0:
               minter = cpt
               break
         




# poly = Polygon(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12)
# l = Line(p9, p10)
# r = (p10, p9)
# print(poly)
# showinter = r.intersection(poly)
# print(showinter[0])
# seg= Segment(p5, showinter[0])
# seg= Segment(p5, p6)
# dist = showinter[0].distance(seg)
# print(dist)
# tab = [p1, p2]
# dist1 = tab[0].distance(tab[1])
# print(dist1)
# showinter = seg.intersection(poly)
# print(showinter)
# cutSection = poly.cut_section(seg)

# print(cutSection)