from sympy import Point, Line, Polygon, Ray, Segment
import copy

# e1, e2, e3, e4, e5, e6, e7, e8, e9, e10, e11, e12 =(4, 10), (8, 10), (8, 8), (4, 8), (4, 7),\
#                                                    (8, 7),(9, 5), (9, 2), (6, 5), (6, 2), (1, 4), (1, 8)

# arcs = [('e1', 'e2'), ('e2', 'e3'), ('e3', 'e4'), ('e4', 'e5'), ('e5', 'e6'), ('e6', 'e7'),
#         ('e7', 'e8'), ('e8', 'e9'), ('e9', 'e10'), ('e10', 'e11'), ('e11', 'e12'), ('e12', 'e1')]

lis = [(4, 10), (8, 10), (8, 8), (4, 8), (4, 7), (8, 7),(9, 5), (9, 2), (6, 5), (6, 2), (1, 4), (1, 8)]
p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12 = map(Point, lis)
liste0 = p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12
liste1 = copy.deepcopy(liste0)
poly = Polygon(*liste0)
print(poly)

arcs = [(p1, p2), (p2, p3), (p3, p4), (p4, p5), (p5, p6), (p6, p7),
         (p7, p8), (p8, p9), (p9, p10), (p10, p11), (p11, p12), (p12, p1)]

num = len(liste0)
concave_points = []
concave_index = []
witness_list = []
for i in range(num):
    witness_list.append(0)


def ccw(A, B, C):
    """Tests whether the turn formed by A, B, and C is ccw"""
    return (B.x - A.x) * (C.y - A.y) > (B.y - A.y) * (C.x - A.x)

for i in range(num):
    if (i == 0):
         a, b, c = liste0[num-1], liste0[i], liste0[i+1]
    elif (i == num-1):
       a, b, c = liste0[i-1], liste0[i], liste0[0]
    else:
         a, b, c = liste0[i-1], liste0[i], liste0[i+1]
    if ccw(a, b, c):
        concave_points.append(liste0[i])
        concave_index.append(i)
        print(i+1, ':', liste0[i])

index = concave_index[2]

print(liste0[index])
if index == 0:
    r = Ray(liste0[num - 1], liste0[index])
elif index == num:
    r = Ray(liste0[0], liste0[index])
else:
    r = Ray(liste0[index + 1], liste0[index])
print(r)
show_inter = r.intersection(poly)
short_dist = 1000000
near = 0
for i in range(len(show_inter)-1):
    dist = liste0[index].distance(show_inter[i])
    if short_dist > dist:
        short_dist = dist
        near = i
    print(short_dist, dist)
print(short_dist, near, show_inter[near])

print(show_inter)
inter_seg = 0
segment = 0
for cpt in range(num):
    if cpt == num:
        j = 0
    else:
        j = cpt + 1
    segment = Segment(liste0[cpt], liste0[j])
    dist = show_inter[near].distance(segment)
    if dist == 0:
        inter_seg = cpt
        break
print(segment)
# for cpt in range(num):
#         if cpt == num:
#                 j = 0
#         else:
#                 j = cpt + 1
#         segment = Segment(liste[cpt], liste[j])
#         dist = show_inter[near].distance(segment)
#         if dist == 0:
#                 # inter_seg = cpt
#                 if (show_inter[near] == liste[cpt]) or (show_inter[near] == liste[j]):
#                     arcs.insert(index, (liste[index], show_inter[near]))
#                 else:
#                     arcs.insert(index, (liste[index], show_inter[near]))
#                     arcs.insert(cpt + 1, (liste[cpt], show_inter[near]))
#                     arcs.insert(cpt + 2, (show_inter[near], liste[j]))
#
#                 break
