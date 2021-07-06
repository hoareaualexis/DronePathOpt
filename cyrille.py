# take second element for sort
def takeSecond(elem):
    return elem[1]

# random list
random = [(2, 2), (3, 4), (4, 1), (1, 3)]

# sort list with key
random.sort(key=takeSecond)

# print list
print('Sorted list:', random)

# sorting using custom key
employees = [
    {'Name': 'Alan Turing', 'age': 25, 'salary': 10000},
    {'Name': 'Sharon Lin', 'age': 30, 'salary': 8000},
    {'Name': 'John Hopkins', 'age': 18, 'salary': 1000},
    {'Name': 'Mikhail Tal', 'age': 40, 'salary': 15000},
]

# custom functions to get employee info
def get_name(employee):
    return employee.get('Name')


def get_age(employee):
    return employee.get('age')


def get_salary(employee):
    return employee.get('salary')


# sort by name (Ascending order)
employees.sort(key=get_name)
print(employees, end='\n\n')

# sort by Age (Ascending order)
employees.sort(key=get_age)
print(employees, end='\n\n')

# sort by salary (Descending order)
employees.sort(key=get_salary, reverse=True)
print(employees, end='\n\n')

# sorting using custom key
employees = [
    {'Name': 'Alan Turing', 'age': 25, 'salary': 10000},
    {'Name': 'Sharon Lin', 'age': 30, 'salary': 8000},
    {'Name': 'John Hopkins', 'age': 18, 'salary': 1000},
    {'Name': 'Mikhail Tal', 'age': 40, 'salary': 15000},
]

# sort by name (Ascending order)
employees.sort(key=lambda x: x.get('Name'))
print(employees, end='\n\n')

# sort by Age (Ascending order)
employees.sort(key=lambda x: x.get('age'))
print(employees, end='\n\n')

# sort by salary (Descending order)
employees.sort(key=lambda x: x.get('salary'), reverse=True)
print(employees, end='\n\n')


def intersection_points(lis):

    concaves_pt = concave_points(lis)
    liste, arcs, poly = poly_def(lis)
    total_arcs, total_liste = [], []

    n = len(liste)

    nbre = len(concaves_pt)

    config_list = list(itertools.product([0, 1], repeat=nbre))

    print(config_list)

    for conf in config_list:

        conf_arcs = copy.deepcopy(arcs)

        conf_liste = copy.deepcopy(liste)

        arcs_intern = []
        j = 0
        for con_pt in concaves_pt:

            index = liste.index(con_pt)

            if conf[j] == 0:
                r = section_droite1(liste, n, index)
            else:
                r = section_droite2(liste, n, index)
            j += 1

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

                near_point = short_distance(show_inter, conf_liste, index)

            # print('semi_droite and inter_point : ', r, 'et', near_point)
            # print('show_inter: ', show_inter, '\n')
            ##Ajout du nouvel arc
            arcs_add(near_point, index, conf_arcs, conf_liste, arcs_intern)

        total_arcs.append(conf_arcs)
        total_liste.append(conf_liste)
    for arcs_ in total_arcs:
        for sh in arcs_:
            print(sh)
        print('\n')

    return total_arcs, total_liste





