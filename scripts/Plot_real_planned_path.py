import math
import numpy as np
from matplotlib import pyplot as plt, patches

# mal * 100
def mult_100(list):
    ll = []
    for i in range(len(list)):
        ll.append([list[i][0] * 100, list[i][1] * 100])

    return ll

#Read a text Data
def Read_text_data(dateiname):
    liste = []
    list = []
    with open(dateiname, 'r') as datei:
        for zeile in datei:
            werte = zeile.strip().split()
            for wert in werte:
                try:
                    liste.append(float(wert))
                except ValueError:
                    pass

        for i in range(0, len(liste), 2):

            wert1 = float(liste[i])
            wert2 = float(liste[i+1])
            list.append([wert1, wert2])

    return list
#save a list in text data
def save_list(liste, dateiname):
    with open(dateiname, 'w') as datei:
        for element in liste:
            datei.write(str(element) + '\n')

def merge_dicts(dict1, dict2):
    merged_dict = dict1.copy()  # Create a copy of dict1

    for key, value in dict2.items():
        if key in merged_dict:
            if isinstance(value, dict) and isinstance(merged_dict[key], dict):
                merged_dict[key] = merge_dicts(merged_dict[key], value)
            elif isinstance(value, list) and isinstance(merged_dict[key], list):
                merged_dict[key].extend(value)
            else:
                merged_dict[key] = value
        else:
            merged_dict[key] = value

    return merged_dict

x1, y1 = [-1, 12], [1, 4]
x2, y2 = [1, 10], [3, 2]


cubes_rot_blue = {
    "w1": [50 , 60 , "BLUE"],
    "w2": [100  , 70, "BLUE"],
    "w3": [40 , 110, "BLUE"],
    "w4": [80 ,  30, "RED"],
    "w5": [120 ,  30, "RED"],
    "w6": [110 ,  110, "RED"]

}

#find out which cubes's positions are smaler aer bigger than 75
x_smaller75 = []
x_bigger75 = []

for x in cubes_rot_blue:
    j = cubes_rot_blue[x]
    if j[0] <= 75:
        x_smaller75.append(x)
    else:
        x_bigger75.append(x)

# separate the dict cubes_rot_blue to tow dict
x_smaller75_d = {}
x_bigger75_d = {}

for ob, value in cubes_rot_blue.items():

    if ob  in x_smaller75:
        x_smaller75_d[ob] = value
    if ob in x_bigger75:
        x_bigger75_d[ob] = value

# sort based on the first digits then on the second digits
#smaller than or equal 75
sorted_x_smaller75_d = dict(sorted(x_smaller75_d.items(), key=lambda x: int(str(x[1][0]))))
sorted_y_smaller75_d = dict(sorted(sorted_x_smaller75_d.items(), key=lambda x: int(str(x[1][1]))))

#bigger than 75
sorted_x_bigger75_d = dict(sorted(x_bigger75_d.items(), key=lambda x: int(str(x[1][0])), reverse=True))
sorted_y_bigger75_d = dict(sorted(sorted_x_bigger75_d.items(), key=lambda x: int(str(x[1][1])),reverse=True))

#merge the two dict together
#merged_sorted = {**sorted_y_smaller75_d, **sorted_y_bigger75_d}
merged_sorted = merge_dicts(sorted_y_smaller75_d, sorted_y_bigger75_d)
x1 = [0 , 1]
x2 = [0, 6]

#current_postion
r=10
b=10

#next_postion

#dict for the planned and "driven" trajectory
planned_trajectory = {}


fig = plt.figure()
ax = fig.add_subplot()


#plot the cubes
width = 2
height = 2
for k in cubes_rot_blue:
    f = cubes_rot_blue[k]
    # Berechnung der Eckpunkte des Rechtecks
    left = f[0] - width/2
    right = f[0]  + width/2
    bottom = f[1]  - height/2
    top = f[1]  + height/2


    if f[2] == "RED":

        # plot colored cube
        plt.plot([left, right, right, left, left], [bottom, bottom, top, top, bottom], color='red')
        plt.fill([left, right, right, left, left], [bottom, bottom, top, top, bottom], color='red', alpha=0.3)
    else:
        # plot colored cube
        plt.plot([left, right, right, left, left], [bottom, bottom, top, top, bottom], color='blue')
        plt.fill([left, right, right, left, left], [bottom, bottom, top, top, bottom], color='blue', alpha=0.3)


g=0
for i in merged_sorted:
    g = g + 1
    # next_postion
    j = merged_sorted[i]

    last_ob = list(merged_sorted.keys())[-1]
    last_ob_x_smaller75_d = list(sorted_y_smaller75_d.keys())[-1]
    last_ob_x_bigger75_d = list(sorted_y_bigger75_d.keys())[-1]
    fisrt_ob_x_bigger75_d = list(sorted_y_bigger75_d.keys())[0]

    # next next position
    if i != last_ob:
        gg = list(merged_sorted.keys())[g]

    jj = merged_sorted[gg]

    # -7.5 damit der JetBot am Kreis haelt und einen Kreis Faefrt und dann den Kreis verlaesst

    radius = 12.5

    if i in x_smaller75_d:
        #from the start point
        if r in range(10, 20) and b in range(10, 20):
            x1 = [r  , j[0]]
            x2 = [b , j[1] -radius]

            if  j[0] >= jj[0]  :
                # current postion
                r = j[0] - radius
                b = j[1]

            if  j[0] < jj[0]  :
                # current postion
                r = j[0] + radius
                b = j[1]

        elif i != last_ob_x_smaller75_d :

            x1 = [r  , j[0] ]
            x2 = [b , j[1] -radius ]

            if j[0] >= jj[0]:
                # current postion
                r = j[0] - radius
                b = j[1]

            if j[0] < jj[0]:
                # current postion
                r = j[0] + radius
                b = j[1]

        else:
            x1 = [r  , j[0] ]
            x2 = [b , j[1] -radius ]

            if j[0] >= jj[0]:
                # current postion
                r = j[0] - radius
                b = j[1]

            if j[0] < jj[0]:
                # current postion
                r = j[0] + radius
                b = j[1]

    if i in x_bigger75_d:
        if i == fisrt_ob_x_bigger75_d:

            x1 = [r  , j[0] -radius ]
            x2 = [b , j[1] ]

            if j[0] >= jj[0]:
                # current postion
                r = j[0] - radius
                b = j[1]

            if j[0] < jj[0]:
                # current postion
                r = j[0] + radius
                b = j[1]

        #elif i ==last_ob_x_bigger75_d:
        #    x1 = [r, j[0] ]
        #    x2 = [b, j[1] + radius ]

            # current postion
        #    r = j[0]
        #    b = j[1]- radius


        else:
            x1 = [r, j[0] ]
            x2 = [b, j[1] + radius ]

            if j[0] >= jj[0]:
                # current postion
                r = j[0] - radius
                b = j[1]

            if j[0] < jj[0]:
                # current postion
                r = j[0] + radius
                b = j[1]



    circle2 = plt.Circle((j[0], j[1]), radius, fill=False  , edgecolor='black', facecolor='yellow')

    planned_trajectory['point'+ i] = []
    planned_trajectory["point" + i] += [x1[0], x2[0]]
    planned_trajectory["point" + i] += [x1[1], x2[1]]
    planned_trajectory['kreis' + i] = [j[0], j[1], radius ]

    if x1[1] != r and x2[1] != b:
        planned_trajectory['kurve' + i] = [r, b]

    line = plt.plot(x1 , x2, color='black' )
    plt.arrow(x1[0], x2[0], x1[1] - x1[0], x2[1]- x2[0],
              width=1, length_includes_head=True, head_width=2, color='black')
    ax.add_patch(circle2)

    # sample at 0, 1/3rd, and 2/3rd of curve
    t = np.linspace(0, 2*np.pi, 100)
    adx0, adx1 = 0, len(t) // 3
    adx2 = adx1 * 2

    arrow0 = x1[adx0 + 1], x2[adx0 + 1], x1[adx0 + 1] - x1[adx0], x2[adx0 + 1] - x2[adx0]

    plt.arrow(*arrow0 , shape='full', lw=0, length_includes_head=True, head_width=0.12)




    if i == last_ob:
        x1 = [j[0]- radius,j[0]- radius]
        x2 = [j[1], 10]
        planned_trajectory['start_point90' ] = []
        planned_trajectory["start_point90" ] += [x1[0], x2[0]]
        planned_trajectory["start_point90"] += [x1[1], x2[1]]


        plt.plot(x1, x2, color='black')
        plt.arrow( (j[0]- radius) ,(j[1]) , (j[0]- radius) - (j[0]- radius),10- j[1],
                  width=1, length_includes_head=True, head_width=2, color='black')

        x1 = [j[0]- radius,10]
        x2 = [10, 10]
        planned_trajectory['start_point' ] = []
        planned_trajectory["start_point" ] += [x1[0], x2[0]]
        planned_trajectory["start_point"] += [x1[1], x2[1]]

        plt.plot(x1, x2, color='black',label='Planned_Trajectory')
        plt.arrow((j[0]- radius) , 10, 10 - (j[0]- radius) ,10- 10,
                  width=1, length_includes_head=True, head_width=2, color='black')


# Read the real position to plot the real Path
#dateiname = 'pos_real.txt'
#t_pos_real = Read_text_data(dateiname)

#t_pos_real = mult_100(t_pos_real)
# Liste der Trajektorie
#t_pos_real= [[13.158985000000001, 15.440338], [15.438499, 13.011807000000001], [32.989996999999995, 12.967989999999999], [32.989996999999995, 12.967989999999999], [40.368067, 36.297913], [40.368067, 36.297913], [29.930975999999998, 39.017629], [29.930975999999998, 39.017629], [29.930975999999998, 39.017629], [32.509121, 47.661659], [32.509121, 47.661659], [62.861005999999996, 36.688861], [62.861005999999996, 36.688861], [56.453892, 39.364759], [62.507729999999995, 68.85332], [62.507729999999995, 68.85332], [54.289337, 73.84623099999999], [54.289337, 73.84623099999999], [60.81860100000001, 78.686155], [60.81860100000001, 78.686155], [45.688602, 97.10982200000001], [45.688602, 97.10982200000001], [38.394224, 108.14814], [38.394224, 108.14814], [38.394224, 108.14814], [51.28135700000001, 105.913174], [51.28135700000001, 105.913174], [39.14327, 129.484127], [39.14327, 129.484127], [50.088072, 124.30224199999999], [70.921434, 119.12391600000001], [70.921434, 119.12391600000001], [145.73676799999998, 146.572958], [145.73676799999998, 146.572958], [107.170949, 108.927975], [107.170949, 108.927975], [107.173024, 108.929502], [102.427786, 120.393873], [102.427786, 120.393873], [96.272192, 112.039519], [79.588519, 102.580241], [79.588519, 102.580241], [79.588519, 102.580241], [81.638734, 97.645633], [81.638734, 97.645633], [73.153473, 79.550208], [73.153473, 79.550208], [78.76659099999999, 85.841144], [100.62335300000001, 82.82868], [100.62335300000001, 82.82868], [121.391293, 82.293913], [121.391293, 82.293913], [121.391293, 82.293913], [124.009592, 77.766755], [124.009592, 77.766755], [119.40481700000001, 51.996971], [119.40481700000001, 51.996971], [115.88224600000001, 53.995093000000004], [97.318553, 47.903939], [97.318553, 47.903939], [92.019661, 38.776517], [92.019661, 38.776517], [92.019661, 38.776517], [103.926249, 43.142956999999996], [103.926249, 43.142956999999996], [76.72797700000001, 8.605307], [10.412435, 9.408408999999999]]

#print("positon real",t_pos_real)
f = open("pos_real.txt", "r")
ids = []
for i in f.readlines():
    z = i[0]
    h = i[1]
    l = i[2:7]
    k =  i[14:19]

    #sub_id = list(map(float,l))
    #sub_id_k = list(map(float, k.split(",")))
    ids.append(float(l))
    ids.append(float (k))


new_list = [ids[i:i+2] for i in range(0, len(ids), 2)]
t_pos_real = mult_100(new_list)




for k in merged_sorted :
    a = merged_sorted[k]
    circle2 = plt.Circle((a[0], a[1]), 10, fill=False, edgecolor='black', facecolor='yellow')
# Koordinaten extrahieren
# x_coords = [coord[0] for coord in t_pos_real]
# y_coords = [coord[1] for coord in t_pos_real]


for h in t_pos_real:
    x_coords = h[0]

for j in t_pos_real:
    y_coords = j[1]

# Pfeile zeichnen
for i in range(len(t_pos_real) - 1):
        dx = t_pos_real[i + 1][0] - t_pos_real[i][0]
        dy = t_pos_real[i + 1][1] - t_pos_real[i][1]
        plt.arrow(t_pos_real[i][0], t_pos_real[i][1], dx, dy, length_includes_head=True, head_width=1, color='red')
        #plt.plot(t_pos_real[i][0], t_pos_real[i][1], marker='o', color='red')

plt.arrow(10, 10, 1, 1 ,length_includes_head=True, head_width=1, color='red',label='Real_Trajectory')

# Convert dictionary to list
planned_trajectory_list = planned_trajectory.items()
merged_sorted_list = merged_sorted.items()




merged_sorted_result = [(key, [value / 100 if isinstance(value, float  ) else value for value in values]) for key, values in merged_sorted_list]
merged_sorted_result = [(key, [value / 100 if isinstance(value, int  ) else value for value in values]) for key, values in merged_sorted_result]
planned_trajectory_result = [(key, [value / 100 if isinstance(value, float) else value for value in values]) for key, values in planned_trajectory_list]
planned_trajectory_result = [(key, [value / 100 if isinstance(value, int) else value for value in values]) for key, values in planned_trajectory_result]


print(merged_sorted_list)
print(merged_sorted_result)
print(planned_trajectory_list)
print(planned_trajectory_result)


dateiname = 'merged_sorted_result.txt'
save_list(merged_sorted_result, dateiname)


dateiname = 'planned_trajectory_result.txt'
save_list(planned_trajectory_result, dateiname)

# Set plot
plt.xlim(0, 149.5)
plt.ylim(0, 149.5)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Arena\n planned path and real Trajectory')
plt.legend()

plt.grid()

plt.show()

