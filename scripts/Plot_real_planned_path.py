import numpy as np
from matplotlib import pyplot as plt, patches

# multiply a list with 100
def mult_100(list):
    ll = []
    for i in range(len(list)):
        ll.append([list[i][0] * 100, list[i][1] * 100])

    return ll


# save a list in text data
def save_list(liste, dateiname):
    with open(dateiname, 'w') as datei:
        for element in liste:
            datei.write(str(element) + '\n')

# merge two dict
def merge_dicts(dict1, dict2):
    # Create a copy of dict1
    merged_dict = dict1.copy()

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

# Cube's positions and color
cubes_rot_blue = {
    "w1": [50 , 60 , "BLUE"],
    "w2": [100  , 70, "BLUE"],
    "w3": [40 , 110, "BLUE"],
    "w4": [80 ,  30, "RED"],
    "w5": [120 ,  30, "RED"],
    "w6": [110 ,  110, "RED"]

}

# find out which cubes's positions are smaler aer bigger than 75
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
# smaller than or equal 75
sorted_x_smaller75_d = dict(sorted(x_smaller75_d.items(), key=lambda x: int(str(x[1][0]))))
sorted_y_smaller75_d = dict(sorted(sorted_x_smaller75_d.items(), key=lambda x: int(str(x[1][1]))))

# bigger than 75
sorted_x_bigger75_d = dict(sorted(x_bigger75_d.items(), key=lambda x: int(str(x[1][0])), reverse=True))
sorted_y_bigger75_d = dict(sorted(sorted_x_bigger75_d.items(), key=lambda x: int(str(x[1][1])),reverse=True))

# merge the two dict together
merged_sorted = merge_dicts(sorted_y_smaller75_d, sorted_y_bigger75_d)
x1 = [0 , 1]
x2 = [0, 6]

# current_postion
r=10
b=10

# dict for the planned and "driven" trajectory
planned_trajectory = {}

# Set plot
fig = plt.figure()
ax = fig.add_subplot()


#plot the cubes
width = 2
height = 2
for k in cubes_rot_blue:
    f = cubes_rot_blue[k]
    # Calculation of the vertices of the rectangle
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

# plot the planned trajectory
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

    # circle radius around the cube
    radius = 12.5

    # calculate the current position depending on the position of the next cube
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


    # plot a circle around the cube
    circle2 = plt.Circle((j[0], j[1]), radius, fill=False  , edgecolor='black', facecolor='yellow')

    planned_trajectory['point'+ i] = []
    planned_trajectory["point" + i] += [x1[0], x2[0]]
    planned_trajectory["point" + i] += [x1[1], x2[1]]
    planned_trajectory['kreis' + i] = [j[0], j[1], radius ]

    # if the jetbot should drive a curve before it go to the next cube
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



    # after the last cube we  have to go back to the start point
    # calculate the path to the start point
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
f = open("pos_real.txt", "r")
ids = []
for i in f.readlines():
    z = i[0]
    h = i[1]
    l = i[2:7]
    k =  i[14:19]

    ids.append(float(l))
    ids.append(float (k))


new_list = [ids[i:i+2] for i in range(0, len(ids), 2)]
t_pos_real = mult_100(new_list)




for k in merged_sorted :
    a = merged_sorted[k]
    circle2 = plt.Circle((a[0], a[1]), 10, fill=False, edgecolor='black', facecolor='yellow')


for h in t_pos_real:
    x_coords = h[0]

for j in t_pos_real:
    y_coords = j[1]

# draw arrows
for i in range(len(t_pos_real) - 1):
        dx = t_pos_real[i + 1][0] - t_pos_real[i][0]
        dy = t_pos_real[i + 1][1] - t_pos_real[i][1]
        plt.arrow(t_pos_real[i][0], t_pos_real[i][1], dx, dy, length_includes_head=True, head_width=1, color='red')

plt.arrow(10, 10, 1, 1 ,length_includes_head=True, head_width=1, color='red',label='Real_Trajectory')

# Convert dictionary to list
planned_trajectory_list = planned_trajectory.items()
merged_sorted_list = merged_sorted.items()



# cm to m
merged_sorted_result = [(key, [value / 100 if isinstance(value, float  ) else value for value in values]) for key, values in merged_sorted_list]
merged_sorted_result = [(key, [value / 100 if isinstance(value, int  ) else value for value in values]) for key, values in merged_sorted_result]
planned_trajectory_result = [(key, [value / 100 if isinstance(value, float) else value for value in values]) for key, values in planned_trajectory_list]
planned_trajectory_result = [(key, [value / 100 if isinstance(value, int) else value for value in values]) for key, values in planned_trajectory_result]

print("planned cubes in order their position in cm")
print(merged_sorted_list)
print("planned cubes in order their position in m")
print(merged_sorted_result)
print("planned trajectory in order, positions in cm")
print(planned_trajectory_list)
print("planned trajectory in order, positions in m")
print(planned_trajectory_result)

# save the planned cube in order
dateiname = 'merged_sorted_result.txt'
save_list(merged_sorted_result, dateiname)

# save the planned trajectory
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
