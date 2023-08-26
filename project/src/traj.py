import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
def qd(time):
    x = np.array([0,
                (4664065662093477*time**5)/590295810358705651712 - time**4/2025 + (22*time**3)/2025 - (8*time**2)/81 + (32*time)/81 - 47/81,
                1,
                -(4664065662093477*time**5)/590295810358705651712 + (17*time**4)/10125 - (286*time**3)/2025 + (476*time**2)/81 - (9800*time)/81 + 80000/81,
                0,
                0])

    y = np.array([0,
                0,
                (4664065662093477*time**5)/590295810358705651712 - (11*time**4)/10125 + (118*time**3)/2025 - (616*time**2)/405 + (1568*time)/81 - 7808/81,
                1,
                -(4664065662093477*time**5)/590295810358705651712 + (23*time**4)/10125 - (526*time**3)/2025 + (1196*time**2)/81 - (33800*time)/81 + 5159302209836171/1099511627776,
                0])

    z = np.array([(time**3*(6*time**2 - 75*time + 250))/3125, 
                    1,
                    1,
                    1,
                    1,
                    1])

    x_dot = np.array([0,
 
(23320328310467385*time**4)/590295810358705651712 - (4*time**3)/2025 + (22*time**2)/675 - (16*time)/81 + 32/81,
 
0,
 
- (23320328310467385*time**4)/590295810358705651712 + (68*time**3)/10125 - (286*time**2)/675 + (952*time)/81 - 9800/81,
 
0,
0])

    y_dot= np.array([0,
 
0,
 
(23320328310467385*time**4)/590295810358705651712 - (44*time**3)/10125 + (118*time**2)/675 - (1232*time)/405 + 1568/81,
 
0,
 
- (23320328310467385*time**4)/590295810358705651712 + (92*time**3)/10125 - (526*time**2)/675 + (2392*time)/81 - 33800/81,
0])

    z_dot = np.array([(6*time**4)/625 - (12*time**3)/125 + (6*time**2)/25,
 
0,
 
0,
 
0,
 
0,
0])

    x_ddot = np.array([0,
 
(23320328310467385*time**3)/147573952589676412928 - (4*time**2)/675 + (44*time)/675 - 16/81,
 
0,
 
- (23320328310467385*time**3)/147573952589676412928 + (68*time**2)/3375 - (572*time)/675 + 952/81,
 
0,
0])

    y_ddot = np.array([0,
 
0,
 
(23320328310467385*time**3)/147573952589676412928 - (44*time**2)/3375 + (236*time)/675 - 1232/405,
 
0,
 
- (23320328310467385*time**3)/147573952589676412928 + (92*time**2)/3375 - (1052*time)/675 + 2392/81,
0])

    z_ddot = np.array([(24*time**3)/625 - (36*time**2)/125 + (12*time)/25,
 
0,
 
0,
 
0,
 
0,
0])

    if time <= 5:
        idx = 0
    elif 5 < time <= 20:
        idx = 1
    elif 20 < time <= 35:
        idx = 2
    elif 35 < time <= 50:
        idx = 3
    elif 50 < time <= 65:
        idx = 4
    else:
        idx = 5

    return x[idx], y[idx], z[idx], x_dot[idx], y_dot[idx], z_dot[idx], x_ddot[idx], y_ddot[idx], z_ddot[idx]

# print(qd(35))
# x=[];
# y=[];
# z=[];
# x_dot=[];
# y_dot=[];
# z_dot=[];
# x_ddot=[];
# y_ddot=[];
# z_ddot=[];
# t_list = np.linspace(0.0,65.0, 6500)
# for t in t_list:
#     desired = qd(t)
#     x.append(desired[0])
#     y.append(desired[1])
#     z.append(desired[2])
#     x_dot.append(desired[3])
#     y_dot.append(desired[4])
#     z_dot.append(desired[5])
#     x_ddot.append(desired[6])
#     y_ddot.append(desired[7])
#     z_ddot.append(desired[8])
# fig, axs = plt.subplots(3,3)
# fig.suptitle('Vertically stacked subplots')
# axs[0,0].plot(t_list, x)
# axs[0,1].plot(t_list, x_dot)
# axs[0,2].plot(t_list, x_ddot)
# axs[1,0].plot(t_list, y)
# axs[1,1].plot(t_list, y_dot)
# axs[1,2].plot(t_list, y_ddot)
# axs[2,0].plot(t_list, z)
# axs[2,1].plot(t_list, z_dot)
# axs[2,2].plot(t_list, z_ddot)
# plt.show()