"""this is file docstring"""
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import math as m


def transformation_DHrow(
        link_length: float, link_twist: float, link_offset: float, joint_angle: float) -> np.array:
    """ This function takes the 4 parameters (a0, alpha0, d1, theta1) from a row of modified DH table and outputs the
    transformation matrix (numpy array) from {i-1} to {i} with respect to {i-1]
    """
    a0 = link_length
    alpha0 = link_twist
    d1 = link_offset
    theta1 = joint_angle
    T_0_1 = np.array([[m.cos(theta1), -m.sin(theta1), 0, a0], [m.sin(theta1)*m.cos(alpha0), m.cos(theta1)*m.cos(alpha0),
    -m.sin(alpha0), -m.sin(alpha0)*d1], [m.sin(theta1)*m.sin(alpha0), m.cos(theta1)*m.sin(alpha0), m.cos(alpha0),
    m.cos(alpha0)*d1], [0, 0, 0, 1]])
    return T_0_1


def transformation_via_DHtable(DHtable: np.ndarray) -> np.array:
    """ This function takes any number of modified DH table rows as NumPy Array - where the rows are each link and the
    columns are link length(a0), link twist(alpha0), link offset(d1), and joint angle(theta1), in that order -
    and the output is the transformation matrix from the base frame {0} to the end effector frame {EE}
    """
    size_dh = DHtable.shape  # outputs tuple (r,c)
    num_rows = size_dh[0]
    EEtransformation = np.identity(4)  # initiate matrix and allocate space
    r = 0
    while r < num_rows:
        row = DHtable[r, :]
        # transformation about "current axis"
        EEtransformation = np.matmul(EEtransformation, transformation_DHrow(row[0], row[1], row[2], row[3]))
        r = r + 1
    return EEtransformation


def rrprrr_DHtable(theta1: float, theta2: float, d2: float, d3: float, theta4: float, theta5: float, theta6: float)\
        -> np.ndarray:
    """ This function forms the proper DH table based on the mutable joint parameters - angles are in degrees and
    distances are in TODO units
    """
    dhTable = np.array([[0, 0, 0, m.radians(theta1)], [0, m.radians(-90), d2, m.radians(90+theta2)], [0, m.radians(90),
                        d3, 0], [0, 0, 0, m.radians(theta4)], [0, m.radians(-90), 0, m.radians(90+theta5)], [0,
                        m.radians(90), 0, m.radians(theta6)]])
    return dhTable


# MAIN SCRIPT
# Joint Limitations
theta1_limit_bound = 180  # degrees
theta2_limit_bound = 90  # degrees
d3_limit_low = 1  # length?
d3_limit_high = 3  # length?
theta4_limit_bound = 180  # degrees
theta5_limit_bound = 25  # degrees
theta6_limit_bound = 180  # degrees

# this error control assumes that uses will enter numeric values ONLY
while True:
    theta1_var = float(input('\nPlease enter the angle value between +-{} degrees for Theta1 of the RRPRRR '
                             'Manipulator: '.format(theta1_limit_bound)))
    if abs(theta1_var) <= theta1_limit_bound:
        break
    print('ERROR! Theta1 must be an angle degree between +-{} degrees'.format(theta1_limit_bound))
while True:
    theta2_var = float(input('\nPlease enter the angle value between +-{} degrees for Theta2 of the RRPRRR Manipulator:'
                             ' '.format(theta2_limit_bound)))
    if abs(theta2_var) <= theta2_limit_bound:
        break
    print('ERROR! Theta2 must be an angle degree between +-{} degrees'.format(theta2_limit_bound))
while True:
    d3_var = float(input('\nPlease enter the distance value between {} and {} for d3 of the RRPRRR Manipulator: '
                         .format(d3_limit_low, d3_limit_high)))
    if d3_limit_low <= d3_var <= d3_limit_high:
        break
    print('ERROR! d3 must be a distance within [{}, {}] degrees'.format(d3_limit_low, d3_limit_high))
while True:
    theta4_var = float(input('\nPlease enter the angle value between +-{} degrees for Theta4 of the RRPRRR Manipulator:'
                             ' '.format(theta4_limit_bound)))
    if abs(theta4_var) <= theta4_limit_bound:
        break
    print('ERROR! Theta4 must be an angle degree between +-{} degrees'.format(theta4_limit_bound))
while True:
    theta5_var = float(input('\nPlease enter the angle value between +-{} degrees for Theta5 of the RRPRRR Manipulator:'
                             ' '.format(theta5_limit_bound)))
    if abs(theta5_var) <= theta5_limit_bound:
        break
    print('ERROR! Theta5 must be an angle degree between +-{} degrees'.format(theta5_limit_bound))
while True:
    theta6_var = float(input('\nPlease enter the angle value between +-{} degrees for Theta6 of the RRPRRR Manipulator:'
                             ' '.format(theta6_limit_bound)))
    if abs(theta6_var) <= theta6_limit_bound:
        break
    print('ERROR! Theta6 must be an angle degree between +-{} degrees'.format(theta6_limit_bound))

# generate DH table of RRPRRRR manipulator from user input
# theta1_var = 45
# theta2_var = 45
# d3_var = 3
# theta4_var = 45
# theta5_var = 25
# theta6_var = 45

# ORIGINAL POSITION
theta1_init = 0
theta2_init = 0
d3_init = 1
theta4_init = 0
theta5_init = 0
theta6_init = 0

d2_fixed = 1

# PLOT
fig = plt.figure()
axes = p3.Axes3D(fig)

# SIMULATION PARAMETERS
divi = 80  # number of steps each variable change has

theta1_step = np.linspace(0, theta1_var, divi).reshape([divi, 1])
theta2_step = np.linspace(0, theta2_var, divi).reshape([divi, 1])
d3_step = np.linspace(1, d3_var, divi).reshape([divi, 1])
theta4_step = np.linspace(0, theta4_var, divi).reshape([divi, 1])
theta5_step = np.linspace(0, theta5_var, divi).reshape([divi, 1])
theta6_step = np.linspace(0, theta6_var, divi).reshape([divi, 1])

def manipulator_sim(frame):
    dh_table = rrprrr_DHtable(float(theta1_step[frame]), float(theta2_step[frame]), d2_fixed, float(d3_step[frame]),
                              float(theta4_step[frame]), float(theta5_step[frame]), float(theta6_step[frame]))

    # create all transformations matrices for arm
    T01 = transformation_via_DHtable(dh_table[[0], :])
    T02 = transformation_via_DHtable(dh_table[0:2, :])
    T03 = transformation_via_DHtable(dh_table[0:3, :])
    T04 = transformation_via_DHtable(dh_table[0:4, :])
    T05 = transformation_via_DHtable(dh_table[0:5, :])
    T0EE = transformation_via_DHtable(dh_table[0:6, :])

    # CLEAR PLOT for each frame
    axes.clear()

    # AXIS/PLOT LAYOUT
    floor = -2.5  # Lowest point on robot, hence "floor", since position of {0} is not lowest point
    axes.set_xlabel('X-axis')
    axes.set_xlim3d([-4, 4])
    axes.set_ylabel('Y-axis')
    axes.set_ylim3d([-4, 4])
    axes.set_zlabel('Z-axis')
    axes.set_zlim3d([floor, 4])
    axes.set_title('RRPRRR Manipulator Simulation')

    # BASE LINK (base to R1) - fixed
    axes.plot([0, 0], [0, 0], [floor, floor+.5], linewidth=5, color='black')

    # First LINK (R1 to R2) - variable via Theta1
    link1_mtx = np.array([[0, -1, -1, -1, 0, 0], [0, 0, 0, d2_fixed+1, d2_fixed+1, d2_fixed],
                          [floor+.5, floor+.5, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1]])
    link1 = np.matmul(T01, link1_mtx)
    axes.plot(link1[0, :], link1[1, :], link1[2, :], linewidth=5, color='orange')
    # print(T01)

    # Second LINK (R2 to P3) = variable via Theta2
    link2_mtx = np.array([[0, 0], [0, -float(d3_step[frame])+1-.2], [0, 0], [1, 1]])  # SPECIAL CASE
    link2 = np.matmul(T02, link2_mtx)
    axes.plot(link2[0, :], link2[1, :], link2[2, :], linewidth=3, color='orange')
    # print(T02)

    # Third LINK (P1 to R3)
    link3_mtx = np.array([[0, 0, 0, 0, 0], [0, -.4, -.4, 0, 0], [-0.8, -0.8, -0.45, -0.45, -0.35], [1, 1, 1, 1, 1]])
    link3 = np.matmul(T03, link3_mtx)
    axes.plot(link3[0, :], link3[1, :], link3[2, :], linewidth=2, color='orange')
    # print(T03)

    # Fourth LINK (R3 to R4)
    link4_mtx = np.array([[0, 0, 0, 0], [0, -.4, -.4, -.2], [-.35, -.35, -0, -0], [1, 1, 1, 1]])
    link4 = np.matmul(T04, link4_mtx)
    axes.plot(link4[0, :], link4[1, :], link4[2, :], linewidth=2, color='grey')
    # print(T04)

    # Fifth LINK (R4 to R5)
    link5_mtx = np.array([[0, 0, 0], [0, 0, -0.3], [-.2, 0, 0], [1, 1, 1]])
    link5 = np.matmul(T05, link5_mtx)
    axes.plot(link5[0, :], link5[1, :], link5[2, :], linewidth=2, color='orange')
    # print(T05)

    # Sixth LINK (R5 to EE)
    link6_mtx = np.array([[0, -.2, -.2, 0, 0], [0, 0, 0, 0, 0],
                          [.3, .3, .6, .6, .8], [1, 1, 1, 1, 1]])
    link6 = np.matmul(T0EE, link6_mtx)
    axes.plot(link6[0, :], link6[1, :], link6[2, :], linewidth=2, color='grey')

    # EE
    EE_base_mtx = np.array([[.1, -.1], [0, 0], [.8, .8], [1, 1]])
    EE_base = np.matmul(T0EE, EE_base_mtx)
    axes.plot(EE_base[0, :], EE_base[1, :], EE_base[2, :], linewidth=1, color='black')

    EE1_mtx = np.array([[.05, .05], [0, 0], [.8, 1], [1, 1]])
    EE1 = np.matmul(T0EE, EE1_mtx)
    axes.plot(EE1[0, :], EE1[1, :], EE1[2, :], linewidth=1, color='black')

    EE2_mtx = np.array([[-.05, -.05], [0, 0], [.8, 1], [1, 1]])
    EE2 = np.matmul(T0EE, EE2_mtx)
    axes.plot(EE2[0, :], EE2[1, :], EE2[2, :], linewidth=1, color='black')

    # EE Frame Location {6}
    EE_frame_mtx = np.array([[0, 0.005], [0, 0.005], [0, 0.005], [1, 1]])
    EE_frame = np.matmul(T0EE, EE_frame_mtx)
    axes.plot(EE_frame[0, :], EE_frame[1, :], EE_frame[2, :], linewidth=4, color='red')

    # create all transformations matrices for arm for
    dh_table_init = rrprrr_DHtable(theta1_init, theta2_init, d2_fixed, d3_init, theta4_init, theta5_init, theta6_init)
    T01_init = transformation_via_DHtable(dh_table_init[[0], :])
    T02_init = transformation_via_DHtable(dh_table_init[0:2, :])
    T03_init = transformation_via_DHtable(dh_table_init[0:3, :])
    T04_init = transformation_via_DHtable(dh_table_init[0:4, :])
    T05_init = transformation_via_DHtable(dh_table_init[0:5, :])
    T0EE_init = transformation_via_DHtable(dh_table_init[0:6, :])

    # plot links
    link1 = np.matmul(T01_init, link1_mtx)
    axes.plot(link1[0, :], link1[1, :], link1[2, :], linewidth=1, color='black')
    link2_mtx_init = np.array([[0, 0], [0, -d3_init + 1 - .2], [0, 0], [1, 1]])  # Special Case for link length
    link2 = np.matmul(T02_init, link2_mtx_init)
    axes.plot(link2[0, :], link2[1, :], link2[2, :], linewidth=1, color='black')
    link3 = np.matmul(T03_init, link3_mtx)
    axes.plot(link3[0, :], link3[1, :], link3[2, :], linewidth=1, color='black')
    link4 = np.matmul(T04_init, link4_mtx)
    axes.plot(link4[0, :], link4[1, :], link4[2, :], linewidth=1, color='black')
    link5 = np.matmul(T05_init, link5_mtx)
    axes.plot(link5[0, :], link5[1, :], link5[2, :], linewidth=1, color='black')
    link6 = np.matmul(T0EE_init, link6_mtx)
    axes.plot(link6[0, :], link6[1, :], link6[2, :], linewidth=1, color='black')
    EE_base = np.matmul(T0EE_init, EE_base_mtx)
    axes.plot(EE_base[0, :], EE_base[1, :], EE_base[2, :], linewidth=1, color='black')
    EE1 = np.matmul(T0EE_init, EE1_mtx)
    axes.plot(EE1[0, :], EE1[1, :], EE1[2, :], linewidth=1, color='black')
    EE2 = np.matmul(T0EE_init, EE2_mtx)
    axes.plot(EE2[0, :], EE2[1, :], EE2[2, :], linewidth=1, color='black')
    EE_frame = np.matmul(T0EE_init, EE_frame_mtx)
    axes.plot(EE_frame[0, :], EE_frame[1, :], EE_frame[2, :], linewidth=4, color='red')


# CREATE ANIMATION
time_int = 1  # milliseconds (ms)
# manipulator_animation = animation.FuncAnimation(fig, manipulator_sim, frames=divi, interval=1)
manipulator_animation = animation.FuncAnimation(fig, manipulator_sim, frames=divi, interval=time_int, repeat=False)
# manipulator_animation.save('RRPRRR_sim.mp4')
plt.show()

# OUTPUT DATA
# all transformations relative to previous frame {i-1}
dh_table = rrprrr_DHtable(theta1_var, theta2_var, d2_fixed, d3_var, theta4_var, theta5_var, theta6_var)
T01 = transformation_via_DHtable(dh_table[[0], :])
T12 = transformation_via_DHtable(dh_table[1:2, :])
T23 = transformation_via_DHtable(dh_table[2:3, :])
T34 = transformation_via_DHtable(dh_table[3:4, :])
T45 = transformation_via_DHtable(dh_table[4:5, :])
T56 = transformation_via_DHtable(dh_table[5:6, :])
T0EE = transformation_via_DHtable(dh_table[0:6, :])
print('\nDH Table for RRPRRR:')
print(dh_table)
print('\nTransformation Matrix T01:')
print(T01)
print('\nTransformation Matrix T12:')
print(T12)
print('\nTransformation Matrix T23:')
print(T23)
print('\nTransformation Matrix T34:')
print(T34)
print('\nTransformation Matrix T45:')
print(T45)
print('\nTransformation Matrix T56:')
print(T56)
print('\nEnd Effector Final Position and Orientation:')
print('{XYZ Position}')
print(T0EE[0:3, 3:4])
print('{Orientation}')
print(T0EE[0:3, 0:3])

# VELOCITY KINEMATICS and JACOBIAN
# any position along the manipulator's path to its final point can have its velocity kinematics calculated
movement_matrix = np.concatenate((theta1_step, theta2_step, d3_step, theta4_step, theta5_step, theta6_step), axis=1)
while True:
    pos_index = int(input(
        'Please enter the position integer index from 0 to {} to calculated EE velocities (Notes: all joints reach '
        'their final positions at the same time): '.format(divi-1)))
    if 0 <= pos_index < divi:
        break
    print('ERROR! Position must be chosen 0 to {} degrees'.format(divi-1))

movement_pos = movement_matrix[pos_index, :]
theta1_pos = movement_pos[0]
theta2_pos = movement_pos[1]
d3_pos = movement_pos[2]
theta4_pos = movement_pos[3]
theta5_pos = movement_pos[4]
theta6_pos = movement_pos[5]

dh_table_movement_pos = rrprrr_DHtable(theta1_pos, theta2_pos, d2_fixed, d3_pos, theta4_pos, theta5_pos, theta6_pos)
# all transformations relative to base frame {0}
T01 = transformation_via_DHtable(dh_table_movement_pos[[0], :])
T02 = transformation_via_DHtable(dh_table_movement_pos[0:2, :])
T03 = transformation_via_DHtable(dh_table_movement_pos[0:3, :])
T04 = transformation_via_DHtable(dh_table_movement_pos[0:4, :])
T05 = transformation_via_DHtable(dh_table_movement_pos[0:5, :])
T06 = transformation_via_DHtable(dh_table_movement_pos[0:6, :])


def skewsym(vector: np.array) -> np.array:
    """ This function returns the skew symmetric matrix of a 3d vector"""
    x = float(vector[0])
    y = float(vector[1])
    z = float(vector[2])
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


# pull rotation vectors from transformation matrixes
Z01 = T01[0:3, 2].reshape([3, 1])
Z02 = T02[0:3, 2].reshape([3, 1])
Z03 = T03[0:3, 2].reshape([3, 1])
Z04 = T04[0:3, 2].reshape([3, 1])
Z05 = T05[0:3, 2].reshape([3, 1])
Z06 = T06[0:3, 2].reshape([3, 1])

T46 = np.matmul(T45, T56)
T36 = np.matmul(T34, T46)
T26 = np.matmul(T23, T36)
T16 = np.matmul(T12, T26)

P16 = T16[0:3, 3].reshape([3, 1])
P26 = T26[0:3, 3].reshape([3, 1])
P36 = T36[0:3, 3].reshape([3, 1])
P46 = T46[0:3, 3].reshape([3, 1])
P56 = T56[0:3, 3].reshape([3, 1])

# create the components of 6xN Jacobian matrix
jv = np.concatenate(((np.dot(skewsym(Z01), P16)), np.dot(skewsym(Z02), P26), Z03, np.dot(skewsym(Z04), P46),
                     np.dot(skewsym(Z05), P56), np.zeros([3, 1])), axis=1)
jw = np.concatenate((Z01, Z02, np.zeros([3, 1]), Z04, Z05, Z06), axis=1)
# combine Jv (3xN) and Jw (3xN) to create Jacobian
jacobian = np.concatenate((jv, jw), axis=0)
print('\nJacobian at the given joint paramters:')
print(jacobian)

# Velocity Kinematic Portion
# need theta_dots - (final-initial)/(time taken) is constant velocity
time = divi*(time_int/1000)
theta1_dot = (theta1_var - theta1_init)/time  # can use equation because constant time is used for all motions
theta2_dot = (theta2_var - theta2_init)/time  # mm/second
d3_dot = (d3_var - d3_init)/time
theta4_dot = (theta4_var - theta4_init)/time
theta5_dot = (theta5_var - theta5_init)/time
theta6_dot = (theta6_var - theta6_init)/time

q_dot = np.array([[theta1_dot], [theta2_dot], [d3_dot], [theta4_dot], [theta5_dot], [theta6_dot]])

EE_dot = np.matmul(jacobian, q_dot)
EEv = EE_dot[0:3, :]
EEw = EE_dot[3:6, :]
print('EE Linear Velocity')
print(EEv)
print('EE Angular Velocity')
print(EEw)
