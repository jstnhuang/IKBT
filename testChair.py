from ikbtbasics.ik_classes import *
from ikbtfunctions.ik_robots import * 
from ikbtbasics.kin_cl import *
from math import *
import os

testing = False

robot = 'Chair_Helper'


((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_1, l_2, l_3, l_4)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4'))
((a_2, a_3)) = sp.symbols(('a_2', 'a_3'))
sp.var('Px Py Pz')



#   Get the robot model 
[dh, vv, params, pvals, unknowns] = robot_params(robot)  # see ik_robots.py 

[M, R, unknowns] = kinematics_pickle(robot, dh, params, pvals, vv, unknowns, testing)

# need to type in the poses
pose = {d_1: 2.0, th_2: 30*deg, th_3: 50*deg, th_4: 135*deg, th_5:70*deg}


params_num = {l_1: 2, l_2: 1, l_4: 3} 
#params_num = pvals

l_1 = 2
l_2 = 1
l_4 = 3

def convert_to_degrees(solution_list):
    solution_in_degree = []
    for a_set in solution_list:
        curr_set = []
        for curr_rad in a_set:
            if curr_rad > 2*pi:
                curr_rad = curr_rad % (2*pi)
            curr_set.append(degrees(curr_rad))
        solution_in_degree.append(curr_set)
    return solution_in_degree

def verify_T_matrices(pose_list, variable_template, M, params_num):
    T_mat_list = []

    for one_pose in pose_list:
        pose = {}
        for i in range(len(variable_template)):
            pose[variable_template[i]] = one_pose[i]
        # print pose
        T = forward_kinematics_N_2(M, pose, params_num)
        T_mat_list.append(T)
    return T_mat_list


def forward_kinematics_N_2(M, pose, params):
    pp = pose.copy()
    pp.update(params)    # combine the pose and the params
    # print pp
    T1 = M.T_06.subs(pp)    # substitue for all symbols
    # print T1
    # test to make sure all symbols are substituted with numeric values
    #Num_check(T1)     # this quits if fails

    T2 = np.matrix(T1)   # convert from sympy to numpy matrix
    return T2

#T = forward_kinematics_N(M, pose, pvals)
T = forward_kinematics_N_2(M, pose, params_num)






#define the input vars
r_11 = T[0,0]
r_12 = T[0,1]
r_13 = T[0,2]
r_21 = T[1,0]
r_22 = T[1,1]
r_23 = T[1,2]
r_31 = T[2,0]
r_32 = T[2,1]
r_33 = T[2,2]
Px = T[0,3]
Py = T[1,3]
Pz = T[2,3]

    

#
# Caution:    Generated code is not yet validated
#
    #Variable:  d_1
d_1 = Pz - l_4*r_33


#Variable:  th_2
th_2 = atan2((Px - l_1 - l_4*r_13)/l_2, -(Py - l_4*r_23)/l_2)


#Variable:  th_3
th_3s2 = atan2(-r_33, -r_13*cos(th_2) - r_23*sin(th_2))
th_3s1 = atan2(r_33, r_13*cos(th_2) + r_23*sin(th_2))


#Variable:  th_4
th_4s1 = atan2(-r_33/sin(th_3s2), r_13*sin(th_2) - r_23*cos(th_2))
th_4s2 = atan2(-r_33/sin(th_3s1), r_13*sin(th_2) - r_23*cos(th_2))


#Variable:  th_5
th_5s1 = atan2((-r_12*sin(th_2) + r_22*cos(th_2))/sin(th_4s1), (r_11*sin(th_2) - r_21*cos(th_2))/sin(th_4s1))
th_5s2 = atan2((-r_12*sin(th_2) + r_22*cos(th_2))/sin(th_4s2), (r_11*sin(th_2) - r_21*cos(th_2))/sin(th_4s2))

##################################
#
#package the solutions into a list for each set
#
###################################

solution_list = []
#(note trailing commas allowed in python
solution_list.append( [  d_1,  th_2,  th_3s2,  th_4s1,  th_5s1,  ] )
#(note trailing commas allowed in python
solution_list.append( [  d_1,  th_2,  th_3s1,  th_4s2,  th_5s2,  ] )

 
#
#package the solutions into a list for each set
#
###################################

# convert to degree
#solution_list = convert_to_degrees(solution_list)

# create a template
# write down the symbol for each variable in order (solution_list)
variable_temp = sp.var(['d_1', 'th_2', 'th_3', 'th_4', 'th_5'])

# test if each pose can generate the same T matrices

T_matrices = verify_T_matrices(solution_list, variable_temp, M, params_num)


# save_dir = 'solution_num/'
# if not os.path.isdir(save_dir):
#     os.mkdir(save_dir)

# save the original version
# np.save('solution_num/Puma_Solutions.npy', solution_list)
# np.save('solution_num/Puma_T_matrices.npy', T_matrices)

# print the truncated/round up version

# print poses/solutions
# to print float up to 5 decimal places
class prettyfloat(float):
    def __repr__(self):
        #return "%0.5f" % self
        return "{:10.5f}".format(self)

# print all sets of poses     
for a_set in solution_list:
    truncated_pose = map(prettyfloat, a_set)
    print truncated_pose

# this indexing is only for Numpy Array
def printNumArray(a):
    for row in range(a.shape[0]):
        row_ls = a[row].tolist()
        truncate = map(prettyfloat, row_ls[0])
        print truncate
    print '\n'

    
# print the original T matrix
print "the original T matrix"
printNumArray(T)

print "T matrices calculated from solution poses"
for one_T in T_matrices:
    printNumArray(one_T)




