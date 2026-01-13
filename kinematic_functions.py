import numpy as np
import sys
import math


class KinematicFunctions():




   def calculate_dh_transform(self, joint_positions):


       ####################################### Your Code Starts Here #######################################


       # TODO: DH parameters for UR3e. Modify these parameters according to the robot's configuration # a , alpha, d, theta
       t1, t2, t3, t4, t5, t6, t7 = ([
           np.radians(135),
           joint_positions[0]-np.radians(135),
           joint_positions[1],
           joint_positions[2],
           joint_positions[3]-np.radians(90),
           joint_positions[4],
           joint_positions[5]
       ])
       # Array of the seven angles.
       thetas = [t1, t2, t3, t4, t5, t6, t7]


       # Creates the arrays of the DH parameters. d a and alpha. Based on the DH table.
       d = np.array([0, 0.162, 0.027, 0, 0.104, 0.083, 0.151])
       a = np.array([0.15*np.sqrt(2), 0, 0.244, 0.213, 0, 0, 0.0535])
       alpha = np.array([0, -90, 0, 0, -90, 90, 0])




       # Determine Individual Transforms Uses a loop and a blank array (A) to build the set of
       # matrices and store them in an array.
       A = [None] * 7
       for i in range(7):
           theta = thetas[i]
           ct = np.cos(theta)
           st = np.sin(theta)                  
           ca = np.cos(np.radians(alpha[i]))
           sa = np.sin(np.radians(alpha[i]))




           # Building Individual Transforms using the formula.
           A[i] = np.array([
               [ct, -st * ca,  st * sa, a[i] * ct],
               [st,  ct * ca, -ct * sa, a[i] * st],
               [0,        sa,       ca,      d[i]],
               [0,         0,        0,         1]
           ])




       # Final transformation matrix (uses np.eye to initialize an identity matrix to begin the
       # transformations on. Then applies each transform from the array.
       transform = np.eye(4)
       for arr in A:
           transform = transform @ arr




       ############################# Your Code Ends Here #######################################
      


       return transform




   def inverse_kinematics(self, xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
      
       return_value = np.array([0, 0, 0, 0, 0, 0])


       ####################################### Your Code Starts Here #######################################


       # TODO: Function that calculates an elbow up inverse kinematics solution for the UR3


       # Step 1: find gripper position relative to the base of UR3, and set theta_5 equal to -pi/2
       frame_w = np.array([-0.15, 0.15, 0.01])
       theta_5 = -np.pi/2
       grip_pos = np.array([xWgrip, yWgrip, zWgrip]) - np.array(frame_w)
       l1 = 0.152
       l2 = 0.120
       l3 = 0.244
       l4 = 0.093
       l5 = 0.213
       l6 = 0.104
       l7 = 0.083
       l8 = 0.092
       l9 = 0.0535
       l10 = 0.059




       # Step 2: find x_cen, y_cen, z_cen
       yaw = np.radians(yaw_WgripDegree)
       xcen = grip_pos[0] - (l9 * np.cos(yaw))
       ycen = grip_pos[1] - (l9 * np.sin(yaw))
       zcen = grip_pos[2]




       # Step 3: find theta_1
       theta_c = np.arctan2(ycen, xcen)


       r_c = np.sqrt(np.square(xcen) + np.square(ycen))
       s_b = (l6 + 0.027) / r_c
       c_b = np.sqrt(1 - np.square(s_b))
       theta_b = np.arctan2(s_b, c_b)




       theta_1 = theta_c - theta_b




       # Step 4: find theta_6
       theta_6 = theta_1 + np.pi/2 - yaw




       # Step 5: find x3_end, y3_end, z3_end




       x3end = xcen - (l7 * np.cos(theta_1)) + ((l6 + 0.027) * np.sin(theta_1))
       y3end = ycen - (l7 * np.sin(theta_1)) - ((l6 + 0.027) * np.cos(theta_1))
       z3end = zcen + (l8 + l10)




       # Step 6: find theta_2, theta_3, theta_4
       r = np.sqrt(np.square(x3end) + np.square(y3end) + np.square(z3end - l1))




       c21 = (np.square(l3) + np.square(r) - np.square(l5)) / (2 * l3 * r)
       s21 = np.sqrt(1 - np.square(c21))
       s22 = (z3end - l1) / r
       c22 = np.sqrt(1 - np.square(s22))
       theta_2 = -1 * (np.arctan2(s21, c21) + np.arctan2(s22, c22))




       c3 = (np.square(l3) + np.square(l5) - np.square(r)) / (2 * l3 * l5)
       s3 = np.sqrt(1 - np.square(c3))
       theta_3 = np.pi - np.arctan2(s3, c3)




       theta_4 = -1 * (theta_2 + theta_3)




       ############################# Your Code Ends Here #######################################


       # print theta values (in degree) calculated from inverse kinematics
      
       # print("Correct Joint Angles: ")
       # print(str(theta_1*180/np.pi) + " " + str(theta_2*180/np.pi) + " " + \
       #         str(theta_3*180/np.pi) + " " + str(theta_4*180/np.pi) + " " + \
       #         str(theta_5*180/np.pi) + " " + str(theta_6*180/np.pi))


       # obtain return_value from forward kinematics function
       return_value = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]


       return return_value



