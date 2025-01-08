import numpy as np
from IPython.display import clear_output
from scipy.optimize import least_squares

def trans_matrix(t):
    M = np.eye(4,4)
    M[0,3] = t[0,0]
    M[1,3] = t[1,0]
    M[2,3] = t[2,0]
    return M

# Function for the distance between two points
def distance(p1, p2):
    return np.linalg.norm(p2 - p1)

# Apply transformation to a point middle plate
def transform(p, angle, t):
    R = rot_matrix(angle)
    T = trans_matrix(t)
    return T @ R @ p
    # vec_trans = p+t
    # vec_trans[3,0] = 1
    # return (R @ vec_trans)

def rot_matrix(angle,axis='y'):
    if axis == 'x':
        R = np.array([
        [1, 0, 0, 0],
        [0,np.cos(angle),-np.sin(angle), 0],
        [0,np.sin(angle), np.cos(angle), 0],
        [0, 0, 0, 1]])
    elif axis == 'y':
        R = np.array([
        [np.cos(angle), 0, np.sin(angle), 0],
        [0, 1, 0, 0],
        [-np.sin(angle), 0, np.cos(angle), 0],
        [0, 0, 0, 1]])
    elif axis == 'z':
        R = np.array([
        [np.cos(angle),-np.sin(angle),0, 0],
        [np.sin(angle), np.cos(angle),0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]])
    else:
        R = np.eye(4,4)
        print('invalid axis given')
    return R

def trans_w1(n1):
    d1 = 250
    h  = 0.1
    alpha = (n1*h)/d1
    t = np.array([[125],[0],[0],[1]])
    T = trans_matrix(-t) @ rot_matrix(alpha,'z') @ trans_matrix(t)
    return T

def trans_w2(n2):
    d2 = 88.57649915763257
    h  = 0.1
    gamma  = (n2*h)/d2
    t = np.array([[-125],[0],[0],[1]])
    angle_A = 0.3561858765 #angle between wedge12 and 13
    rot_a = angle_A / 2
    T = trans_matrix(-t) @ rot_matrix(-rot_a,'y') @ rot_matrix(-gamma,'x') @ rot_matrix(rot_a,'y') @trans_matrix(t)
    return T

def trans_w3(n3):
    d3 = 88.57649915763257
    h  = 0.1
    gamma  = (n3*h)/d3
    t = np.array([[-125],[0],[0],[1]])
    angle_A = 0.3561858765 #angle between wedge12 and 13
    rot_a = angle_A / 2
    T = trans_matrix(-t) @ rot_matrix(rot_a,'y') @ rot_matrix(gamma,'x') @ rot_matrix(-rot_a,'y') @trans_matrix(t)
    return T

def solve(values):
	def residuals(vars):
		u, w, yaw = vars
		t = np.array([[u],[0],[w],[1]])

		# Points in layer 1
		p11 = np.array([[values['p11x']],[0], [values['p11z']],[1]])
		p12 = np.array([[values['p12x']],[0], [values['p12z']],[1]])
		p13 = np.array([[values['p13x']],[0], [values['p13z']],[1]])

		# Initial points in layer 2
		p21_0 = np.array([[values['p21x_0']],[0], [values['p21z_0']],[1]])
		p22_0 = np.array([[values['p22x_0']],[0], [values['p22z_0']],[1]])
		p23_0 = np.array([[values['p23x_0']],[0], [values['p23z_0']],[1]])

		# Transformed points in layer 2
		p21_i = transform(p21_0, yaw, t)
		p22_i = transform(p22_0, yaw, t)
		p23_i = transform(p23_0, yaw, t)

		# Calculate distances
		d1_0 = distance(p11, p21_0) #before
		d1_i = distance(p11, p21_i) #after

		d2_0 = distance(p12, p22_0)
		d2_i = distance(p12, p22_i)

		d3_0 = distance(p13, p23_0)
		d3_i = distance(p13, p23_i)

		# Constraints 

		# eq1, eq2, eq3 are the equations for the distances between the turnbuckles attached in layer 1 and layer 2

		# dist_1 is the distance inc/dec in the first turnbuckle 
		# dist_2 is the distance inc/dec in the second turnbuckle
		# dist_3 is the distance inc/dec in the third turnbuckle
		# if the distance is 0, then the corresponding eq is not changed
		eq1 = d1_0 + values['dist_1'] - d1_i
		eq2 = d2_0 + values['dist_2'] - d2_i
		eq3 = d3_0 + values['dist_3'] - d3_i
			
		# eq4, eq5, eq6 are the equations for the distances between the turnbuckles points in layer 2
		d12_0 = distance(p21_0, p22_0)
		d12_i = distance(p21_i, p22_i)

		d23_0 = distance(p22_0, p23_0)
		d23_i = distance(p22_i, p23_i)

		d13_0 = distance(p21_0, p23_0)
		d13_i = distance(p21_i, p23_i)

		# they are constant even after the transformation
		eq4 = d12_0 - d12_i
		eq5 = d23_0 - d23_i
		eq6 = d13_0 - d13_i

		# all the equations are returned to the least_squares function
		return [eq1, eq2, eq3, eq4, eq5, eq6]
	
	# define bounds for (u, w, yaw) the translation should not go beyond 3 mm
	bounds = ([-3, -3, -np.inf], [3, 3, np.inf])
	initial_guess = [0.0, 0.0, 0.0]

	# This gives the best u,w,yaw for the given constraints
	solution = least_squares(residuals, initial_guess, bounds=bounds)
	u, w, yaw = solution.x

	return yaw, u, w

def change_wedge(turns):
    w1,w2,w3 = turns
    magnet_c = np.array([[0],[205],[0],[1]])
    m_c_l3 = trans_w2(w2) @ trans_w3(w3) @ trans_w1(w1) @ magnet_c
    delta_y = np.round(m_c_l3[1,0],10)-205
    pitch =  np.round(m_c_l3[2,0]/205,10) # around x 
    roll =   np.round(m_c_l3[0,0]/205,10) # around z
    return np.array([delta_y,pitch,roll])

def move_girder(turns):
    t1,t2,t3,w1,w2,w3 = np.array(turns)
    magnet_b = np.array([[0],[0],[0],[1]])
    magnet_c = np.array([[0],[205],[0],[1]])
    # Distance changed by one rotation of turnbuckle
    delta_dist = 0.25
    # Initial points in layer 2
    p21_0 = np.array([-30, -60])
    p22_0 = np.array([-30, 60])
    p23_0 = np.array([150, 50])

    # Values for the constraints
    values = {
        'p11x': 75, 'p11z': -60, 					# x,z coordinates of the first turnbuckle in layer 1
        'p12x': 75, 'p12z': 60,					# x,z coordinates of the second turnbuckle in layer 1
        'p13x': 150, 'p13z': -55,					# x,z coordinates of the third turnbuckle in layer 1

        'p21x_0': p21_0[0], 'p21z_0': p21_0[1],		# x,z coordinates of the first turnbuckle in layer 2
        'p22x_0': p22_0[0], 'p22z_0': p22_0[1],	 	# x,z coordinates of the second turnbuckle in layer 2
        'p23x_0': p23_0[0], 'p23z_0': p23_0[1], 	# x,z coordinates of the third turnbuckle in layer 2

        'dist_1': t1 * delta_dist,			# distance inc/dec in the first turnbuckle
        'dist_2': t2 * delta_dist,			# distance inc/dec in the second turnbuckle
        'dist_3': t3 * delta_dist			# distance inc/dec in the third turnbuckle
    }
        
    yaw,u,w = solve(values) 
    t = np.array([[u],[0],[w],[1]])

    # magnet_c = transform(magnet_c,yaw,t) @ trans_w3(w3) @ trans_w2(w2) @ trans_w1(w1) 
    # magnet_b = transform(magnet_b,yaw,t) @ trans_w3(w3) @ trans_w2(w2) @ trans_w1(w1) 
    m_c_l3 = trans_w1(w1) @ trans_w3(w3) @ trans_w2(w2) @  magnet_c
    m_c_new = transform(m_c_l3,yaw,t)

    # delta_x = np.round(magnet_c[0,0],10)
    # delta_y = np.round(magnet_b[1,0],10)
    # delta_z = np.round(magnet_c[2,0],10)
    # yaw = np.round(yaw,10)
    # pitch =  np.round(magnet_c[2,0]/205,10) # around x 
    # roll =  np.round(magnet_c[0,0]/205,10) # around z
    delta_x = np.round(m_c_new[0,0],10)
    delta_y = np.round(m_c_new[1,0],10)-205
    delta_z = np.round(m_c_new[2,0],10)
    yaw = np.round(yaw,10)
    pitch =  np.round(m_c_l3[2,0]/205,10) # around x 
    roll =   np.round(m_c_l3[0,0]/205,10) # around z
    return np.array([delta_x,delta_y,delta_z,pitch,yaw,roll])

# def move_girder_inv(target,penalty_weight=0):
#     def residual_function(inputs):
#         # Compute the output of move_girder
#         output = move_girder(inputs)
#         # Calculate residuals (difference between output and target)
#         weights = np.array([1, 1, 1, 1, 1, 1])
#         residuals = (output - target).flatten()
#         residuals = residuals * weights
#         penalty = penalty_weight * np.sum((inputs - np.round(inputs))**2)
#         return residuals + penalty

#     initial_guess = [0, 0, 0, 0, 0, 0]  # Initial guess for [t1, t2, t3, w1, w2, w3]
#     # Use least_squares to minimize the residuals
#     lower_bounds = [-12] * len(initial_guess)  
#     upper_bounds = [12] * len(initial_guess)   

#     # Set the bounds as a tuple
#     bounds = (lower_bounds, upper_bounds)

#     result = least_squares(residual_function, initial_guess, bounds=bounds) 
#     if result.success:
#         return result.x  # Return the inputs that minimize the residuals
#     else:
#         raise ValueError("Least squares optimization failed to converge.")

def move_girder_inv(target):
    x,y,z,pitch,yaw,roll = target
    l3_target = np.array([y,pitch,roll])
    l2_target = [x,z,yaw]
    def residual_function(inputs):
        # Compute the output of move_girder
        output = change_wedge(inputs)
        # Calculate residuals (difference between output and target)
        residuals = (output - l3_target).flatten()
        return residuals
    initial_guess = [0, 0, 0]  # Initial guess for [w1, w2, w3]
    # Use least_squares to minimize the residuals
    lower_bounds = [-12] * len(initial_guess)  
    upper_bounds = [12] * len(initial_guess)   

    # Set the bounds as a tuple
    bounds = (lower_bounds, upper_bounds)

    result = least_squares(residual_function, initial_guess, bounds=bounds) 
    w1,w2,w3 = result.x
    return w1,w2,w3