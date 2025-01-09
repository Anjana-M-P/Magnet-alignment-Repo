from sympy import symbols, Matrix, eye, sin, cos

# Define symbolic variables
t1, t2, t3, w1, w2, w3 = symbols('t1 t2 t3 w1 w2 w3')
alpha, gamma, angle_A = symbols('alpha gamma angle_A')
h = 0.1
d1, d2, d3 = 250, 88.57649915763257, 88.57649915763257

# Translation matrix
def trans_matrix(t):
    M = eye(4)
    M[0, 3] = t[0]
    M[1, 3] = t[1]
    M[2, 3] = t[2]
    return M

# Rotation matrix with small angle approximation
def rot_matrix(angle, axis='y'):
    if axis == 'x':
        R = Matrix([
            [1, 0, 0, 0],
            [0, 1, -angle, 0],
            [0, angle, 1, 0],
            [0, 0, 0, 1]])
    elif axis == 'y':
        R = Matrix([
            [1, 0, angle, 0],
            [0, 1, 0, 0],
            [-angle, 0, 1, 0],
            [0, 0, 0, 1]])
    elif axis == 'z':
        R = Matrix([
            [1, -angle, 0, 0],
            [angle, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
    else:
        R = eye(4)
    return R


# Translation transformations
def trans_t3(t3):
    l = -0.25 * t3
    t = Matrix([0, 0, l])
    T = trans_matrix(t)
    return T

def trans_t2(t2):
    l = -0.25 * t2
    yaw = -l / 120
    t_t3 = Matrix([150, 0, 50])
    T = trans_matrix(t_t3) @ rot_matrix(yaw, axis='y') @ trans_matrix(-t_t3)
    return T

def trans_t1(t1):
    l = -0.25 * t1
    yaw = l / 120
    t_t3 = Matrix([150, 0, 50])
    T = trans_matrix(t_t3) @ rot_matrix(yaw, axis='y') @ trans_matrix(-t_t3)
    return T

# Wedge transformations
def trans_w1(n1):
    alpha = (n1 * h) / d1
    t = Matrix([125, 0, 0])
    T = trans_matrix(-t) @ rot_matrix(alpha, 'z') @ trans_matrix(t)
    return T

def trans_w2(n2):
    gamma = (n2 * h) / d2
    t = Matrix([-125, 0, 0])
    angle_A = 0.3561858765 
    rot_a = angle_A / 2
    T = trans_matrix(-t) @ rot_matrix(-rot_a, 'y') @ rot_matrix(-gamma, 'x') @ rot_matrix(rot_a, 'y') @ trans_matrix(t)
    return T

def trans_w3(n3):
    gamma = (n3 * h) / d3
    t = Matrix([-125, 0, 0])
    angle_A = 0.3561858765
    rot_a = angle_A / 2
    T = trans_matrix(-t) @ rot_matrix(rot_a, 'y') @ rot_matrix(gamma, 'x') @ rot_matrix(-rot_a, 'y') @ trans_matrix(t)
    return T

# Combined transformation
def move_girder(turns):
    t1, t2, t3, w1, w2, w3 = turns
    T = trans_w3(w3) @ trans_w2(w2) @ trans_w1(w1) @ trans_t3(t3) @ trans_t2(t2) @ trans_t1(t1)
    return T
