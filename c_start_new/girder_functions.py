import numpy as np

def trans_matrix(t):
    M = np.eye(4,4)
    M[0,3] = t[0,0]
    M[1,3] = t[1,0]
    M[2,3] = t[2,0]
    return M

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

def trans_t3(t3):
    l = -0.25*t3
    t = np.array([[0],[0],[l]])
    # Assuming pure translation in z for t3
    T =  trans_matrix(t)
    # T = trans_matrix(t)
    return T

def trans_t2(t2):
    l = -0.25*t2
    yaw = -l/120
    t_t3 = np.array([[150],[0],[50]])
    T = trans_matrix(t_t3) @ rot_matrix(yaw,axis='y') @ trans_matrix(-t_t3)
    return T

def trans_t1(t1):
    l = -0.25*t1
    yaw = l/120
    t_t3 = np.array([[150],[0],[50]])
    T = trans_matrix(t_t3) @ rot_matrix(yaw,axis='y') @ trans_matrix(-t_t3)
    return  T

def trans_w1(n1):
    d1 = 250
    h  = 0.1
    alpha = -(n1*h)/d1
    t = np.array([[125],[0],[0],[1]])
    T = trans_matrix(-t) @ rot_matrix(alpha,'z') @ trans_matrix(t)
    return T

def trans_w2(n2):
    d2 = 88.57649915763257
    h  = 0.1
    gamma  = -(n2*h)/d2
    t = np.array([[-125],[0],[0],[1]])
    angle_A = 0.3561858765 #angle between wedge12 and 13
    rot_a = angle_A / 2
    T = trans_matrix(-t) @ rot_matrix(-rot_a,'y') @ rot_matrix(-gamma,'x') @ rot_matrix(rot_a,'y') @trans_matrix(t)
    return T

def trans_w3(n3):
    d3 = 88.57649915763257
    h  = 0.1
    gamma  = -(n3*h)/d3
    t = np.array([[-125],[0],[0],[1]])
    angle_A = 0.3561858765 #angle between wedge12 and 13
    rot_a = angle_A / 2
    T = trans_matrix(-t) @ rot_matrix(rot_a,'y') @ rot_matrix(gamma,'x') @ rot_matrix(-rot_a,'y') @trans_matrix(t)
    return T


def move_girder(turns):
    t1, t2, t3, w1, w2, w3 = turns
    T = trans_w3(w3) @ trans_w2(w2) @ trans_w1(w1) @ trans_t3(t3) @ trans_t2(t2) @ trans_t1(t1)
    return T
