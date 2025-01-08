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
    print(yaw)
    t_t3 = np.array([[150],[0],[50]])
    T = trans_matrix(t_t3) @ rot_matrix(yaw,axis='y') @ trans_matrix(-t_t3)
    return T

def trans_t1(t1):
    l = -0.25*t1
    yaw = l/120
    print(yaw)
    t_t3 = np.array([[150],[0],[50]])
    T = trans_matrix(t_t3) @ rot_matrix(yaw,axis='y') @ trans_matrix(-t_t3)
    return  T

